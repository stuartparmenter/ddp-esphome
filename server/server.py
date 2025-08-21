# © Copyright 2025 Stuart Parmenter
# SPDX-License-Identifier: MIT
#!/usr/bin/env python3
"""
server.py
- WebSocket control at /control?w=..&h=..&out=..&src=..[&pace=N][&ema=A][&expand=(0|1|2)][&loop=1][&hw=auto]
- Sends full RGB888 frames via DDP (UDP/4048). One frame = multiple <=1440B packets.
- pace=Hz up-samples only (duplicates). If pace==0 or omitted, native cadence is used.
"""

import asyncio
import json
import os
import platform
import shutil
import struct
import subprocess
import time
import contextlib
from urllib.parse import urlparse, parse_qs, unquote
try:
    from urllib.request import url2pathname  # Windows-safe file:///C:/... -> C:\...
except Exception:
    def url2pathname(p): return p  # fallback
from typing import Tuple, Optional, Dict, Any

import numpy as np
import imageio.v3 as iio
from PIL import Image, ImageOps
import websockets

# Optional PyAV (preferred for videos/VFR)
try:
    import av  # PyAV
    _HAVE_PYAV = True
except Exception:
    _HAVE_PYAV = False

MIN_DELAY_MS = 10

DEFAULT_CONFIG: Dict[str, Any] = {
    "hw": {"prefer": "auto"},
    "video": {"expand_mode": 1},  # 0=never, 1=auto(limited), 2=force
    "playback": {"loop": True},
    "log": {"send_ms": False},
    "net": {"win_timer_res": True}
}

def _load_config_file(path: str) -> Dict[str, Any]:
    ext = os.path.splitext(path)[1].lower()
    if not os.path.exists(path):
        return {}
    try:
        if ext in (".yaml", ".yml"):
            try:
                import yaml  # type: ignore
            except Exception:
                print("[config] PyYAML not installed; skipping YAML.")
                return {}
            with open(path, "r", encoding="utf-8") as f:
                return yaml.safe_load(f) or {}
        elif ext == ".toml":
            try:
                import tomllib  # 3.11+
            except Exception:
                try:
                    import tomli as tomllib  # type: ignore
                except Exception:
                    print("[config] tomllib/tomli not installed; skipping TOML.")
                    return {}
            with open(path, "rb") as f:
                return tomllib.load(f) or {}
        elif ext == ".json":
            with open(path, "r", encoding="utf-8") as f:
                return json.load(f) or {}
        else:
            print(f"[config] Unknown config extension: {ext}")
            return {}
    except Exception as e:
        print(f"[config] Failed to load {path}: {e}")
        return {}

def _deep_update(dst: Dict[str, Any], src: Dict[str, Any]) -> Dict[str, Any]:
    for k, v in src.items():
        if isinstance(v, dict) and isinstance(dst.get(k), dict):
            _deep_update(dst[k], v)
        else:
            dst[k] = v
    return dst

def load_config(path: Optional[str]) -> Dict[str, Any]:
    cfg = json.loads(json.dumps(DEFAULT_CONFIG))
    if path:
        _deep_update(cfg, _load_config_file(path))
    else:
        auto = "ws_ddp_proxy.yaml"
        if os.path.exists(auto):
            _deep_update(cfg, _load_config_file(auto))
    return cfg

def _resolve_local_path(srcu: str) -> Optional[str]:
    u = urlparse(srcu)
    if u.scheme in ("", "file"):
        p = u.path if u.scheme == "file" else srcu
        if os.name == "nt" and len(p) >= 3 and p[0] == "/" and p[2] == ":":
            p = p[1:]
        return url2pathname(p)
    return None

def _truthy(s: str) -> bool:
    return s.lower() not in ("0", "false", "no", "off", "")

def _get_bool(q, key: str, default: bool) -> bool:
    v = q.get(key, [None])[0]
    return default if v is None else _truthy(str(v))

def _get_int(q, key: str, default: int) -> int:
    v = q.get(key, [None])[0]
    try:
        return int(v) if v is not None else default
    except Exception:
        return default

def _get_float(q, key: str, default: float) -> float:
    v = q.get(key, [None])[0]
    try:
        return float(v) if v is not None else default
    except Exception:
        return default

def _ffmpeg_has_hwaccel(name: str) -> bool:
    exe = shutil.which("ffmpeg")
    if not exe:
        return False
    try:
        out = subprocess.check_output([exe, "-hide_banner", "-hwaccels"], text=True, stderr=subprocess.STDOUT)
        return any(line.strip().lower() == name.lower() for line in out.splitlines())
    except Exception:
        return False

def pick_hw_backend(prefer: str | None = None) -> tuple[Optional[str], dict]:
    sys = platform.system().lower()
    prefer = (prefer or "auto").lower()

    def ok(n): return _ffmpeg_has_hwaccel(n)

    if prefer not in ("", "auto", "none"):
        return (prefer if ok(prefer) else None, {})

    if sys == "windows":
        if ok("cuda"): return "cuda", {}
        if ok("d3d11va"): return "d3d11va", {}
        if ok("qsv"): return "qsv", {}
        return None, {}
    if sys == "darwin":
        if ok("videotoolbox"): return "videotoolbox", {}
        return None, {}
    if ok("vaapi"): return "vaapi", {"device": "/dev/dri/renderD128"}
    if ok("qsv"): return "qsv", {}
    if ok("cuda"): return "cuda", {}
    return None, {}

def _resize_pad_to_rgb_bytes(img: Image.Image, size: Tuple[int, int]) -> bytes:
    w, h = size
    if img.mode != "RGB":
        img = img.convert("RGB")
    im = ImageOps.contain(
        img, size,
        method=Image.Resampling.NEAREST if getattr(img, "is_animated", False)
        else Image.Resampling.BILINEAR
    )
    if im.size != size:
        canvas = Image.new("RGB", size, (0, 0, 0))
        canvas.paste(im, ((w - im.size[0]) // 2, (h - im.size[1]) // 2))
        im = canvas
    return np.asarray(im, dtype=np.uint8).tobytes()

def _apply_rotation_rgb_array(arr: np.ndarray, rotate_deg: int) -> np.ndarray:
    if rotate_deg % 360 == 0:
        return arr
    k = (rotate_deg // 90) % 4
    if k == 1: return np.rot90(arr, 1)
    if k == 2: return np.rot90(arr, 2)
    if k == 3: return np.rot90(arr, 3)
    return arr

def _rotation_from_stream_and_frame(vstream, frame) -> int:
    try:
        rot = getattr(frame, "rotation", None)
        if isinstance(rot, int):
            return ((rot % 360) + 360) % 360
    except Exception:
        pass
    try:
        md = getattr(vstream, "metadata", {})
        if md:
            r = md.get("rotate")
            if r is not None:
                return ((int(str(r)) % 360) + 360) % 360
    except Exception:
        pass
    return 0

def _iter_frames_imageio(srcu: str, size: Tuple[int, int], loop_video: bool):
    default_delay_ms = 100
    try:
        props = iio.improps(srcu)
        fps = getattr(props, "fps", None)
        if fps and fps > 0:
            default_delay_ms = max(MIN_DELAY_MS, int(round(1000.0 / float(fps))))
    except Exception:
        pass

    plugin = "pillow" if srcu.lower().endswith(".gif") else None

    while True:
        try:
            reader = iio.imiter(srcu, plugin=plugin) if plugin else iio.imiter(srcu)
            saw = False
            for frame in reader:
                saw = True
                im = Image.fromarray(np.asarray(frame)).convert("RGB")
                rgb888 = _resize_pad_to_rgb_bytes(im, size)
                delay_ms = default_delay_ms
                try:
                    if hasattr(frame, "meta"):
                        d = frame.meta.get("duration")
                        if d is not None:
                            delay_ms = int(d * 1000) if isinstance(d, float) else int(d)
                except Exception:
                    pass
                yield rgb888, max(MIN_DELAY_MS, delay_ms)
            if not loop_video:
                break
            if not saw:
                raise FileNotFoundError(f"cannot decode frames: {srcu}")
        except Exception as e2:
            msg = str(e2).lower()
            if isinstance(e2, FileNotFoundError) or "no such file" in msg or "not found" in msg:
                raise FileNotFoundError(f"cannot open source: {srcu}") from e2
            raise RuntimeError(f"imageio error: {e2}") from e2
        if not loop_video:
            break

def _open_with_hwaccel(srcu: str, prefer: str | None):
    if not _HAVE_PYAV:
        return av.open(srcu, mode="r"), None, None  # type: ignore
    kind, kw = pick_hw_backend(prefer)
    container = av.open(srcu, mode="r")
    vstream = next((s for s in container.streams if s.type == "video"), None)
    if vstream is None:
        raise RuntimeError("no video stream")
    if not kind:
        return container, vstream, None
    try:
        hw = av.hwdevice.Context.create(kind, kw.get("device", None)) if kw else av.hwdevice.Context.create(kind)
        dec = av.codec.CodecContext.create(vstream.codec.name, "r")
        dec.options = vstream.codec_context.options
        dec.hw_device_ctx = hw
        return container, vstream, dec
    except Exception as e:
        print(f"[hwaccel disabled: {kind} not available: {e}]")
        return container, vstream, None

def _iter_frames_pyav(srcu: str, size: Tuple[int, int], loop_video: bool,
                      *, expand_mode: int, hw_prefer: str | None):
    while True:
        try:
            container, vstream, dec = _open_with_hwaccel(srcu, hw_prefer)
            if vstream is None:
                raise RuntimeError("no video stream")

            avg_ms = None
            if vstream.average_rate:
                try:
                    fps = float(vstream.average_rate)
                    if fps > 0:
                        avg_ms = max(MIN_DELAY_MS, int(round(1000.0 / fps)))
                except Exception:
                    pass

            last_pts_s: Optional[float] = None

            for packet in container.demux(vstream):
                frames = dec.decode(packet) if dec else packet.decode()
                for frame in frames:
                    arr = frame.to_ndarray(format="rgb24")

                    # limited-range expand?
                    do_expand = (expand_mode == 2)
                    if expand_mode == 1:
                        rng = getattr(frame, "color_range", None)
                        try:
                            rng_name = rng.name if hasattr(rng, "name") else str(rng)
                        except Exception:
                            rng_name = None
                        do_expand = bool(rng_name and "mpeg" in str(rng_name).lower())
                    if do_expand:
                        arr = np.clip((arr.astype(np.int16) - 16) * 255 // 219, 0, 255).astype(np.uint8)
                    else:
                        if arr.dtype != np.uint8:
                            arr = arr.astype(np.uint8)

                    # rotation
                    rotate_deg = _rotation_from_stream_and_frame(vstream, frame)
                    if rotate_deg:
                        arr = _apply_rotation_rgb_array(arr, rotate_deg)

                    im = Image.fromarray(arr).convert("RGB")
                    rgb888 = _resize_pad_to_rgb_bytes(im, size)

                    delay_ms = avg_ms if avg_ms is not None else 100
                    pts_s = None
                    if frame.pts is not None:
                        tb = frame.time_base or vstream.time_base
                        if tb is not None:
                            pts_s = float(frame.pts * tb)
                    if pts_s is not None:
                        if last_pts_s is None:
                            delay_ms = avg_ms if avg_ms is not None else 33
                        else:
                            delta_ms = int(round((pts_s - last_pts_s) * 1000.0))
                            if delta_ms <= 0 and avg_ms is not None:
                                delta_ms = avg_ms
                            delay_ms = max(MIN_DELAY_MS, delta_ms)
                        last_pts_s = pts_s

                    yield rgb888, delay_ms

            container.close()
            if not loop_video:
                break
        except Exception as e:
            msg = str(e).lower()
            if isinstance(e, FileNotFoundError) or "no such file" in msg or "not found" in msg:
                raise FileNotFoundError(f"cannot open source: {srcu}") from e
            raise RuntimeError(f"av error: {e}") from e
        if not loop_video:
            break

def iter_frames(src: str, size: Tuple[int, int], loop_video: bool = True,
                *, expand_mode: int, hw_prefer: str | None):
    srcu = unquote(src)
    low = srcu.lower()
    if _HAVE_PYAV and not any(low.endswith(ext) for ext in (".png", ".jpg", ".jpeg", ".bmp", ".webp", ".gif")):
        yield from _iter_frames_pyav(srcu, size, loop_video, expand_mode=expand_mode, hw_prefer=hw_prefer)
    else:
        yield from _iter_frames_imageio(srcu, size, loop_video)

def ddp_send_frame(sock, addr, rgb_bytes: bytes, output_id: int, seq: int):
    MAX_DATA = 1440
    off = 0
    total = len(rgb_bytes)
    while off < total:
        chunk = rgb_bytes[off: off + MAX_DATA]
        push = 0x01 if (off + len(chunk) >= total) else 0x00
        flags = 0x40 | push
        pixcfg = 0x2C  # RGB, 8bpc
        hdr = struct.pack("!BBB B I H",
                          flags,
                          seq & 0xFF,
                          pixcfg,
                          output_id & 0xFF,
                          off,
                          len(chunk))
        sock.sendto(hdr + chunk, addr)
        off += len(chunk)

# -------------------------
# Paced sender (upsample)
# -------------------------
async def _paced_sender(sock, addr, output_id, *, pace_hz, ema_alpha, get_latest_bytes):
    """Send at fixed rate, using latest bytes supplier. Returns when cancelled."""
    loop_ = asyncio.get_running_loop()
    next_tick = loop_.time()
    seq = 0
    tick_s = 1.0 / pace_hz
    ema_buf: Optional[np.ndarray] = None

    while True:
        latest = get_latest_bytes()
        if latest is not None:
            payload = latest
            if ema_alpha > 0.0:
                cur = np.frombuffer(latest, dtype=np.uint8)
                if ema_buf is None or ema_buf.shape != cur.shape:
                    ema_buf = cur.copy()
                else:
                    ema_buf = (ema_buf.astype(np.float32) * (1.0 - ema_alpha) +
                               cur.astype(np.float32) * ema_alpha).astype(np.uint8)
                # For “between-frame” ticks, send the ema_buf; otherwise latest.
                payload = ema_buf.tobytes()
            t0 = time.perf_counter()
            ddp_send_frame(sock, addr, payload, output_id, seq)
            # (optional) you can add a per-tick log switch if needed
            seq = (seq + 1) & 0x0F

        next_tick += tick_s
        sleep_for = max(0.0, next_tick - loop_.time())
        if sleep_for > 0:
            await asyncio.sleep(sleep_for)

# -------------------------
# Main task per connection
# -------------------------
async def ddp_task(target_ip: str, target_port: int, out_id: int, *, size, src, opts):
    """If pace_hz>0: run producer (native cadence) + paced sender (fixed Hz).
       Else: single native-cadence loop (what worked for you)."""
    import socket
    w, h = size
    srcu = unquote(src)
    local_path = _resolve_local_path(srcu)
    if local_path is not None and not os.path.exists(local_path):
        raise FileNotFoundError(f"no such file: {local_path}")

    frames = iter_frames(
        src, (w, h),
        loop_video=opts["loop"],
        expand_mode=opts["expand_mode"],
        hw_prefer=opts["hw"]
    )

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    addr = (target_ip, target_port)

    pace_hz   = int(opts.get("pace_hz", 0))
    ema_alpha = float(opts.get("ema_alpha", 0.0))
    log_send  = bool(opts.get("log_send_ms", False))

    try:
        if pace_hz > 0:
            # ---- paced mode: producer + sender ----
            latest: Dict[str, Optional[bytes]] = {"buf": None}
            latest_lock = asyncio.Lock()

            async def producer():
                # Honor native cadence by sleeping per frame delay_ms
                for rgb888, delay_ms in frames:
                    async with latest_lock:
                        latest["buf"] = rgb888
                    await asyncio.sleep(max(1, int(delay_ms)) / 1000.0)

            def get_latest_bytes():
                # lockless read; bytes assignment is atomic enough for CPython
                return latest["buf"]

            sender_task = asyncio.create_task(
                _paced_sender(sock, addr, out_id,
                              pace_hz=pace_hz, ema_alpha=max(0.0, min(ema_alpha, 1.0)),
                              get_latest_bytes=get_latest_bytes)
            )
            try:
                await producer()  # returns when source ends (or loops forever)
            finally:
                sender_task.cancel()
                with contextlib.suppress(Exception):
                    await sender_task

        else:
            # ---- native mode: send once per source frame ----
            loop_ = asyncio.get_running_loop()
            next_deadline = loop_.time()
            seq = 0
            for rgb888, delay_ms in frames:
                t0 = time.perf_counter()
                ddp_send_frame(sock, addr, rgb888, out_id, seq)
                if log_send:
                    send_ms = int((time.perf_counter() - t0) * 1000)
                    print(f"[ddp {out_id}] seq={seq} size={w}x{h} send={send_ms}ms bytes={len(rgb888)}")
                next_deadline += max(1, int(delay_ms)) / 1000.0
                sleep_for = max(0.0, next_deadline - loop_.time())
                if sleep_for > 0:
                    await asyncio.sleep(sleep_for)
                seq = (seq + 1) & 0x0F

    finally:
        sock.close()

# =========================
# WebSocket control
# =========================
CONFIG: Dict[str, Any] = DEFAULT_CONFIG

async def handle_control(ws):
    path = getattr(getattr(ws, "request", None), "path", "/")
    q = parse_qs(urlparse(path).query)
    try:
        w   = int(q.get("w",   ["64"])[0])
        h   = int(q.get("h",   ["64"])[0])
        out = int(q.get("out", ["1"])[0])
        src = q.get("src", [""])[0]
        ddp_port = int(q.get("ddp_port", ["4048"])[0])
        if not src:
            ip = getattr(ws, "remote_address", ("?",))[0]
            print(f"! control error from {ip}: missing src (path='{path}')")
            await ws.close(code=4001, reason="missing src"); return
        if not (1 <= w <= 255 and 1 <= h <= 255 and 0 <= out <= 255):
            ip = getattr(ws, "remote_address", ("?",))[0]
            print(f"! control error from {ip}: bad dimensions/out (w={w}, h={h}, out={out}, path='{path}')")
            await ws.close(code=4002, reason="bad dimensions or out"); return
    except Exception:
        ip = getattr(ws, "remote_address", ("?",))[0]
        print(f"! control error from {ip}: bad query ({e!r}) (path='{path}')")
        await ws.close(code=4000, reason="bad query"); return

    # Early local-file fail
    srcu = unquote(src)
    local_path = _resolve_local_path(srcu)
    if local_path is not None and not os.path.exists(local_path):
        await ws.close(code=4404, reason=f"no such file: {local_path}"); return

    # expand: 0=never, 1=auto, 2=force
    expand_flag = q.get("expand", [str(CONFIG["video"]["expand_mode"])])[0]
    expand_mode = 2 if expand_flag in ("2", "force") else (1 if expand_flag not in ("0","false","False") else 0)

    loop_video = _get_bool(q, "loop", bool(CONFIG["playback"]["loop"]))

    hw_prefer = q.get("hw", [str(CONFIG["hw"]["prefer"])])[0]
    if hw_prefer.lower() in ("none", "off", "cpu"):
        hw_prefer = None

    pace_hz = _get_int(q, "pace", 0)  # upsample only
    ema_alpha = _get_float(q, "ema", 0.0)

    opts = {
        "loop": loop_video,
        "expand_mode": expand_mode,
        "hw": hw_prefer,
        "log_send_ms": bool(CONFIG["log"].get("send_ms", False)),
        "pace_hz": pace_hz,
        "ema_alpha": max(0.0, min(ema_alpha, 1.0)),
    }

    target_ip = ws.remote_address[0]
    print(f"* control from {target_ip}: out={out} size={w}x{h} ddp_port={ddp_port} "
          f"src={src} pace={pace_hz} ema={opts['ema_alpha']}")
    task = asyncio.create_task(ddp_task(target_ip, ddp_port, out, size=(w, h), src=src, opts=opts))

    try:
        await ws.wait_closed()
    finally:
        task.cancel()

async def dispatch(ws):
    path = getattr(getattr(ws, "request", None), "path", "/")
    if path.startswith("/control"):
        await handle_control(ws)
    else:
        await ws.close(code=4003, reason="unknown path")

def _win_timer_res(enable=True):
    if os.name == "nt":
        try:
            import ctypes
            if enable:
                ctypes.windll.winmm.timeBeginPeriod(1)
            else:
                ctypes.windll.winmm.timeEndPeriod(1)
        except Exception as e:
            print(f"[warn] timeBeginPeriod/timeEndPeriod failed: {e}")

async def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=8788)
    ap.add_argument("--config", default=None, help="Path to YAML/TOML/JSON config (default: ws_ddp_proxy.yaml if present)")
    args = ap.parse_args()

    global CONFIG
    CONFIG = load_config(args.config)
    print(f"* loaded config: {CONFIG}")

    print("* ws_ddp_proxy on ws://{}:{}/control?w=..&h=..&out=..&src=..".format(args.host, args.port))
    async with websockets.serve(
        dispatch, args.host, args.port,
        max_size=2**22,
        compression=None,
        ping_interval=20, ping_timeout=20
    ):
        await asyncio.Future()

if __name__ == "__main__":
    try:
        if bool(DEFAULT_CONFIG["net"]["win_timer_res"]):
            _win_timer_res(True)
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
    finally:
        if bool(DEFAULT_CONFIG["net"]["win_timer_res"]):
            _win_timer_res(False)
