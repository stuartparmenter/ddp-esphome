
# © Copyright 2025 Stuart Parmenter
# SPDX-License-Identifier: MIT
#!/usr/bin/env python3

import asyncio
import json
import os
import platform
import shutil
import struct
import subprocess
import time
import contextlib
import traceback
from urllib.parse import urlparse, unquote
try:
    from urllib.request import url2pathname
except Exception:
    def url2pathname(p): return p
from typing import Tuple, Optional, Dict, Any

import numpy as np
import imageio.v3 as iio
from PIL import Image, ImageOps
import websockets
from websockets.exceptions import ConnectionClosed, ConnectionClosedOK, ConnectionClosedError

import av
from av.filter import Graph as AvFilterGraph
av.logging.set_level(av.logging.INFO)

# -------------------------
# Metrics helpers
# -------------------------

class _RateMeter:
    """Rolling rate/jitter meter using a short timestamp window."""
    def __init__(self, window_s: float = 2.5):
        self.window_s = float(window_s)
        self.ts: list[float] = []

    def tick(self, t: float) -> None:
        self.ts.append(t)
        cut = t - self.window_s
        i = 0
        for i, v in enumerate(self.ts):
            if v >= cut:
                break
        if i > 0:
            del self.ts[:i]

    def rate_hz(self) -> float:
        n = len(self.ts)
        if n < 2:
            return 0.0
        duration = self.ts[-1] - self.ts[0]
        return (n - 1) / duration if duration > 0 else 0.0

    def jitter_ms(self) -> float:
        import math
        n = len(self.ts)
        if n < 3:
            return 0.0
        diffs = [ (self.ts[i] - self.ts[i-1]) for i in range(1, n) ]
        mean = sum(diffs) / len(diffs)
        var = sum((d - mean)**2 for d in diffs) / (len(diffs) - 1)
        return math.sqrt(var) * 1000.0

MIN_DELAY_MS = 10.0  # float ms

DEFAULT_CONFIG: Dict[str, Any] = {
    "hw": {"prefer": "auto"},
    "video": {"expand_mode": 1},  # 0=never, 1=auto(limited), 2=force
    "playback": {"loop": True},
    "log": {"send_ms": False, "rate_ms": 1000, "detail": False, "metrics": True},
    "net": {"win_timer_res": True, "spread_packets": True, "spread_max_fps": 120, "spread_min_ms": 3.0}
}

# -------------------------
# Config helpers
# -------------------------

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


# -------------------------
# Parsing / small utilities
# -------------------------

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


def _ffmpeg_exe_path() -> Optional[str]:
    exe = shutil.which("ffmpeg")
    if exe:
        return exe
    try:
        import imageio_ffmpeg  # type: ignore
        return imageio_ffmpeg.get_ffmpeg_exe()
    except Exception:
        return None


def _ffmpeg_has_hwaccel(name: str) -> bool:
    exe = _ffmpeg_exe_path()
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

    ALIASES = {"d3d11": "d3d11va", "gpu": "cuda"}
    def ok(n): return _ffmpeg_has_hwaccel(n)
    def norm(n): return ALIASES.get(n, n)

    if prefer not in ("", "auto", "none"):
        pn = norm(prefer)
        return (pn if ok(pn) else None, {})

    if sys == "windows":
        for cand in ("cuda", "d3d11va", "qsv"):
            if ok(cand): return cand, {}
        return None, {}
    if sys == "darwin":
        if ok("videotoolbox"): return "videotoolbox", {}
        return None, {}
    for cand in ("vaapi", "qsv", "cuda"):
        if ok(cand): return cand, {}
    return None, {}


# -------------------------
# Imaging helpers
# -------------------------

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


# -------------------------
# Frame iteration
# -------------------------

def _iter_frames_imageio(srcu: str, size: Tuple[int, int], loop_video: bool):
    default_delay_ms = 1000.0 / 10.0
    try:
        props = iio.improps(srcu)
        fps = getattr(props, "fps", None)
        if fps and fps > 0:
            default_delay_ms = max(MIN_DELAY_MS, 1000.0 / float(fps))
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
                            if isinstance(d, (int, float)):
                                delay_ms = float(d) * (1000.0 if float(d) <= 10.0 else 1.0)
                except Exception:
                    pass
                yield rgb888, max(MIN_DELAY_MS, float(delay_ms))
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
    kind, _kw = pick_hw_backend(prefer)
    try:
        if kind:
            try:
                from av.codec.hwaccel import HWAccel
            except Exception as ie:
                raise RuntimeError(f"hwaccel API unavailable: {ie}")
            container = av.open(srcu, mode="r", hwaccel=HWAccel(device_type=kind))
            print(f"[hw] selected {kind} for decode")
        else:
            print("[hw] using CPU decode (no HW accel selected)")
            container = av.open(srcu, mode="r")
        vstream = next((s for s in container.streams if s.type == "video"), None)
        if vstream is None:
            raise RuntimeError("no video stream")
        return container, vstream, None
    except Exception as e:
        print(f"[hwaccel disabled: {kind or 'auto'} not available: {e}]")
        container = av.open(srcu, mode="r")
        vstream = next((s for s in container.streams if s.type == "video"), None)
        if vstream is None:
            raise RuntimeError("no video stream")
        return container, vstream, None


def _iter_frames_pyav(srcu: str, size: Tuple[int, int], loop_video: bool,
                      *, expand_mode: int, hw_prefer: str | None):
    TW, TH = size

    def _tb_num_den(tb) -> Tuple[int, int]:
        if tb is None:
            return (1, 1000)
        n = getattr(tb, "num", None); d = getattr(tb, "den", None)
        if n is not None and d is not None:
            return int(n), int(d)
        n = getattr(tb, "numerator", None); d = getattr(tb, "denominator", None)
        if n is not None and d is not None:
            return int(n), int(d)
        try:
            n, d = tb
            return int(n), int(d)
        except Exception:
            return (1, 1000)

    try:
        from av.error import BlockingIOError as AvBlockingIOError  # type: ignore
    except Exception:
        AvBlockingIOError = None  # type: ignore

    while True:
        try:
            container, vstream, dec = _open_with_hwaccel(srcu, hw_prefer)
            if vstream is None:
                raise RuntimeError("no video stream")

            avg_ms: Optional[float] = None
            if vstream.average_rate:
                try:
                    fps = float(vstream.average_rate)
                    if fps > 0:
                        avg_ms = max(MIN_DELAY_MS, 1000.0 / fps)
                except Exception:
                    pass

            graph = None
            src_in = sink_out = None

            def _ensure_graph_for(frame) -> None:
                nonlocal graph, src_in, sink_out
                if graph is not None:
                    return

                g = AvFilterGraph()

                tb_n, tb_d = _tb_num_den(frame.time_base or vstream.time_base)
                src = g.add(
                    "buffer",
                    args=(
                        f"video_size={frame.width}x{frame.height}:"
                        f"pix_fmt=rgb24:"
                        f"time_base={tb_n}/{tb_d}:"
                        f"pixel_aspect=1/1"
                    ),
                )
                last = src

                rot = _rotation_from_stream_and_frame(vstream, frame)
                if rot in (90, 180, 270):
                    if rot == 90:
                        t = g.add("transpose", args="clock"); last.link_to(t); last = t
                    elif rot == 270:
                        t = g.add("transpose", args="cclock"); last.link_to(t); last = t
                    else:  # 180
                        t1 = g.add("transpose", args="clock")
                        t2 = g.add("transpose", args="clock")
                        last.link_to(t1); t1.link_to(t2); last = t2

                sc = g.add("scale", args=f"{TW}:{TH}:flags=bilinear:force_original_aspect_ratio=decrease")
                last.link_to(sc); last = sc
                pd = g.add("pad", args=f"{TW}:{TH}:(ow-iw)/2:(oh-ih)/2:color=black")
                last.link_to(pd); last = pd

                sink = g.add("buffersink")
                last.link_to(sink)
                g.configure()

                graph = g
                src_in = src
                sink_out = sink

            last_pts_s: Optional[float] = None

            for packet in container.demux(vstream):
                frames = dec.decode(packet) if dec else packet.decode()
                for frame in frames:
                    arr = frame.to_ndarray(format="rgb24")

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
                    elif arr.dtype is not np.uint8:
                        arr = arr.astype(np.uint8)

                    f_rgb = av.VideoFrame.from_ndarray(arr, format="rgb24")
                    if getattr(f_rgb, "time_base", None) is None and getattr(vstream, "time_base", None):
                        f_rgb.time_base = vstream.time_base
                    _ensure_graph_for(f_rgb)
                    src_in.push(f_rgb)  # type: ignore[name-defined]

                    out_frames = []
                    while True:
                        try:
                            of = sink_out.pull()  # type: ignore[name-defined]
                            out_frames.append(of)
                        except Exception as pe:
                            if (AvBlockingIOError and isinstance(pe, AvBlockingIOError)) \
                               or getattr(pe, "errno", None) in (11, 35) \
                               or "resource temporarily unavailable" in str(pe).lower() \
                               or "eagain" in str(pe).lower():
                                break
                            raise

                    for of in out_frames:
                        rgb888 = of.to_ndarray(format="rgb24").tobytes()

                        delay_ms: float = float(avg_ms) if (avg_ms is not None) else 1000.0 / 10.0
                        pts_s = None
                        if of.pts is not None:
                            tb_n, tb_d = _tb_num_den(of.time_base or vstream.time_base)
                            pts_s = float(of.pts) * (tb_n / tb_d)
                        if pts_s is not None:
                            if last_pts_s is None:
                                delay_ms = float(avg_ms) if (avg_ms is not None) else 33.33
                            else:
                                delta_ms = (pts_s - last_pts_s) * 1000.0
                                if delta_ms <= 0 and (avg_ms is not None):
                                    delta_ms = float(avg_ms)
                                delay_ms = max(MIN_DELAY_MS, float(delta_ms))
                            last_pts_s = pts_s

                        yield rgb888, float(delay_ms)

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
    if not any(low.endswith(ext) for ext in (".png", ".jpg", ".jpeg", ".bmp", ".webp", ".gif")):
        yield from _iter_frames_pyav(srcu, size, loop_video, expand_mode=expand_mode, hw_prefer=hw_prefer)
    else:
        yield from _iter_frames_imageio(srcu, size, loop_video)


# -------------------------
# DDP packetizer
# -------------------------

_DDP_MAX_DATA = 1440
_DDP_HDR = struct.Struct("!BBB B I H")
_DDP_PIXEL_CFG_RGB888 = 0x2C

def _ddp_iter_packets(rgb_bytes: bytes, output_id: int, seq: int):
    mv = memoryview(rgb_bytes)
    total = len(mv)
    off = 0
    push_mask = 0x01
    ddp_base_flags = 0x40
    while off < total:
        end = min(off + _DDP_MAX_DATA, total)
        chunk = mv[off:end]
        is_last = end >= total
        flags = ddp_base_flags | (push_mask if is_last else 0)
        payload_len = len(chunk)
        pkt = bytearray(_DDP_HDR.size + payload_len)
        _DDP_HDR.pack_into(pkt, 0,
            flags,
            seq & 0xFF,
            _DDP_PIXEL_CFG_RGB888,
            output_id & 0xFF,
            off,
            payload_len
        )
        pkt[_DDP_HDR.size:] = chunk.tobytes()
        yield bytes(pkt)
        off = end


# -------------------------
# Async UDP transport
# -------------------------

class _UDPSender(asyncio.DatagramProtocol):
    def __init__(self):
        self.transport: asyncio.DatagramTransport | None = None
        self.packets_sent = 0

    def connection_made(self, transport: asyncio.BaseTransport) -> None:
        self.transport = transport

    def error_received(self, exc: BaseException) -> None:
        print(f"[udp] error_received: {exc!r}")

    def connection_lost(self, exc: BaseException | None) -> None:
        if exc:
            print(f"[udp] connection_lost: {exc!r}")

    def sendto(self, data: bytes, addr):
        if self.transport is not None:
            self.transport.sendto(data, addr)
            self.packets_sent += 1


# -------------------------
# Paced sender (upsample)
# -------------------------

async def _paced_sender(loop, q, *, pace_hz, ema_alpha, get_latest_bytes, out_id, proto):
    next_t = loop.time()
    tick_s = 1.0 / pace_hz
    ema_buf: Optional[np.ndarray] = None
    seq = 0

    while True:
        buf = get_latest_bytes()
        if buf is not None:
            if ema_alpha > 0.0:
                cur = np.frombuffer(buf, dtype=np.uint8)
                if ema_buf is None or ema_buf.shape != cur.shape:
                    ema_buf = cur.astype(np.float32, copy=True)
                else:
                    ema_buf *= (1.0 - ema_alpha)
                    ema_buf += cur.astype(np.float32) * ema_alpha
                outb = ema_buf.astype(np.uint8, copy=False).tobytes()
            else:
                outb = buf

            for pkt in _ddp_iter_packets(outb, out_id, seq):
                if q.full():
                    try:
                        q.get_nowait(); q.task_done()
                    except asyncio.QueueEmpty:
                        pass
                await q.put(pkt)
            seq = (seq + 1) & 0x0F

        await asyncio.sleep(max(0.0, next_t - loop.time()))
        next_t += tick_s


# -------------------------
# Main task per connection (CORRECTED)
# -------------------------

async def ddp_task(target_ip: str, target_port: int, out_id: int, *, size, src, opts):
    """If pace_hz>0: producer (native) + paced sender. Else: native-cadence push.
       Non-blocking UDP with bounded queue. Optional per-frame packet spreading.
    """
    import socket, time
    from collections import deque

    w, h = size
    frames = iter_frames(
        src, (w, h),
        loop_video=opts["loop"],
        expand_mode=opts["expand_mode"],
        hw_prefer=opts["hw"]
    )

    loop = asyncio.get_running_loop()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    addr = (target_ip, target_port)

    # Config
    log_cfg = CONFIG.get("log", {})
    log_metrics = bool(log_cfg.get("metrics", True))
    log_detail  = bool(log_cfg.get("detail", False))
    log_rate_ms = int(log_cfg.get("rate_ms", 1000))
    spread_enabled = bool(CONFIG.get("net", {}).get("spread_packets", False))
    spread_max_fps = int(CONFIG.get("net", {}).get("spread_max_fps", 120))

    # Meters
    pkt_meter = _RateMeter()
    frm_meter = _RateMeter()
    q_occ_samples = deque(maxlen=200)
    q_drops = 0

    transport = None
    try:
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 1 << 20)
        except OSError:
            pass

        sock.bind(("0.0.0.0", 0))

        proto = _UDPSender()
        transport, _ = await loop.create_datagram_endpoint(lambda: proto, sock=sock)

        max_in_flight = 4096
        q: asyncio.Queue[bytes] = asyncio.Queue(maxsize=max_in_flight)

        async def drainer():
            while True:
                pkt = await q.get()
                try:
                    proto.sendto(pkt, addr)
                    if log_metrics:
                        pkt_meter.tick(time.perf_counter())
                finally:
                    q.task_done()

        drain_task = asyncio.create_task(drainer())

        pace_hz   = int(opts.get("pace_hz", 0))
        ema_alpha = float(opts.get("ema_alpha", 0.0))

        frames_emitted = 0
        packets_enqueued = 0
        last_log = time.perf_counter()
        seq = 0

        async def enqueue_frame(rgb888: bytes, seq_val: int,
                                *, packet_spacing_s: Optional[float] = None,
                                group_n: int = 1):
            nonlocal packets_enqueued, q_drops
            total_len = len(rgb888)
            off = 0
            slot_idx = 0
            start_ts = loop.time() if (packet_spacing_s and packet_spacing_s > 0.0) else None
            while off < total_len:
                # group_n packets per timeslot (to avoid micro-sleeps)
                group_left = group_n if group_n > 0 else 1
                end = min(off + _DDP_MAX_DATA, total_len)
                chunk = rgb888[off:end]
                is_last = end >= total_len
                flags = 0x40 | (0x01 if is_last else 0)
                pkt = bytearray(_DDP_HDR.size + len(chunk))
                _DDP_HDR.pack_into(pkt, 0,
                    flags, seq_val & 0xFF, _DDP_PIXEL_CFG_RGB888, out_id & 0xFF, off, len(chunk)
                )
                pkt[_DDP_HDR.size:] = chunk

                # enqueue immediately; we'll sleep after sending 'group_n' packets
                if q.full():
                    try:
                        q.get_nowait(); q.task_done(); q_drops += 1
                    except asyncio.QueueEmpty:
                        pass
                await q.put(bytes(pkt))
                packets_enqueued += 1
                group_left -= 1
                off = end
                # If we've sent a group and spacing is configured, sleep until next slot
                if group_left <= 0 and start_ts is not None and packet_spacing_s and packet_spacing_s > 0.0:
                    slot_idx += 1
                    target = start_ts + slot_idx * packet_spacing_s
                    await asyncio.sleep(max(0.0, target - loop.time()))
                    group_left = group_n if group_n > 0 else 1

        if pace_hz > 0:
            # Pacing path
            latest: Dict[str, Optional[bytes]] = {"buf": None}
            latest_lock = asyncio.Lock()

            async def producer():
                nonlocal frames_emitted, last_log, seq
                for rgb888, delay_ms in frames:
                    async with latest_lock:
                        latest["buf"] = rgb888
                    frames_emitted += 1
                    if log_metrics:
                        frm_meter.tick(time.perf_counter())
                    seq = (seq + 1) & 0x0F

                    now = time.perf_counter()
                    if now - last_log >= (log_rate_ms / 1000.0):
                        if log_metrics:
                            fps = frm_meter.rate_hz()
                            pps = pkt_meter.rate_hz()
                            pkt_jit = pkt_meter.jitter_ms()
                            frm_jit = frm_meter.jitter_ms()
                            q_avg = (sum(q_occ_samples)/len(q_occ_samples)) if q_occ_samples else 0
                            q_max = max(q_occ_samples) if q_occ_samples else 0
                            spread_tag = (" (spread)" if (spread_enabled and pace_hz <= spread_max_fps) else "")
                            print(f"[send] out={out_id} pace={pace_hz}Hz fps={fps:.2f} pps={pps:.0f} "
                                  f"pkt_jit={pkt_jit:.1f}ms frm_jit={frm_jit:.1f}ms "
                                  f"q_avg={q_avg:.0f}/{max_in_flight} q_max={q_max} "
                                  f"enq={packets_enqueued} tx={proto.packets_sent} drops={q_drops}{spread_tag}")
                        else:
                            print(f"[send] out={out_id} pace={pace_hz}Hz frames={frames_emitted} enq={packets_enqueued} tx={proto.packets_sent}")
                        frames_emitted = 0
                        q_occ_samples.clear()
                        last_log = now

                    await asyncio.sleep(max(MIN_DELAY_MS/1000.0, float(delay_ms)/1000.0))

            async def sampler():
                tick = 1.0 / pace_hz
                next_t = loop.time()
                while True:
                    buf = latest["buf"]
                    if buf is not None:
                        if spread_enabled and pace_hz <= spread_max_fps:
                            pkt_count = (len(buf) + _DDP_MAX_DATA - 1) // _DDP_MAX_DATA
                            spacing = (tick / pkt_count) if pkt_count > 0 else None
                            if spacing and spacing > 0.0:
                                min_s = float(CONFIG.get('net', {}).get('spread_min_ms', 3.0)) / 1000.0
                                import math
                                group_n = max(1, int(math.ceil(min_s / spacing)))
                                spacing = spacing * group_n
                            else:
                                group_n = 1
                        else:
                            spacing = None
                            group_n = 1
                        await enqueue_frame(buf, seq, packet_spacing_s=spacing, group_n=group_n)

                    if log_metrics:
                        q_occ_samples.append(q.qsize())

                    await asyncio.sleep(max(0.0, next_t - loop.time()))
                    next_t += tick

            prod_task = asyncio.create_task(producer())
            pace_task = asyncio.create_task(sampler())
            try:
                await prod_task
            finally:
                pace_task.cancel()
                with contextlib.suppress(asyncio.CancelledError):
                    await pace_task

        else:
            # Native cadence
            last_log = time.perf_counter()
            next_t = loop.time()
            for rgb888, delay_ms in frames:
                delay_s = max(MIN_DELAY_MS/1000.0, float(delay_ms)/1000.0)

                if spread_enabled:
                    inst_fps = (1.0/delay_s) if delay_s > 0 else 1e9
                    if inst_fps <= spread_max_fps:
                        pkt_count = (len(rgb888) + _DDP_MAX_DATA - 1) // _DDP_MAX_DATA
                        spacing = (delay_s / pkt_count) if pkt_count > 0 else None
                        if spacing and spacing > 0.0:
                            min_s = float(CONFIG.get('net', {}).get('spread_min_ms', 3.0)) / 1000.0
                            import math
                            group_n = max(1, int(math.ceil(min_s / spacing)))
                            spacing = spacing * group_n
                        else:
                            group_n = 1
                    else:
                        spacing = None
                        group_n = 1
                else:
                    spacing = None
                    group_n = 1

                await enqueue_frame(rgb888, seq, packet_spacing_s=spacing, group_n=group_n)
                seq = (seq + 1) & 0x0F
                frames_emitted += 1

                if log_metrics:
                    frm_meter.tick(time.perf_counter())
                    q_occ_samples.append(q.qsize())

                now = time.perf_counter()
                if now - last_log >= (log_rate_ms / 1000.0):
                    if log_metrics:
                        fps = frm_meter.rate_hz()
                        pps = pkt_meter.rate_hz()
                        pkt_jit = pkt_meter.jitter_ms()
                        frm_jit = frm_meter.jitter_ms()
                        q_avg = (sum(q_occ_samples)/len(q_occ_samples)) if q_occ_samples else 0
                        q_max = max(q_occ_samples) if q_occ_samples else 0
                        target = 1000.0 / max(float(delay_ms), 1.0)
                        spread_tag = (" (spread)" if spacing else "")
                        print(f"[send] out={out_id} native fps={fps:.2f} (~{target:.1f} tgt) pps={pps:.0f} "
                              f"pkt_jit={pkt_jit:.1f}ms frm_jit={frm_jit:.1f}ms "
                              f"q_avg={q_avg:.0f}/{max_in_flight} q_max={q_max} "
                              f"enq={packets_enqueued} tx={proto.packets_sent} drops={q_drops}{spread_tag}")
                    else:
                        print(f"[send] out={out_id} native frames={frames_emitted} enq={packets_enqueued} tx={proto.packets_sent}")
                    frames_emitted = 0
                    q_occ_samples.clear()
                    last_log = now

                next_t += delay_s
                await asyncio.sleep(max(0.0, next_t - loop.time()))

        await q.join()
        drain_task.cancel()
        with contextlib.suppress(asyncio.CancelledError):
            await drain_task

    except Exception as e:
        print(f"[send] out={out_id} fatal error: {e!r}")
        traceback.print_exc()
        raise
    finally:
        try:
            if transport is not None:
                transport.close()
        except Exception:
            pass
        try:
            sock.close()
        except Exception:
            pass


# =========================
# WebSocket control
# =========================
CONFIG: Dict[str, Any] = DEFAULT_CONFIG


def _is_benign_disconnect(exc: BaseException) -> bool:
    if isinstance(exc, OSError) and getattr(exc, "winerror", None) in (64, 121):
        return True
    return isinstance(exc, (ConnectionClosedOK, ConnectionClosedError))


def _parse_expand(val, default):
    s = str(val if val is not None else default).lower()
    if s in ("2", "force"):  return 2
    if s in ("0", "false", "never"): return 0
    return 1  # auto


def _parse_hw(val, default):
    s = (val if val is not None else default)
    return None if str(s).lower() in ("none","off","cpu") else str(s)


def _parse_pace_hz(v):
    try:
        n = int(v or 0)
    except Exception:
        raise ValueError(f"pace must be integer Hz (got {v!r})")
    if n < 0:
        raise ValueError("pace must be >= 0")
    return n


async def handle_control(ws):
    streams: dict[int, asyncio.Task] = {}
    streams_lock = asyncio.Lock()
    session = {
        "device_id": None,
        "ip": getattr(ws, "remote_address", ("?",))[0],
    }

    async def start_stream(msg):
        _require_fields(msg, ("out","w","h","src"), "start_stream")
        out = int(msg["out"])
        w = int(msg["w"]); h = int(msg["h"])
        if w <= 0 or h <= 0:
            raise ValueError(f"start_stream requires positive w/h (got {w}x{h})")
        src = str(msg["src"])
        ddp_port = int(msg.get("ddp_port", 4048))

        pace_hz = _parse_pace_hz(msg.get("pace", 0))
        opts = {
            "loop": _truthy(str(msg.get("loop", CONFIG["playback"]["loop"]))),
            "expand_mode": _parse_expand(msg.get("expand"), CONFIG["video"]["expand_mode"]),
            "hw": _parse_hw(msg.get("hw"), CONFIG["hw"]["prefer"]),
            "log_send_ms": bool(CONFIG["log"].get("send_ms", False)),
            "pace_hz": pace_hz,
            "ema_alpha": max(0.0, min(float(msg.get("ema", 0.0)), 1.0)),
        }

        srcu = unquote(src)
        local_path = _resolve_local_path(srcu)
        if local_path is not None and not os.path.exists(local_path):
            print(f"* start_stream {session['ip']} dev={session['device_id']} out={out} requested missing file: {local_path}")
            raise FileNotFoundError(f"no such file: {local_path}")

        async with streams_lock:
            t_old = streams.pop(out, None)
            if t_old:
                t_old.cancel()
                with contextlib.suppress(asyncio.CancelledError, Exception):
                    await t_old

        print(f"* start_stream {session['ip']} dev={session['device_id']} out={out} size={w}x{h} ddp_port={ddp_port} "
              f"src={src} pace={opts['pace_hz']} ema={opts['ema_alpha']} expand={opts['expand_mode']} loop={opts['loop']} hw={opts['hw']}")

        task = asyncio.create_task(ddp_task(session["ip"], ddp_port, out, size=(w, h), src=src, opts=opts))

        def _on_done(t: asyncio.Task):
            try:
                exc = t.exception()
            except asyncio.CancelledError:
                return
            except Exception as e:
                print(f"[task] out={out} exception(): {e!r}")
                return
            if exc:
                print(f"[task] out={out} crashed: {exc!r}")
        task.add_done_callback(_on_done)

        async with streams_lock:
            streams[out] = task

        await _ws_send(ws, {"type": "ack", "out": out, "applied": {
            "src": src, "pace": opts["pace_hz"], "ema": opts["ema_alpha"],
            "expand": opts["expand_mode"], "loop": opts["loop"], "hw": opts["hw"],
        }})

    async def stop_stream(msg):
        _require_fields(msg, ("out",), "stop_stream")
        out = int(msg["out"])
        async with streams_lock:
            t = streams.pop(out, None)
            if t:
                t.cancel()
                with contextlib.suppress(asyncio.CancelledError, Exception):
                    await t
        print(f"* stop_stream {session['ip']} dev={session['device_id']} out={out}")
        await _ws_send(ws, {"type": "ack", "out": out})

    async def update_stream(msg):
        _require_fields(msg, ("out",), "update")
        out = int(msg["out"])
        async with streams_lock:
            if out not in streams:
                raise ValueError(f"update for unknown out={out} (no active stream)")

        base = {"type": "start_stream", "out": out}
        applied = {}
        for k in ("w","h","ddp_port","src","pace","ema","expand","loop","hw"):
            if k in msg:
                base[k] = msg[k]
                applied[k] = msg[k]

        await stop_stream({"out": out})
        await start_stream(base)

        await _ws_send(ws, {"type": "ack", "out": out, "applied": applied})

    # --- handshake ---
    try:
        raw = await ws.recv()
        hello = json.loads(raw)
    except ConnectionClosed as e:
        print(f"* disconnect during handshake from {session['ip']} ({getattr(e, 'code', '')} {getattr(e, 'reason', '')})")
        return
    except Exception as e:
        await _ws_send(ws, {"type": "error", "code": "proto", "message": f"invalid hello: {e}"})
        with contextlib.suppress(Exception):
            await ws.close(code=4001, reason="protocol")
        return

    if hello.get("type") != "hello":
        await _ws_send(ws, {"type":"error","code":"proto","message":"expect 'hello' first"})
        await ws.close(code=4001, reason="protocol")
        return

    session["device_id"] = hello.get("device_id","unknown")
    print(f"* hello from {session['ip']} dev={session['device_id']} proto={hello.get('proto')}")
    await _ws_send(ws, {"type":"hello_ack","server_version":"lvgl-ddp-stream/1"})

    try:
        async for raw in ws:
            try:
                msg = json.loads(raw)
                t = msg.get("type")
                if t == "start_stream":
                    await start_stream(msg)
                elif t == "stop_stream":
                    await stop_stream(msg)
                elif t == "update":
                    await update_stream(msg)
                elif t == "ping":
                    await _ws_send(ws, {"type":"pong","t": msg.get("t")})
                else:
                    await _ws_send(ws, {"type":"error","code":"bad_type","message":f"unknown type {t}"})
            except (ValueError, FileNotFoundError) as e:
                await _ws_send(ws, {"type":"error","code":"bad_request","message":str(e)})
            except Exception as e:
                await _ws_send(ws, {"type":"error","code":"server_error","message":str(e)})
    except Exception as loop_exc:
        if _is_benign_disconnect(loop_exc):
            reason = getattr(loop_exc, 'reason', '') or (getattr(loop_exc, 'args', [''])[0])
            print(f"* disconnect {session['ip']} ({type(loop_exc).__name__}: {reason})")
        else:
            print(f"[warn] control loop error from {session['ip']}: {loop_exc!r}")
    finally:
        async with streams_lock:
            for out, t in list(streams.items()):
                t.cancel()
            with contextlib.suppress(asyncio.CancelledError, Exception):
                if streams:
                    await asyncio.gather(*streams.values(), return_exceptions=True)


async def dispatch(ws):
    path = getattr(getattr(ws, "request", None), "path", "/")
    try:
        if str(path).startswith("/control"):
            await handle_control(ws)
        else:
            await ws.close(code=4003, reason="unknown path")
    except Exception as e:
        if not _is_benign_disconnect(e):
            print(f"[warn] connection handler error: {e!r}")
        with contextlib.suppress(Exception):
            await ws.close()


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

    try:
        if bool(CONFIG["net"].get("win_timer_res", True)):
            _win_timer_res(True)

        print("* ws_ddp_proxy on ws://{}:{}/control".format(args.host, args.port))
        async with websockets.serve(
            dispatch, args.host, args.port,
            max_size=2**22,
            compression=None,
            ping_interval=20, ping_timeout=20,
            close_timeout=1.0
        ):
            await asyncio.Future()
    finally:
        if bool(CONFIG["net"].get("win_timer_res", True)):
            _win_timer_res(False)


# -------------------------
# Small shared helpers
# -------------------------

async def _ws_send(ws, obj) -> bool:
    try:
        await ws.send(json.dumps(obj, separators=(",", ":")))
        return True
    except (ConnectionClosed, OSError):
        return False


def _require_fields(msg, fields, verb):
    missing = [f for f in fields if f not in msg]
    if missing:
        raise ValueError(f"{verb} requires {', '.join(fields)} (missing: {', '.join(missing)})")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
