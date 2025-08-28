# © Copyright 2025 Stuart Parmenter
# SPDX-License-Identifier: MIT
#!/usr/bin/env python3

import asyncio
import contextlib
import json
import os
import platform
import shutil
import struct
import subprocess
import time
import traceback
from typing import Optional, Dict, Any
from urllib.parse import urlparse, unquote

try:
    from urllib.request import url2pathname
except Exception:  # pragma: no cover
    def url2pathname(p): return p  # type: ignore[misc]

import numpy as np
from PIL import Image, ImageOps
import imageio.v3 as iio
import websockets
from websockets.exceptions import ConnectionClosed, ConnectionClosedOK, ConnectionClosedError

# --- PyAV (video decode + filters) ---
import av
from av.filter import Graph as AvFilterGraph
av.logging.set_level(av.logging.INFO)

# --- YouTube resolver (required) ---
import yt_dlp  # type: ignore


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
        diffs = [(self.ts[i] - self.ts[i - 1]) for i in range(1, n)]
        mean = sum(diffs) / len(diffs)
        var = sum((d - mean) ** 2 for d in diffs) / (len(diffs) - 1)
        return math.sqrt(var) * 1000.0


MIN_DELAY_MS = 10.0  # clamp very small frame delays

DEFAULT_CONFIG: Dict[str, Any] = {
    "hw": {"prefer": "auto"},
    "video": {
        "expand_mode": 1,  # 0=never, 1=auto(limited->full), 2=force
        "fit": "auto-pad",  # "pad" | "cover" | "auto-pad" | "auto-cover"
        "autocrop": {
            "enabled": True,
            "probe_frames": 8,
            "luma_thresh": 22,
            "max_bar_ratio": 0.20,
            "min_bar_px": 2
        }
    },
    "playback": {"loop": True},
    "log": {"send_ms": False, "rate_ms": 1000, "detail": False, "metrics": True},
    "net": {
        "win_timer_res": True,
        "spread_packets": True,
        "spread_max_fps": 60,
        "spread_min_ms": 3.0,
        "spread_max_sleeps": 0
    }
}


def _compute_spacing_and_group(pkt_count: int, frame_interval_s: float) -> tuple[Optional[float], int]:
    """
    Compute (spacing_s, group_n) for packet spreading.

    spacing_s: sleep between packet *groups* (None => no spreading)
    group_n:   number of packets sent per timeslot (then sleep once)

    Enforces:
      - net.spread_min_ms: minimum spacing to avoid micro-sleeps
      - net.spread_max_sleeps: maximum sleeps per frame

    Works for both native and paced paths.
    """
    import math

    if pkt_count <= 0 or frame_interval_s <= 0.0:
        return (None, 1)

    net_cfg = CONFIG.get("net", {})
    min_s = float(net_cfg.get("spread_min_ms", 3.0)) / 1000.0
    max_sleeps = int(net_cfg.get("spread_max_sleeps", 6))

    # Ideal per-packet spacing if we slept once per packet
    ideal = frame_interval_s / float(pkt_count)

    # Start by grouping to satisfy minimum spacing
    group_n = 1
    if 0.0 < ideal < min_s:
        group_n = max(1, int(math.ceil(min_s / ideal)))

    # Then cap total sleeps per frame
    if max_sleeps > 0:
        per_sleep = max(1, int(math.ceil(pkt_count / max_sleeps)))
        group_n = max(group_n, per_sleep)

    spacing = ideal * group_n
    if spacing > frame_interval_s:
        spacing = frame_interval_s

    return (spacing, group_n)


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
# Small utilities
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


def pick_hw_backend(prefer: Optional[str] = None):
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
            if ok(cand):
                return cand, {}
        return (None, {})
    if sys == "darwin":
        if ok("videotoolbox"):
            return "videotoolbox", {}
        return (None, {})
    for cand in ("vaapi", "qsv", "cuda"):
        if ok(cand):
            return cand, {}
    return (None, {})


# --- YouTube helpers ---

def _is_youtube_url(u: str) -> bool:
    try:
        p = urlparse(u)
        host = (p.netloc or "").lower()
        return any(h in host for h in (
            "youtube.com", "youtu.be", "youtube-nocookie.com"
        ))
    except Exception:
        return False


def _headers_dict_to_ffmpeg_opt(h: Dict[str, str]) -> str:
    """
    FFmpeg's libavformat expects a single CRLF-delimited string in the `headers`
    option (same as `-headers` in ffmpeg CLI). Must end with a trailing CRLF.
    """
    if not h:
        return ""
    lines = []
    for k, v in h.items():
        if not k or v is None:
            continue
        k = str(k).strip()
        v = str(v).strip()
        if k and v:
            lines.append(f"{k}: {v}")
    return "\r\n".join(lines) + "\r\n"  # avoid "No trailing CRLF" warning


def _resolve_stream_url(srcu: str) -> tuple[str, Dict[str, str]]:
    """
    Resolve YouTube (and similar) page URLs into a direct media URL + HTTP headers
    suitable for av.open(..., options={ 'headers': 'K: V\\r\\n...' }).
    Non-YouTube URLs are returned unchanged.
    """
    if not _is_youtube_url(srcu):
        return srcu, {}

    # Prefer compact, HTTP progressive when possible; fall back to HLS/DASH
    ydl_opts = {
        "quiet": True,
        "no_warnings": True,
        "noprogress": True,
        "extract_flat": False,
        "format": (
            "best[protocol^=http][vcodec!=none][height<=720]/"
            "best[protocol*=m3u8][vcodec!=none][height<=720]/"
            "bv*[height<=720]/best"
        ),
    }

    with yt_dlp.YoutubeDL(ydl_opts) as ydl:
        info = ydl.extract_info(srcu, download=False)
        if info is None:
            return srcu, {}
        if "entries" in info and info["entries"]:
            info = info["entries"][0]
        media_url = info.get("url") or srcu
        headers = {}
        rh = info.get("http_headers") or {}
        if not rh and "formats" in info and isinstance(info["formats"], list):
            for f in info["formats"]:
                if f.get("url") == media_url and f.get("http_headers"):
                    rh = f["http_headers"]
                    break
        if rh:
            headers_str = _headers_dict_to_ffmpeg_opt({k: v for k, v in rh.items()})
            if headers_str:
                headers["headers"] = headers_str
        return media_url, headers


# -------------------------
# Imaging helpers
# -------------------------

def _resize_pad_to_rgb_bytes(img: Image.Image, size: tuple[int, int]) -> bytes:
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

def _iter_frames_imageio(srcu: str, size: tuple[int, int], loop_video: bool):
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


def _open_with_hwaccel(srcu: str, prefer: Optional[str], options: Optional[Dict[str, str]] = None):
    kind, _kw = pick_hw_backend(prefer)
    options = options or {}
    try:
        if kind:
            try:
                from av.codec.hwaccel import HWAccel
            except Exception as ie:
                raise RuntimeError(f"hwaccel API unavailable: {ie}")
            container = av.open(srcu, mode="r", hwaccel=HWAccel(device_type=kind), options=options)
            print(f"[hw] selected {kind} for decode")
        else:
            print("[hw] using CPU decode (no HW accel selected)")
            container = av.open(srcu, mode="r", options=options)
        vstream = next((s for s in container.streams if s.type == "video"), None)
        if vstream is None:
            raise RuntimeError("no video stream")
        return container, vstream
    except Exception as e:
        print(f"[hwaccel disabled: {kind or 'auto'} not available: {e}]")
        container = av.open(srcu, mode="r", options=options)
        vstream = next((s for s in container.streams if s.type == "video"), None)
        if vstream is None:
            raise RuntimeError("no video stream")
        return container, vstream


def _choose_decode_preference(
    srcu: str,
    prefer: Optional[str],
    out_size: tuple[int, int],
    expand_mode: int
) -> Optional[str]:
    """
    Decide CPU vs HW decode based on *input* properties (resolution/fps/codec),
    output size, and expand_mode. Returns:
      - "auto"  -> let pick_hw_backend() choose HW if available (CUDA, QSV, D3D11VA, VAAPI, etc.)
      - "cpu"/"none"/None -> force CPU decode
      - or the explicit user preference if provided (honored as-is)
    Also prints a short diagnostic explaining the choice.
    """
    if prefer and str(prefer).lower() not in ("", "auto"):
        print(f"[decode] prefer={prefer!r} explicitly requested -> honoring")
        return prefer

    # If this is a YouTube page URL, we will resolve to HLS/DASH; skip probing and choose auto.
    if _is_youtube_url(srcu):
        kind, _ = pick_hw_backend("auto")
        print(f"[decode] source=YouTube page -> choosing HW 'auto' (resolved stream likely HLS/DASH); auto maps to {kind or 'none'}")
        return "auto"

    TW, TH = out_size

    # Probe input to get width/height/fps/codec
    in_w = in_h = 0
    in_fps: Optional[float] = None
    in_codec = ""
    try:
        pc = av.open(srcu, mode="r")
        try:
            pvs = next((s for s in pc.streams if s.type == "video"), None)
            if pvs is not None:
                in_w = int(getattr(getattr(pvs, "codec_context", None), "width", 0) or getattr(pvs, "width", 0) or 0)
                in_h = int(getattr(getattr(pvs, "codec_context", None), "height", 0) or getattr(pvs, "height", 0) or 0)
                try:
                    in_fps = float(pvs.average_rate) if pvs.average_rate else None
                except Exception:
                    in_fps = None
                in_codec = (getattr(getattr(pvs, "codec", None), "name", "") or "").lower()
        finally:
            with contextlib.suppress(Exception):
                pc.close()
    except Exception as e:
        print(f"[decode] probe failed ({e!r}); defaulting to auto -> pick_hw_backend()")
        return "auto"

    # Heuristics
    BIG_PIXELS = 1920 * 1080
    HIGH_FPS = 50.0
    SMALL_OUT = 128 * 128
    HARD_CODECS = ("hevc", "h265", "av1", "vp9")

    input_big = (in_w * in_h) >= BIG_PIXELS
    input_fast = (in_fps or 0.0) >= HIGH_FPS
    hard_codec = any(k in in_codec for k in HARD_CODECS)
    output_tiny = (TW * TH) <= SMALL_OUT
    expand_needed = expand_mode in (1, 2)

    reasons = []
    if input_big:
        reasons.append(">=1080p input")
    if input_fast:
        reasons.append(">=50 fps input")
    if hard_codec:
        reasons.append(f"hard codec ({in_codec or 'unknown'})")
    if output_tiny and expand_needed:
        reasons.append("tiny output + range-expand")

    if input_big or input_fast or hard_codec:
        kind, _ = pick_hw_backend("auto")
        why = ", ".join(reasons) or "input suggests HW"
        print(
            f"[decode] in={in_w}x{in_h}@{(in_fps or 0):.2f}fps codec={in_codec or 'unknown'} "
            f"out={TW}x{TH} expand={expand_mode} -> choosing HW ('auto') "
            f"because {why}; auto maps to {kind or 'none'}"
        )
        return "auto"

    why = ", ".join(reasons) or "small/slow input; HW adds overhead here"
    print(
        f"[decode] in={in_w}x{in_h}@{(in_fps or 0):.2f}fps codec={in_codec or 'unknown'} "
        f"out={TW}x{TH} expand={expand_mode} -> choosing CPU because {why}"
    )
    return "cpu"


def _iter_frames_pyav(
    srcu: str,
    size: tuple[int, int],
    loop_video: bool,
    *,
    expand_mode: int,
    hw_prefer: Optional[str]
):
    """
    PyAV video path with a REBUILDABLE filter graph.

    Why rebuild? HLS/DASH (e.g., YouTube) can change width/height/pix_fmt/SAR/rotation mid-play.
    A fixed filter graph gets confused and you see "48px wide inside 64" artifacts.
    We detect changes and rebuild the graph on the next frame.
    """
    TW, TH = size

    # --- Auto-crop (black bar) config/state ---
    ac_cfg = CONFIG.get("video", {}).get("autocrop", {}) or {}
    ac_enabled = bool(ac_cfg.get("enabled", True))
    ac_probe_frames = int(ac_cfg.get("probe_frames", 8))
    ac_thresh = int(ac_cfg.get("luma_thresh", 22))
    ac_max_ratio = float(ac_cfg.get("max_bar_ratio", 0.20))
    ac_min_px = int(ac_cfg.get("min_bar_px", 2))
    ac_samples = {"l": [], "r": [], "t": [], "b": []}
    ac_decided = False
    ac_crop = {"l": 0, "r": 0, "t": 0, "b": 0}  # in source pixel coords
    ac_seen = 0

    def _estimate_black_bars(frame_w: int, frame_h: int, gray: np.ndarray,
                             thresh: int, min_px: int, max_ratio: float) -> dict:
        """Return {'l','r','t','b'} estimated black bar widths in pixels for one frame."""
        h, w = gray.shape  # (H, W)
        # Walk inward from each edge until median luma > thresh
        def walk_left():
            cap = int(w * max_ratio); x = 0
            while x < min(cap, w - 1):
                if int(np.median(gray[:, x:min(x+4, w)])) > thresh: break
                x += 1
            return max(min_px if x >= min_px else 0, min(x, cap))
        def walk_right():
            cap = int(w * max_ratio); x = 0
            while x < min(cap, w - 1):
                if int(np.median(gray[:, max(0, w-1-(x+3)):w-x])) > thresh: break
                x += 1
            return max(min_px if x >= min_px else 0, min(x, cap))
        def walk_top():
            cap = int(h * max_ratio); y = 0
            while y < min(cap, h - 1):
                if int(np.median(gray[y:min(y+4, h), :])) > thresh: break
                y += 1
            return max(min_px if y >= min_px else 0, min(y, cap))
        def walk_bot():
            cap = int(h * max_ratio); y = 0
            while y < min(cap, h - 1):
                if int(np.median(gray[max(0, h-1-(y+3)):h-y, :])) > thresh: break
                y += 1
            return max(min_px if y >= min_px else 0, min(y, cap))
        return {"l": walk_left(), "r": walk_right(), "t": walk_top(), "b": walk_bot()}


    try:
        from av.error import BlockingIOError as AvBlockingIOError  # type: ignore
    except Exception:
        AvBlockingIOError = None  # type: ignore

    first_graph_log_done = False

    while True:
        try:
            # Resolve YouTube page URLs to fresh direct media + headers on each (re)open
            # (googlevideo links expire; resolving here ensures we always reopen with a valid URL)
            real_srcu, http_opts = _resolve_stream_url(srcu)

            # Decide CPU vs HW based on input size/fps/codec (still routed through pick_hw_backend)
            prefer = _choose_decode_preference(real_srcu, hw_prefer, (TW, TH), expand_mode)
            container, vstream = _open_with_hwaccel(real_srcu, prefer, options=http_opts)
            if vstream is None:
                raise RuntimeError("no video stream")

            # Default frame delay if timestamps are missing
            avg_ms: Optional[float] = None
            if vstream.average_rate:
                try:
                    fps = float(vstream.average_rate)
                    if fps > 0:
                        avg_ms = max(MIN_DELAY_MS, 1000.0 / fps)
                except Exception:
                    pass

            # --- Rebuildable filter graph state ---
            graph = None
            src_in = sink_out = None
            g_props = {"w": None, "h": None, "fmt": None, "sar": (1, 1), "rot": 0}

            # Helpers
            def _tb_num_den(tb) -> tuple[int, int]:
                if tb is None:
                    return (1, 1000)
                for a, b in (("num", "den"), ("numerator", "denominator")):
                    n = getattr(tb, a, None); d = getattr(tb, b, None)
                    if n is not None and d is not None:
                        return int(n), int(d)
                try:
                    n, d = tb
                    return int(n), int(d)
                except Exception:
                    return (1, 1000)

            def _sar_of(obj) -> tuple[int, int]:
                # Try frame SAR first; fall back to codec_context SAR; else 1:1
                try:
                    sar = getattr(obj, "sample_aspect_ratio", None)
                    n = getattr(sar, "num", getattr(sar, "numerator", None))
                    d = getattr(sar, "den", getattr(sar, "denominator", None))
                    if n and d and n > 0 and d > 0:
                        return int(n), int(d)
                except Exception:
                    pass
                try:
                    cc = getattr(vstream, "codec_context", None)
                    sar = getattr(cc, "sample_aspect_ratio", None)
                    n = getattr(sar, "num", getattr(sar, "numerator", None))
                    d = getattr(sar, "den", getattr(sar, "denominator", None))
                    if n and d and n > 0 and d > 0:
                        return int(n), int(d)
                except Exception:
                    pass
                return (1, 1)


            def _ensure_graph_for(frame) -> None:
                """(Re)build the filter graph if geometry/SAR/format/rotation changed."""
                nonlocal graph, src_in, sink_out, first_graph_log_done, g_props

                w, h = int(frame.width), int(frame.height)
                fmt_name = getattr(frame.format, "name", "rgb24")
                sar_n, sar_d = _sar_of(frame)
                rot = _rotation_from_stream_and_frame(vstream, frame)

                # Track whether we've already applied the autocrop in the current graph
                applied_ac = g_props.get("ac_applied", False)
                want_ac = bool(ac_enabled and ac_decided and any(v > 0 for v in ac_crop.values()))

                need_rebuild = (
                    graph is None or
                    g_props["w"] != w or
                    g_props["h"] != h or
                    g_props["fmt"] != fmt_name or
                    g_props["sar"] != (sar_n, sar_d) or
                    g_props["rot"] != rot or
                    (want_ac and not applied_ac)
                )
                if not need_rebuild:
                    return

                old = g_props.copy()
                g_props.update({"w": w, "h": h, "fmt": fmt_name, "sar": (sar_n, sar_d), "rot": rot, "ac_applied": want_ac})
                print(f"[graph] rebuild: {old} -> {g_props} (ac={ac_crop if (ac_enabled and ac_decided) else 'pending'})")

                if not first_graph_log_done:
                    try:
                        cc = getattr(vstream, "codec_context", None)
                        in_sar = getattr(cc, "sample_aspect_ratio", None)
                        print(f"[graph] input codec SAR={in_sar} size={w}x{h}")
                    except Exception:
                        pass
                    first_graph_log_done = True

                g = AvFilterGraph()

                tb_n, tb_d = _tb_num_den(frame.time_base or vstream.time_base)
                fr_n, fr_d = _tb_num_den(getattr(vstream, "average_rate", None))
                rate_arg = f":frame_rate={fr_n}/{fr_d}" if (fr_n and fr_d) else ""

                # buffersrc with input SAR (we normalize later)
                src = g.add(
                    "buffer",
                    args=(
                        f"video_size={w}x{h}:"
                        f"pix_fmt={fmt_name}:"
                        f"time_base={tb_n}/{tb_d}:"
                        f"pixel_aspect={sar_n}/{sar_d}" + rate_arg
                    ),
                )
                last = src

                # (A) autocrop baked bars in source pixel coords, if decided
                if want_ac:
                    L, R, T, B = ac_crop["l"], ac_crop["r"], ac_crop["t"], ac_crop["b"]
                    cw = max(1, w - (L + R))
                    ch = max(1, h - (T + B))
                    n = g.add("crop", args=f"{cw}:{ch}:{L}:{T}")
                    last.link_to(n); last = n

                # unsqueeze PAR -> setsar=1
                n = g.add("scale", args="iw*sar:ih")
                last.link_to(n); last = n
                n = g.add("setsar", args="1")
                last.link_to(n); last = n

                # rotate (metadata) if needed
                if rot in (90, 180, 270):
                    if rot == 90:
                        n = g.add("transpose", args="clock"); last.link_to(n); last = n
                    elif rot == 270:
                        n = g.add("transpose", args="cclock"); last.link_to(n); last = n
                    else:  # 180
                        t1 = g.add("transpose", args="clock")
                        t2 = g.add("transpose", args="clock")
                        last.link_to(t1); t1.link_to(t2); last = t2

                # expand TV->PC range before final downscale if requested
                expand_args = ""
                if expand_mode == 2:
                    expand_args = ":in_range=tv:out_range=pc"
                elif expand_mode == 1:
                    expand_args = ":in_range=auto:out_range=pc"

                # Fit selection
                fit_mode = str(CONFIG.get("video", {}).get("fit", "auto-cover")).lower()
                if fit_mode in ("cover", "auto-cover"):
                    n = g.add("scale", args=f"{TW}:{TH}:flags=bilinear:force_original_aspect_ratio=increase" + expand_args)
                    last.link_to(n); last = n
                    n = g.add("crop", args=f"{TW}:{TH}:(in_w-{TW})/2:(in_h-{TH})/2")
                    last.link_to(n); last = n
                else:
                    n = g.add("scale", args=f"{TW}:{TH}:flags=bilinear:force_original_aspect_ratio=decrease" + expand_args)
                    last.link_to(n); last = n
                    n = g.add("pad", args=f"{TW}:{TH}:(ow-iw)/2:(oh-ih)/2:color=black")
                    last.link_to(n); last = n

                n = g.add("setdar", args="1")
                last.link_to(n); last = n
                n = g.add("format", args="rgb24")
                last.link_to(n); last = n

                sink = g.add("buffersink")
                last.link_to(sink)
                g.configure()

                graph = g
                src_in = src
                sink_out = sink

            last_pts_s: Optional[float] = None

            for packet in container.demux(vstream):
                # decode() may return 0..N frames (depending on codec & B-frames)
                frames = packet.decode()
                for frame in frames:
                    # --- Auto-crop sampling on early frames (before building graph) ---
                    if ac_enabled and not ac_decided and ac_seen < ac_probe_frames:
                        try:
                            gray = frame.to_ndarray(format="gray")
                            cand = _estimate_black_bars(frame.width, frame.height, gray,
                                                        ac_thresh, ac_min_px, ac_max_ratio)
                            # Adjust for rotation metadata: detector looked at pre-rotate coords,
                            # but we crop pre-rotate; remap by rotation so L/R/T/B stay correct.
                            r = _rotation_from_stream_and_frame(vstream, frame)
                            if r == 90:
                                cand = {"l": cand["t"], "r": cand["b"], "t": cand["r"], "b": cand["l"]}
                            elif r == 270:
                                cand = {"l": cand["b"], "r": cand["t"], "t": cand["l"], "b": cand["r"]}
                            elif r == 180:
                                cand = {"l": cand["r"], "r": cand["l"], "t": cand["b"], "b": cand["t"]}
                            for k in ("l", "r", "t", "b"):
                                ac_samples[k].append(int(cand[k]))
                        except Exception:
                            pass
                        finally:
                            ac_seen += 1
                            if ac_seen >= ac_probe_frames:
                                import statistics as _st
                                ac_crop = {k: int(_st.median(v) if v else 0) for k, v in ac_samples.items()}
                                ac_decided = True
                                # Force a rebuild next frame to apply crop
                                graph = None
                    _ensure_graph_for(frame)
                    src_in.push(frame)  # type: ignore[name-defined]

                    # Pull 0..N filtered frames
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

                        # Compute inter-frame delay using PTS if available; otherwise avg_ms fallback
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
                                if avg_ms is not None:
                                    low = 0.75 * avg_ms
                                    high = 1.25 * avg_ms
                                    delta_ms = max(low, min(high, delta_ms))
                                elif delta_ms <= 0:
                                    delta_ms = 33.33
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
            # If the original source is a YouTube *page* URL, treat demux/open errors
            # as transient: refresh the googlevideo URL on the next loop iteration.
            if _is_youtube_url(srcu):
                print(f"[retry] YouTube demux/open error; refreshing URL and retrying: {e!r}")
                # 'container' may or may not be defined here; suppress either way.
                with contextlib.suppress(Exception):
                    container.close()
                time.sleep(0.5)
                continue
            raise RuntimeError(f"av error: {e}") from e

        if not loop_video:
            break


def iter_frames(
    src: str,
    size: tuple[int, int],
    loop_video: bool = True,
    *,
    expand_mode: int,
    hw_prefer: Optional[str]
):
    srcu = unquote(src)
    low = srcu.lower()
    if not any(low.endswith(ext) for ext in (".png", ".jpg", ".jpeg", ".bmp", ".webp", ".gif")):
        yield from _iter_frames_pyav(srcu, size, loop_video, expand_mode=expand_mode, hw_prefer=hw_prefer)
    else:
        yield from _iter_frames_imageio(srcu, size, loop_video)


# -------------------------
# DDP packetizer
# -------------------------

# Max payload per DDP packet. 1440 keeps UDP datagrams < 1500B MTU
# (IP+UDP+DDP header overhead), reducing fragmentation on typical links.
_DDP_MAX_DATA = 1440

# DDP header layout (big-endian):
#   flags: 0x40 => header present, 0x01 => PUSH (end-of-frame)
#   seq:   0..255 sequence number (low 8 bits used)
#   cfg:   pixel config; 0x2C = RGB888 (per 3waylabs DDP spec)
#   out_id: destination output/canvas id (0..255)
#   offset: byte offset within the frame buffer
#   length: payload bytes in this packet
_DDP_HDR = struct.Struct("!BBB B I H")  # flags, seq, cfg, out_id, offset, length (network byte order)
_DDP_PIXEL_CFG_RGB888 = 0x2C


def _ddp_iter_packets(rgb_bytes: bytes, output_id: int, seq: int):
    mv = memoryview(rgb_bytes)
    total = len(mv)
    off = 0
    push_mask = 0x01
    ddp_base_flags = 0x40  # header present (per DDP spec); we'll OR in PUSH (0x01) on the last packet of a frame
    while off < total:
        end = min(off + _DDP_MAX_DATA, total)
        chunk = mv[off:end]
        is_last = end >= total
        flags = ddp_base_flags | (push_mask if is_last else 0)
        payload_len = len(chunk)
        pkt = bytearray(_DDP_HDR.size + payload_len)
        _DDP_HDR.pack_into(
            pkt, 0,
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
        self.transport: Optional[asyncio.DatagramTransport] = None
        self.packets_sent = 0

    def connection_made(self, transport: asyncio.BaseTransport) -> None:
        self.transport = transport  # type: ignore[assignment]

    def error_received(self, exc: BaseException) -> None:
        print(f"[udp] error_received: {exc!r}")

    def connection_lost(self, exc: BaseException | None) -> None:
        if exc:
            print(f"[udp] connection_lost: {exc!r}")

    def sendto(self, data: bytes, addr):
        if self.transport is not None:
            self.transport.sendto(data, addr)  # type: ignore[union-attr]
            self.packets_sent += 1


# -------------------------
# Main DDP task
# -------------------------

async def ddp_task(target_ip: str, target_port: int, out_id: int, *, size, src, opts):
    """If pace_hz>0: producer (native) + paced sampler. Else: native-cadence push.
       Non-blocking UDP with bounded queue. Optional per-frame packet spreading.
    """
    import socket
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
    log_rate_ms = int(log_cfg.get("rate_ms", 1000))
    spread_enabled = bool(CONFIG.get("net", {}).get("spread_packets", True))
    spread_max_fps = int(CONFIG.get("net", {}).get("spread_max_fps", 60))

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
            # Dedicated consumer for the UDP queue so producers never block on sendto().
            # Keeps packet pacing accurate even if the NIC jitters.
            while True:
                pkt = await q.get()
                try:
                    proto.sendto(pkt, addr)
                    if log_metrics:
                        pkt_meter.tick(time.perf_counter())
                finally:
                    q.task_done()

        drain_task = asyncio.create_task(drainer())

        pace_hz = int(opts.get("pace_hz", 0))
        ema_alpha = float(opts.get("ema_alpha", 0.0))

        frames_emitted = 0
        packets_enqueued = 0
        last_log = time.perf_counter()
        seq = 0

        async def enqueue_frame(rgb888: bytes, seq_val: int,
                                *, packet_spacing_s: Optional[float] = None,
                                   group_n: int = 1):
            nonlocal packets_enqueued, q_drops
            group_n = max(1, int(group_n))
            start_ts = loop.time() if (packet_spacing_s and packet_spacing_s > 0.0) else None
            slot_idx = 0
            group_left = group_n

            for pkt in _ddp_iter_packets(rgb888, out_id, seq_val):
                if q.full():
                    try:
                        q.get_nowait()
                        q.task_done()
                        q_drops += 1
                    except asyncio.QueueEmpty:
                        pass
                await q.put(pkt)
                packets_enqueued += 1

                group_left -= 1
                if group_left <= 0 and start_ts is not None and packet_spacing_s and packet_spacing_s > 0.0:
                    slot_idx += 1
                    target = start_ts + slot_idx * packet_spacing_s
                    await asyncio.sleep(max(0.0, target - loop.time()))
                    group_left = group_n

        if pace_hz > 0:
            # --- Pacing path ---
            # Producer decodes at source cadence, sampler emits at fixed pace_hz (optionally with EMA smoothing).
            # Pacing path (producer + sampler). EMA is applied in the sampler.
            latest: dict[str, Optional[bytes]] = {"buf": None}
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
                        frames_emitted = 0
                        q_occ_samples.clear()
                        last_log = now

                    await asyncio.sleep(max(MIN_DELAY_MS/1000.0, float(delay_ms)/1000.0))

            async def sampler():
                tick = 1.0 / pace_hz
                next_t = loop.time()
                ema_buf_f32: Optional[np.ndarray] = None

                while True:
                    buf = latest["buf"]
                    if buf is not None:
                        outb = buf
                        if ema_alpha > 0.0:
                            cur = np.frombuffer(buf, dtype=np.uint8)
                            if ema_buf_f32 is None or ema_buf_f32.shape != cur.shape:
                                ema_buf_f32 = cur.astype(np.float32, copy=True)
                            else:
                                ema_buf_f32 *= (1.0 - ema_alpha)
                                ema_buf_f32 += cur.astype(np.float32) * ema_alpha
                            outb = ema_buf_f32.astype(np.uint8, copy=False).tobytes()

                        if spread_enabled and pace_hz <= spread_max_fps:
                            pkt_count = (len(outb) + _DDP_MAX_DATA - 1) // _DDP_MAX_DATA
                            spacing, group_n = _compute_spacing_and_group(pkt_count, tick)
                        else:
                            spacing, group_n = (None, 1)
                        await enqueue_frame(outb, seq, packet_spacing_s=spacing, group_n=group_n)

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
            # --- Native cadence ---
            # Emit frames at the source-provided delay_ms without resampling.
            next_t = loop.time()
            for rgb888, delay_ms in frames:
                delay_s = max(MIN_DELAY_MS/1000.0, float(delay_ms)/1000.0)

                if spread_enabled:
                    inst_fps = (1.0 / delay_s) if delay_s > 0 else 1e9
                    if inst_fps <= spread_max_fps:
                        pkt_count = (len(rgb888) + _DDP_MAX_DATA - 1) // _DDP_MAX_DATA
                        spacing, group_n = _compute_spacing_and_group(pkt_count, delay_s)
                    else:
                        spacing, group_n = (None, 1)
                else:
                    spacing, group_n = (None, 1)

                await enqueue_frame(rgb888, seq, packet_spacing_s=spacing, group_n=group_n)
                seq = (seq + 1) & 0x0F
                frames_emitted += 1

                if log_metrics:
                    frm_meter.tick(time.perf_counter())
                    q_occ_samples.append(q.qsize())

                now = time.perf_counter()
                if now - last_log >= (log_rate_ms / 1000.0):
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
                    frames_emitted = 0
                    q_occ_samples.clear()
                    last_log = now

                next_t += delay_s
                await asyncio.sleep(max(0.0, next_t - loop.time()))

        # Try to flush the queue quickly; don't hang forever if we're being cancelled.
        try:
            await asyncio.wait_for(q.join(), timeout=0.5)
        except Exception:
            pass
        finally:
            if drain_task:
                drain_task.cancel()
                with contextlib.suppress(asyncio.CancelledError):
                    await drain_task

    except asyncio.CancelledError:
        try:
            if drain_task:
                drain_task.cancel()
                with contextlib.suppress(asyncio.CancelledError):
                    await drain_task
        except Exception:
            pass
        raise
    except Exception as e:
        print(f"[send] out={out_id} fatal error: {e!r}")
        traceback.print_exc()
        try:
            if drain_task:
                drain_task.cancel()
                with contextlib.suppress(asyncio.CancelledError):
                    await drain_task
        except Exception:
            pass
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


# -------------------------
# WebSocket control
# -------------------------

CONFIG: Dict[str, Any] = DEFAULT_CONFIG


def _is_benign_disconnect(exc: BaseException) -> bool:
    if isinstance(exc, OSError) and getattr(exc, "winerror", None) in (64, 121):
        return True
    return isinstance(exc, (ConnectionClosedOK, ConnectionClosedError))


def _parse_expand(val, default):
    s = str(val if val is not None else default).lower()
    if s in ("2", "force"):
        return 2
    if s in ("0", "false", "never"):
        return 0
    return 1  # auto


def _parse_hw(val, default):
    s = (val if val is not None else default)
    return None if str(s).lower() in ("none", "off", "cpu") else str(s)


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
        _require_fields(msg, ("out", "w", "h", "src"), "start_stream")
        out = int(msg["out"])
        w = int(msg["w"])
        h = int(msg["h"])
        if w <= 0 or h <= 0:
            raise ValueError(f"start_stream requires positive w/h (got {w}x{h})")
        src = str(msg["src"])
        ddp_port = int(msg.get("ddp_port", 4048))

        pace_hz = _parse_pace_hz(msg.get("pace", 0))
        opts = {
            "loop": _truthy(str(msg.get("loop", CONFIG["playback"]["loop"]))),
            "expand_mode": _parse_expand(msg.get("expand"), CONFIG["video"]["expand_mode"]),
            "hw": _parse_hw(msg.get("hw"), CONFIG["hw"]["prefer"]),
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
        for k in ("w", "h", "ddp_port", "src", "pace", "ema", "expand", "loop", "hw"):
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
        await _ws_send(ws, {"type": "error", "code": "proto", "message": "expect 'hello' first"})
        await ws.close(code=4001, reason="protocol")
        return

    session["device_id"] = hello.get("device_id", "unknown")
    print(f"* hello from {session['ip']} dev={session['device_id']} proto={hello.get('proto')}")
    await _ws_send(ws, {"type": "hello_ack", "server_version": "lvgl-ddp-stream/1"})

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
                    await _ws_send(ws, {"type": "pong", "t": msg.get("t")})
                else:
                    await _ws_send(ws, {"type": "error", "code": "bad_type", "message": f"unknown type {t}"})
            except (ValueError, FileNotFoundError) as e:
                await _ws_send(ws, {"type": "error", "code": "bad_request", "message": str(e)})
            except Exception as e:
                await _ws_send(ws, {"type": "error", "code": "server_error", "message": str(e)})
    except Exception as loop_exc:
        if _is_benign_disconnect(loop_exc):
            reason = getattr(loop_exc, 'reason', '') or (getattr(loop_exc, 'args', [''])[0])
            print(f"* disconnect {session['ip']} ({type(loop_exc).__name__}: {reason})")
        else:
            print(f"[warn] control loop error from {session['ip']}: {loop_exc!r}")
    finally:
        async with streams_lock:
            for _out, t in list(streams.items()):
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
