# lvgl-ddp-stream

[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)
[![ESPHome](https://img.shields.io/badge/ESPHome-2025.8-blue)](https://esphome.io/)
[![ESP-IDF](https://img.shields.io/badge/ESP--IDF-5.x-orange)](https://docs.espressif.com/projects/esp-idf/)



Low-latency bridge to stream images/video into **LVGL** canvases via **DDP** (UDP/4048).

Includes:
- Python WebSocket control server: `server/server.py`
- ESPHome custom components (ESP-IDF only): `ddp_stream`, `ws_ddp_control`
- A ready-to-use LVGL example page: `esphome/examples/page-ddp-stream.yaml`

> Tested with **ESPHome 2025.8** (ESP-IDF). Arduino is **not** supported.

---

## 1) Setup (Python environment + dependencies)

This project isn’t packaged yet - you just need to create a virtual environment and install dependencies.

*Note:* In the future we’ll provide a Home Assistant add-on so you can just click-to-install. For now, run the Python server manually as shown below.

We use **two files** for dependencies:

- `requirements.txt` - loose, user-friendly version ranges
- `constraints.txt` - exact versions we have tested (for reproducibility)

```bash
python -m venv .venv
# Windows
.venv\Scripts\activate
# macOS/Linux
source .venv/bin/activate

# Install with tested pins (recommended)
pip install -r server/requirements.txt -c server/constraints.txt
```

If you want to install with just the loose ranges, omit the constraints file:

```bash
pip install -r server/requirements.txt
```

**Prereqs:** Python 3.10+, and `ffmpeg` available on your PATH.  
(`imageio[ffmpeg]` installs an ffmpeg binary automatically on Windows/macOS/Linux, but if you hit errors, install ffmpeg manually and ensure it is on your PATH.)

---

## 2) Run the server

Start the WebSocket control server (it sends DDP frames to your ESP device):

```bash
python server/server.py --host 0.0.0.0 --port 8788
```

Your ESP device should then connect to a URL like:
```
ws://<server-ip>:8788/control?w=<W>&h=<H>&out=<ID>&src=<SRC>[&pace=30][&ema=0.2][&expand=auto][&loop=1][&ddp_port=4048][&hw=auto]
```

### Control parameters
- `w`, `h` - canvas width/height (1-255)
- `out` - DDP stream ID (0-255); maps to a specific LVGL canvas on the ESP
- `src` - media source
  - Local file next to the server script: `mario.gif`
  - Absolute file path: `file:///C:/media/clip.mp4` or `file:///home/user/video.mp4`
  - HTTP/HTTPS URL: `https://example.com/clip.mp4` (GIF/JPEG/PNG also supported)
- Optional:
  - `pace` - integer FPS to upsample low-FPS sources (e.g. `30`; `0` disables)
  - `ema` - smoothing factor `0.0-1.0` (e.g. `0.2`)
  - `expand` - `0|never`, `1|auto` (default), `2|force`
  - `loop` - `0|1` (repeat)
  - `ddp_port` - DDP UDP port (default `4048`)
  - `hw` - decoder preference: `auto|none|qsv|vaapi|videotoolbox|d3d11va|cuda`

Example defaults file (`ws_ddp_proxy.yaml`):
```yaml
hw:
  prefer: auto
video:
  expand_mode: 1
playback:
  loop: true
log:
  send_ms: false
net:
  win_timer_res: true
```

---

## 3) ESPHome example (ESP-IDF only)

A ready-made page is provided at:
```
esphome/examples/page-ddp-stream.yaml
```

It uses declarative canvas binding in `ddp_stream` and structured options in `ws_ddp_control`.  
Move sensitive values like `WS_HOST` and `VIDEO_SRC` to your `secrets.yaml`.

---

## 4) Quick examples

- **Local GIF next to the server**
  ```yaml
  src: "mario.gif"
  ```

- **HTTP video**
  ```yaml
  src: "https://commondatastorage.googleapis.com/gtv-videos-bucket/sample/BigBuckBunny.mp4"
  ```

- **Absolute file path**
  - Windows: `src: "file:///C:/media/clip.mp4"`
  - Linux/macOS: `src: "file:///home/user/media/clip.mp4"`

---

## 5) ESPHome configuration

### `ddp_stream`

```yaml
ddp_stream:
  id: ddp
  port: 4048
  streams:
    - id: 1
      canvas_id: canvas64
      width: 64
      height: 64
```

### `ws_ddp_control`

```yaml
ws_ddp_control:
  - id: ws
    ws_host: ${WS_HOST}
    ws_port: ${WS_PORT}
    width: ${WIDTH}
    height: ${HEIGHT}
    out: ${OUT_ID}
    src: ${VIDEO_SRC}
    pace: 30
    ema: 0.2
    expand: auto
    loop: true
    ddp_port: 4048
    hw: auto
```

---

## 6) Troubleshooting

- **ESP shows “connected” but blank screen**  
  - Ensure the canvas is bound via `ddp_stream.streams:`  
  - Confirm `ddp_stream.port` matches the server `ddp_port` (default 4048).  
  - Check server logs for frame sends.

- **Wrong colors (red/blue swapped)**  
  - Set or unset `LV_COLOR_16_SWAP` in your LVGL config to match panel wiring.

- **Choppy GIFs**  
  - Use `pace=30` and `ema=0.2` in `ws_ddp_control`.

- **Cannot connect**  
  - Verify firewall allows TCP `8788` and UDP `4048`.  
  - Confirm correct LAN IP in `ws_host`.

- **Windows firewall**  
  - Allow **TCP 8788** and **UDP 4048** in Windows Defender Firewall for the Python executable running the server.

- **Windows jitter**  
  - Enable `win_timer_res` in config to improve timing precision.

---

## 7) Updating dependency pins

When you test newer libraries locally, refresh `constraints.txt`:

```bash
pip install -U -r server/requirements.txt
# run your tests
pip freeze | grep -E "^(av|imageio|imageio-ffmpeg|numpy|pillow|psutil|PyYAML|websockets)=="
# paste those exact lines into server/constraints.txt
```

---

## 8) Security notes

- DDP runs over UDP and is unauthenticated. Run only on a trusted LAN or VPN.  
- Do not expose the server port (8788) or DDP port (4048) to the public internet.

---


---

## Planned Home Assistant Add-on

We plan to provide a **Home Assistant add-on** so you can run the DDP server with a single click, managed entirely from the Home Assistant UI.

The add-on will:
- Run the `lvgl-ddp-stream` server as a background service
- Handle Python dependencies and ffmpeg automatically
- Expose configuration (ports, defaults, media path) through the HA add-on options
- Update along with your Home Assistant system

Until then, use the manual Python setup described above.



---

## Acknowledgements

This project was inspired in part by:
- [WLED Video Sync](https://github.com/Aircoookie/WLED/wiki/UDP-Realtime-Control#videosync)
- [WLED DDP protocol reference](https://kno.wled.ge/interfaces/ddp/)

Many thanks to those projects for pioneering low-latency LED streaming approaches.



---

## Related Projects & Documentation

- [LVGL Documentation](https://docs.lvgl.io/) - Lightweight embedded graphics library used for rendering.
- [ESPHome](https://esphome.io/) - Framework for building firmware for ESP32/ESP8266 devices.
- [ESP-IDF](https://docs.espressif.com/projects/esp-idf/) - Official Espressif IoT Development Framework.


## License

MIT - see [LICENSE](LICENSE).
