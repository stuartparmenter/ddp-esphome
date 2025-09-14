# lvgl-ddp-stream

[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)
[![ESPHome](https://img.shields.io/badge/ESPHome-2025.8-blue)](https://esphome.io/)
[![ESP-IDF](https://img.shields.io/badge/ESP--IDF-5.x-orange)](https://docs.espressif.com/projects/esp-idf/)

Low-latency bridge to stream images/video into **LVGL** canvases via **DDP** (UDP/4048).

> **Heads up (v0.3.0): the control server moved!**  
> The Python/WebSocket media proxy has been split into its own repo:  
> **https://github.com/stuartparmenter/media-proxy**

This repository now contains:
- ESPHome custom components (ESP-IDF only): `ddp_stream`, `ws_ddp_control`
- A ready-to-use LVGL example page: `esphome/examples/page-ddp-stream.yaml`

> Tested with **ESPHome 2025.8** (ESP-IDF). Arduino is **not** supported.

---

## 1) Server (media proxy)
The media ingestion/resize/encode + WebSocket control server now lives in **`media-proxy`**.  
Follow its README for installation, CLI, config, and Home Assistant add-on:  
https://github.com/stuartparmenter/media-proxy

---

## 2) ESPHome example (ESP-IDF only)

A ready-made page is provided at:
```
esphome/examples/page-ddp-stream.yaml
```

It uses declarative canvas binding in `ddp_stream` and structured options in `ws_ddp_control`.  
Move sensitive values like `WS_HOST` and `VIDEO_SRC` to your `secrets.yaml`.  
Point `ws_host`/`ws_port` at your running **media-proxy** instance.

---

## 3) Quick examples

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

## 4) ESPHome configuration

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

## 5) Troubleshooting

- **ESP shows “connected” but blank screen**  
  - Ensure the canvas is bound via `ddp_stream.streams:`  
  - Confirm `ddp_stream.port` matches the server `ddp_port` (default 4048).  
  - Check server logs in **media-proxy** for frame sends.

- **Choppy GIFs**  
  - Use `pace=30` and `ema=0.2` in `ws_ddp_control`.

- **Cannot connect**  
  - Verify firewall allows TCP `8788` (media-proxy control) and UDP `4048` (DDP).  
  - Confirm correct LAN IP in `ws_host`.

---

## 6) Security notes

- DDP runs over UDP and is unauthenticated. Run only on a trusted LAN or VPN.  
- Do not expose the media-proxy control port (8788) or DDP port (4048) to the public internet.

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
