# lvgl-ddp-stream

[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)
[![ESPHome](https://img.shields.io/badge/ESPHome-2025.8-blue)](https://esphome.io/)
[![ESP-IDF](https://img.shields.io/badge/ESP--IDF-5.x-orange)](https://docs.espressif.com/projects/esp-idf/)

ESPHome external components for streaming images/video to **LVGL** canvases via **DDP** (Distributed Display Protocol) over UDP.

## Components

This repository provides two ESPHome components:

- **`ddp_stream`**: UDP server that receives DDP packets and renders them to LVGL canvases
- **`ws_ddp_control`**: WebSocket client for orchestrating video playback via a media proxy server

> Tested with **ESPHome 2025.8** (ESP-IDF only). Arduino is **not** supported.

### Standalone Usage

The `ddp_stream` component can be used independently with any system that sends DDP packets to UDP port 4048. This includes tools like [WLEDVideoSync](https://github.com/Aircoookie/WLED/wiki/UDP-Realtime-Control#videosync) (though compatibility hasn't been extensively tested). You only need `ws_ddp_control` if you want to use the companion media proxy server for video processing and WebSocket orchestration.

### RGB Format Notes

- **Default format**: RGB888 (24-bit, flag 0x2C) is the default format used by the media proxy server (as specified in the DDP protocol)
- **Recommended format**: RGB565 (16-bit, flags 0x61/0x62) is typically better for ESP32 devices since LVGL normally runs in 565 mode and uses less bandwidth
- **Extension support**: RGB565 formats are an extension to the DDP spec; our media proxy may be the only implementation that supports them
- **Automatic rendering**: The `ddp_stream` component automatically renders whatever pixel format it receives (RGB888, RGB565LE, or RGB565BE) as long as the DDP packet flags are set correctly

---

A ready-made example configuration is provided at:
```
esphome/examples/page-ddp-stream.yaml
```

This example demonstrates both components working together with a media proxy server. Move sensitive values like `WS_HOST` and `VIDEO_SRC` to your `secrets.yaml`.

---

### `ddp_stream`

```yaml
ddp_stream:
  id: ddp
  port: 4048
  streams:
    - id: stream_1              # Required: ESPHome component ID for this stream
      canvas: canvas64          # Required: LVGL canvas object name
      width: 64                 # Optional: Override canvas width
      height: 64                # Optional: Override canvas height
      # stream: auto            # Optional: DDP stream number (auto-generated if omitted)
```

### `ws_ddp_control`

```yaml
ws_ddp_control:
  id: ws                        # Required: Component ID
  ddp: ddp                      # Required: Reference to ddp_stream component
  ws_host: ${WS_HOST}           # Optional: WebSocket host (can use url instead)
  ws_port: 8788                 # Optional: WebSocket port (default: 8788)
  device_id: "esp32-device"     # Optional: Device identifier (default: "unknown")
  url: ""                       # Optional: Full WebSocket URL override
  outputs:                      # Optional: List of output streams
    - id: output_1              # Required: ESPHome component ID for this output
      ddp_stream: stream_1      # Required: Reference to ddp_stream's stream component
      src: ${VIDEO_SRC}         # Required: Video source
      width: 64                 # Optional: Output width (auto-detected if omitted)
      height: 64                # Optional: Output height (auto-detected if omitted)
      format: rgb888            # Optional: Pixel format (rgb888, rgb565, rgb565le, rgb565be)
      pace: 30                  # Optional: Frame pacing (0-240)
      ema: 0.2                  # Optional: EMA smoothing (0.0-1.0)
      expand: auto              # Optional: Scaling (never, auto, force)
      loop: true                # Optional: Loop video playbook
      hw: auto                  # Optional: Hardware acceleration (auto, none, cuda, qsv, vaapi, videotoolbox, d3d11va)
```

### Using Actions

```yaml
# Runtime source change example
- ws_ddp_control.set_src:
    id: output_1                # Reference the WsDdpOutput component directly
    src: "new_video.mp4"
```

---

## Media Proxy Server

For video processing and WebSocket orchestration, this project works with a companion media proxy server:

**Repository**: https://github.com/stuartparmenter/media-proxy

The media proxy handles:
- Media ingestion, resizing, and encoding
- WebSocket control interface
- Home Assistant add-on support

---

## Security notes

- DDP runs over UDP and is unauthenticated. Run only on a trusted LAN or VPN.  
- Do not expose the media-proxy control port (8788) or DDP port (4048) to the public internet.

---

## Acknowledgements

This project was inspired in part by:
- [WLED Video Sync](https://github.com/Aircoookie/WLED/wiki/UDP-Realtime-Control#videosync)
- [DDP Protocol Specification](http://www.3waylabs.com/ddp/)

Many thanks to those projects for pioneering low-latency LED streaming approaches.

---

## Related Projects & Documentation

- [LVGL Documentation](https://docs.lvgl.io/) - Lightweight embedded graphics library used for rendering.
- [ESPHome](https://esphome.io/) - Framework for building firmware for ESP32/ESP8266 devices.
- [ESP WebSocket Client](https://github.com/espressif/esp-protocols/tree/master/components/esp_websocket_client) - WebSocket client component used by `ws_ddp_control`.

## License

MIT - see [LICENSE](LICENSE).
