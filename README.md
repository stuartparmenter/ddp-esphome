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

The `ddp_stream` component can be used independently with any system that sends DDP packets to UDP port 4048. This includes:

- **[LedFx](https://github.com/LedFx/LedFx)** - Real-time music visualization software with excellent DDP support. Works great with this component. **Note:** LedFx always uses stream ID 1, so configure your `ddp_stream` with `stream: 1` for compatibility.
- **[WLEDVideoSync](https://github.com/Aircoookie/WLED/wiki/UDP-Realtime-Control#videosync)** - Though compatibility hasn't been extensively tested.

You only need `ws_ddp_control` if you want to use the companion media proxy server for video processing and WebSocket orchestration.

### RGB Format Notes

- **Default format**: RGB888 (24-bit, flag 0x2C) is the default format used by the media proxy server (as specified in the DDP protocol)
- **Recommended format**: RGB565 (16-bit, flags 0x61/0x62) is typically better for ESP32 devices since LVGL normally runs in 565 mode and uses less bandwidth
- **Extension support**: RGB565 formats are an extension to the DDP spec; our media proxy may be the only implementation that supports them
- **Automatic rendering**: The `ddp_stream` component automatically renders whatever pixel format it receives (RGB888, RGB565LE, or RGB565BE) as long as the DDP packet flags are set correctly

---

## Examples

Two ready-made example configurations are provided:

- **[ddp-receiver-only.yaml](esphome/examples/ddp-receiver-only.yaml)** - Minimal standalone DDP receiver (no WebSocket control). Perfect for use with [LedFx](https://github.com/LedFx/LedFx) or other DDP senders.
- **[ddp-websocket-full.yaml](esphome/examples/ddp-websocket-full.yaml)** - Complete setup with WebSocket control client for use with the media proxy server.

Move sensitive values like Wi-Fi credentials and video sources to your `secrets.yaml`.

---

## Configuration Reference

### `ddp_stream`

```yaml
ddp_stream:
  id: ddp
  port: 4048
  streams:
    - id: stream_1
      canvas: canvas64
      # stream: 1               # Uncomment and set to 1 for LedFx compatibility
```

**Parameters:**
- `id`: Component identifier for referencing in other components
- `port`: UDP port to listen on for DDP packets
- `back_buffers`: Default number of back buffers for smoother rendering (0=no buffering, 1-2=buffered)
- `streams`: List of DDP stream configurations
  - `id`: ESPHome component ID for this stream output
  - `canvas`: Name of the LVGL canvas object to render to
  - `stream`: DDP stream number (1-249). Use `1` for LedFx compatibility. Auto-generated if omitted.
  - `width`/`height`: Override canvas dimensions if needed
  - `back_buffers`: Per-stream back buffer override

### `ws_ddp_control`

```yaml
ws_ddp_control:
  id: ws
  ddp: ddp
  ws_host: "homeassistant.local"
  ws_port: 8788
  outputs:
    - id: output_1
      ddp_stream: stream_1
      src: "video.mp4"
      format: rgb565            # Use rgb565 for better performance on ESP32
```

**Parameters:**
- `id`: Component identifier for referencing in actions
- `ddp`: Reference to the `ddp_stream` component
- `ws_host`: Hostname or IP address of media proxy server
- `ws_port`: WebSocket port on media proxy server
- `device_id`: Identifier sent to media proxy for device tracking
- `url`: Full WebSocket URL (overrides `ws_host` and `ws_port` if provided)
- `outputs`: List of video output configurations
  - `id`: ESPHome component ID for this output
  - `ddp_stream`: Reference to a `ddp_stream` stream component
  - `src`: Video source (local file, HTTP/HTTPS URL, RTSP stream, etc.)
  - `width`/`height`: Output dimensions (auto-detected from canvas if omitted)
  - `format`: Pixel format for transmission. The `ddp_stream` component auto-detects rgb565le vs rgb565be. (rgb888 for quality, rgb565 for bandwidth)
  - `pace`: Frame timing adjustment in milliseconds (negative=speed up, positive=slow down). **Server has good defaults; rarely needs adjustment.**
  - `ema`: Exponential moving average smoothing for frame timing stability. **Server has good defaults; rarely needs adjustment.**
  - `expand`: Color range expansion from TV range (16-235) to PC range (0-255). **Server has good defaults; rarely needs adjustment.**
  - `fit`: How to fit video to canvas (auto=letterbox/pillarbox, pad=always letterbox, cover=fill and crop). **Server defaults to auto.**
  - `loop`: Whether to loop video playback
  - `hw`: Hardware acceleration method (depends on media proxy server capabilities). **Server has good defaults; rarely needs adjustment.**

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
- [LedFx](https://github.com/LedFx/LedFx) - Real-time music visualization with DDP protocol support
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
