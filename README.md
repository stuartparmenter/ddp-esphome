# lvgl-ddp-stream

[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)
[![ESPHome](https://img.shields.io/badge/ESPHome-2025.8-blue)](https://esphome.io/)
[![ESP-IDF](https://img.shields.io/badge/ESP--IDF-5.x-orange)](https://docs.espressif.com/projects/esp-idf/)

ESPHome external components for streaming images/video to **LVGL** canvases via **DDP** (Distributed Display Protocol) over UDP.

**📚 Documentation:**
- [DDP Protocol Specification](http://www.3waylabs.com/ddp/) - Official DDP protocol documentation
- [DDP mDNS Discovery Spec](DDP-MDNS-DISCOVERY.md) - Zero-configuration network discovery (our extension to DDP)

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

### Pixel Format Notes

- **RGB888** (24-bit, DDP type `0x0B`): Standard DDP format, 8 bits per channel. Used by most DDP implementations including WLED and LedFx.
- **RGB565** (16-bit, DDP type `0x61` little-endian / `0x62` big-endian): Extension to DDP spec. Recommended for ESP32 since LVGL typically runs in 16-bit color mode. Reduces bandwidth by ~33%.
- **Automatic rendering**: The `ddp_stream` component automatically renders whatever pixel format it receives (RGB888, RGB565LE, or RGB565BE) as long as the DDP packet type flags are set correctly.

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
      receiving:                # Optional: binary sensor for monitoring
        name: "Stream 1 Receiving"
        device_class: connectivity
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
  - `receiving`: Optional binary sensor to track if stream is receiving data
    - Supports all standard ESPHome binary sensor options (name, device_class, filters, etc.)
    - State is `ON` when packets are actively being received
    - State is `OFF` when no packets received for 1 second
    - Useful for monitoring stream connectivity and debugging

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

## Network Discovery

The `ddp_stream` component automatically advertises via **mDNS/Zeroconf** as `_ddp._udp.local.` for zero-configuration discovery on the local network.

**📖 Full Discovery Specification:** See [DDP-MDNS-DISCOVERY.md](DDP-MDNS-DISCOVERY.md) for complete technical details, TXT record format, and implementation guidelines.

**Note:** While other DDP senders could use mDNS to find devices, most current implementations don't implement discovery yet.

### Quick Start

Devices advertise with these TXT records:
- `txtvers=1` - TXT record format version
- `protovers=1` - DDP protocol version
- `fmts=rgb888,rgb565` - Supported pixel formats
- `ids=1,2,3` - Available DDP Destination IDs
- `id1=64x64` - Optional: Dimensions for each ID

### Configuring Discovery

To advertise display dimensions via mDNS, configure explicit `width` and `height`:

```yaml
ddp_stream:
  streams:
    - canvas: my_canvas
      stream: 1
      width: 64      # Advertised via mDNS for discovery
      height: 64     # Without these, dimensions are determined from canvas at runtime
```

**Note:** If width/height are not specified, the stream will still work (dimensions come from the bound canvas), but senders won't know the resolution in advance.

**Network considerations:** mDNS works reliably on most home/office networks but may be filtered on enterprise networks with strict multicast policies.

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
