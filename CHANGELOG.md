# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.7.0] - 2025-10-12

### Changed
- **BREAKING**: Repository renamed from `lvgl-ddp-stream` to `ddp-esphome` to better reflect broader scope (LVGL + LED strips)
  - GitHub automatically redirects old URLs
  - Update your `external_components` source: `github://stuartparmenter/ddp-esphome@main`

### Removed
- **BREAKING**: Removed deprecated `ddp_stream` component (replaced by modular `ddp` + `ddp_canvas`/`ddp_light_effect` architecture)
- **BREAKING**: Removed deprecated `ws_ddp_control` component (replaced by `media_proxy_control`)
- **Note**: Users still using these components should stay on **v0.6.1** or migrate to the new modular architecture (see examples for migration guidance)

## [0.6.1] - 2025-10-10

### Fixed
- Handle RGBW correctly in ddp_canvas and ddp_light_effect

## [0.6.0] - 2025-10-10

### Added
- New modular architecture with `ddp`, `ddp_canvas`, `ddp_light_effect`, and `media_proxy_control` components
- Support for multiple renderers per DDP stream (e.g., mirror to canvas + LEDs simultaneously)
- `ddp_light_effect` for AddressableLight LED strips with RGB/RGBW support
- Polymorphic stream references in `media_proxy_control` (component IDs or integers)
- mDNS/Zeroconf network discovery as `_ddp._udp.local.` service
- DDP mDNS Discovery Specification document (DDP-MDNS-DISCOVERY.md)

### Changed
- Refactored monolithic `ddp_stream` into modular components
- Separated rendering (ddp_canvas, ddp_light_effect) from protocol (ddp)

### Deprecated
- `ddp_stream` component (use `ddp` + `ddp_canvas` or `ddp_light_effect`)
- `ws_ddp_control` component (use `media_proxy_control`)

## [0.5.3] - 2025-10-08

### Added
- Optional binary sensor to monitor if data is being received on streams
- Fit control setting (auto, pad, cover) for video/image display
- New DDP receiver-only example configuration

## [0.5.2] - 2025-10-06

### Fixed
- Use correct DDP byte for rgb888 format

### Changed
- Allow stream to be set to 1

## [0.5.1] - 2025-09-29

### Changed
- Optimize ESPHome component loop usage
- Only log when there is activity in the window

## [0.5.0] - 2025-09-23

### Changed
- Refactor how we use IDs to make them more ESPHome-friendly

### Fixed
- Crash fix in DDP stream when WiFi connection drops

## [0.4.3] - 2025-09-21

### Added
- Update example to use `set_src` action

### Changed
- Clean up README documentation and structure
- Make `src` required and add `set_src` action

## [0.4.2] - 2025-09-21

### Fixed
- Fix disconnect crash

## [0.4.1] - 2025-09-20

### Changed
- Rework reconnecting to make sure that we always reconnect appropriately

### Fixed
- Attempt to fix crash that occurs sometimes on disconnect

## [0.4.0] - 2025-09-16

### Changed
- Make most schema options actually optional and don't send them to the server unless specified (so the server defaults can actually kick in)

## [0.3.1] - 2025-09-14

### Changed
- Auto retry starts until there is a valid size

## [0.3.0] - 2025-09-10

### Removed
- **BREAKING**: The Python WebSocket control server has moved out of this repository to a new standalone project: **stuartparmenter/media-proxy**. All code under `server/` has been removed here. Please use the new repo for installation, configuration, and Home Assistant add-on packaging.
  - Repo: https://github.com/stuartparmenter/media-proxy

### Changed
- This repository now focuses solely on the ESPHome external components (`ddp_stream`, `ws_ddp_control`) and examples
- README has been updated to point to the new server repository

## [0.2.2] - 2025-08-28

### Added
- Support for optional RGB565 DDP streams to reduce bandwidth & conversion costs

### Fixed
- Fix rendering when LV_COLOR_DEPTH=32
- Fix deprecated Image.fromarray(..., mode="RGB") usage

## [0.2.1] - 2025-08-27

### Added
- Config for number of back buffers
- Better support for still images (with loop=false) send a few DDP frames over (to avoid the client missing any), backing off over a few seconds before stopping
- Improved resizing of stills for low DPI

## [0.2.0] - 2025-08-26

### Added
- Support for YouTube videos
- Auto-pad (default) and auto-cover modes for better fitting
- Triple buffering and drawing optimizations
- Metrics logs for performance monitoring
- Hardware acceleration auto-detection

### Changed
- **BREAKING**: Usage of `ddp_stream` and `ws_ddp_control` have changed to enable a single websocket connection for multiple streams. Please see updated examples.
- Re-resolve and restart YouTube videos if they fail
- Rework front_buf to properly use the passed in canvas's buffer avoiding an extra buffer allocation
- Change buffer allocations to use lv_mem_alloc
- Better hw accel auto-detection, optimized graph to pull frame to CPU later
- Optimize packet handling and rendering paths by swapping colors while decoding rather than in a full buffer during render
- Simplify to use a single websocket connection for control of multiple videos
- Adjust packet waits to try and optimize processing
- Avoid double sends when first connecting and send all the attributes through to the server when updating

### Fixed
- Hardware acceleration and rework UDP packet sending to reduce jitter and improve throughput to client
- Huge performance improvements to streaming (from ~3fps -> ~30) via both client and server side fixes
- Clear initial canvas to avoid flashing
- Improved error handling

## [0.1.0] - 2025-08-20

### Added
- Initial public release
- Python WebSocket DDP server (`server.py`)
- ESPHome external components: `ddp_stream`, `ws_ddp_control`
- Example LVGL page: `page-ddp-stream.yaml`

[unreleased]: https://github.com/stuartparmenter/ddp-esphome/compare/v0.7.0...HEAD
[0.7.0]: https://github.com/stuartparmenter/ddp-esphome/compare/v0.6.1...v0.7.0
[0.6.1]: https://github.com/stuartparmenter/ddp-esphome/compare/v0.6.0...v0.6.1
[0.6.0]: https://github.com/stuartparmenter/ddp-esphome/compare/v0.5.3...v0.6.0
[0.5.3]: https://github.com/stuartparmenter/ddp-esphome/compare/v0.5.2...v0.5.3
[0.5.2]: https://github.com/stuartparmenter/ddp-esphome/compare/v0.5.1...v0.5.2
[0.5.1]: https://github.com/stuartparmenter/ddp-esphome/compare/v0.5.0...v0.5.1
[0.5.0]: https://github.com/stuartparmenter/ddp-esphome/compare/v0.4.3...v0.5.0
[0.4.3]: https://github.com/stuartparmenter/ddp-esphome/compare/v0.4.2...v0.4.3
[0.4.2]: https://github.com/stuartparmenter/ddp-esphome/compare/v0.4.1...v0.4.2
[0.4.1]: https://github.com/stuartparmenter/ddp-esphome/compare/v0.4.0...v0.4.1
[0.4.0]: https://github.com/stuartparmenter/ddp-esphome/compare/v0.3.1...v0.4.0
[0.3.1]: https://github.com/stuartparmenter/ddp-esphome/compare/v0.3.0...v0.3.1
[0.3.0]: https://github.com/stuartparmenter/ddp-esphome/compare/v0.2.2...v0.3.0
[0.2.2]: https://github.com/stuartparmenter/ddp-esphome/compare/v0.2.1...v0.2.2
[0.2.1]: https://github.com/stuartparmenter/ddp-esphome/compare/v0.2.0...v0.2.1
[0.2.0]: https://github.com/stuartparmenter/ddp-esphome/compare/v0.1.0...v0.2.0
[0.1.0]: https://github.com/stuartparmenter/ddp-esphome/releases/tag/v0.1.0
