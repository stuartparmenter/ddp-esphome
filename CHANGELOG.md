# Changelog

All notable changes to this project will be documented in this file.

## [0.2.0] - 2025-08-23
### Breaking
- Usage of ddp_stream and ws_ddp_control have changed to enable a single websocket connection for multiple streams.  Please see updated examples.

### Fixed
- Huge performance improvements to streaming (from ~3fps -> ~30) via both client and server side fixes.

## [0.1.0] - 2025-08-20
### Added
- Initial public release
- Python WebSocket DDP server (`server.py`)
- ESPHome external components: `ddp_stream`, `ws_ddp_control`
- Example LVGL page: `page-ddp-stream.yaml`
