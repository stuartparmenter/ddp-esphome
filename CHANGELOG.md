# Changelog

All notable changes to this project will be documented in this file.

## [0.3.0] - 2025-09-14
### Breaking
- The Python WebSocket control server has moved out of this repository to a new standalone project: **stuartparmenter/media-proxy**. All code under `server/` has been removed here. Please use the new repo for installation, configuration, and Home Assistant add-on packaging.
  - Repo: https://github.com/stuartparmenter/media-proxy
### Notes
- This repository now focuses solely on the ESPHome external components (`ddp_stream`, `ws_ddp_control`) and examples.
- README has been updated to point to the new server repository.

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
