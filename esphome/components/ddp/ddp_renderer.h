// © Copyright 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT

#pragma once

#include <cstdint>

namespace esphome {
namespace ddp {

// Pixel format enumeration
enum class PixelFormat {
  RGB888,      // 3 bytes per pixel: R, G, B
  RGB565_BE,   // 2 bytes per pixel: RGB565 big-endian
  RGB565_LE,   // 2 bytes per pixel: RGB565 little-endian
  RGBW,        // 4 bytes per pixel: R, G, B, W
  UNKNOWN
};

// Abstract renderer interface
// All DDP renderers (canvas, light effect, etc.) implement this interface
//
// THREADING MODEL:
// - on_data() and on_push() are called from UDP receive task (FreeRTOS)
// - These methods MUST NOT call LVGL APIs or ESPHome component methods
// - Safe operations: write to pre-allocated buffers, set atomic flags, pure math
// - LVGL/ESPHome APIs should be deferred to Component::loop() via atomic flags
class DdpRenderer {
 public:
  virtual ~DdpRenderer() = default;

  // Called from UDP receive task for each data packet as it arrives (streaming)
  // SAFE: Write to pre-allocated buffers, pixel conversion, update atomics
  // UNSAFE: LVGL APIs, ESPHome APIs, large allocations
  //
  // offset_px: Pixel offset in frame where this data starts
  // pixels: Raw pixel data in DDP wire format (RGB888/RGB565/RGBW)
  // format: Source pixel format from DDP packet
  // pixel_count: Number of pixels in this packet
  virtual void on_data(size_t offset_px, const uint8_t* pixels,
                       PixelFormat format, size_t pixel_count) = 0;

  // Called from UDP receive task when PUSH flag received (end of frame)
  // SAFE: Set atomic flags to signal Component::loop()
  // UNSAFE: LVGL APIs (e.g., lv_obj_invalidate) - just set flags!
  virtual void on_push() = 0;

  // Get the DDP stream ID this renderer listens to
  virtual uint8_t get_stream_id() const = 0;

  // Get renderer's expected dimensions (for mDNS and media_proxy_control)
  // Returns true if dimensions are known, false otherwise
  // If false, width and height are not modified
  virtual bool get_dimensions(int* width, int* height) const = 0;

  // Get renderer's name for logging
  virtual const char* get_name() const = 0;
};

}  // namespace ddp
}  // namespace esphome
