// © Copyright 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT
//
// Light effect renderer - renders DDP to AddressableLight strips

#include "ddp_light_effect.h"
#include "esphome/components/ddp/ddp_pixel_convert.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include <cstring>
#include <algorithm>

namespace esphome {
namespace ddp {

static const char* TAG = "ddp.light_effect";

// -------- DdpRenderer interface (UDP TASK CONTEXT) --------

void DdpLightEffect::on_data(size_t offset_px, const uint8_t* pixels,
                              PixelFormat format, size_t pixel_count) {
  // UDP TASK CONTEXT - no ESPHome APIs allowed!

  // Allocate buffer if not yet initialized
  if (!frame_buffer_ && frame_pixels_ > 0) {
    frame_buffer_ = allocator_.allocate(frame_pixels_ * 4);  // RGBW
    if (!frame_buffer_) {
      ESP_LOGE(TAG, "Failed to allocate frame buffer (%zu bytes)", frame_pixels_ * 4);
      return;
    }
    std::memset(frame_buffer_, 0, frame_pixels_ * 4);
  }

  if (!frame_buffer_ || frame_pixels_ == 0) return;

  // Bounds check
  if (offset_px + pixel_count > frame_pixels_) {
    return;  // Silently drop out-of-bounds data
  }

  uint8_t* dst_rgbw = frame_buffer_ + (offset_px * 4);

  // Convert all formats to RGBW on arrival (single conversion)
  if (format == PixelFormat::RGBW) {
    // Direct copy - already RGBW
    std::memcpy(dst_rgbw, pixels, pixel_count * 4);

  } else if (format == PixelFormat::RGB888) {
    // RGB888 → RGBW (calculate white from RGB average)
    const uint8_t* sp = pixels;
    for (size_t i = 0; i < pixel_count; ++i) {
      uint8_t r = sp[0];
      uint8_t g = sp[1];
      uint8_t b = sp[2];
      dst_rgbw[i * 4 + 0] = r;
      dst_rgbw[i * 4 + 1] = g;
      dst_rgbw[i * 4 + 2] = b;
      dst_rgbw[i * 4 + 3] = (r + g + b) / 3;  // W = average
      sp += 3;
    }

  } else if (format == PixelFormat::RGB565_BE || format == PixelFormat::RGB565_LE) {
    // RGB565 → RGBW (expand to RGB888, then calculate white)
    const bool src_be = (format == PixelFormat::RGB565_BE);
    const uint8_t* sp = pixels;

    for (size_t i = 0; i < pixel_count; ++i) {
      uint16_t v = src_be ? (uint16_t)((sp[0] << 8) | sp[1])
                          : (uint16_t)((sp[1] << 8) | sp[0]);
      uint8_t r5 = (uint8_t)((v >> 11) & 0x1F);
      uint8_t g6 = (uint8_t)((v >> 5)  & 0x3F);
      uint8_t b5 = (uint8_t)( v        & 0x1F);

      uint8_t r = (uint8_t)((r5 << 3) | (r5 >> 2));
      uint8_t g = (uint8_t)((g6 << 2) | (g6 >> 4));
      uint8_t b = (uint8_t)((b5 << 3) | (b5 >> 2));

      dst_rgbw[i * 4 + 0] = r;
      dst_rgbw[i * 4 + 1] = g;
      dst_rgbw[i * 4 + 2] = b;
      dst_rgbw[i * 4 + 3] = (r + g + b) / 3;  // W = average
      sp += 2;
    }
  }
}

void DdpLightEffect::on_push() {
  // UDP TASK CONTEXT - just set atomic flag
  frame_ready_.store(true, std::memory_order_release);
}

bool DdpLightEffect::get_dimensions(int* w, int* h) const {
  // LED strips are 1D: report as num_leds × 1
  auto* it = get_addressable_();
  if (it) {
    if (w) *w = it->size();
    if (h) *h = 1;
    return true;
  }

  return false;
}

// -------- AddressableLightEffect interface --------

void DdpLightEffect::start() {
  AddressableLightEffect::start();

  // Note: Renderer registration happens at codegen time (in Python __init__.py)
  // so that DdpComponent::setup() has complete renderer info for mDNS

  // Calculate frame buffer size from LED count
  int num_leds = 0;
  if (get_dimensions(&num_leds, nullptr)) {
    frame_pixels_ = (size_t)num_leds;
  }

  ESP_LOGI(TAG, "Started DDP light effect '%s' for stream %u (%d LEDs)",
           name_.c_str(), stream_id_, num_leds);
}

void DdpLightEffect::stop() {
  // Note: We don't unregister since registration is permanent (happens at codegen time)
  ESP_LOGI(TAG, "Stopped DDP light effect '%s' for stream %u", name_.c_str(), stream_id_);

  // Free frame buffer (RGBW = 4 bytes per pixel)
  if (frame_buffer_) {
    allocator_.deallocate(frame_buffer_, frame_pixels_ * 4);
    frame_buffer_ = nullptr;
  }

  AddressableLightEffect::stop();
}

void DdpLightEffect::apply(light::AddressableLight& it, const Color& current_color) {
  // MAIN THREAD CONTEXT - safe to call ESPHome APIs

  // Check if we have a frame ready
  if (!frame_ready_.exchange(false, std::memory_order_acquire)) {
    return;
  }

  if (!frame_buffer_ || frame_pixels_ == 0) return;

  // Get LED count
  int num_leds = it.size();

  // Limit to available LEDs
  size_t leds_to_set = std::min((size_t)num_leds, frame_pixels_);

  // Apply RGBW pixels directly (buffer is always RGBW after on_data conversion)
  const uint8_t* data = frame_buffer_;
  for (size_t i = 0; i < leds_to_set; i++) {
    it[i] = Color(data[i * 4 + 0],  // R
                  data[i * 4 + 1],  // G
                  data[i * 4 + 2],  // B
                  data[i * 4 + 3]); // W
  }

  // Schedule LED update
  it.schedule_show();
}

}  // namespace ddp
}  // namespace esphome
