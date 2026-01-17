// © Copyright 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT
//
// Monochromatic light effect renderer - aggregates DDP pixels to single brightness

#include "ddp_monochromatic_light_effect.h"
#include "esphome/core/log.h"
#include <algorithm>
#include <type_traits>

namespace esphome::ddp {

static const char *TAG = "ddp.mono_effect";

// Helper to extract const char* from either std::string or const char*
template<typename T> static inline const char *get_cstr(const T &val) {
  if constexpr (std::is_pointer_v<T>) {
    return val;  // Already const char*
  } else {
    return val.c_str();  // std::string, call .c_str()
  }
}

// -------- DdpRenderer interface (UDP TASK CONTEXT) --------

void DdpMonochromaticLightEffect::on_data(size_t offset_px, const uint8_t *pixels, PixelFormat format,
                                          size_t pixel_count) {
  // UDP TASK CONTEXT - no ESPHome APIs allowed!

  // Track maximum brightness across all pixels in the frame
  uint8_t max_bri = max_brightness_.load(std::memory_order_relaxed);

  if (format == PixelFormat::RGBW) {
    // RGBW: 4 bytes per pixel
    for (size_t i = 0; i < pixel_count; ++i) {
      uint8_t r = pixels[i * 4 + 0];
      uint8_t g = pixels[i * 4 + 1];
      uint8_t b = pixels[i * 4 + 2];
      uint8_t w = pixels[i * 4 + 3];
      // Add white channel to RGB brightness
      uint8_t rgb_bri = rgb_to_brightness(r, g, b, brightness_method_);
      uint16_t total = static_cast<uint16_t>(rgb_bri) + w;
      uint8_t bri = total > 255 ? 255 : static_cast<uint8_t>(total);
      max_bri = std::max(max_bri, bri);
    }
  } else if (format == PixelFormat::RGB888) {
    // RGB888: 3 bytes per pixel
    for (size_t i = 0; i < pixel_count; ++i) {
      uint8_t bri = rgb_to_brightness(pixels[i * 3 + 0], pixels[i * 3 + 1], pixels[i * 3 + 2], brightness_method_);
      max_bri = std::max(max_bri, bri);
    }
  } else if (format == PixelFormat::RGB565_BE || format == PixelFormat::RGB565_LE) {
    // RGB565: 2 bytes per pixel
    bool big_endian = (format == PixelFormat::RGB565_BE);
    for (size_t i = 0; i < pixel_count; ++i) {
      uint8_t r, g, b;
      expand_rgb565_to_rgb888(pixels + i * 2, big_endian, r, g, b);
      uint8_t bri = rgb_to_brightness(r, g, b, brightness_method_);
      max_bri = std::max(max_bri, bri);
    }
  }

  max_brightness_.store(max_bri, std::memory_order_relaxed);
}

void DdpMonochromaticLightEffect::on_push() {
  // UDP TASK CONTEXT - just set atomic flag
  frame_ready_.store(true, std::memory_order_release);
}

bool DdpMonochromaticLightEffect::get_dimensions(int *w, int *h) const {
  // Monochromatic lights are treated as a single pixel
  if (w)
    *w = 1;
  if (h)
    *h = 1;
  return true;
}

const char *DdpMonochromaticLightEffect::get_name() const {
  // Access base class name_ member directly (protected)
  // Old ESPHome: name_ is std::string, call .c_str()
  // New ESPHome (PR #11487): name_ is const char*, return directly
  return get_cstr(name_);
}

// -------- LightEffect interface --------

void DdpMonochromaticLightEffect::start() {
  LightEffect::start();

  // Note: Renderer registration happens at codegen time (in Python __init__.py)
  // so that DdpComponent::setup() has complete renderer info for mDNS

  ESP_LOGI(TAG, "Started DDP monochromatic effect '%s' for stream %u", get_name(), stream_id_);
}

void DdpMonochromaticLightEffect::stop() {
  ESP_LOGI(TAG, "Stopped DDP monochromatic effect '%s' for stream %u", get_name(), stream_id_);
  LightEffect::stop();
}

void DdpMonochromaticLightEffect::apply() {
  // MAIN THREAD CONTEXT - safe to call ESPHome APIs

  // Check if we have a frame ready
  if (!frame_ready_.exchange(false, std::memory_order_acquire)) {
    return;
  }

  // Get and reset the aggregated brightness
  uint8_t bri = max_brightness_.exchange(0, std::memory_order_relaxed);

  // Apply brightness to the light
  auto call = this->state_->turn_on();
  call.set_brightness_if_supported(bri / 255.0f);
  call.set_transition_length_if_supported(0);
  call.set_publish(false);
  call.set_save(false);
  call.perform();
}

}  // namespace esphome::ddp
