// © Copyright 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT

#pragma once

#include "esphome/core/component.h"
#include "esphome/components/light/light_effect.h"
#include "ddp.h"
#include "ddp_renderer.h"
#include "ddp_pixel_convert.h"

#include <atomic>

namespace esphome::ddp {

// DDP renderer that outputs to monochromatic/RGB lights (non-addressable)
// Aggregates all received pixels to a single brightness value
class DdpMonochromaticLightEffect : public light::LightEffect, public DdpRenderer {
 public:
  explicit DdpMonochromaticLightEffect(const char *name) : LightEffect(name) {}

  // Configuration
  void set_parent(DdpComponent *parent) { parent_ = parent; }
  void set_stream_id(uint8_t id) { stream_id_ = id; }
  void set_brightness_method(BrightnessMethod method) { brightness_method_ = method; }

  // DdpRenderer interface (UDP TASK CONTEXT)
  void on_data(size_t offset_px, const uint8_t *pixels, PixelFormat format, size_t pixel_count) override;
  void on_push() override;
  uint8_t get_stream_id() const override { return stream_id_; }
  bool get_dimensions(int *w, int *h) const override;
  const char *get_name() const override;

  // LightEffect interface (MAIN THREAD)
  void start() override;
  void stop() override;
  void apply() override;

 protected:
  DdpComponent *parent_{nullptr};
  uint8_t stream_id_{0};
  BrightnessMethod brightness_method_{BrightnessMethod::LUMINANCE};

  // Aggregated brightness from all pixels in current frame
  std::atomic<uint8_t> max_brightness_{0};
  std::atomic<bool> frame_ready_{false};
};

}  // namespace esphome::ddp
