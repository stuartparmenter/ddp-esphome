// © Copyright 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT

#pragma once

#include "esphome/core/component.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/ddp/ddp.h"
#include "esphome/components/ddp/ddp_renderer.h"

extern "C" {
  #include "lvgl.h"
}

#include <functional>
#include <atomic>

namespace esphome {
namespace ddp {

// DDP renderer that outputs to LVGL canvas with triple-buffering
class DdpCanvas : public Component, public DdpRenderer {
 public:
  // Configuration
  void set_parent(DdpComponent* parent) { parent_ = parent; }
  void set_stream_id(uint8_t id) { stream_id_ = id; }
  void set_canvas_getter(std::function<lv_obj_t*()> getter) { getter_ = std::move(getter); }
  void set_size(int w, int h) { w_ = w; h_ = h; }
  void set_back_buffers(uint8_t n) { back_buffers_ = (n > 2 ? 2 : n); }
  void set_receiving_sensor(binary_sensor::BinarySensor* sensor) { receiving_sensor_ = sensor; }

  // DdpRenderer interface (UDP TASK CONTEXT)
  void on_data(size_t offset_px, const uint8_t* pixels,
               PixelFormat format, size_t pixel_count) override;
  void on_push() override;
  uint8_t get_stream_id() const override { return stream_id_; }
  bool get_dimensions(int* w, int* h) const override;
  const char* get_name() const override { return "DdpCanvas"; }
#ifdef DDP_METRICS
  const RendererMetrics* get_metrics() const override { return &metrics_; }
  void reset_windowed_metrics() override;
#endif

  // ESPHome lifecycle
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::LATE; }

 protected:
  // Canvas binding helpers
  void bind_if_possible_();
  void ensure_buffers_();
  void free_ready_accum_();
  static void on_canvas_size_changed_(lv_event_t* e);

  DdpComponent* parent_{nullptr};
  uint8_t stream_id_{0};

  // LVGL canvas
  std::function<lv_obj_t*()> getter_;
  lv_obj_t* canvas_{nullptr};
  int w_{-1}, h_{-1};
  uint8_t back_buffers_{2};  // Default: 2 (triple-buffer)

  // Triple-buffering for smooth rendering
  // All buffers use LVGL's native color format (LV_COLOR_DEPTH=16 or 32)
  uint16_t* front_buf_{nullptr};  // Currently displayed (owned by canvas)
  uint16_t* ready_buf_{nullptr};  // Next to present (filled on PUSH)
  uint16_t* accum_buf_{nullptr};  // RX writes here
  size_t buf_px_{0};
  std::atomic<bool> have_ready_{false};
  std::atomic<bool> need_copy_to_front_{false};
  std::atomic<bool> need_invalidate_{false};
  std::atomic<bool> need_sensor_update_{false};  // Deferred sensor update from UDP task

  bool bound_{false};

  // Optional receiving sensor
  binary_sensor::BinarySensor* receiving_sensor_{nullptr};
  bool last_receiving_state_{false};
  std::atomic<int64_t> last_frame_us_{0};  // Track last frame time for timeout

#ifdef DDP_METRICS
  // Performance metrics
  RendererMetrics metrics_;

  // Frame assembly tracking (for coverage calculation)
  std::atomic<size_t> frame_bytes_accum_{0};
  std::atomic<int64_t> frame_first_pkt_us_{0};
  std::atomic<int64_t> ready_set_us_{0};  // When frame became ready (for queue wait)
#endif
};

}  // namespace ddp
}  // namespace esphome
