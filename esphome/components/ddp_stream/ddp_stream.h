// © Copyright 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT

#pragma once
#include "esphome/core/component.h"

extern "C" {
  #include "lvgl.h"
}

#include <map>
#include <vector>
#include <cstdint>
#include <atomic>
#include <functional>

#include <lwip/sockets.h>
#include <lwip/inet.h>
#include <lwip/netdb.h>
#include <errno.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifndef DDP_STREAM_METRICS
#define DDP_STREAM_METRICS 1
#endif

namespace esphome {
namespace ddp_stream {

// Forward declaration
class DdpStream;

class DdpStreamOutput : public Component {
 public:
  void set_ddp_stream_id(uint8_t ddp_stream_id) { ddp_stream_id_ = ddp_stream_id; }
  uint8_t get_ddp_stream_id() const { return ddp_stream_id_; }

  void set_parent(DdpStream* parent) { parent_ = parent; }

  // Stream-specific configuration
  void set_canvas_getter(std::function<lv_obj_t*()> getter) { getter_ = std::move(getter); }
  void set_size(int w, int h) { w_ = w; h_ = h; }
  void set_back_buffers(uint8_t n) { back_buffers_ = (n > 2 ? 2 : n); }

  // Stream information
  bool get_size(int *w, int *h) const;

  // ESPHome lifecycle
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::LATE; }

 protected:
  uint8_t ddp_stream_id_{0};
  DdpStream* parent_{nullptr};

  // Stream configuration (from old Binding struct)
  std::function<lv_obj_t*()> getter_;
  lv_obj_t* canvas_{nullptr};
  int w_{-1}, h_{-1};
  uint8_t back_buffers_{2};     // 0=none, 1=double, 2=triple (default)

  // Triple buffering (RGB565), allocated via lv_mem_alloc (PSRAM-aware)
  uint16_t* front_buf_{nullptr};  // currently displayed (owned by the canvas)
  uint16_t* ready_buf_{nullptr};  // next to present (filled on PUSH)
  uint16_t* accum_buf_{nullptr};  // RX writes current frame here
  size_t    buf_px_{0};           // buffer length in pixels (for all three)
  std::atomic<bool> have_ready_{false}; // a frame is queued for present
  std::atomic<bool> need_copy_to_front_{false}; // for back_buffers==1
  std::atomic<bool> need_invalidate_{false};    // for back_buffers==0/1

#if DDP_STREAM_METRICS
  // Totals
  uint64_t rx_pkts_{0};
  uint64_t rx_bytes_{0};         // DDP pixel payload bytes (header excluded)
  uint64_t rx_wire_bytes_{0};    // UDP payload bytes (header included)

  // Frame assembly
  size_t   frame_bytes_accum_{0};
  size_t   frame_px_accum_{0};
  int64_t  frame_first_pkt_us_{0};
  bool     frame_seen_any_{false};

  // Frame accounting
  uint32_t frames_started_{0};
  uint32_t frames_push_{0};
  uint32_t frames_ok_{0};
  uint32_t frames_incomplete_{0};
  uint32_t frames_presented_{0};
  uint32_t frames_overrun_{0};

  // Sequencing
  uint8_t  last_seq_push_{0};
  bool     have_last_seq_{false};
  uint32_t push_seq_misses_{0};

  uint8_t  last_seq_pkt_{0};
  bool     have_last_seq_pkt_{false};
  uint32_t pkt_seq_gaps_{0};

  // EWMA
  double   present_lat_us_ewma_{0.0};
  double   coverage_ewma_{0.0};

  // RX slice behavior
  uint32_t rx_wakeups_{0};
  uint64_t rx_pkts_in_slices_{0};

  // Windowed (2s) counters
  int64_t  log_t0_us_{0};
  uint32_t win_frames_push_{0};
  uint32_t win_frames_presented_{0};
  uint64_t win_rx_bytes_{0};        // payload only
  uint64_t win_rx_wire_bytes_{0};   // UDP payload (wire)
  uint32_t win_pkt_gap_{0};
  uint32_t win_overrun_{0};

  // Ready→present queue wait
  int64_t  ready_set_us_{0};
  double   qwait_ms_ewma_{0.0};
  double   build_ms_ewma_{0.0};

  // When we enqueue a present, remember the frame's first packet ts for present-lat calc.
  int64_t  present_first_pkt_us_{0};

  // max inter-packet gap within a frame
  double   intra_ms_max_{0.0};
  int64_t  last_pkt_us_{0};
#endif

  bool bound_{false};

  friend class DdpStream;
};

#pragma pack(push, 1)
struct DdpHeader {
  uint8_t  flags;        // bit6=DDP(0x40), bit0=PUSH(0x01)
  uint8_t  seq;          // low 4 bits commonly used in DDP
  uint8_t  pixcfg;       // 0x2C = RGB888; EXT: 0x61 = RGB565(BE), 0x62 = RGB565(LE)
  uint8_t  id;           // stream/output id
  uint32_t offset_be;    // big-endian byte offset (RGB888)
  uint16_t length_be;    // big-endian payload length
};
#pragma pack(pop)

// Pixel config constants (DDP 'cfg' byte)
static constexpr uint8_t DDP_PIXCFG_RGB888     = 0x2C;
// Custom extension values to indicate 16-bpp transport:
static constexpr uint8_t DDP_PIXCFG_RGB565_BE  = 0x61;
static constexpr uint8_t DDP_PIXCFG_RGB565_LE  = 0x62;

class DdpStream : public Component {
 public:
  void set_port(uint16_t p) { port_ = p; }
  uint16_t get_port() const { return port_; }
  void set_default_back_buffers(uint8_t n) { default_back_buffers_ = (n > 2 ? 2 : n); }

  void register_stream(DdpStreamOutput* stream, uint8_t ddp_stream_id);

  void add_stream_binding(uint8_t id,
                          std::function<lv_obj_t*()> getter,
                          int w /* = -1 */, int h /* = -1 */);

  void set_stream_canvas(uint8_t id, lv_obj_t* canvas, int w, int h);
  void set_stream_back_buffers(uint8_t id, uint8_t n);
  void set_back_buffers_for(uint8_t id, uint8_t n) { set_stream_back_buffers(id, n); }

  bool get_stream_size(uint8_t id, int *w, int *h) const;
  uint8_t get_stream_id(uint8_t id) const;

  void setup() override;
  void dump_config() override;
  void loop() override;
  float get_setup_priority() const override { return setup_priority::AFTER_CONNECTION; }

 private:

  void ensure_socket_();
  void open_socket_();
  void close_socket_();

  static void recv_task_trampoline(void* arg);
  void recv_task();

  void bind_if_possible_(DdpStreamOutput &stream);
  void ensure_binding_buffers_(DdpStreamOutput &stream);
  void free_ready_accum_(DdpStreamOutput &stream);
  static void on_canvas_size_changed_(lv_event_t *e);

  DdpStreamOutput* find_stream_(uint8_t id);

  // Packet dispatcher + helpers
  void handle_packet_(const uint8_t *buf, size_t len);
  void handle_push_(DdpStreamOutput &stream, const DdpHeader* h);
  void handle_rgb888_(DdpStreamOutput &stream, const DdpHeader* h, const uint8_t* payload, size_t len);
  void handle_rgb565_(DdpStreamOutput &stream, const DdpHeader* h, const uint8_t* payload, size_t len);

  uint16_t port_{4048};
  int sock_{-1};
  TaskHandle_t task_{nullptr};
  std::atomic<bool> task_should_exit_{false};
  std::atomic<bool> task_stopping_{false};
  uint8_t default_back_buffers_{2};
  uint8_t  preferred_fmt_{0};  // 0=RGB888 (default), 1=RGB565

  bool udp_opened_{false};
  std::map<uint8_t, DdpStreamOutput*> streams_;

  // Loop optimization: disable when idle
  std::atomic<bool> loop_is_disabled_{false};
  int64_t last_activity_us_{0};
  static constexpr int64_t IDLE_TIMEOUT_US = 1'000'000;  // 1 second
};

}  // namespace ddp_stream
}  // namespace esphome
