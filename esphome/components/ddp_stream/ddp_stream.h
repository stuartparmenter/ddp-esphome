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

#pragma pack(push, 1)
struct DdpHeader {
  uint8_t  flags;        // bit6=DDP(0x40), bit0=PUSH(0x01)
  uint8_t  seq;          // low 4 bits commonly used in DDP
  uint8_t  pixcfg;       // 0x2C for RGB888
  uint8_t  id;           // stream/output id
  uint32_t offset_be;    // big-endian byte offset (RGB888)
  uint16_t length_be;    // big-endian payload length
};
#pragma pack(pop)

class DdpStream : public Component {
 public:
  void set_port(uint16_t p) { port_ = p; }
  uint16_t get_port() const { return port_; }

  void add_stream_binding(uint8_t id,
                          std::function<lv_obj_t*()> getter,
                          int w /* = -1 */, int h /* = -1 */);

  void set_stream_canvas(uint8_t id, lv_obj_t* canvas, int w, int h);

  bool get_stream_size(uint8_t id, int *w, int *h) const;

  void setup() override;
  void dump_config() override;
  void loop() override;
  float get_setup_priority() const override { return setup_priority::LATE; }

 private:
  struct Binding {
    std::function<lv_obj_t*()> getter;
    lv_obj_t* canvas{nullptr};
    int w{-1}, h{-1};

    // Triple buffering (RGB565), allocated via lv_mem_alloc (PSRAM-aware)
    uint16_t* front_buf{nullptr};  // currently displayed (owned by the canvas)
    uint16_t* ready_buf{nullptr};  // next to present (filled on PUSH)
    uint16_t* accum_buf{nullptr};  // RX writes current frame here
    size_t    buf_px{0};           // buffer length in pixels (for all three)
    std::atomic<bool> have_ready{false}; // a frame is queued for present

#if DDP_STREAM_METRICS
    // Totals
    uint64_t rx_pkts{0};
    uint64_t rx_bytes{0};

    // Frame assembly
    size_t   frame_bytes_accum{0};
    size_t   frame_px_accum{0};
    int64_t  frame_first_pkt_us{0};
    bool     frame_seen_any{false};

    // Frame accounting
    uint32_t frames_started{0};
    uint32_t frames_push{0};
    uint32_t frames_ok{0};
    uint32_t frames_incomplete{0};
    uint32_t frames_presented{0};
    uint32_t frames_overrun{0};

    // Sequencing
    uint8_t  last_seq_push{0};
    bool     have_last_seq{false};
    uint32_t push_seq_misses{0};

    uint8_t  last_seq_pkt{0};
    bool     have_last_seq_pkt{false};
    uint32_t pkt_seq_gaps{0};

    // EWMA
    double   present_lat_us_ewma{0.0};
    double   coverage_ewma{0.0};

    // RX slice behavior
    uint32_t rx_wakeups{0};
    uint64_t rx_pkts_in_slices{0};

    // Windowed (2s) counters
    int64_t  log_t0_us{0};
    uint32_t win_frames_push{0};
    uint32_t win_frames_presented{0};
    uint64_t win_rx_bytes{0};
    uint32_t win_pkt_gap{0};
    uint32_t win_overrun{0};

    // Ready→present queue wait
    int64_t  ready_set_us{0};
    double   qwait_ms_ewma{0.0};
    double   build_ms_ewma{0.0};

    // NEW: max inter-packet gap within a frame (diagnostic)
    double   intra_ms_max{0.0};
    int64_t  last_pkt_us{0};
#endif

    bool bound{false};
  };

  void ensure_socket_();
  void open_socket_();
  void close_socket_();

  static void recv_task_trampoline(void* arg);
  void recv_task();

  void bind_if_possible_(Binding &b);
  void ensure_binding_buffers_(Binding &b);
  void free_ready_accum_(Binding &b);
  static void on_canvas_size_changed_(lv_event_t *e);

  Binding* find_binding_(uint8_t id);
  void handle_packet_(const uint8_t *buf, size_t len);

  uint16_t port_{4048};
  int sock_{-1};
  TaskHandle_t task_{nullptr};

  bool udp_opened_{false};
  std::map<uint8_t, Binding> bindings_;
};

}  // namespace ddp_stream
}  // namespace esphome
