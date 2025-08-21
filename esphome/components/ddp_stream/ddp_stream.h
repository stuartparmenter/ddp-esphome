// © Copyright 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT

#pragma once
#include "esphome/core/component.h"

extern "C" {
  #include "lvgl.h"
}

// STL
#include <map>
#include <vector>
#include <cstdint>
#include <atomic>
#include <functional>

// lwIP / sockets
#include <lwip/sockets.h>
#include <lwip/inet.h>
#include <lwip/netdb.h>
#include <errno.h>

// FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace esphome {
namespace ddp_stream {

// -------- DDP header (packed, big-endian fields) --------
#pragma pack(push, 1)
struct DdpHeader {
  uint8_t  flags;     // 0x40=data, bit0=PUSH
  uint8_t  seq;       // low 4 bits sequence
  uint8_t  pixcfg;    // e.g. 0x2C for RGB888 (per your sender)
  uint8_t  id;        // stream/output id
  uint32_t offset_be; // big-endian byte offset
  uint16_t length_be;    // big-endian payload length
};
#pragma pack(pop)

// Optional helpers if you need host-endian values in .cpp
static inline uint16_t be16toh_u16(uint16_t x) {
  return (uint16_t)(((x & 0x00FFu) << 8) | ((x & 0xFF00u) >> 8));
}
static inline uint32_t be32toh_u32(uint32_t x) {
  return ((x & 0x000000FFu) << 24) |
         ((x & 0x0000FF00u) <<  8) |
         ((x & 0x00FF0000u) >>  8) |
         ((x & 0xFF000000u) >> 24);
}

class DdpStream : public Component {
 public:
  // ---- Public configuration ----
  void set_port(uint16_t p) { port_ = p; }

  // Immediate bind (kept for compatibility; safe to call from on_load)
  void set_stream_canvas(uint8_t id, lv_obj_t* canvas, int w, int h);

  // Declarative/deferred bind (for YAML `streams:`)
  void add_stream_binding(uint8_t id,
                          std::function<lv_obj_t*()> getter,
                          int w, int h) {
    pending_.push_back({id, std::move(getter), w, h});
  }

  // ---- ESPHome lifecycle ----
  void setup() override;          // opens socket (via ensure_socket_) and schedules deferred binds
  void dump_config() override;
  void loop() override;           // RX handled in a task; declared (not inlined) to avoid ODR clash
  float get_setup_priority() const override {
    // Run late so LVGL can construct pages; we still defer binds below.
    return setup_priority::LATE;
  }

 private:
  // ------------------- DDP sink (per stream id) -------------------
  struct StreamSink {
    lv_obj_t* canvas{nullptr};
    int w{0}, h{0};
    std::vector<uint16_t> front;     // RGB565
    std::vector<uint16_t> back;      // RGB565
    std::atomic<bool> have_new{false};
    uint8_t last_seq{0};
  };

  // ------------------- Deferred binding -------------------
  struct Pending {
    uint8_t id;
    std::function<lv_obj_t*()> getter;   // returns &id(canvas)
    int w, h;
  };
  std::vector<Pending> pending_;
  void process_pending_();               // resolves getters once LVGL widgets exist

  // ------------------- Networking -------------------
  void ensure_socket_();                 // open/close socket as needed
  void open_socket_();                   // create socket, set options, spawn RX task
  void close_socket_();                  // stop RX task, close socket

  static void recv_task_trampoline(void* arg);
  void recv_task();                      // blocking UDP loop, decodes DDP, flips buffers

  // ------------------- Helpers -------------------
  static inline uint16_t rgb888_to_565(uint8_t r, uint8_t g, uint8_t b) {
    return (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
  }

  // ------------------- State -------------------
  uint16_t port_{4048};
  int sock_{-1};
  TaskHandle_t task_{nullptr};

  // swap helper if LV_COLOR_16_SWAP is set (used during blit)
  std::vector<uint16_t> tmp_;

  std::map<uint8_t, StreamSink> sinks_;
};

}  // namespace ddp_stream
}  // namespace esphome
