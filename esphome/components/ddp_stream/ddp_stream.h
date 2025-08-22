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
  uint8_t  flags;        // 0x40=data, bit0=PUSH
  uint8_t  seq;          // low 4 bits sequence
  uint8_t  pixcfg;       // e.g. 0x2C for RGB888
  uint8_t  id;           // stream/output id
  uint32_t offset_be;    // big-endian byte offset
  uint16_t length_be;    // big-endian payload length
};
#pragma pack(pop)

class DdpStream : public Component {
 public:
  // ---- Public configuration ----
  void set_port(uint16_t p) { port_ = p; }
  uint16_t get_port() const { return port_; }

  // Bind canvas by getter; if w/h == -1 we auto-resolve from LVGL when available.
  void add_stream_binding(uint8_t id,
                          std::function<lv_obj_t*()> getter,
                          int w /* = -1 */, int h /* = -1 */);

  // Immediate bind (rarely needed; safe to call from on_load)
  void set_stream_canvas(uint8_t id, lv_obj_t* canvas, int w, int h);

  // Query the resolved size (returns false if canvas not ready yet).
  bool get_stream_size(uint8_t id, int *w, int *h) const;

  // ---- ESPHome lifecycle ----
  void setup() override;          // opens socket (via ensure_socket_) and resolves bindings
  void dump_config() override;
  void loop() override;           // flips front/back and blits to LVGL
  float get_setup_priority() const override {
    // Run late so LVGL can construct pages; we still resolve canvases/sizes in intervals.
    return setup_priority::LATE;
  }

 private:
  // ------------------- Binding (per output id) -------------------
  struct Binding {
    std::function<lv_obj_t*()> getter;   // returns &id(canvas)
    lv_obj_t* canvas{nullptr};           // cached when available
    int w{-1}, h{-1};                    // -1 => auto from LVGL object
    std::vector<uint16_t> front;         // RGB565
    std::vector<uint16_t> back;          // RGB565
    std::atomic<bool> have_new{false};
    uint8_t last_seq{0};
    bool ready{false};                   // true once canvas, size, and buffers are set
  };

  // ------------------- Networking -------------------
  void ensure_socket_();                 // open/close socket as needed
  void open_socket_();                   // create socket, set options, spawn RX task
  void close_socket_();                  // stop RX task, close socket

  static void recv_task_trampoline(void* arg);
  void recv_task();                      // blocking UDP loop, decodes DDP, writes into back

  // ------------------- Binding helpers -------------------
  void bind_if_possible_(Binding &b);    // attempt full bind
  void ensure_binding_buffers_(Binding &b);
  static void on_canvas_size_changed_(lv_event_t *e);

  Binding* find_binding_(uint8_t id);    // nullptr if not found

  void handle_packet_(const uint8_t *buf, size_t len); // decoder

  // ------------------- State -------------------
  uint16_t port_{4048};
  int sock_{-1};
  TaskHandle_t task_{nullptr};

  bool udp_opened_{false};
  std::vector<uint16_t> tmp_;
  std::map<uint8_t, Binding> bindings_;
};

}  // namespace ddp_stream
}  // namespace esphome
