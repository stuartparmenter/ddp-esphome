// © Copyright 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT

#include "ddp_stream.h"
#include "esphome/core/log.h"
#include "esphome/components/network/util.h"  // network::is_connected()
#include <algorithm>  // std::min
#include <cstdio>

namespace esphome {
namespace ddp_stream {

static const char* TAG = "ddp_stream";

// -------- public API --------

void DdpStream::add_stream_binding(uint8_t id,
                                   std::function<lv_obj_t*()> getter,
                                   int w, int h) {
  Binding &dst = bindings_[id];  // default-constructed Binding
  dst.getter = std::move(getter);
  dst.w = w;
  dst.h = h;
  this->bind_if_possible_(dst);
}

void DdpStream::set_stream_canvas(uint8_t id, lv_obj_t* canvas, int w, int h) {
  Binding &dst = bindings_[id];
  dst.getter = [canvas]() -> lv_obj_t* { return canvas; };
  dst.canvas = canvas;
  dst.w = w;
  dst.h = h;
  this->bind_if_possible_(dst);
}

bool DdpStream::get_stream_size(uint8_t id, int *w, int *h) const {
  auto it = bindings_.find(id);
  if (it == bindings_.end()) return false;
  const Binding &b = it->second;
  if (b.w <= 0 || b.h <= 0) return false;
  if (w) *w = b.w;
  if (h) *h = b.h;
  return true;
}

// -------- lifecycle --------

void DdpStream::setup() {
  ESP_LOGI(TAG, "setup this=%p port=%u", this, (unsigned) port_);

  // Keep trying to open/close the socket with network state (run indefinitely)
  this->set_interval("ddp_net", 500, [this]() { this->ensure_socket_(); });

  // Resolve canvases/sizes until all bindings are ready, then stop this interval
  this->set_interval("ddp_bind", 250, [this]() {
    bool all_ready = true;
    for (auto &kv : bindings_) {
      Binding &b = kv.second;
      bool before = b.ready;
      this->bind_if_possible_(b);
      bool after = b.ready;
      if (!before && after) {
        ESP_LOGI(TAG, "stream %u bound to canvas=%p (%dx%d)",
                 (unsigned) kv.first, (void*) b.canvas, b.w, b.h);
        lv_canvas_fill_bg(b.canvas, lv_color_black(), LV_OPA_COVER);
      }
      all_ready = all_ready && after;
    }
    if (all_ready) {
      ESP_LOGI(TAG, "all DDP bindings resolved; stopping binding resolver");
      this->cancel_interval("ddp_bind");
    }
  });

  ESP_LOGI(TAG, "initialized; waiting for network to open UDP %u", this->port_);
}

void DdpStream::dump_config() {
  ESP_LOGCONFIG(TAG, "DDP stream:");
  ESP_LOGCONFIG(TAG, "  UDP port: %u", port_);
  for (auto &kv : bindings_) {
    const Binding &b = kv.second;
    ESP_LOGCONFIG(TAG, "  stream id %u -> %dx%d canvas=%p (buf=%zu)",
                  kv.first, b.w, b.h, (void*) b.canvas, b.front.size());
  }
}

void DdpStream::loop() {
  // Present any completed frames on LVGL thread
  for (auto &kv : bindings_) {
    Binding &b = kv.second;

    // quick reject: nothing new
    if (!b.have_new.exchange(false))
      continue;

    // must have a canvas and resolved size
    if (!b.canvas || b.w <= 0 || b.h <= 0)
      continue;

    // flip buffers; decoder already wrote in LVGL's byte order
    b.front.swap(b.back);

    const int w = b.w;
    const int h = b.h;

    // copy to canvas and invalidate just the updated area
    lv_canvas_copy_buf(b.canvas, b.front.data(), 0, 0, w, h);

    lv_area_t a;
    a.x1 = 0; a.y1 = 0;
    a.x2 = w - 1; a.y2 = h - 1;
    lv_obj_invalidate_area(b.canvas, &a);
  }
}

// -------- socket/RX --------

void DdpStream::ensure_socket_() {
  bool net_up = network::is_connected();
  if (net_up) {
    if (sock_ < 0) open_socket_();
  } else {
    if (sock_ >= 0) close_socket_();
  }
}

void DdpStream::open_socket_() {
  if (sock_ >= 0) return;

  // Create UDP socket
  int s = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (s < 0) {
    ESP_LOGW(TAG, "socket() failed errno=%d", errno);
    return;
  }

  // Allow quick rebinds after restart
  int yes = 1;
  (void) setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

  // Bind to INADDR_ANY:port_
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_port = htons(port_);
  if (bind(s, (sockaddr*)&addr, sizeof(addr)) < 0) {
    ESP_LOGW(TAG, "bind() failed errno=%d", errno);
    ::close(s);
    return;
  }

  sock_ = s;

  if (!udp_opened_) {
    ESP_LOGI(TAG, "DDP listening on UDP %u", port_);
    udp_opened_ = true;
  }

  // Start RX task
  if (task_ == nullptr) {
    xTaskCreatePinnedToCore(&DdpStream::recv_task_trampoline, "ddp_rx",
                            4096, this, 5, &task_, tskNO_AFFINITY);
  }
}


void DdpStream::close_socket_() {
  if (task_ != nullptr) {
    TaskHandle_t t = task_;
    task_ = nullptr;
    if (sock_ >= 0) {
      ::shutdown(sock_, SHUT_RDWR);
      ::close(sock_);
      sock_ = -1;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
    vTaskDelete(t);
  } else if (sock_ >= 0) {
    ::shutdown(sock_, SHUT_RDWR);
    ::close(sock_);
    sock_ = -1;
  }
  ESP_LOGI(TAG, "DDP socket closed");
}

void DdpStream::recv_task_trampoline(void* arg) {
  reinterpret_cast<DdpStream*>(arg)->recv_task();
}

void DdpStream::recv_task() {
  std::vector<uint8_t> buf(2048);
  while (true) {
    if (task_ == nullptr) break;
    int s = sock_;
    if (s < 0) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }
    ssize_t n = ::recv(s, buf.data(), buf.size(), 0);
    if (n <= 0) {
      vTaskDelay(pdMS_TO_TICKS(1));
      continue;
    }
    this->handle_packet_(buf.data(), (size_t) n);
  }
}

// -------- binding helpers --------

void DdpStream::bind_if_possible_(Binding &b) {
  if (b.ready) return;
  if (!b.canvas && b.getter) b.canvas = b.getter();
  if (!b.canvas) return;
  if (b.w <= 0 || b.h <= 0) {
    int W = (int) lv_obj_get_width(b.canvas);
    int H = (int) lv_obj_get_height(b.canvas);
    if (b.w <= 0) b.w = W;
    if (b.h <= 0) b.h = H;
  }
  if (b.w <= 0 || b.h <= 0) return;
  ensure_binding_buffers_(b);
  if (b.front.empty() || b.back.empty()) return;
  lv_obj_add_event_cb(b.canvas, &DdpStream::on_canvas_size_changed_, LV_EVENT_SIZE_CHANGED, this);
  b.ready = true;
}

void DdpStream::ensure_binding_buffers_(Binding &b) {
  if (b.w <= 0 || b.h <= 0) return;
  const size_t px = (size_t) b.w * (size_t) b.h;
  if (b.front.size() != px) b.front.assign(px, 0);
  if (b.back.size()  != px) b.back.assign(px, 0);
}

DdpStream::Binding* DdpStream::find_binding_(uint8_t id) {
  auto it = bindings_.find(id);
  if (it == bindings_.end()) return nullptr;
  return &it->second;
}

void DdpStream::on_canvas_size_changed_(lv_event_t *e) {
  auto *canvas = lv_event_get_target(e);
  auto *self   = static_cast<DdpStream*>(lv_event_get_user_data(e));
  if (!self || !canvas) return;
  for (auto &kv : self->bindings_) {
    Binding &b = kv.second;
    if (b.canvas != canvas) continue;
    bool auto_any = (b.w <= 0 || b.h <= 0);
    if (!auto_any) return;
    int W = (int) lv_obj_get_width(canvas);
    int H = (int) lv_obj_get_height(canvas);
    if (b.w <= 0) b.w = W;
    if (b.h <= 0) b.h = H;
    self->ensure_binding_buffers_(b);
    break;
  }
}

// -------- DDP decoder --------

static inline uint16_t rgb888_to_565(uint8_t r, uint8_t g, uint8_t b) {
  return (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
}

void DdpStream::handle_packet_(const uint8_t *raw, size_t n) {
  // A DDP packet = 1 header + 0..1440 payload bytes.
  // Multiple packets make up a full frame (RGB888).

  if (n < sizeof(DdpHeader)) return;           // too short to even hold a header
  auto *h = reinterpret_cast<const DdpHeader*>(raw);

  // --- 1) Basic header checks ---
  bool ver  = (h->flags & 0x40) != 0;          // bit 6 must be set for "DDP data"
  bool push = (h->flags & 0x01) != 0;          // bit 0 = "PUSH": last packet of frame
  if (!ver) return;                            // ignore non-DDP data packets

  uint8_t id = h->id;                          // stream/output id (0–255)
  Binding *b = this->find_binding_(id);
  if (!b || !b->canvas || b->w <= 0 || b->h <= 0) return;  // not bound yet

  // --- 2) Offset/length ---
  uint32_t offset = ntohl(h->offset_be);       // byte offset in the frame
  uint16_t len    = ntohs(h->length_be);       // payload length in bytes
  if (len == 0) return;
  if (sizeof(DdpHeader) + len > n) return;     // truncated packet

  const uint8_t* p = raw + sizeof(DdpHeader);  // pointer to pixel data

  // --- 3) Validate alignment ---
  // This implementation expects RGB888 (3 bytes per pixel).
  if ((offset % 3) != 0 || (len % 3) != 0) return;

  size_t pixel_off = offset / 3;               // starting pixel index
  size_t px        = len / 3;                  // number of pixels in this packet
  size_t max_px    = (size_t) b->w * (size_t) b->h;
  if (pixel_off >= max_px) return;             // starts outside canvas
  size_t write_px  = std::min(px, max_px - pixel_off);

  // --- 4) Ensure buffers are sized ---
  this->ensure_binding_buffers_(*b);
  if (b->back.empty()) return;

  // --- 5) Convert RGB888 -> RGB565 (with optional LVGL byte swap) ---
  uint16_t* dst = b->back.data() + pixel_off;
  const uint8_t* sp = p;
  for (size_t i = 0; i < write_px; ++i) {
    const uint16_t c = rgb888_to_565(sp[0], sp[1], sp[2]);  // 24-bit → 16-bit
#if defined(LV_COLOR_16_SWAP) && LV_COLOR_16_SWAP
    dst[i] = (uint16_t)((c >> 8) | (c << 8));           // swap bytes if LVGL configured so
#else
    dst[i] = c;
#endif
    sp += 3;
  }

  // --- 6) If PUSH flag set, mark frame as ready for display ---
  if (push) {
    b->last_seq = h->seq;                     // sequence number from header
    b->have_new.store(true);                  // signal loop() to flip buffers & blit
  }
}


} // namespace ddp_stream
} // namespace esphome
