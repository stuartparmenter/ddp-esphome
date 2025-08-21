// © Copyright 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT

#include "ddp_stream.h"
#include "esphome/core/log.h"
#include "esphome/components/network/util.h"  // network::is_connected()

namespace esphome {
namespace ddp_stream {

static const char* TAG = "ddp_stream";

void DdpStream::setup() {
  // Don’t touch lwIP yet. Just schedule periodic checks to (re)open when ready.
  this->set_interval(500, [this]() { this->ensure_socket_(); });
  ESP_LOGI(TAG, "initialized; waiting for network to open UDP %u", this->port_);
  this->set_timeout(0, [this]() { this->process_pending_(); });
}

void DdpStream::dump_config() {
  ESP_LOGCONFIG(TAG, "DDP stream:");
  ESP_LOGCONFIG(TAG, "  UDP port: %u", port_);
  for (auto &kv : sinks_) {
    ESP_LOGCONFIG(TAG, "  stream id %u -> %dx%d canvas=%p",
                  kv.first, kv.second.w, kv.second.h, (void*)kv.second.canvas);
  }
}

void DdpStream::process_pending_() {
  std::vector<Pending> retry;
  for (auto &p : pending_) {
    lv_obj_t* obj = p.getter ? p.getter() : nullptr;
    if (obj) {
      ESP_LOGD(TAG, "binding stream id=%u to canvas=%p (%dx%d)", p.id, (void*)obj, p.w, p.h);
      this->set_stream_canvas(p.id, obj, p.w, p.h);
    } else {
      retry.push_back(std::move(p));
    }
  }
  pending_.swap(retry);
  if (!pending_.empty()) {
    // Try again shortly until pages/widgets exist
    this->set_timeout(50, [this]() { this->process_pending_(); });
  }
}

// Bind/resize a sink and (re)allocate frame buffers.
void DdpStream::set_stream_canvas(uint8_t id, lv_obj_t* canvas, int w, int h) {
  StreamSink &s = sinks_[id];

  s.canvas = canvas;
  s.w = w;
  s.h = h;

  // (Re)allocate front/back RGB565 buffers for this stream
  const size_t px = static_cast<size_t>(w) * static_cast<size_t>(h);
  s.front.assign(px, 0);
  s.back.assign(px, 0);

  s.have_new.store(false);
  s.last_seq = 0;

  ESP_LOGI(TAG, "stream %u bound to canvas=%p (%dx%d)", id, (void*)canvas, w, h);

  // Optional: clear the canvas once on bind (uncomment if you want a blank frame immediately)
  if (canvas) {
    lv_canvas_fill_bg(canvas, lv_color_black(), LV_OPA_COVER);
  }
}

void DdpStream::loop() {
  // Present any completed frames on LVGL thread
  for (auto &kv : sinks_) {
    StreamSink &s = kv.second;
    if (s.have_new.exchange(false)) {
      s.front.swap(s.back);
      if (!s.canvas) continue;
      int cw = (int) lv_obj_get_width(s.canvas);
      int ch = (int) lv_obj_get_height(s.canvas);
      int w = std::min(cw, s.w), h = std::min(ch, s.h);
      if (w <= 0 || h <= 0) continue;

#if defined(LV_COLOR_16_SWAP) && LV_COLOR_16_SWAP
      tmp_.resize((size_t)w * h);
      for (int i = 0; i < w*h; ++i) {
        uint16_t c = s.front[i];
        tmp_[i] = (uint16_t)((c >> 8) | (c << 8));
      }
      lv_canvas_copy_buf(s.canvas, tmp_.data(), 0, 0, w, h);
#else
      lv_canvas_copy_buf(s.canvas, s.front.data(), 0, 0, w, h);
#endif
      lv_obj_invalidate(s.canvas);
    }
  }
}

void DdpStream::ensure_socket_() {
  // Open when connected; close when disconnected.
  bool net_up = network::is_connected();
  if (net_up) {
    if (sock_ < 0) open_socket_();
  } else {
    if (sock_ >= 0) close_socket_();
  }
}

void DdpStream::open_socket_() {
  if (sock_ >= 0) return;

  int s = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (s < 0) {
    ESP_LOGW(TAG, "socket() failed errno=%d", errno);
    return;
  }

  // Optional: allow rebinding quickly after link flap
  int yes = 1;
  setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

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

  // Spawn RX task now that socket is ready
  if (task_ == nullptr) {
    xTaskCreatePinnedToCore(&DdpStream::recv_task_trampoline, "ddp_rx",
                            4096, this, 5, &task_, tskNO_AFFINITY);
  }
  ESP_LOGI(TAG, "DDP listening on UDP %u", port_);
}

void DdpStream::close_socket_() {
  // Stop RX task first to avoid using a closing socket
  if (task_ != nullptr) {
    TaskHandle_t t = task_;
    task_ = nullptr;
    // Closing the socket will unblock recv(); then the task will return.
    // vTaskDelete as a fallback if it lingers.
    // (We’ll close socket before deleting task to wake it.)
    if (sock_ >= 0) {
      ::shutdown(sock_, SHUT_RDWR);
      ::close(sock_);
      sock_ = -1;
    }
    // Give RX task a moment to exit
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
    // If task_ was cleared (close requested), exit.
    if (task_ == nullptr) {
      break;
    }
    int s = sock_;
    if (s < 0) {
      // Socket closed underneath us — yield and exit (close will delete task)
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    ssize_t n = ::recv(s, buf.data(), buf.size(), 0);
    if (n <= 0) {
      // EAGAIN/EINTR or socket closed; yield a bit to avoid tight loop
      vTaskDelay(pdMS_TO_TICKS(1));
      continue;
    }

    if ((size_t)n < sizeof(DdpHeader)) continue;
    auto *h = reinterpret_cast<DdpHeader*>(buf.data());
    bool ver = (h->flags & 0x40) != 0;
    bool push = (h->flags & 0x01) != 0;
    if (!ver) continue;

    uint8_t id = h->id;
    auto it = sinks_.find(id);
    if (it == sinks_.end()) continue;

    StreamSink &sinc = it->second;
    if (!sinc.canvas || sinc.w == 0 || sinc.h == 0) continue;

    uint32_t offset = ntohl(h->offset_be);
    uint16_t len = ntohs(h->length_be);
    if (len == 0) continue;
    if (sizeof(DdpHeader) + len > (size_t)n) continue;

    const uint8_t* p = buf.data() + sizeof(DdpHeader);
    // RGB888 expected; offset & len in bytes
    if ((offset % 3) != 0 || (len % 3) != 0) continue;
    size_t pixel_off = offset / 3;
    size_t px = len / 3;
    size_t max_px = (size_t)sinc.w * sinc.h;
    if (pixel_off >= max_px) continue;
    size_t write_px = std::min(px, max_px - pixel_off);

    uint16_t* dst = sinc.back.data() + pixel_off;
    for (size_t i = 0; i < write_px; ++i) {
      uint8_t r = p[3*i+0], g = p[3*i+1], b = p[3*i+2];
      dst[i] = rgb888_to_565(r, g, b);
    }

    if (push) {
      sinc.last_seq = h->seq;
      sinc.have_new.store(true);
    }
  } // while
}

} // namespace ddp_stream
} // namespace esphome
