// © Copyright 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT

#include "ddp_stream.h"
#include "esphome/core/log.h"
#include "esphome/components/network/util.h"
#include <algorithm>
#include <cstdio>
#include <cstring>
#include "esp_timer.h"
#include <sys/time.h>

namespace esphome {
namespace ddp_stream {

static const char* TAG = "ddp_stream";

// DdpStreamOutput implementation
void DdpStreamOutput::setup() {
  if (parent_) {
    parent_->register_stream(this, ddp_stream_id_);
  }
}

void DdpStreamOutput::dump_config() {
  ESP_LOGCONFIG(TAG, "DdpStreamOutput: ddp_stream_id=%u", ddp_stream_id_);
}

bool DdpStreamOutput::get_size(int *w, int *h) const {
  if (w_ <= 0 || h_ <= 0) return false;
  if (w) *w = w_;
  if (h) *h = h_;
  return true;
}

static_assert(LV_COLOR_DEPTH == 16 || LV_COLOR_DEPTH == 32, "LV_COLOR_DEPTH must be 16 or 32");
static constexpr size_t BYTES_PER_PIXEL = LV_COLOR_DEPTH / 8;

#ifndef DDP_STREAM_METRICS
#define DDP_STREAM_METRICS 1
#endif

// ---------- helpers ----------

static uint16_t LUT_R5[256];
static uint16_t LUT_G6[256];
static uint16_t LUT_B5[256];
static bool     LUT_INITD = false;

static inline void init_rgb_luts_once() {
  if (LUT_INITD) return;
  for (int i = 0; i < 256; ++i) {
    LUT_R5[i] = (uint16_t)((i & 0xF8) << 8);
    LUT_G6[i] = (uint16_t)((i & 0xFC) << 3);
    LUT_B5[i] = (uint16_t)( i >> 3);
  }
  LUT_INITD = true;
}

#if DDP_STREAM_METRICS
static inline double ewma(double prev, double sample, double alpha = 0.2) {
  return (prev == 0.0) ? sample : (alpha * sample + (1.0 - alpha) * prev);
}
#endif

static const char* img_cf_name(uint8_t cf) {
  switch (cf) {
    case LV_IMG_CF_TRUE_COLOR:                  return "TRUE_COLOR";
    case LV_IMG_CF_TRUE_COLOR_CHROMA_KEYED:     return "TRUE_COLOR_CK";
    case LV_IMG_CF_TRUE_COLOR_ALPHA:            return "TRUE_COLOR_ALPHA";
    case LV_IMG_CF_INDEXED_1BIT:                return "INDEXED_1BIT";
    case LV_IMG_CF_INDEXED_2BIT:                return "INDEXED_2BIT";
    case LV_IMG_CF_INDEXED_4BIT:                return "INDEXED_4BIT";
    case LV_IMG_CF_INDEXED_8BIT:                return "INDEXED_8BIT";
    case LV_IMG_CF_ALPHA_1BIT:                  return "ALPHA_1BIT";
    case LV_IMG_CF_ALPHA_2BIT:                  return "ALPHA_2BIT";
    case LV_IMG_CF_ALPHA_4BIT:                  return "ALPHA_4BIT";
    case LV_IMG_CF_ALPHA_8BIT:                  return "ALPHA_8BIT";
    default:                                    return "UNKNOWN";
  }
}

// -------- public API --------

void DdpStream::register_stream(DdpStreamOutput* stream, uint8_t ddp_stream_id) {
  if (!stream) return;
  streams_[ddp_stream_id] = stream;
  this->bind_if_possible_(*stream);
}

void DdpStream::add_stream_binding(uint8_t id,
                                   std::function<lv_obj_t*()> getter,
                                   int w, int h) {
  auto it = streams_.find(id);
  if (it == streams_.end()) return;
  DdpStreamOutput &dst = *it->second;
  dst.getter_ = std::move(getter);
  dst.w_ = w;
  dst.h_ = h;
  dst.back_buffers_ = default_back_buffers_;
  this->bind_if_possible_(dst);
}

void DdpStream::set_stream_canvas(uint8_t id, lv_obj_t* canvas, int w, int h) {
  auto it = streams_.find(id);
  if (it == streams_.end()) return;
  DdpStreamOutput &dst = *it->second;
  dst.getter_ = [canvas]() -> lv_obj_t* { return canvas; };
  dst.canvas_ = canvas;
  dst.w_ = w;
  dst.h_ = h;
  if (dst.back_buffers_ > 2) dst.back_buffers_ = 2;
  this->bind_if_possible_(dst);
}

void DdpStream::set_stream_back_buffers(uint8_t id, uint8_t n) {
  auto it = streams_.find(id);
  if (it == streams_.end()) return;
  DdpStreamOutput &stream = *it->second;
  uint8_t nb = (n > 2) ? 2 : n;
  if (stream.back_buffers_ == nb) return;
  // Re-provision buffers according to the new mode
  stream.back_buffers_ = nb;
  free_ready_accum_(stream);
  ensure_binding_buffers_(stream);  // will (re)allocate as needed for mode/size
}

bool DdpStream::get_stream_size(uint8_t id, int *w, int *h) const {
  auto it = streams_.find(id);
  if (it == streams_.end()) return false;
  const DdpStreamOutput &stream = *it->second;
  if (stream.w_ <= 0 || stream.h_ <= 0) return false;
  if (w) *w = stream.w_;
  if (h) *h = stream.h_;
  return true;
}

uint8_t DdpStream::get_stream_id(uint8_t id) const {
  return id;
}

// -------- lifecycle --------

void DdpStream::setup() {
  ESP_LOGI(TAG, "setup this=%p port=%u", this, (unsigned) port_);
  init_rgb_luts_once();

  this->set_interval("ddp_net", 500, [this]() { this->ensure_socket_(); });

  this->set_interval("ddp_bind", 250, [this]() {
    bool all_ready = true;
    for (auto &kv : streams_) {
      DdpStreamOutput &stream = *kv.second;
      bool before = stream.bound_;
      this->bind_if_possible_(stream);
      bool after = stream.bound_;
      if (!before && after) {
        ESP_LOGI(TAG, "stream %u bound to canvas=%p (%dx%d)",
                 (unsigned) kv.first, (void*) stream.canvas_, stream.w_, stream.h_);
        lv_canvas_fill_bg(stream.canvas_, lv_color_black(), LV_OPA_COVER);
#if DDP_STREAM_METRICS
        stream.log_t0_us_ = esp_timer_get_time();
#endif
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

  for (auto &kv : streams_) {
    const DdpStreamOutput &stream = *kv.second;
    ESP_LOGCONFIG(TAG, "  stream id %u -> %dx%d canvas=%p (buf_px=%u) back_buffers=%u",
                  kv.first, stream.w_, stream.h_, (void*) stream.canvas_, (unsigned) stream.buf_px_, (unsigned) stream.back_buffers_);
  }
}

void DdpStream::loop() {
  bool had_activity = false;

  for (auto &kv : streams_) {
    DdpStreamOutput &stream = *kv.second;

    // Triple-buffer path: copy ready->front
    if (stream.back_buffers_ == 2) {
      if (!stream.have_ready_.exchange(false)) continue;
      if (!stream.canvas_ || stream.buf_px_ == 0) continue;
      if (stream.buf_px_ && stream.front_buf_ && stream.ready_buf_) {
        std::memcpy(stream.front_buf_, stream.ready_buf_, stream.buf_px_ * BYTES_PER_PIXEL);
      }
      stream.need_invalidate_.store(true, std::memory_order_relaxed);
    }

    // Double-buffer path: do memcpy on the LVGL/main thread
    if (stream.back_buffers_ == 1 && stream.need_copy_to_front_.exchange(false)) {
      if (stream.front_buf_ && stream.accum_buf_ && stream.buf_px_) {
        std::memcpy(stream.front_buf_, stream.accum_buf_, stream.buf_px_ * BYTES_PER_PIXEL);
      }
      stream.need_invalidate_.store(true, std::memory_order_relaxed);
    }

    // If any mode requested an invalidate, do it here (safe w.r.t LVGL refresh)
    bool did_present = false;
    if (stream.need_invalidate_.exchange(false)) {
      if (stream.canvas_) {
        lv_obj_invalidate(stream.canvas_);
        did_present = true;
        had_activity = true;
      }
    }

#if DDP_STREAM_METRICS
    if (did_present) {
      // queue wait time (ready -> present)
      if (stream.ready_set_us_) {
        int64_t now_us = esp_timer_get_time();
        double qwait_ms = (double)(now_us - stream.ready_set_us_) / 1000.0;
        stream.qwait_ms_ewma_ = ewma(stream.qwait_ms_ewma_, qwait_ms, 0.2);
        stream.ready_set_us_ = 0;
      }
      // present latency (first packet -> present), using the snapshot we took when queueing present
      if (stream.present_first_pkt_us_ != 0) {
        int64_t now = esp_timer_get_time();
        double lat_us = (double)(now - stream.present_first_pkt_us_);
        stream.present_lat_us_ewma_ = ewma(stream.present_lat_us_ewma_, lat_us);

        // clear both: we've accounted this frame
        stream.present_first_pkt_us_ = 0;
        stream.frame_seen_any_ = false;
        stream.frame_first_pkt_us_ = 0;
      }
      stream.frames_presented_ += 1;
      stream.win_frames_presented_ += 1;
    }
#endif
  }

#if DDP_STREAM_METRICS
  // Periodic windowed log (every ~2s)
  int64_t now = esp_timer_get_time();
  for (auto &kv : streams_) {
    uint8_t id = kv.first;
    DdpStreamOutput &stream = *kv.second;
    if (!stream.bound_) continue;
    if (stream.log_t0_us_ == 0) stream.log_t0_us_ = now;
    int64_t dt_us = now - stream.log_t0_us_;
    if (dt_us >= 2'000'000) {
      double dt_s = (double)dt_us / 1e6;

      double push_fps = stream.win_frames_push_ / dt_s;
      double pres_fps = stream.win_frames_presented_ / dt_s;
      double rx_mbps      = (stream.win_rx_bytes_ * 8.0) / (dt_s * 1e6);
      double rx_wire_mbps = (stream.win_rx_wire_bytes_ * 8.0) / (dt_s * 1e6);
      double cov_pct  = stream.coverage_ewma_ * 100.0;
      double lat_ms   = stream.present_lat_us_ewma_ / 1000.0;
      double qwait_ms = stream.qwait_ms_ewma_;
      double pkts_per_wakeup = (stream.rx_wakeups_ > 0)
        ? ((double)stream.rx_pkts_in_slices_ / (double)stream.rx_wakeups_)
        : 0.0;

      // Only log if there was activity in this window
      if (stream.win_frames_push_ > 0 || stream.win_frames_presented_ > 0 || stream.win_rx_bytes_ > 0) {
        ESP_LOGI(TAG,
          "id=%u rx_pkts=%llu rx_B=%llu win{push=%.1ffps pres=%.1ffps rx=%.2fMb/s wire=%.2fMb/s pkt_gap=%u overrun=%u} "
          "tot{push=%u pres=%u overrun=%u} cov(avg)=%.1f%% build(avg)=%.1f ms lat(avg)=%.1f ms qwait(avg)=%.1f ms intra(max)=%.1f ms "
          "rx_slc{wakeups=%u pkts/burst=%.2f}",
          (unsigned)id,
          (unsigned long long)stream.rx_pkts_,
          (unsigned long long)stream.rx_bytes_,
          push_fps, pres_fps, rx_mbps, rx_wire_mbps, stream.win_pkt_gap_, stream.win_overrun_,
          stream.frames_push_, stream.frames_presented_, stream.frames_overrun_,
          cov_pct, stream.build_ms_ewma_, lat_ms, qwait_ms, stream.intra_ms_max_,
          stream.rx_wakeups_, pkts_per_wakeup
        );
      }

      // reset window; keep totals & EWMA
      stream.rx_wakeups_ = 0;
      stream.rx_pkts_in_slices_ = 0;
      stream.win_frames_push_ = 0;
      stream.win_frames_presented_ = 0;
      stream.win_rx_bytes_ = 0;
      stream.win_rx_wire_bytes_ = 0;
      stream.win_pkt_gap_ = 0;
      stream.win_overrun_ = 0;
      stream.intra_ms_max_ = 0.0;
      stream.log_t0_us_ = now;
    }
  }
#endif

  // Loop optimization: disable when idle for 1 second
  if (had_activity) {
    last_activity_us_ = esp_timer_get_time();
    loop_is_disabled_.store(false, std::memory_order_relaxed);
  } else if (last_activity_us_ != 0) {
    int64_t idle_us = esp_timer_get_time() - last_activity_us_;
    if (idle_us > IDLE_TIMEOUT_US) {
      this->disable_loop();
      loop_is_disabled_.store(true, std::memory_order_release);
      last_activity_us_ = 0;
    }
  }
}

// -------- socket/RX --------

void DdpStream::ensure_socket_() {
  bool net_up = network::is_connected();
  if (net_up) {
    // Don't reopen while a previous task is still winding down.
    if (sock_ < 0 && !task_stopping_.load()) open_socket_();
  } else {
    if (sock_ >= 0) close_socket_();
  }
}

void DdpStream::open_socket_() {
  if (sock_ >= 0) return;

  int s = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (s < 0) { ESP_LOGW(TAG, "socket() failed errno=%d", errno); return; }

  int yes = 1;
  (void) setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_port = htons(port_);
  if (bind(s, (sockaddr*)&addr, sizeof(addr)) < 0) {
    ESP_LOGW(TAG, "bind() failed errno=%d", errno);
    ::close(s);
    return;
  }

  // Larger receive buffer to smooth bursts (kept; harmless for latency)
  int rcv = 64 * 1024; // 64 KB
  (void) setsockopt(s, SOL_SOCKET, SO_RCVBUF, &rcv, sizeof(rcv));

  sock_ = s;

  if (!udp_opened_) {
    ESP_LOGI(TAG, "DDP listening on UDP %u", port_);
    udp_opened_ = true;
  }

  if (task_ == nullptr) {
    task_should_exit_.store(false);
    task_stopping_.store(false);
    // Higher prio + pin to opposite core from typical LVGL usage
    xTaskCreatePinnedToCore(&DdpStream::recv_task_trampoline, "ddp_rx",
                            8192, this, 6, &task_, 1);
  }
}

void DdpStream::close_socket_() {
  if (task_ != nullptr) {
    task_stopping_.store(true);
    // Signal task to exit and close socket to break out of select()
    task_should_exit_.store(true);
    if (sock_ >= 0) { ::shutdown(sock_, SHUT_RDWR); ::close(sock_); sock_ = -1; }

    // Wait for task to exit gracefully with timeout
    TaskHandle_t t = task_;

    // Give the task some time to exit cleanly
    for (int i = 0; i < 100; i++) {
      if (eTaskGetState(t) == eDeleted) break;
      vTaskDelay(pdMS_TO_TICKS(1));
    }

    // Force delete if still running
    if (eTaskGetState(t) != eDeleted) {
      vTaskDelete(t);
    }
    task_ = nullptr;
    task_stopping_.store(false);
  } else if (sock_ >= 0) {
    ::shutdown(sock_, SHUT_RDWR); ::close(sock_); sock_ = -1;
  }

  for (auto &kv : streams_) {
    free_ready_accum_(*kv.second);
  }

  ESP_LOGI(TAG, "DDP socket closed");
}

void DdpStream::recv_task_trampoline(void* arg) {
  if (arg == nullptr) return;
  DdpStream* stream = reinterpret_cast<DdpStream*>(arg);
  stream->recv_task();
}

void DdpStream::recv_task() {
  std::vector<uint8_t> buf(2048);

  while (true) {
    if (task_should_exit_.load()) break;
    int s = sock_;
    if (s < 0) { vTaskDelay(pdMS_TO_TICKS(10)); continue; }

    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(s, &rfds);
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 500;  // wait ~0.5ms to reduce burst splitting

    int r = lwip_select(s + 1, &rfds, nullptr, nullptr, &tv);
    if (r > 0 && FD_ISSET(s, &rfds)) {
      const int MAX_PKTS_PER_SLICE = 128;
      const int MAX_SLICE_US       = 4500;  // tolerate ragged Wi‑Fi bursts
      int handled = 0;
      int64_t slice_start = esp_timer_get_time();
      // Attribute per-slice stats to only the streams that actually received packets
      std::map<DdpStreamOutput*, uint32_t> slice_counts;

      while (handled < MAX_PKTS_PER_SLICE) {
        ssize_t n = ::recv(s, buf.data(), buf.size(), MSG_DONTWAIT);
        if (n > 0) {
          this->handle_packet_(buf.data(), (size_t)n);

#if DDP_STREAM_METRICS
          // Attribute bytes/packet to the bound stream if possible.
          if ((size_t)n >= sizeof(DdpHeader)) {
            const DdpHeader* h = reinterpret_cast<const DdpHeader*>(buf.data());
            uint16_t len = ntohs(h->length_be); // pixel payload
            if (DdpStreamOutput *b = this->find_stream_(h->id)) {
              b->rx_pkts_          += 1;
              b->rx_bytes_         += (uint64_t)len;
              b->win_rx_bytes_     += (uint64_t)len;
              b->rx_wire_bytes_    += (uint64_t)n;
              b->win_rx_wire_bytes_+= (uint64_t)n;
              slice_counts[b]     += 1;
            }
          }
#endif
          // Only break if THIS packet was a PUSH (end of frame).
          if ((size_t)n >= sizeof(DdpHeader)) {
            const DdpHeader* h = reinterpret_cast<const DdpHeader*>(buf.data());
            if ((h->flags & 0x41) == 0x41) { // ver|PUSH
              handled++;
              break;
            }
          }

          handled++;
          if ((esp_timer_get_time() - slice_start) >= MAX_SLICE_US)
            break;
          continue;
        }
        break; // no more queued data immediately
      }

#if DDP_STREAM_METRICS
      for (auto &kvb : slice_counts) {
        DdpStreamOutput *b = kvb.first;
        uint32_t cnt = kvb.second;
        if (!b) continue;
        b->rx_wakeups_ += 1;
        b->rx_pkts_in_slices_ += (uint64_t)cnt;
      }
#endif
      taskYIELD();
    } else {
      vTaskDelay(pdMS_TO_TICKS(1));
    }
  }
  ESP_LOGI(TAG, "ddp_rx task exiting");
  vTaskDelete(nullptr);
}

// -------- binding helpers --------

void DdpStream::bind_if_possible_(DdpStreamOutput &stream) {
  if (stream.bound_) return;
  if (!stream.canvas_ && stream.getter_) stream.canvas_ = stream.getter_();
  if (!stream.canvas_) return;
  if (stream.w_ <= 0 || stream.h_ <= 0) {
    int W = (int) lv_obj_get_width(stream.canvas_);
    int H = (int) lv_obj_get_height(stream.canvas_);
    if (stream.w_ <= 0) stream.w_ = W;
    if (stream.h_ <= 0) stream.h_ = H;
  }
  if (stream.w_ <= 0 || stream.h_ <= 0) return;
  ensure_binding_buffers_(stream);
  // In mode 2 we require ready+accum; in mode 1 we require accum; in mode 0 we only require front.
  if (!stream.front_buf_) return;
  if (stream.back_buffers_ == 2 && (!stream.ready_buf_ || !stream.accum_buf_)) return;
  if (stream.back_buffers_ == 1 && (!stream.accum_buf_)) return;

  // Clear immediately to avoid first-show flash.
  lv_canvas_fill_bg(stream.canvas_, lv_color_black(), LV_OPA_COVER);

  lv_obj_add_event_cb(stream.canvas_, &DdpStream::on_canvas_size_changed_, LV_EVENT_SIZE_CHANGED, this);
  stream.bound_ = true;
}

void DdpStream::ensure_binding_buffers_(DdpStreamOutput &stream) {
  if (!stream.canvas_) return;

  // 1) Adopt the canvas' existing buffer as our front (owned by LVGL canvas)
  auto *img = (lv_img_dsc_t*) lv_canvas_get_img(stream.canvas_);
  if (!img || !img->data || img->header.w <= 0 || img->header.h <= 0) {
    // Canvas not fully initialized yet; try again later
    return;
  }

  const size_t px = (size_t)img->header.w * (size_t)img->header.h;

  // Log whenever the canvas buffer pointer changes or size changes
  if (stream.front_buf_ != (uint16_t*)img->data || stream.buf_px_ != px) {
    ESP_LOGI(TAG,
      "Using canvas buffer: cv=%p img=%p data=%p w=%d h=%d cf=%u(%s) LV_COLOR_DEPTH=%d buf_px(old=%u -> new=%u)",
      (void*)stream.canvas_, (void*)img, (void*)img->data,
      (int)img->header.w, (int)img->header.h,
      (unsigned)img->header.cf, img_cf_name(img->header.cf),
      (int)LV_COLOR_DEPTH,
      (unsigned)stream.buf_px_, (unsigned)px);
  }

  stream.front_buf_ = (uint16_t*) img->data;

  // 2) (Re)allocate only ready/accum when size changes; front is canvas-owned.
  if (stream.buf_px_ != px) {
    free_ready_accum_(stream);
    stream.buf_px_ = px;
  }
  size_t bytes = px * BYTES_PER_PIXEL;
  if (stream.back_buffers_ == 2 && !stream.ready_buf_) {
    stream.ready_buf_ = static_cast<uint16_t*>(lv_mem_alloc(bytes));
    if (!stream.ready_buf_) { free_ready_accum_(stream); stream.buf_px_ = 0; return; }
    std::memset(stream.ready_buf_, 0, bytes);
  }

  if ((stream.back_buffers_ >= 1) && !stream.accum_buf_) {
    stream.accum_buf_ = static_cast<uint16_t*>(lv_mem_alloc(bytes));
    if (!stream.accum_buf_) { free_ready_accum_(stream); stream.buf_px_ = 0; return; }
    std::memset(stream.accum_buf_, 0, bytes);
  }

  // If we downsized mode (e.g., to 1 or 0), make sure extra buffers are freed.
  if (stream.back_buffers_ < 2 && stream.ready_buf_) { lv_mem_free(stream.ready_buf_); stream.ready_buf_ = nullptr; }
  if (stream.back_buffers_ < 1 && stream.accum_buf_) { lv_mem_free(stream.accum_buf_); stream.accum_buf_ = nullptr; }
}

DdpStreamOutput* DdpStream::find_stream_(uint8_t id) {
  auto it = streams_.find(id);
  if (it == streams_.end()) return nullptr;
  return it->second;
}

void DdpStream::free_ready_accum_(DdpStreamOutput &stream) {
  if (stream.ready_buf_) { lv_mem_free(stream.ready_buf_); stream.ready_buf_ = nullptr; }
  if (stream.accum_buf_) { lv_mem_free(stream.accum_buf_); stream.accum_buf_ = nullptr; }
  stream.have_ready_.store(false);
}

void DdpStream::on_canvas_size_changed_(lv_event_t *e) {
  auto *canvas = lv_event_get_target(e);
  auto *self   = static_cast<DdpStream*>(lv_event_get_user_data(e));
  if (!self || !canvas) return;
  for (auto &kv : self->streams_) {
    DdpStreamOutput &stream = *kv.second;
    if (stream.canvas_ != canvas) continue;
    bool auto_any = (stream.w_ <= 0 || stream.h_ <= 0);
    if (!auto_any) return;
    int W = (int) lv_obj_get_width(canvas);
    int H = (int) lv_obj_get_height(canvas);
    if (stream.w_ <= 0) stream.w_ = W;
    if (stream.h_ <= 0) stream.h_ = H;
    // Re-adopt the canvas buffer and resize ready/accum if needed.
    self->ensure_binding_buffers_(stream);
    break;
  }
}

// -------- DDP decoder (dispatcher + helpers) --------

void DdpStream::handle_packet_(const uint8_t *raw, size_t n) {
  if (n < sizeof(DdpHeader)) return;
  auto *h = reinterpret_cast<const DdpHeader*>(raw);

  bool ver  = (h->flags & 0x40) != 0;
  bool push = (h->flags & 0x01) != 0;
  if (!ver) return;

  uint8_t id = h->id;
  DdpStreamOutput *stream = this->find_stream_(id);
  if (!stream || !stream->canvas_ || stream->buf_px_ == 0) return;

  uint32_t offset = ntohl(h->offset_be);
  uint16_t len    = ntohs(h->length_be);
  uint8_t  cfg    = h->pixcfg;

  // PUSH-only packet: present whatever we've accumulated
  if (len == 0) { if (push) handle_push_(*stream, h); return; }

  if (sizeof(DdpHeader) + len > n) return;
  const uint8_t* p = raw + sizeof(DdpHeader);

#if DDP_STREAM_METRICS
  // Start-of-frame metrics init (first data packet in frame)
  if (!stream->frame_seen_any_) {
    stream->frames_started_ += 1;
    stream->frame_first_pkt_us_ = esp_timer_get_time();
    stream->frame_bytes_accum_ = 0;
    stream->frame_px_accum_ = 0;
    stream->frame_seen_any_ = true;
    stream->intra_ms_max_ = 0.0;
    stream->last_pkt_us_ = 0;
  }
#endif

  // Handle payload by format
  if (cfg == DDP_PIXCFG_RGB888) {
    handle_rgb888_(*stream, h, p, len);
  } else if (cfg == DDP_PIXCFG_RGB565_LE || cfg == DDP_PIXCFG_RGB565_BE) {
    handle_rgb565_(*stream, h, p, len);
  } else {
    return; // unknown cfg; ignore
  }

#if DDP_STREAM_METRICS
  // Per-packet sequencing (gap detection)
  uint8_t cur_pkt = (uint8_t)(h->seq & 0x0F);
  if (stream->have_last_seq_pkt_) {
    uint8_t prev = stream->last_seq_pkt_ & 0x0F;
    uint8_t delta = (uint8_t)((cur_pkt - prev) & 0x0F);
    if (delta > 1) {
      uint32_t add = (uint32_t)(delta - 1);
      stream->pkt_seq_gaps_ += add;
      stream->win_pkt_gap_  += add;
    }
  }
  stream->last_seq_pkt_ = cur_pkt;
  stream->have_last_seq_pkt_ = true;

  // Track max intra-packet gap (diagnostic for burstiness/Jitter)
  int64_t nowp = esp_timer_get_time();
  if (stream->last_pkt_us_ != 0) {
    double gap_ms = (double)(nowp - stream->last_pkt_us_) / 1000.0;
    if (gap_ms > stream->intra_ms_max_) stream->intra_ms_max_ = gap_ms;
  }
  stream->last_pkt_us_ = nowp;
#endif

  // If this packet says PUSH, finalize coverage and queue for present
  if (push) {
#if DDP_STREAM_METRICS
    stream->frames_push_ += 1;
    stream->win_frames_push_ += 1;

    size_t expected_bytes = stream->buf_px_ * ((cfg == DDP_PIXCFG_RGB888) ? 3 : 2);
    double cov = (expected_bytes > 0)
      ? std::min(1.0, (double)stream->frame_bytes_accum_ / (double)expected_bytes)
      : 0.0;
    stream->coverage_ewma_ = ewma(stream->coverage_ewma_, cov);
    const double COMPLETE_THRESH = 0.95;
    if (cov >= COMPLETE_THRESH) stream->frames_ok_ += 1;
    else                        stream->frames_incomplete_ += 1;

    // Build time (first packet -> PUSH)
    if (stream->frame_first_pkt_us_) {
      int64_t now_us = esp_timer_get_time();
      double build_ms = (double)(now_us - stream->frame_first_pkt_us_) / 1000.0;
      stream->build_ms_ewma_ = ewma(stream->build_ms_ewma_, build_ms, 0.2);
    }
#endif

    if (stream->back_buffers_ == 2 && stream->have_ready_.load()) {
#if DDP_STREAM_METRICS
      stream->frames_overrun_ += 1;
      stream->win_overrun_ += 1;
#endif
      // overwrite policy: keep newest
    }

    if (stream->back_buffers_ == 2) {
      std::swap(stream->ready_buf_, stream->accum_buf_);
      stream->have_ready_.store(true);
    } else if (stream->back_buffers_ == 1) {
      stream->need_copy_to_front_.store(true, std::memory_order_relaxed);
      stream->need_invalidate_.store(true, std::memory_order_relaxed);
    } else { // back_buffers == 0
      stream->need_invalidate_.store(true, std::memory_order_relaxed);
    }

    // Wake loop if it was disabled due to inactivity
    if (loop_is_disabled_.load(std::memory_order_acquire)) {
      this->enable_loop_soon_any_context();
    }
#if DDP_STREAM_METRICS
  stream->ready_set_us_ = esp_timer_get_time();
  // Remember when the frame actually started, to compute present latency later.
  if (stream->frame_first_pkt_us_) stream->present_first_pkt_us_ = stream->frame_first_pkt_us_;
#endif
  }
}

// Present path used by PUSH-only packets
void DdpStream::handle_push_(DdpStreamOutput &stream, const DdpHeader* h) {
  // Wake loop if it was disabled due to inactivity
  if (loop_is_disabled_.load(std::memory_order_acquire)) {
    this->enable_loop_soon_any_context();
  }

#if DDP_STREAM_METRICS
  stream.frames_push_ += 1;
  stream.win_frames_push_ += 1;
  if (stream.frame_first_pkt_us_) {
    int64_t now_us = esp_timer_get_time();
    double build_ms = (double)(now_us - stream.frame_first_pkt_us_) / 1000.0;
    stream.build_ms_ewma_ = ewma(stream.build_ms_ewma_, build_ms, 0.2);
  }
#endif

  if (stream.back_buffers_ == 2) {
    if (stream.have_ready_.load()) {
#if DDP_STREAM_METRICS
      stream.frames_overrun_ += 1;
      stream.win_overrun_ += 1;
#endif
    }
    if (stream.accum_buf_) {
      std::swap(stream.ready_buf_, stream.accum_buf_);
      stream.have_ready_.store(true);
#if DDP_STREAM_METRICS
      stream.ready_set_us_ = esp_timer_get_time();
      if (stream.frame_first_pkt_us_) stream.present_first_pkt_us_ = stream.frame_first_pkt_us_;
#endif
    }
  } else if (stream.back_buffers_ == 1) {
    stream.need_copy_to_front_.store(true, std::memory_order_relaxed);
    stream.need_invalidate_.store(true, std::memory_order_relaxed);
#if DDP_STREAM_METRICS
    stream.ready_set_us_ = esp_timer_get_time();
    if (stream.frame_first_pkt_us_) stream.present_first_pkt_us_ = stream.frame_first_pkt_us_;
#endif
  } else { // 0
    stream.need_invalidate_.store(true, std::memory_order_relaxed);
#if DDP_STREAM_METRICS
    stream.ready_set_us_ = esp_timer_get_time();
    if (stream.frame_first_pkt_us_) stream.present_first_pkt_us_ = stream.frame_first_pkt_us_;
#endif
  }
}

// RGB888 -> RGB565 (dst is canvas/native 565)
void DdpStream::handle_rgb888_(DdpStreamOutput &stream, const DdpHeader* h, const uint8_t* p, size_t len) {
  if ((ntohl(h->offset_be) % 3) != 0 || (len % 3) != 0) return;

  size_t pixel_off = ntohl(h->offset_be) / 3;
  size_t px        = len / 3;
  size_t max_px    = stream.buf_px_;
  if (pixel_off >= max_px) return;
  size_t write_px  = std::min(px, max_px - pixel_off);

#if LV_COLOR_DEPTH == 16
  uint16_t* dst = (stream.back_buffers_ == 0) ? (stream.front_buf_ + pixel_off)
                                        : (stream.accum_buf_ ? stream.accum_buf_ + pixel_off : nullptr);
  if (!dst) return;
  const uint8_t* sp = p;
  #if defined(LV_COLOR_16_SWAP) && LV_COLOR_16_SWAP
    for (size_t i = 0; i < write_px; ++i) {
      uint16_t c = LUT_R5[sp[0]] | LUT_G6[sp[1]] | LUT_B5[sp[2]];
      dst[i] = (uint16_t)((c >> 8) | (c << 8));
      sp += 3;
    }
  #else
    for (size_t i = 0; i < write_px; ++i) {
      dst[i] = (uint16_t)(LUT_R5[sp[0]] | LUT_G6[sp[1]] | LUT_B5[sp[2]]);
      sp += 3;
    }
  #endif
#elif LV_COLOR_DEPTH == 32
  if ((stream.back_buffers_ == 0 && !stream.front_buf_) || (stream.back_buffers_ != 0 && !stream.accum_buf_)) return;
  lv_color32_t* dst32 = reinterpret_cast<lv_color32_t*>(
      (stream.back_buffers_ == 0) ? stream.front_buf_ : stream.accum_buf_);
  dst32 += pixel_off;
  const uint8_t* sp = p;
  for (size_t i = 0; i < write_px; ++i) {
    lv_color32_t c;
    c.ch.red   = sp[0];
    c.ch.green = sp[1];
    c.ch.blue  = sp[2];
    c.ch.alpha = 0xFF;
    dst32[i] = c;
    sp += 3;
  }
#endif

#if DDP_STREAM_METRICS
  stream.frame_bytes_accum_ += len;
  stream.frame_px_accum_    += write_px;
#endif
}

// RGB565 passthrough with explicit byte-wise swap on mismatch
void DdpStream::handle_rgb565_(DdpStreamOutput &stream, const DdpHeader* h, const uint8_t* p, size_t len) {
  if ((ntohl(h->offset_be) % 2) != 0 || (len % 2) != 0) return;

  size_t pixel_off = ntohl(h->offset_be) / 2;
  size_t px        = len / 2;
  size_t max_px    = stream.buf_px_;
  if (pixel_off >= max_px) return;
  size_t write_px  = std::min(px, max_px - pixel_off);

#if LV_COLOR_DEPTH == 16
  uint16_t* dst = (stream.back_buffers_ == 0) ? (stream.front_buf_ + pixel_off)
                                        : (stream.accum_buf_ ? stream.accum_buf_ + pixel_off : nullptr);
  if (!dst) return;
  const bool src_be = (h->pixcfg == DDP_PIXCFG_RGB565_BE);
  const bool want_be =
  #if defined(LV_COLOR_16_SWAP) && LV_COLOR_16_SWAP
      true;
  #else
      false;
  #endif
  const uint8_t* sp = p;
  if (src_be == want_be) {
    std::memcpy(dst, sp, write_px * BYTES_PER_PIXEL);
  } else {
    ESP_LOGW(TAG, "Mismatch of rgb565 endian format, using slow path");
    uint8_t *d8 = reinterpret_cast<uint8_t*>(dst);
    for (size_t i = 0; i < write_px; ++i) {
      d8[2*i + 0] = sp[1];
      d8[2*i + 1] = sp[0];
      sp += 2;
    }
  }
#elif LV_COLOR_DEPTH == 32
  if ((stream.back_buffers_ == 0 && !stream.front_buf_) || (stream.back_buffers_ != 0 && !stream.accum_buf_)) return;
  lv_color32_t* dst32 = reinterpret_cast<lv_color32_t*>(
      (stream.back_buffers_ == 0) ? stream.front_buf_ : stream.accum_buf_);
  dst32 += pixel_off;
  const bool src_be = (h->pixcfg == DDP_PIXCFG_RGB565_BE);
  const uint8_t* sp = p;
  for (size_t i = 0; i < write_px; ++i) {
    uint16_t v = src_be ? (uint16_t)((sp[0] << 8) | sp[1]) : (uint16_t)((sp[1] << 8) | sp[0]);
    uint8_t r5 = (uint8_t)((v >> 11) & 0x1F);
    uint8_t g6 = (uint8_t)((v >> 5)  & 0x3F);
    uint8_t b5 = (uint8_t)( v        & 0x1F);
    lv_color32_t c;
    c.ch.red   = (uint8_t)((r5 << 3) | (r5 >> 2));
    c.ch.green = (uint8_t)((g6 << 2) | (g6 >> 4));
    c.ch.blue  = (uint8_t)((b5 << 3) | (b5 >> 2));
    c.ch.alpha = 0xFF;
    dst32[i] = c;
    sp += 2;
  }
#endif

#if DDP_STREAM_METRICS
  stream.frame_bytes_accum_ += len;
  stream.frame_px_accum_    += write_px;
#endif
}

} // namespace ddp_stream
} // namespace esphome
