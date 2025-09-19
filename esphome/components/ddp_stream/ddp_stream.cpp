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

void DdpStream::add_stream_binding(uint8_t id,
                                   std::function<lv_obj_t*()> getter,
                                   int w, int h) {
  Binding &dst = bindings_[id];
  dst.getter = std::move(getter);
  dst.w = w;
  dst.h = h;
  dst.back_buffers = default_back_buffers_;
  this->bind_if_possible_(dst);
}

void DdpStream::set_stream_canvas(uint8_t id, lv_obj_t* canvas, int w, int h) {
  Binding &dst = bindings_[id];
  dst.getter = [canvas]() -> lv_obj_t* { return canvas; };
  dst.canvas = canvas;
  dst.w = w;
  dst.h = h;
  if (dst.back_buffers > 2) dst.back_buffers = 2;
  this->bind_if_possible_(dst);
}

void DdpStream::set_stream_back_buffers(uint8_t id, uint8_t n) {
  auto it = bindings_.find(id);
  if (it == bindings_.end()) return;
  Binding &b = it->second;
  uint8_t nb = (n > 2) ? 2 : n;
  if (b.back_buffers == nb) return;
  // Re-provision buffers according to the new mode
  b.back_buffers = nb;
  free_ready_accum_(b);
  ensure_binding_buffers_(b);  // will (re)allocate as needed for mode/size
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
  init_rgb_luts_once();

  this->set_interval("ddp_net", 500, [this]() { this->ensure_socket_(); });

  this->set_interval("ddp_bind", 250, [this]() {
    bool all_ready = true;
    for (auto &kv : bindings_) {
      Binding &b = kv.second;
      bool before = b.bound;
      this->bind_if_possible_(b);
      bool after = b.bound;
      if (!before && after) {
        ESP_LOGI(TAG, "stream %u bound to canvas=%p (%dx%d)",
                 (unsigned) kv.first, (void*) b.canvas, b.w, b.h);
        lv_canvas_fill_bg(b.canvas, lv_color_black(), LV_OPA_COVER);
#if DDP_STREAM_METRICS
        b.log_t0_us = esp_timer_get_time();
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

  for (auto &kv : bindings_) {
    const Binding &b = kv.second;
    ESP_LOGCONFIG(TAG, "  stream id %u -> %dx%d canvas=%p (buf_px=%u) back_buffers=%u",
                  kv.first, b.w, b.h, (void*) b.canvas, (unsigned) b.buf_px, (unsigned) b.back_buffers);
  }
}

void DdpStream::loop() {
  for (auto &kv : bindings_) {
    Binding &b = kv.second;

    // Triple-buffer path: copy ready->front
    if (b.back_buffers == 2) {
      if (!b.have_ready.exchange(false)) continue;
      if (!b.canvas || b.buf_px == 0) continue;
      if (b.buf_px && b.front_buf && b.ready_buf) {
        std::memcpy(b.front_buf, b.ready_buf, b.buf_px * BYTES_PER_PIXEL);
      }
      b.need_invalidate.store(true, std::memory_order_relaxed);
    }

    // Double-buffer path: do memcpy on the LVGL/main thread
    if (b.back_buffers == 1 && b.need_copy_to_front.exchange(false)) {
      if (b.front_buf && b.accum_buf && b.buf_px) {
        std::memcpy(b.front_buf, b.accum_buf, b.buf_px * BYTES_PER_PIXEL);
      }
      b.need_invalidate.store(true, std::memory_order_relaxed);
    }

    // If any mode requested an invalidate, do it here (safe w.r.t LVGL refresh)
    bool did_present = false;
    if (b.need_invalidate.exchange(false)) {
      if (b.canvas) {
        lv_obj_invalidate(b.canvas);
        did_present = true;
      }
    }

#if DDP_STREAM_METRICS
    if (did_present) {
      // queue wait time (ready -> present)
      if (b.ready_set_us) {
        int64_t now_us = esp_timer_get_time();
        double qwait_ms = (double)(now_us - b.ready_set_us) / 1000.0;
        b.qwait_ms_ewma = ewma(b.qwait_ms_ewma, qwait_ms, 0.2);
        b.ready_set_us = 0;
      }
      // present latency (first packet -> present), using the snapshot we took when queueing present
      if (b.present_first_pkt_us != 0) {
        int64_t now = esp_timer_get_time();
        double lat_us = (double)(now - b.present_first_pkt_us);
        b.present_lat_us_ewma = ewma(b.present_lat_us_ewma, lat_us);

        // clear both: we've accounted this frame
        b.present_first_pkt_us = 0;
        b.frame_seen_any = false;
        b.frame_first_pkt_us = 0;
      }
      b.frames_presented += 1;
      b.win_frames_presented += 1;
    }
#endif
  }

#if DDP_STREAM_METRICS
  // Periodic windowed log (every ~2s)
  int64_t now = esp_timer_get_time();
  for (auto &kv : bindings_) {
    uint8_t id = kv.first;
    Binding &b = kv.second;
    if (!b.bound) continue;
    if (b.log_t0_us == 0) b.log_t0_us = now;
    int64_t dt_us = now - b.log_t0_us;
    if (dt_us >= 2'000'000) {
      double dt_s = (double)dt_us / 1e6;

      double push_fps = b.win_frames_push / dt_s;
      double pres_fps = b.win_frames_presented / dt_s;
      double rx_mbps      = (b.win_rx_bytes * 8.0) / (dt_s * 1e6);
      double rx_wire_mbps = (b.win_rx_wire_bytes * 8.0) / (dt_s * 1e6);
      double cov_pct  = b.coverage_ewma * 100.0;
      double lat_ms   = b.present_lat_us_ewma / 1000.0;
      double qwait_ms = b.qwait_ms_ewma;
      double pkts_per_wakeup = (b.rx_wakeups > 0)
        ? ((double)b.rx_pkts_in_slices / (double)b.rx_wakeups)
        : 0.0;

      ESP_LOGI(TAG,
        "id=%u rx_pkts=%llu rx_B=%llu win{push=%.1ffps pres=%.1ffps rx=%.2fMb/s wire=%.2fMb/s pkt_gap=%u overrun=%u} "
        "tot{push=%u pres=%u overrun=%u} cov(avg)=%.1f%% build(avg)=%.1f ms lat(avg)=%.1f ms qwait(avg)=%.1f ms intra(max)=%.1f ms "
        "rx_slc{wakeups=%u pkts/burst=%.2f}",
        (unsigned)id,
        (unsigned long long)b.rx_pkts,
        (unsigned long long)b.rx_bytes,
        push_fps, pres_fps, rx_mbps, rx_wire_mbps, b.win_pkt_gap, b.win_overrun,
        b.frames_push, b.frames_presented, b.frames_overrun,
        cov_pct, b.build_ms_ewma, lat_ms, qwait_ms, b.intra_ms_max,
        b.rx_wakeups, pkts_per_wakeup
      );

      // reset window; keep totals & EWMA
      b.rx_wakeups = 0;
      b.rx_pkts_in_slices = 0;
      b.win_frames_push = 0;
      b.win_frames_presented = 0;
      b.win_rx_bytes = 0;
      b.win_rx_wire_bytes = 0;
      b.win_pkt_gap = 0;
      b.win_overrun = 0;
      b.intra_ms_max = 0.0;
      b.log_t0_us = now;
    }
  }
#endif
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
    // Higher prio + pin to opposite core from typical LVGL usage
    xTaskCreatePinnedToCore(&DdpStream::recv_task_trampoline, "ddp_rx",
                            6144, this, 6, &task_, 1);
  }
}

void DdpStream::close_socket_() {
  if (task_ != nullptr) {
    // Signal task to exit and close socket to break out of select()
    task_should_exit_.store(true);
    if (sock_ >= 0) { ::shutdown(sock_, SHUT_RDWR); ::close(sock_); sock_ = -1; }

    // Wait for task to exit gracefully with timeout
    TaskHandle_t t = task_;
    task_ = nullptr;

    // Give the task some time to exit cleanly
    for (int i = 0; i < 100; i++) {
      if (eTaskGetState(t) == eDeleted) break;
      vTaskDelay(pdMS_TO_TICKS(1));
    }

    // Force delete if still running
    if (eTaskGetState(t) != eDeleted) {
      vTaskDelete(t);
    }
  } else if (sock_ >= 0) {
    ::shutdown(sock_, SHUT_RDWR); ::close(sock_); sock_ = -1;
  }

  for (auto &kv : bindings_) {
    free_ready_accum_(kv.second);
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
      // Attribute per-slice stats to only the bindings that actually received packets
      std::map<Binding*, uint32_t> slice_counts;

      while (handled < MAX_PKTS_PER_SLICE) {
        ssize_t n = ::recv(s, buf.data(), buf.size(), MSG_DONTWAIT);
        if (n > 0) {
          this->handle_packet_(buf.data(), (size_t)n);

#if DDP_STREAM_METRICS
          // Attribute bytes/packet to the bound stream if possible.
          if ((size_t)n >= sizeof(DdpHeader)) {
            const DdpHeader* h = reinterpret_cast<const DdpHeader*>(buf.data());
            uint16_t len = ntohs(h->length_be); // pixel payload
            if (Binding *b = this->find_binding_(h->id)) {
              b->rx_pkts          += 1;
              b->rx_bytes         += (uint64_t)len;
              b->win_rx_bytes     += (uint64_t)len;
              b->rx_wire_bytes    += (uint64_t)n;
              b->win_rx_wire_bytes+= (uint64_t)n;
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
        Binding *b = kvb.first;
        uint32_t cnt = kvb.second;
        if (!b) continue;
        b->rx_wakeups += 1;
        b->rx_pkts_in_slices += (uint64_t)cnt;
      }
#endif
      taskYIELD();
    } else {
      vTaskDelay(pdMS_TO_TICKS(1));
    }
  }
}

// -------- binding helpers --------

void DdpStream::bind_if_possible_(Binding &b) {
  if (b.bound) return;
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
  // In mode 2 we require ready+accum; in mode 1 we require accum; in mode 0 we only require front.
  if (!b.front_buf) return;
  if (b.back_buffers == 2 && (!b.ready_buf || !b.accum_buf)) return;
  if (b.back_buffers == 1 && (!b.accum_buf)) return;

  // Clear immediately to avoid first-show flash.
  lv_canvas_fill_bg(b.canvas, lv_color_black(), LV_OPA_COVER);

  lv_obj_add_event_cb(b.canvas, &DdpStream::on_canvas_size_changed_, LV_EVENT_SIZE_CHANGED, this);
  b.bound = true;
}

void DdpStream::ensure_binding_buffers_(Binding &b) {
  if (!b.canvas) return;

  // 1) Adopt the canvas' existing buffer as our front (owned by LVGL canvas)
  auto *img = (lv_img_dsc_t*) lv_canvas_get_img(b.canvas);
  if (!img || !img->data || img->header.w <= 0 || img->header.h <= 0) {
    // Canvas not fully initialized yet; try again later
    return;
  }

  const size_t px = (size_t)img->header.w * (size_t)img->header.h;

  // Log whenever the canvas buffer pointer changes or size changes
  if (b.front_buf != (uint16_t*)img->data || b.buf_px != px) {
    ESP_LOGI(TAG,
      "Using canvas buffer: cv=%p img=%p data=%p w=%d h=%d cf=%u(%s) LV_COLOR_DEPTH=%d buf_px(old=%u -> new=%u)",
      (void*)b.canvas, (void*)img, (void*)img->data,
      (int)img->header.w, (int)img->header.h,
      (unsigned)img->header.cf, img_cf_name(img->header.cf),
      (int)LV_COLOR_DEPTH,
      (unsigned)b.buf_px, (unsigned)px);
  }

  b.front_buf = (uint16_t*) img->data;

  // 2) (Re)allocate only ready/accum when size changes; front is canvas-owned.
  if (b.buf_px != px) {
    free_ready_accum_(b);
    b.buf_px = px;
  }
  size_t bytes = px * BYTES_PER_PIXEL;
  if (b.back_buffers == 2 && !b.ready_buf) {
    b.ready_buf = static_cast<uint16_t*>(lv_mem_alloc(bytes));
    if (!b.ready_buf) { free_ready_accum_(b); b.buf_px = 0; return; }
    std::memset(b.ready_buf, 0, bytes);
  }

  if ((b.back_buffers >= 1) && !b.accum_buf) {
    b.accum_buf = static_cast<uint16_t*>(lv_mem_alloc(bytes));
    if (!b.accum_buf) { free_ready_accum_(b); b.buf_px = 0; return; }
    std::memset(b.accum_buf, 0, bytes);
  }

  // If we downsized mode (e.g., to 1 or 0), make sure extra buffers are freed.
  if (b.back_buffers < 2 && b.ready_buf) { lv_mem_free(b.ready_buf); b.ready_buf = nullptr; }
  if (b.back_buffers < 1 && b.accum_buf) { lv_mem_free(b.accum_buf); b.accum_buf = nullptr; }
}

DdpStream::Binding* DdpStream::find_binding_(uint8_t id) {
  auto it = bindings_.find(id);
  if (it == bindings_.end()) return nullptr;
  return &it->second;
}

void DdpStream::free_ready_accum_(Binding &b) {
  if (b.ready_buf) { lv_mem_free(b.ready_buf); b.ready_buf = nullptr; }
  if (b.accum_buf) { lv_mem_free(b.accum_buf); b.accum_buf = nullptr; }
  b.have_ready.store(false);
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
    // Re-adopt the canvas buffer and resize ready/accum if needed.
    self->ensure_binding_buffers_(b);
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
  Binding *b = this->find_binding_(id);
  if (!b || !b->canvas || b->buf_px == 0) return;

  uint32_t offset = ntohl(h->offset_be);
  uint16_t len    = ntohs(h->length_be);
  uint8_t  cfg    = h->pixcfg;

  // PUSH-only packet: present whatever we've accumulated
  if (len == 0) { if (push) handle_push_(*b, h); return; }

  if (sizeof(DdpHeader) + len > n) return;
  const uint8_t* p = raw + sizeof(DdpHeader);

#if DDP_STREAM_METRICS
  // Start-of-frame metrics init (first data packet in frame)
  if (!b->frame_seen_any) {
    b->frames_started += 1;
    b->frame_first_pkt_us = esp_timer_get_time();
    b->frame_bytes_accum = 0;
    b->frame_px_accum = 0;
    b->frame_seen_any = true;
    b->intra_ms_max = 0.0;
    b->last_pkt_us = 0;
  }
#endif

  // Handle payload by format
  if (cfg == DDP_PIXCFG_RGB888) {
    handle_rgb888_(*b, h, p, len);
  } else if (cfg == DDP_PIXCFG_RGB565_LE || cfg == DDP_PIXCFG_RGB565_BE) {
    handle_rgb565_(*b, h, p, len);
  } else {
    return; // unknown cfg; ignore
  }

#if DDP_STREAM_METRICS
  // Per-packet sequencing (gap detection)
  uint8_t cur_pkt = (uint8_t)(h->seq & 0x0F);
  if (b->have_last_seq_pkt) {
    uint8_t prev = b->last_seq_pkt & 0x0F;
    uint8_t delta = (uint8_t)((cur_pkt - prev) & 0x0F);
    if (delta > 1) {
      uint32_t add = (uint32_t)(delta - 1);
      b->pkt_seq_gaps += add;
      b->win_pkt_gap  += add;
    }
  }
  b->last_seq_pkt = cur_pkt;
  b->have_last_seq_pkt = true;

  // Track max intra-packet gap (diagnostic for burstiness/Jitter)
  int64_t nowp = esp_timer_get_time();
  if (b->last_pkt_us != 0) {
    double gap_ms = (double)(nowp - b->last_pkt_us) / 1000.0;
    if (gap_ms > b->intra_ms_max) b->intra_ms_max = gap_ms;
  }
  b->last_pkt_us = nowp;
#endif

  // If this packet says PUSH, finalize coverage and queue for present
  if (push) {
#if DDP_STREAM_METRICS
    b->frames_push += 1;
    b->win_frames_push += 1;

    size_t expected_bytes = b->buf_px * ((cfg == DDP_PIXCFG_RGB888) ? 3 : 2);
    double cov = (expected_bytes > 0)
      ? std::min(1.0, (double)b->frame_bytes_accum / (double)expected_bytes)
      : 0.0;
    b->coverage_ewma = ewma(b->coverage_ewma, cov);
    const double COMPLETE_THRESH = 0.95;
    if (cov >= COMPLETE_THRESH) b->frames_ok += 1;
    else                        b->frames_incomplete += 1;

    // Build time (first packet -> PUSH)
    if (b->frame_first_pkt_us) {
      int64_t now_us = esp_timer_get_time();
      double build_ms = (double)(now_us - b->frame_first_pkt_us) / 1000.0;
      b->build_ms_ewma = ewma(b->build_ms_ewma, build_ms, 0.2);
    }
#endif

    if (b->back_buffers == 2 && b->have_ready.load()) {
#if DDP_STREAM_METRICS
      b->frames_overrun += 1;
      b->win_overrun += 1;
#endif
      // overwrite policy: keep newest
    }

    if (b->back_buffers == 2) {
      std::swap(b->ready_buf, b->accum_buf);
      b->have_ready.store(true);
    } else if (b->back_buffers == 1) {
      b->need_copy_to_front.store(true, std::memory_order_relaxed);
      b->need_invalidate.store(true, std::memory_order_relaxed);
    } else { // back_buffers == 0
      b->need_invalidate.store(true, std::memory_order_relaxed);
    }
#if DDP_STREAM_METRICS
  b->ready_set_us = esp_timer_get_time();
  // Remember when the frame actually started, to compute present latency later.
  if (b->frame_first_pkt_us) b->present_first_pkt_us = b->frame_first_pkt_us;
#endif
  }
}

// Present path used by PUSH-only packets
void DdpStream::handle_push_(Binding &b, const DdpHeader* h) {
#if DDP_STREAM_METRICS
  b.frames_push += 1;
  b.win_frames_push += 1;
  if (b.frame_first_pkt_us) {
    int64_t now_us = esp_timer_get_time();
    double build_ms = (double)(now_us - b.frame_first_pkt_us) / 1000.0;
    b.build_ms_ewma = ewma(b.build_ms_ewma, build_ms, 0.2);
  }
#endif

  if (b.back_buffers == 2) {
    if (b.have_ready.load()) {
#if DDP_STREAM_METRICS
      b.frames_overrun += 1;
      b.win_overrun += 1;
#endif
    }
    if (b.accum_buf) {
      std::swap(b.ready_buf, b.accum_buf);
      b.have_ready.store(true);
#if DDP_STREAM_METRICS
      b.ready_set_us = esp_timer_get_time();
      if (b.frame_first_pkt_us) b.present_first_pkt_us = b.frame_first_pkt_us;
#endif
    }
  } else if (b.back_buffers == 1) {
    b.need_copy_to_front.store(true, std::memory_order_relaxed);
    b.need_invalidate.store(true, std::memory_order_relaxed);
#if DDP_STREAM_METRICS
    b.ready_set_us = esp_timer_get_time();
    if (b.frame_first_pkt_us) b.present_first_pkt_us = b.frame_first_pkt_us;
#endif
  } else { // 0
    b.need_invalidate.store(true, std::memory_order_relaxed);
#if DDP_STREAM_METRICS
    b.ready_set_us = esp_timer_get_time();
    if (b.frame_first_pkt_us) b.present_first_pkt_us = b.frame_first_pkt_us;
#endif
  }
}

// RGB888 -> RGB565 (dst is canvas/native 565)
void DdpStream::handle_rgb888_(Binding &b, const DdpHeader* h, const uint8_t* p, size_t len) {
  if ((ntohl(h->offset_be) % 3) != 0 || (len % 3) != 0) return;

  size_t pixel_off = ntohl(h->offset_be) / 3;
  size_t px        = len / 3;
  size_t max_px    = b.buf_px;
  if (pixel_off >= max_px) return;
  size_t write_px  = std::min(px, max_px - pixel_off);

#if LV_COLOR_DEPTH == 16
  uint16_t* dst = (b.back_buffers == 0) ? (b.front_buf + pixel_off)
                                        : (b.accum_buf ? b.accum_buf + pixel_off : nullptr);
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
  if ((b.back_buffers == 0 && !b.front_buf) || (b.back_buffers != 0 && !b.accum_buf)) return;
  lv_color32_t* dst32 = reinterpret_cast<lv_color32_t*>(
      (b.back_buffers == 0) ? b.front_buf : b.accum_buf);
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
  b.frame_bytes_accum += len;
  b.frame_px_accum    += write_px;
#endif
}

// RGB565 passthrough with explicit byte-wise swap on mismatch
void DdpStream::handle_rgb565_(Binding &b, const DdpHeader* h, const uint8_t* p, size_t len) {
  if ((ntohl(h->offset_be) % 2) != 0 || (len % 2) != 0) return;

  size_t pixel_off = ntohl(h->offset_be) / 2;
  size_t px        = len / 2;
  size_t max_px    = b.buf_px;
  if (pixel_off >= max_px) return;
  size_t write_px  = std::min(px, max_px - pixel_off);

#if LV_COLOR_DEPTH == 16
  uint16_t* dst = (b.back_buffers == 0) ? (b.front_buf + pixel_off)
                                        : (b.accum_buf ? b.accum_buf + pixel_off : nullptr);
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
  if ((b.back_buffers == 0 && !b.front_buf) || (b.back_buffers != 0 && !b.accum_buf)) return;
  lv_color32_t* dst32 = reinterpret_cast<lv_color32_t*>(
      (b.back_buffers == 0) ? b.front_buf : b.accum_buf);
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
  b.frame_bytes_accum += len;
  b.frame_px_accum    += write_px;
#endif
}

} // namespace ddp_stream
} // namespace esphome
