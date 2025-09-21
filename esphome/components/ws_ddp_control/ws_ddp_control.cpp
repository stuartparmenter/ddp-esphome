// © Copyright 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT

#include "ws_ddp_control.h"
#include "esphome/core/log.h"
#include "esphome/components/network/util.h"
#include <cstring>
#include <algorithm>
#include <cstdio>
#include <cctype>

extern "C" {
  #include "esp_websocket_client.h"
  #include "freertos/FreeRTOS.h"
  #include "freertos/task.h"
}

static const char *TAG = "ws_ddp_control";

namespace esphome {
namespace ws_ddp_control {

// ----------------- local helpers (no header changes required) -----------------
static inline void append_json_str(std::string &dst, const char *key, const std::string &val) {
  dst += "\""; dst += key; dst += "\":\""; dst += val; dst += "\"";
}
static inline void append_json_int(std::string &dst, const char *key, long long v) {
  dst += "\""; dst += key; dst += "\":"; dst += std::to_string(v);
}
static inline void append_json_float(std::string &dst, const char *key, double v) {
  char buf[64]; snprintf(buf, sizeof(buf), "\"%s\":%.6f", key, v); dst += buf;
}
static inline void append_json_bool(std::string &dst, const char *key, bool v) {
  dst += "\""; dst += key; dst += "\":"; dst += (v ? "true" : "false");
}

// Task to cleanup websocket client without blocking main thread
static void cleanup_websocket_task(void *client_handle) {
  if (client_handle) {
    esp_websocket_client_stop((esp_websocket_client_handle_t) client_handle);
    esp_websocket_client_destroy((esp_websocket_client_handle_t) client_handle);
  }
  vTaskDelete(NULL);  // Delete this cleanup task
}

// one canonical place to build the stream JSON (used by start_ and send_update_)
static std::string build_stream_json_(const char *type,
                                      uint8_t out,
                                      int w, int h,
                                      uint16_t ddp_port,
                                      const std::optional<std::string> &src,
                                      const std::optional<std::string> &fmt,
                                      const std::optional<uint8_t> &pixcfg,
                                      const std::optional<int> &pace,
                                      const std::optional<float> &ema,
                                      const std::optional<int> &expand,
                                      const std::optional<bool> &loop,
                                      const std::optional<std::string> &hw) {
  std::string json;
  json.reserve(256);
  json += "{";
  append_json_str(json, "type", type);      json += ",";
  append_json_int(json, "out", out);        json += ",";
  append_json_int(json, "w", w);            json += ",";
  append_json_int(json, "h", h);            json += ",";
  append_json_int(json, "ddp_port", ddp_port);
  auto add = [&](auto fn){ json += ","; fn(); };
  if (src)    add([&](){ append_json_str(json, "src", *src); });
  if (fmt)    add([&](){ append_json_str(json, "fmt", *fmt); });
  if (pixcfg) add([&](){ append_json_int(json, "pixcfg", (int)*pixcfg); });
  if (pace)   add([&](){ append_json_int(json, "pace", *pace); });
  if (ema)    add([&](){ append_json_float(json, "ema", *ema); });
  if (expand) add([&](){ append_json_int(json, "expand", *expand); });
  if (loop)   add([&](){ append_json_bool(json, "loop", *loop); });
  if (hw)     add([&](){ append_json_str(json, "hw", *hw); });
  json += "}";
  return json;
}

static void log_stream_line_(const char *label,
                             uint8_t out,
                             int w, int h,
                             uint16_t ddp_port,
                             const std::optional<std::string> &src,
                             const std::optional<std::string> &fmt,
                             const std::optional<uint8_t> &pixcfg,
                             const std::optional<int> &pace,
                             const std::optional<float> &ema,
                             const std::optional<int> &expand,
                             const std::optional<bool> &loop,
                             const std::optional<std::string> &hw) {
  ESP_LOGI(TAG, "tx %s out=%u size=%dx%d src=%s ddp_port=%u fmt=%s pixcfg=0x%02X "
                "pace=%s ema=%s expand=%s loop=%s hw=%s",
           label,
           (unsigned) out, w, h, (src?src->c_str():"(unset)"), (unsigned) ddp_port,
           (fmt?fmt->c_str():"(unset)"), (unsigned) (pixcfg?*pixcfg:0),
           (pace?std::to_string(*pace).c_str():"(unset)"),
           (ema?std::to_string(*ema).c_str():"(unset)"),
           (expand?std::to_string(*expand).c_str():"(unset)"),
           (loop?(*loop?"true":"false"):"(unset)"),
           (hw?hw->c_str():"(unset)"));
}

// map format string -> ("fmt" field, pixcfg byte). For "rgb565" without endian,
// we borrow endianness from the sink's LVGL (preferred_ddp_pixcfg()).
static inline std::pair<std::string,uint8_t>
resolve_fmt_and_pixcfg_(const std::string &fmt_in, ddp_stream::DdpStream *ddp) {
  std::string f = fmt_in;
  for (auto &c : f) c = (char)tolower((unsigned char)c);
  if (f == "rgb888")
    return { "rgb888", ddp_stream::DDP_PIXCFG_RGB888 };
  if (f == "rgb565le")
    return { "rgb565le", ddp_stream::DDP_PIXCFG_RGB565_LE };
  if (f == "rgb565be")
    return { "rgb565be", ddp_stream::DDP_PIXCFG_RGB565_BE };
  if (f == "rgb565") {
#if defined(LV_COLOR_16_SWAP) && LV_COLOR_16_SWAP
    return { "rgb565be", ddp_stream::DDP_PIXCFG_RGB565_BE };
#else
    return { "rgb565le", ddp_stream::DDP_PIXCFG_RGB565_LE };
#endif
  }
  // fallback
  return { "rgb888", ddp_stream::DDP_PIXCFG_RGB888 };
}

// ------------- trampoline -------------
// NOTE: This runs on ESP-IDF event task, NOT the main ESPHome thread.
// Most operations should be deferred to main thread via set_timeout().
void WsDdpControl::ws_event_trampoline(void *arg,
                                       esp_event_base_t /*base*/,
                                       int32_t event_id,
                                       void * /*event_data*/) {
  auto *self = static_cast<WsDdpControl *>(arg);
  if (!self) return;

  switch (event_id) {
    case WEBSOCKET_EVENT_CONNECTED:
      if (self->running_.load()) break;        // ignore duplicate CONNECTED
      ESP_LOGI(TAG, "connected");
      self->running_.store(true);
      // push all main-thread work off the event task
      self->set_timeout(0, [self]() {
        self->connecting_ = false;
        self->reset_backoff_();  // Reset reconnection backoff on successful connect
        self->send_hello_();
        // replay all active streams on main thread
        for (auto &kv : self->active_) if (kv.second) self->start_(kv.first);
      });
      break;

    case WEBSOCKET_EVENT_DISCONNECTED:
      ESP_LOGW(TAG, "disconnected");
      if (self->running_.load()) {  // Only handle if we were running
        self->running_.store(false);
        self->set_timeout(0, [self]() {
          self->connecting_ = false;
          self->schedule_reconnect_();
        });
      }
      break;

    case WEBSOCKET_EVENT_CLOSED:
      ESP_LOGW(TAG, "connection closed by server");
      if (self->running_.load()) {  // Only handle if we were running
        self->running_.store(false);
        self->set_timeout(0, [self]() {
          self->connecting_ = false;
          self->schedule_reconnect_();
        });
      }
      break;

    case WEBSOCKET_EVENT_ERROR:
      ESP_LOGE(TAG, "connection error");
      if (self->running_.load()) {  // Only handle if we were running
        self->running_.store(false);
        self->set_timeout(0, [self]() {
          self->connecting_ = false;
          self->schedule_reconnect_();
        });
      }
      break;

    default:
      break;
  }
}

// ------------- config/introspection -------------
void WsDdpControl::dump_config() {
  ESP_LOGCONFIG(TAG, "WebSocket control + orchestration:");
  if (!url_const_.empty() || (bool)url_fn_) {
    ESP_LOGCONFIG(TAG, "  url: (templated or static)");
  } else {
    ESP_LOGCONFIG(TAG, "  ws_host: %s", ws_host_const_.c_str());
    ESP_LOGCONFIG(TAG, "  ws_port: %d", ws_port_);
  }
  ESP_LOGCONFIG(TAG, "  device_id: (templated)");
  ESP_LOGCONFIG(TAG, "  outputs: %d", (int) outputs_.size());

  // Print each output with explicit/auto sizing and which optionals are set
  for (const auto &kv : outputs_) {
    uint8_t id = kv.first;
    const OutCfg &o = kv.second;
    ESP_LOGCONFIG(TAG, "  - out=%u size=%dx%d (%s)",
                  (unsigned) id, o.w, o.h,
                  (o.w > 0 && o.h > 0) ? "explicit" : "auto");
    auto str_or = [](const std::optional<std::string> &v){ return v ? v->c_str() : "(unset)"; };
    auto int_or = [](const std::optional<int> &v){ return v ? std::to_string(*v).c_str() : "(unset)"; };
    auto flt_or = [](const std::optional<float> &v){ return v ? std::to_string(*v).c_str() : "(unset)"; };
    auto boo_or = [](const std::optional<bool> &v){ return v ? (*v ? "true" : "false") : "(unset)"; };

    ESP_LOGCONFIG(TAG, "      src=%s pace=%s ema=%s expand=%s loop=%s hw=%s format=%s",
                  str_or(o.src_const),
                  int_or(o.pace),
                  flt_or(o.ema),
                  int_or(o.expand),
                  boo_or(o.loop),
                  str_or(o.hw_const),
                  str_or(o.format_const));
  }
}

std::string WsDdpControl::build_uri_() const {
  if (url_fn_) return url_fn_();
  if (!url_const_.empty()) return url_const_;
  const std::string host = ws_host_fn_ ? ws_host_fn_() : ws_host_const_;
  if (host.empty()) return {};
  char buf[256];
  snprintf(buf, sizeof(buf), "ws://%s:%d/control", host.c_str(), ws_port_);
  return std::string(buf);
}

// ------------- lifecycle -------------
void WsDdpControl::connect() {
  if (running_.load() || connecting_ || client_) return;   // idempotent
  if (!network::is_connected()) {
    ESP_LOGI(TAG, "network not ready; deferring connect with backoff");
    pending_connect_ = true;
    this->set_timeout(reconnect_backoff_ms_, [this](){ this->connect(); });
    reconnect_backoff_ms_ = std::min<uint32_t>(reconnect_backoff_ms_ * 2, 30000);
    return;
  }

  const std::string uri = this->build_uri_();
  if (uri.empty()) {
    ESP_LOGI(TAG, "URI not ready; retrying connect with backoff");
    pending_connect_ = true;
    this->set_timeout(reconnect_backoff_ms_, [this](){ this->connect(); });
    reconnect_backoff_ms_ = std::min<uint32_t>(reconnect_backoff_ms_ * 2, 30000);
    return;
  }

  pending_connect_ = false;
  connecting_ = true;
  this->do_connect_();
}

void WsDdpControl::do_connect_() {
  const std::string uri = this->build_uri_();
  if (uri.empty()) {
    ESP_LOGW(TAG, "connect(): no URI (provide url: or ws_host:/ws_port:)");
    connecting_ = false;
    return;
  }

  esp_websocket_client_config_t cfg{};
  cfg.uri = uri.c_str();
  cfg.network_timeout_ms   = 3000;
  cfg.reconnect_timeout_ms = 0;        // Disable ESP-IDF auto-reconnect, we handle it manually
  cfg.keep_alive_enable    = true;
  cfg.keep_alive_idle      = 10;
  cfg.keep_alive_interval  = 10;
  cfg.keep_alive_count     = 3;

  ESP_LOGI(TAG, "attempt uri=%s", uri.c_str());
  auto client = esp_websocket_client_init(&cfg);
  if (!client) {
    ESP_LOGE(TAG, "init failed");
    reconnect_backoff_ms_ = std::min<uint32_t>(reconnect_backoff_ms_ * 2, 30000);
    connecting_ = false;
    pending_connect_ = true;
    this->set_timeout(reconnect_backoff_ms_, [this](){ this->connect(); });
    return;
  }
  client_ = client;

  esp_err_t err = esp_websocket_register_events(
      (esp_websocket_client_handle_t) client_, WEBSOCKET_EVENT_ANY, WsDdpControl::ws_event_trampoline, this);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "register events failed: 0x%X", (unsigned) err);
    esp_websocket_client_destroy((esp_websocket_client_handle_t) client_);
    client_ = nullptr;
    reconnect_backoff_ms_ = std::min<uint32_t>(reconnect_backoff_ms_ * 2, 30000);
    connecting_ = false;
    pending_connect_ = true;
    this->set_timeout(reconnect_backoff_ms_, [this](){ this->connect(); });
    return;
  }

  err = esp_websocket_client_start((esp_websocket_client_handle_t) client_);
  if (err == ESP_OK) {
    ESP_LOGI(TAG, "started");
    // leave connecting_=true until CONNECTED arrives
  } else {
    ESP_LOGE(TAG, "start failed: 0x%X", (unsigned) err);
    esp_websocket_client_destroy((esp_websocket_client_handle_t) client_);
    client_ = nullptr;
    reconnect_backoff_ms_ = std::min<uint32_t>(reconnect_backoff_ms_ * 2, 30000);
    connecting_ = false;
    pending_connect_ = true;
    this->set_timeout(reconnect_backoff_ms_, [this](){ this->connect(); });
  }
}

void WsDdpControl::disconnect() {
  if (!client_) {
    running_.store(false);
    connecting_ = false;
    pending_connect_ = false;
    return;
  }
  ESP_LOGI(TAG, "disconnect()");
  esp_websocket_client_stop((esp_websocket_client_handle_t) client_);
  esp_websocket_client_destroy((esp_websocket_client_handle_t) client_);
  client_ = nullptr;
  running_.store(false);
  connecting_ = false;
  pending_connect_ = false;
}

void WsDdpControl::schedule_reconnect_() {
  if (connecting_ || pending_connect_) return;  // avoid duplicate reconnects

  // Use exponential backoff, capped at 30 seconds
  reconnect_backoff_ms_ = std::min<uint32_t>(reconnect_backoff_ms_ * 2, 30000);

  ESP_LOGI(TAG, "scheduling reconnect in %ums", (unsigned)reconnect_backoff_ms_);
  pending_connect_ = true;

  // Cleanup old client in background task to avoid blocking main thread
  // esp_websocket_client_stop() can block for seconds waiting for network operations
  void *old_client = client_;
  client_ = nullptr;  // Clear immediately to prevent reuse

  if (old_client) {
    // Create a cleanup task that runs on a separate thread
    TaskHandle_t cleanup_task;
    if (xTaskCreate(cleanup_websocket_task, "ws_cleanup", 2048, old_client, 5, &cleanup_task) != pdPASS) {
      ESP_LOGW(TAG, "failed to create cleanup task, will leak client handle");
    }
  }

  this->set_timeout(reconnect_backoff_ms_, [this]() {
    this->connect();
  });
}

void WsDdpControl::reset_backoff_() {
  // Reset backoff delay on successful connection
  reconnect_backoff_ms_ = 1000;
}

// ------------- protocol helpers -------------
void WsDdpControl::send_hello_() {
  if (!client_) return;
  const std::string dev = this->device_id_();
  char buf[192];
  snprintf(buf, sizeof(buf),
           "{\"type\":\"hello\",\"proto\":\"ddp-ws/1\",\"device_id\":\"%s\"}",
           dev.c_str());
  esp_websocket_client_send_text((esp_websocket_client_handle_t) client_, buf, strlen(buf), portMAX_DELAY);
  ESP_LOGI(TAG, "tx hello device_id=%s", dev.c_str());
}

void WsDdpControl::send_text(const char *json_utf8) {
  if (!client_ || !running_.load()) return;
  if (!json_utf8 || !*json_utf8) return;
  esp_websocket_client_send_text((esp_websocket_client_handle_t) client_, json_utf8, strlen(json_utf8), portMAX_DELAY);
}

// ------------- orchestration API -------------
void WsDdpControl::add_output_basic(uint8_t id, int w, int h) {
  OutCfg cfg; cfg.w = w; cfg.h = h; outputs_[id] = std::move(cfg);
}

void WsDdpControl::start(uint8_t out) {
  active_[out] = true;
  if (!this->is_connected()) {
    ESP_LOGD(TAG, "start: deferring out=%u until WS connects", (unsigned) out);
    if (!pending_connect_) this->connect();
    return;
  }
  this->start_(out);
}
void WsDdpControl::stop(uint8_t out) {
  active_[out] = false;
  this->stop_(out);
}

void WsDdpControl::set_src(uint8_t out, const std::string &s) {
  outputs_[out].src_const = s;
  if (active_[out] && this->is_connected()) this->send_update_(out);
}
void WsDdpControl::set_pace(uint8_t out, int pace) {
  outputs_[out].pace = pace;
  if (active_[out] && this->is_connected()) this->send_update_(out);
}
void WsDdpControl::set_ema(uint8_t out, float ema) {
  outputs_[out].ema = ema;
  if (active_[out] && this->is_connected()) this->send_update_(out);
}
void WsDdpControl::set_expand(uint8_t out, int expand) {
  outputs_[out].expand = expand;
  if (active_[out] && this->is_connected()) this->send_update_(out);
}
void WsDdpControl::set_loop(uint8_t out, bool loop) {
  outputs_[out].loop = loop;
  if (active_[out] && this->is_connected()) this->send_update_(out);
}
void WsDdpControl::set_hw(uint8_t out, const std::string &hw) {
  outputs_[out].hw_const = hw;
  if (active_[out] && this->is_connected()) this->send_update_(out);
}
void WsDdpControl::set_format(uint8_t out, const std::string &fmt) {
  outputs_[out].format_const = fmt;
  if (active_[out] && this->is_connected()) this->send_update_(out);
}

// --------- size/fmt/escape helpers ----------
std::string WsDdpControl::json_escape_(const std::string &s) {
  std::string o; o.reserve(s.size()+8);
  for (char c: s) {
    switch (c) {
      case '\"': o += "\\\""; break;
      case '\\': o += "\\\\"; break;
      case '\b': o += "\\b"; break;
      case '\f': o += "\\f"; break;
      case '\n': o += "\\n"; break;
      case '\r': o += "\\r"; break;
      case '\t': o += "\\t"; break;
      default:
        if ((unsigned char)c < 0x20) { char buf[7]; snprintf(buf, sizeof(buf), "\\u%04x", (unsigned char)c); o += buf; }
        else o += c;
    }
  }
  return o;
}

void WsDdpControl::resolve_size_(uint8_t out, int *w, int *h) const {
  int W = -1, H = -1;
  auto it = outputs_.find(out);
  if (it != outputs_.end()) {
    if (it->second.w > 0) W = it->second.w;
    if (it->second.h > 0) H = it->second.h;
  }
  if ((W <= 0 || H <= 0) && ddp_) {
    int ww=0, hh=0;
    if (ddp_->get_stream_size(out, &ww, &hh)) {
      if (W <= 0) W = ww;
      if (H <= 0) H = hh;
    }
  }
  if (w) *w = (W > 0 ? W : 0);
  if (h) *h = (H > 0 ? H : 0);
}

// Produce a fully-resolved config for a given output (YAML + overrides + ddp)
WsDdpControl::StreamCfg WsDdpControl::compute_stream_cfg_(uint8_t out) const {
  StreamCfg e{};

  // size + port
  this->resolve_size_(out, &e.w, &e.h);
  e.ddp_port = ddp_ ? ddp_->get_port() : 4048;

  // look up base cfg (may be absent if caller validates separately)
  auto oit = outputs_.find(out);
  const OutCfg *o = (oit != outputs_.end()) ? &oit->second : nullptr;

  // strings first (escaped); leave std::nullopt if unspecified
  if (o && o->src_const) e.src = json_escape_(*o->src_const);
  if (o && o->hw_const)  e.hw  = json_escape_(*o->hw_const);

  // numeric/toggles — propagate only if set in YAML
  if (o && o->pace)   e.pace   = *o->pace;
  if (o && o->ema)    e.ema    = *o->ema;
  if (o && o->expand) e.expand = *o->expand;
  if (o && o->loop)   e.loop   = *o->loop;

  // format → (fmt,pixcfg)
  if (o && o->format_const) {
    auto rp   = resolve_fmt_and_pixcfg_(*o->format_const, ddp_);
    e.fmt     = rp.first;
    e.pixcfg  = rp.second;
  }

  return e;
}

// Single place to build/log/send "start_stream" or "update"
void WsDdpControl::send_stream_(const char *type, uint8_t out) {
  if (!client_ || !running_.load()) return;
  auto oit = outputs_.find(out);
  if (oit == outputs_.end()) { ESP_LOGW(TAG, "%s: unknown out=%u", type, (unsigned) out); return; }

  StreamCfg e = this->compute_stream_cfg_(out);
  const std::string json = build_stream_json_(type, out, e.w, e.h, e.ddp_port,
                                              e.src, e.fmt, e.pixcfg,
                                              e.pace, e.ema, e.expand, e.loop, e.hw);
  log_stream_line_(type, out, e.w, e.h, e.ddp_port,
                   e.src, e.fmt, e.pixcfg, e.pace, e.ema, e.expand, e.loop, e.hw);
  this->send_text(json);
}

// ------------- control helpers -------------
void WsDdpControl::start_(uint8_t out) {
  // If size isn't known yet, defer briefly and retry.
  StreamCfg e = this->compute_stream_cfg_(out);
  if (e.w == 0 || e.h == 0) {
    ESP_LOGD(TAG, "start_stream out=%u deferred: size not ready (w=%d h=%d)",
             (unsigned) out, e.w, e.h);
    this->set_timeout(200, [this, out]() {
      // only retry if still active and connected
      if (this->active_[out] && this->is_connected()) this->start_(out);
    });
    return;
  }
  // Size is known: send the real start now.
  this->send_stream_("start_stream", out);
}

void WsDdpControl::send_update_(uint8_t out) {
  this->send_stream_("update", out);
}

void WsDdpControl::stop_(uint8_t out) {
  if (!client_ || !running_.load()) return;
  std::string json;
  json.reserve(48);
  json += "{\"type\":\"stop_stream\",\"out\":";
  json += std::to_string(out);
  json += "}";
  ESP_LOGI(TAG, "tx stop_stream out=%u", (unsigned) out);
  this->send_text(json);
}

}  // namespace ws_ddp_control
}  // namespace esphome
