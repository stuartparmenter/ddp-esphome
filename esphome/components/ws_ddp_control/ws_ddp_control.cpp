// © Copyright 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT

#include "ws_ddp_control.h"
#include "esphome/core/log.h"
#include "esphome/components/network/util.h"
#include <cstring>
#include <algorithm>
#include <cstdio>

extern "C" {
  #include "esp_websocket_client.h"
  #include "freertos/FreeRTOS.h"
}

static const char *TAG = "ws_ddp_control";

namespace esphome {
namespace ws_ddp_control {

// ------------- trampoline -------------
void WsDdpControl::ws_event_trampoline(void *arg,
                                       esp_event_base_t /*base*/,
                                       int32_t event_id,
                                       void * /*event_data*/) {
  auto *self = static_cast<WsDdpControl *>(arg);
  if (!self) return;

  switch (event_id) {
    case WEBSOCKET_EVENT_CONNECTED:
      ESP_LOGI(TAG, "connected");
      self->running_ = true;
      self->send_hello_();
      // replay all active streams
      for (auto &kv : self->active_) if (kv.second) self->start_(kv.first);
      if (self->on_connected_) self->on_connected_();
      break;

    case WEBSOCKET_EVENT_DISCONNECTED:
      ESP_LOGW(TAG, "disconnected");
      self->running_ = false;
      break;

    case WEBSOCKET_EVENT_ERROR:
      ESP_LOGE(TAG, "error");
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
  if (running_) return;
  if (!network::is_connected()) {
    ESP_LOGI(TAG, "network not ready; deferring connect");
    pending_connect_ = true;
    this->set_timeout(500, [this](){ this->connect(); });
    return;
  }
  pending_connect_ = false;
  this->do_connect_();
}

void WsDdpControl::do_connect_() {
  const std::string uri = this->build_uri_();
  if (uri.empty()) {
    ESP_LOGW(TAG, "connect(): no URI (provide url: or ws_host:/ws_port:)");
    return;
  }

  esp_websocket_client_config_t cfg{};
  cfg.uri = uri.c_str();
  cfg.network_timeout_ms   = 3000;
  cfg.reconnect_timeout_ms = 2000;
  cfg.keep_alive_enable    = true;
  cfg.keep_alive_idle      = 10;
  cfg.keep_alive_interval  = 10;
  cfg.keep_alive_count     = 3;

  ESP_LOGI(TAG, "attempt uri=%s", uri.c_str());
  auto client = esp_websocket_client_init(&cfg);
  if (!client) {
    ESP_LOGE(TAG, "init failed");
    static uint32_t backoff_ms = 500;
    backoff_ms = std::min<uint32_t>(backoff_ms * 2, 10000);
    this->set_timeout(backoff_ms, [this](){ this->connect(); });
    return;
  }
  client_ = client;

  esp_err_t err = esp_websocket_register_events(
      (esp_websocket_client_handle_t) client_, WEBSOCKET_EVENT_ANY, WsDdpControl::ws_event_trampoline, this);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "register events failed: 0x%X", (unsigned) err);
    esp_websocket_client_destroy((esp_websocket_client_handle_t) client_);
    client_ = nullptr;
    static uint32_t backoff_ms = 500;
    backoff_ms = std::min<uint32_t>(backoff_ms * 2, 10000);
    this->set_timeout(backoff_ms, [this](){ this->connect(); });
    return;
  }

  err = esp_websocket_client_start((esp_websocket_client_handle_t) client_);
  if (err == ESP_OK) {
    ESP_LOGI(TAG, "started");
  } else {
    ESP_LOGE(TAG, "start failed: 0x%X", (unsigned) err);
    esp_websocket_client_destroy((esp_websocket_client_handle_t) client_);
    client_ = nullptr;
    static uint32_t backoff_ms = 500;
    backoff_ms = std::min<uint32_t>(backoff_ms * 2, 10000);
    this->set_timeout(backoff_ms, [this](){ this->connect(); });
  }
}

void WsDdpControl::disconnect() {
  if (!client_) {
    running_ = false;
    pending_connect_ = false;
    return;
  }
  ESP_LOGI(TAG, "disconnect()");
  esp_websocket_client_stop((esp_websocket_client_handle_t) client_);
  esp_websocket_client_destroy((esp_websocket_client_handle_t) client_);
  client_ = nullptr;
  running_ = false;
  pending_connect_ = false;
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
  if (!client_ || !running_) return;
  if (!json_utf8 || !*json_utf8) return;
  esp_websocket_client_send_text((esp_websocket_client_handle_t) client_, json_utf8, strlen(json_utf8), portMAX_DELAY);
}

// ------------- orchestration API -------------
void WsDdpControl::add_output(uint8_t id, int w, int h, const std::string &src_const,
                              int pace, float ema, int expand, bool loop, const std::string &hw_const) {
  OutCfg cfg;
  cfg.w = w; cfg.h = h;
  cfg.src_const = src_const;
  cfg.pace = pace; cfg.ema = ema; cfg.expand = expand; cfg.loop = loop;
  cfg.hw_const = hw_const;
  outputs_[id] = std::move(cfg);
}

void WsDdpControl::start(uint8_t out) {
  active_[out] = true;
  if (!this->is_connected()) {
    ESP_LOGD(TAG, "start: deferring out=%u until WS connects", (unsigned) out);
    return;
  }
  this->start_(out);
}
void WsDdpControl::stop(uint8_t out) {
  active_[out] = false;
  this->stop_(out);
}

void WsDdpControl::set_src(uint8_t out, const std::string &s) {
  shadow_src_[out] = s;
  if (active_[out] && this->is_connected()) this->send_update_(out);
}
void WsDdpControl::set_pace(uint8_t out, int pace) {
  shadow_pace_[out] = pace;
  if (active_[out] && this->is_connected()) this->send_update_(out);
}
void WsDdpControl::set_ema(uint8_t out, float ema) {
  shadow_ema_[out] = ema;
  if (active_[out] && this->is_connected()) this->send_update_(out);
}
void WsDdpControl::set_expand(uint8_t out, int expand) {
  shadow_expand_[out] = expand;
  if (active_[out] && this->is_connected()) this->send_update_(out);
}
void WsDdpControl::set_loop(uint8_t out, bool loop) {
  shadow_loop_[out] = loop;
  if (active_[out] && this->is_connected()) this->send_update_(out);
}
void WsDdpControl::set_hw(uint8_t out, const std::string &hw) {
  shadow_hw_[out] = hw;
  if (active_[out] && this->is_connected()) this->send_update_(out);
}

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

void WsDdpControl::start_(uint8_t out) {
  if (!client_ || !running_) return;

  auto oit = outputs_.find(out);
  if (oit == outputs_.end()) { ESP_LOGW(TAG, "start: unknown out=%u", out); return; }
  const auto &o = oit->second;

  int w=0, h=0;
  this->resolve_size_(out, &w, &h);

  std::string src = shadow_src_.count(out) ? shadow_src_.at(out) : o.src_const;
  std::string hw  = shadow_hw_.count(out)  ? shadow_hw_.at(out)  : o.hw_const;

  int   pace   = shadow_pace_.count(out)   ? shadow_pace_.at(out)   : o.pace;
  float ema    = shadow_ema_.count(out)    ? shadow_ema_.at(out)    : o.ema;
  int   expand = shadow_expand_.count(out) ? shadow_expand_.at(out) : o.expand;
  bool  loop   = shadow_loop_.count(out)   ? shadow_loop_.at(out)   : o.loop;

  std::string json;
  json.reserve(256);
  json += "{";
  append_json_str(json, "type", "start_stream"); json += ",";
  append_json_int(json, "out", out);             json += ",";
  append_json_int(json, "w", w);                 json += ",";
  append_json_int(json, "h", h);                 json += ",";
  append_json_str(json, "src", json_escape_(src)); json += ",";
  // ddp port comes from the sink
  uint16_t ddp_port = ddp_ ? ddp_->get_port() : 4048;
  append_json_int(json, "ddp_port", ddp_port);   json += ",";
  append_json_int(json, "pace", pace);           json += ",";
  append_json_float(json, "ema", ema);           json += ",";
  append_json_int(json, "expand", expand);       json += ",";
  append_json_bool(json, "loop", loop);          json += ",";
  append_json_str(json, "hw", json_escape_(hw));
  json += "}";

  ESP_LOGI(TAG, "tx start_stream out=%u size=%dx%d src=%s ddp_port=%u pace=%d ema=%.3f expand=%d loop=%s hw=%s",
           (unsigned) out, w, h, src.c_str(), (unsigned) ddp_port, pace, (double) ema, expand, loop ? "true":"false", hw.c_str());
  this->send_text(json);
}

void WsDdpControl::send_update_(uint8_t out) {
  if (!client_ || !running_) return;

  auto oit = outputs_.find(out);
  if (oit == outputs_.end()) { ESP_LOGW(TAG, "update: unknown out=%u", out); return; }
  const auto &o = oit->second;

  std::string src = shadow_src_.count(out) ? shadow_src_.at(out) : o.src_const;
  std::string hw  = shadow_hw_.count(out)  ? shadow_hw_.at(out)  : o.hw_const;

  int   pace   = shadow_pace_.count(out)   ? shadow_pace_.at(out)   : o.pace;
  float ema    = shadow_ema_.count(out)    ? shadow_ema_.at(out)    : o.ema;
  int   expand = shadow_expand_.count(out) ? shadow_expand_.at(out) : o.expand;
  bool  loop   = shadow_loop_.count(out)   ? shadow_loop_.at(out)   : o.loop;

  std::string json;
  json.reserve(256);
  json += "{";
  append_json_str(json, "type", "update"); json += ",";
  append_json_int(json, "out", out);       json += ",";
  append_json_str(json, "src", json_escape_(src)); json += ",";
  append_json_int(json, "pace", pace);     json += ",";
  append_json_float(json, "ema", ema);     json += ",";
  append_json_int(json, "expand", expand); json += ",";
  append_json_bool(json, "loop", loop);    json += ",";
  append_json_str(json, "hw", json_escape_(hw));
  json += "}";

  ESP_LOGI(TAG, "tx update out=%u src=%s pace=%d ema=%.3f expand=%d loop=%s hw=%s",
           (unsigned) out, src.c_str(), pace, (double) ema, expand, loop ? "true":"false", hw.c_str());
  this->send_text(json);
}

void WsDdpControl::stop_(uint8_t out) {
  if (!client_ || !running_) return;
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
