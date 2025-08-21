// © Copyright 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT

#include "ws_ddp_control.h"

#include "esphome/core/log.h"
#include "esphome/components/network/util.h"

#include <algorithm>  // for std::min

extern "C" {
  #include "esp_websocket_client.h"
}

static const char *TAG = "ws_ddp_control";

namespace esphome {
namespace ws_ddp_control {

// ---------- WebSocket event trampoline ----------
static void on_ws_event(void *arg,
                        esp_event_base_t base,
                        int32_t event_id,
                        void *event_data) {
  auto *self = static_cast<WsDdpControl *>(arg);
  if (!self) return;

  switch (event_id) {
    case WEBSOCKET_EVENT_CONNECTED:
      ESP_LOGI(TAG, "connected");
      break;

    case WEBSOCKET_EVENT_DISCONNECTED:
      ESP_LOGW(TAG, "disconnected");
      break;

    case WEBSOCKET_EVENT_ERROR:
      ESP_LOGE(TAG, "error");
      break;

    default:
      break;
  }
}

// ---------- URL builder (from structured fields) ----------

std::string WsDdpControl::build_url_() const {
  const std::string host = ws_host_fn_ ? ws_host_fn_() : ws_host_const_;
  const std::string src  = src_fn_ ? src_fn_() : src_const_;
  const std::string hw   = hw_fn_ ? hw_fn_() : hw_const_;

  if (host.empty() || src.empty()) return std::string();

  char base[384];
  // Required params
  snprintf(base, sizeof(base),
           "ws://%s:%d/control?w=%d&h=%d&out=%d&src=%s&ddp_port=%d",
           host.c_str(), ws_port_, w_, h_, out_, src.c_str(), ddp_port_);
  std::string url(base);

  // Optional params (append only if set)
  if (pace_ > 0) {
    url += "&pace=" + std::to_string(pace_);
  }
  if (ema_ > 0.0f) {
    char buf[32];
    snprintf(buf, sizeof(buf), "&ema=%.3f", static_cast<double>(ema_));
    url += buf;
  }
  if (expand_ == 0) url += "&expand=0";
  else if (expand_ == 2) url += "&expand=2";
  url += (loop_ ? "&loop=1" : "&loop=0");

  if (!hw.empty() && hw != "auto") {
    url += "&hw=" + hw;
  }

  return url;
}

// ---------- ESPHome component lifecycle ----------

void WsDdpControl::dump_config() {
  ESP_LOGCONFIG(TAG, "WebSocket DDP Control:");
  const bool has_direct_url = !url_const_.empty() || (bool)url_fn_;
  if (has_direct_url) {
    ESP_LOGCONFIG(TAG, "  url: (templated or static)");
  } else {
    ESP_LOGCONFIG(TAG, "  ws_host: %s", ws_host_const_.c_str());
    ESP_LOGCONFIG(TAG, "  ws_port: %d", ws_port_);
    ESP_LOGCONFIG(TAG, "  w: %d h: %d out: %d", w_, h_, out_);
    ESP_LOGCONFIG(TAG, "  src: (templated)");
    ESP_LOGCONFIG(TAG, "  pace: %d", pace_);
    ESP_LOGCONFIG(TAG, "  ema: %.3f", (double)ema_);
    ESP_LOGCONFIG(TAG, "  expand: %d", expand_);
    ESP_LOGCONFIG(TAG, "  loop: %s", loop_ ? "true" : "false");
    ESP_LOGCONFIG(TAG, "  ddp_port: %d", ddp_port_);
    ESP_LOGCONFIG(TAG, "  hw: %s", hw_const_.c_str());
  }
}

void WsDdpControl::start() {
  if (running_) return;

  if (!network::is_connected()) {
    ESP_LOGI(TAG, "network not ready; deferring connect");
    pending_start_ = true;
    this->set_timeout(500, [this]() { this->start(); });
    return;
  }

  pending_start_ = false;
  this->do_start_();
}

void WsDdpControl::do_start_() {
  evaluated_.clear();

  // Choose URL (explicit url: takes precedence)
  if (url_fn_) {
    evaluated_ = url_fn_();
  } else if (!url_const_.empty()) {
    evaluated_ = url_const_;
  }
  if (evaluated_.empty()) {
    evaluated_ = build_url_();
  }
  if (evaluated_.empty()) {
    ESP_LOGW(TAG, "start(): no URL (provide url: or ws_* + src: fields)");
    return;
  }

  // Configure client
  esp_websocket_client_config_t cfg = {};
  cfg.uri = evaluated_.c_str();

  // Fail fast when unreachable so we don't starve other tasks
  cfg.network_timeout_ms   = 3000;   // connect/handshake timeout
  cfg.reconnect_timeout_ms = 2000;   // auto-reconnect interval (baseline)
  cfg.keep_alive_enable    = true;
  cfg.keep_alive_idle      = 10;
  cfg.keep_alive_interval  = 10;
  cfg.keep_alive_count     = 3;

  ESP_LOGI(TAG, "attempt url=%s", evaluated_.c_str());

  auto client = esp_websocket_client_init(&cfg);
  if (!client) {
    ESP_LOGE(TAG, "init failed");
    // Backoff and retry
    static uint32_t backoff_ms = 500;
    backoff_ms = std::min<uint32_t>(backoff_ms * 2, 10000);
    this->set_timeout(backoff_ms, [this]() { this->start(); });
    return;
  }
  client_ = client;

  esp_err_t err = esp_websocket_register_events(
      (esp_websocket_client_handle_t) client_, WEBSOCKET_EVENT_ANY, on_ws_event, this);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "register events failed: 0x%X", (unsigned) err);
    esp_websocket_client_destroy((esp_websocket_client_handle_t) client_);
    client_ = nullptr;
    // Backoff and retry
    static uint32_t backoff_ms = 500;
    backoff_ms = std::min<uint32_t>(backoff_ms * 2, 10000);
    this->set_timeout(backoff_ms, [this]() { this->start(); });
    return;
  }

  err = esp_websocket_client_start((esp_websocket_client_handle_t) client_);
  if (err == ESP_OK) {
    running_ = true;
    ESP_LOGI(TAG, "started url=%s", evaluated_.c_str());
    // Reset backoff on success (scoped statics reset each call anyway)
  } else {
    ESP_LOGE(TAG, "start failed: 0x%X", (unsigned) err);
    esp_websocket_client_destroy((esp_websocket_client_handle_t) client_);
    client_ = nullptr;

    // Exponential backoff up to 10s
    static uint32_t backoff_ms = 500;
    backoff_ms = std::min<uint32_t>(backoff_ms * 2, 10000);
    this->set_timeout(backoff_ms, [this]() { this->start(); });
  }
}

void WsDdpControl::stop() {
  if (!client_) {
    running_ = false;
    pending_start_ = false;
    return;
  }
  ESP_LOGI(TAG, "stopping");
  esp_websocket_client_stop((esp_websocket_client_handle_t) client_);
  esp_websocket_client_destroy((esp_websocket_client_handle_t) client_);
  client_ = nullptr;
  running_ = false;
  pending_start_ = false;
}

void WsDdpControl::request_restart() {
  this->stop();
  this->set_timeout(100, [this]() { this->start(); });
}

}  // namespace ws_ddp_control
}  // namespace esphome
