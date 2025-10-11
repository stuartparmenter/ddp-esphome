// © Copyright 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT

#pragma once

#include "esphome/core/component.h"
#include "esphome/components/ddp/ddp.h"

#include <map>
#include <string>
#include <optional>
#include <atomic>
#include <functional>

extern "C" {
  #include "esp_event.h"
  #include "esp_websocket_client.h"
}

namespace esphome {
namespace media_proxy_control {

// Forward declaration
class MediaProxyControl;

// Per-output configuration
class MediaProxyOutput : public Component {
 public:
  void set_stream_id(uint8_t id) { stream_id_ = id; }
  void set_parent(MediaProxyControl* parent) { parent_ = parent; }

  // Configuration setters
  void set_src(const std::string& src);
  void set_pace(int pace);
  void set_ema(float ema);
  void set_expand(int expand);
  void set_loop(bool loop);
  void set_hw(const std::string& hw);
  void set_format(const std::string& fmt);
  void set_fit(const std::string& fit);

  // Actions (for automations)
  void start();
  void stop();

  // ESPHome lifecycle
  void setup() override {}
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::LATE; }

 protected:
  uint8_t stream_id_{0};
  MediaProxyControl* parent_{nullptr};

  // Output configuration
  std::string src_;
  std::optional<int> pace_;
  std::optional<float> ema_;
  std::optional<int> expand_;
  std::optional<bool> loop_;
  std::optional<std::string> hw_;
  std::optional<std::string> format_;
  std::optional<std::string> fit_;

  friend class MediaProxyControl;
};

// Main WebSocket control component
class MediaProxyControl : public Component {
 public:
  // Configuration (templatable)
  void set_ws_host(const std::string& h) { ws_host_ = h; }
  void set_ws_host(std::function<std::string()> fn) { ws_host_fn_ = std::move(fn); }
  void set_ws_port(int port) { ws_port_ = port; }

  void set_url(const std::string& url) { url_ = url; }
  void set_url(std::function<std::string()> fn) { url_fn_ = std::move(fn); }

  void set_device_id(const std::string& id) { dev_id_ = id; }
  void set_device_id(std::function<std::string()> fn) { dev_id_fn_ = std::move(fn); }

  // Dependencies
  void set_ddp(ddp::DdpComponent* ddp) { ddp_ = ddp; }

  // Connection control
  void connect();
  void disconnect();
  bool is_connected() const { return running_.load(); }
  void send_text(const std::string& json_utf8);

  // Output registration
  void register_output(MediaProxyOutput* output, uint8_t stream_id);

  // Stream control (called by MediaProxyOutput)
  void start_stream(uint8_t stream_id);
  void stop_stream(uint8_t stream_id);
  void send_update(uint8_t stream_id);

  // ESPHome lifecycle
  void setup() override {}
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::AFTER_CONNECTION; }

 protected:
  // WebSocket event handling
  static void ws_event_trampoline(void* arg, esp_event_base_t base,
                                   int32_t event_id, void* event_data);

  // Connection management
  void do_connect_();
  void schedule_reconnect_();
  void reset_backoff_();
  std::string build_uri_() const;
  std::string device_id_() const { return dev_id_fn_ ? dev_id_fn_() : dev_id_; }

  // Message sending
  void send_hello_();
  void send_stream_(const char* type, uint8_t stream_id);

  // Helpers
  static std::string json_escape_(const std::string& s);
  void resolve_size_(uint8_t stream_id, int* w, int* h) const;

  // Stream configuration
  struct StreamConfig {
    int w{0}, h{0};
    uint16_t ddp_port{4048};
    std::string src;
    std::optional<std::string> hw;
    std::optional<std::string> fmt;
    std::optional<uint8_t> pixcfg;
    std::optional<int> pace;
    std::optional<float> ema;
    std::optional<int> expand;
    std::optional<bool> loop;
    std::optional<std::string> fit;
  };
  StreamConfig compute_stream_config_(uint8_t stream_id) const;

  // Configuration
  std::string ws_host_;
  std::function<std::string()> ws_host_fn_;
  int ws_port_{8788};

  std::string url_;
  std::function<std::string()> url_fn_;

  std::string dev_id_;
  std::function<std::string()> dev_id_fn_;

  // Dependencies
  ddp::DdpComponent* ddp_{nullptr};

  // Output tracking: stream_id → output pointer + active state
  struct OutputInfo {
    MediaProxyOutput* output{nullptr};
    bool active{false};  // whether stream should be started/running
  };
  std::map<uint8_t, OutputInfo> outputs_;

  // WebSocket state
  esp_websocket_client_handle_t client_{nullptr};
  std::atomic<bool> running_{false};
  std::atomic<bool> connected_{false};
  bool connecting_{false};
  bool pending_connect_{false};

  // Reconnection backoff
  uint32_t reconnect_delay_ms_{1000};
  static constexpr uint32_t MAX_RECONNECT_DELAY_MS = 30000;
};

}  // namespace media_proxy_control
}  // namespace esphome
