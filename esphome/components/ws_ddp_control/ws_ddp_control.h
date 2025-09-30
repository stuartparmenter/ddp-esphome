// © Copyright 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT

#pragma once
#include "esphome/core/component.h"
#include "esphome/core/automation.h"
#include "esphome/components/ddp_stream/ddp_stream.h"

#include <map>
#include <functional>
#include <string>
#include <optional>
#include <atomic>

extern "C" {
  #include "esp_event.h"
}

namespace esphome { namespace ws_ddp_control {

// Forward declarations
class WsDdpControl;

class WsDdpOutput : public Component {
 public:
  void set_ddp_stream_id(uint8_t ddp_stream_id) { ddp_stream_id_ = ddp_stream_id; }
  void set_parent(WsDdpControl* parent) { parent_ = parent; }

  // Per-output control API
  void start();
  void stop();
  void set_src(const std::string &src);
  void set_pace(int pace);
  void set_ema(float ema);
  void set_expand(int expand);
  void set_loop(bool loop);
  void set_hw(const std::string &hw);
  void set_format(const std::string &fmt);

  // ESPHome lifecycle
  void setup() override {}
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::LATE; }

 protected:
  uint8_t ddp_stream_id_{0};
  WsDdpControl* parent_{nullptr};

  // Output configuration
  std::string src_const_;
  std::optional<int> pace_;
  std::optional<float> ema_;
  std::optional<int> expand_;
  std::optional<bool> loop_;
  std::optional<std::string> hw_const_;
  std::optional<std::string> format_const_;

  friend class WsDdpControl;
};

class WsDdpControl : public Component {
 public:
  // ------------- config (templatable) -------------
  void set_ws_host(const std::string &h) { ws_host_const_ = h; }
  void set_ws_host(std::function<std::string()> fn) { ws_host_fn_ = std::move(fn); }
  void set_ws_port(int v){ ws_port_ = v; }

  void set_url(const std::string &url) { url_const_ = url; }
  void set_url(std::function<std::string()> fn) { url_fn_ = std::move(fn); }

  void set_device_id(const std::string &s){ dev_id_const_ = s; }
  void set_device_id(std::function<std::string()> fn){ dev_id_fn_ = std::move(fn); }

  // ------------- deps -------------
  void set_ddp(ddp_stream::DdpStream *ddp) { ddp_ = ddp; }

  // ------------- lifecycle & messaging -------------
  void connect();
  void disconnect();
  bool is_connected() const { return running_.load(); }
  void send_text(const char *json_utf8);
  void send_text(const std::string &json_utf8) { this->send_text(json_utf8.c_str()); }


  // ------------- shared websocket control API -------------
  void register_output(WsDdpOutput* output, uint8_t ddp_stream_id, int w, int h);
  void start_stream(WsDdpOutput* output);
  void stop_stream(WsDdpOutput* output);
  void send_update(WsDdpOutput* output);

  // ESPHome
  void setup() override {}
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::AFTER_CONNECTION; }

 protected:
  // ws event trampoline
  static void ws_event_trampoline(void *arg, esp_event_base_t base, int32_t event_id, void *event_data);

  void do_connect_();
  void schedule_reconnect_();
  void reset_backoff_();
  std::string build_uri_() const;
  std::string device_id_() const { return dev_id_fn_ ? dev_id_fn_() : dev_id_const_; }
  void send_hello_();

  // unified send for start/update
  void send_stream_(const char *type, uint8_t out);

  // control helpers
  void start_(uint8_t out);
  void send_update_(uint8_t out);
  void stop_(uint8_t out);

  static std::string json_escape_(const std::string &s);

  // resolve width/height: explicit override or auto from ddp canvas
  void resolve_size_(uint8_t out, int *w, int *h) const;


  // A fully resolved, ready-to-transmit stream config (YAML + runtime overrides + ddp info)
  struct StreamCfg {
    int w{0}, h{0};
    uint16_t ddp_port{4048};
    std::string src;
    std::optional<std::string> hw;
    std::optional<std::string> fmt;
    std::optional<uint8_t>     pixcfg;
    std::optional<int>         pace;
    std::optional<float>       ema;
    std::optional<int>         expand;   // 0=never,1=auto,2=force
    std::optional<bool>        loop;
  };
  StreamCfg compute_stream_cfg_(uint8_t out) const;

  // config (templatable)
  std::string ws_host_const_;
  std::function<std::string()> ws_host_fn_;
  int ws_port_{8788};

  std::string url_const_;
  std::function<std::string()> url_fn_;

  std::string dev_id_const_{"unknown"};
  std::function<std::string()> dev_id_fn_;

  // deps
  ddp_stream::DdpStream *ddp_{nullptr};

  // ws state
  void *client_{nullptr};
  std::atomic<bool> running_{false};    // Written from both main and event threads
  bool connecting_{false};              // Main-thread owned; event task never writes directly
  bool pending_connect_{false};         // Main-thread owned; event task never writes directly
  uint32_t reconnect_backoff_ms_{1000}; // Current backoff delay

  // outputs
  struct OutputInfo {
    WsDdpOutput* output{nullptr};
    int w{-1}, h{-1};                 // -1 means "auto from canvas"
    bool active{false};
  };
  std::map<uint8_t, OutputInfo> outputs_;
};

template<typename... Ts> class SetSrcAction : public Action<Ts...> {
 public:
  SetSrcAction(WsDdpOutput* output) : output_(output) {}

  TEMPLATABLE_VALUE(std::string, src)

  void play(Ts... x) override {
    std::string src = this->src_.value(x...);
    if (output_) {
      output_->set_src(src);
    }
  }

 protected:
  WsDdpOutput* output_;
};

}}  // namespace esphome::ws_ddp_control
