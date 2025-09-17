// © Copyright 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT

#pragma once
#include "esphome/core/component.h"
#include "esphome/components/ddp_stream/ddp_stream.h"

#include <map>
#include <functional>
#include <string>
#include <optional>

extern "C" {
  #include "esp_event.h"
}

namespace esphome { namespace ws_ddp_control {

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
  bool is_connected() const { return running_; }
  void send_text(const char *json_utf8);
  void send_text(const std::string &json_utf8) { this->send_text(json_utf8.c_str()); }

  // Optional external hook
  void set_on_connected(std::function<void()> cb){ on_connected_ = std::move(cb); }

  // ------------- control API -------------
  void add_output_basic(uint8_t id, int w, int h);

  void start(uint8_t out);
  void stop(uint8_t out);

  void set_src(uint8_t out, const std::string &s);
  void set_pace(uint8_t out, int pace);
  void set_ema(uint8_t out, float ema);
  void set_expand(uint8_t out, int expand);
  void set_loop(uint8_t out, bool loop);
  void set_hw(uint8_t out, const std::string &hw);
  void set_format(uint8_t out, const std::string &fmt);

  // ESPHome
  void setup() override {}
  void dump_config() override;
  void loop() override {}

 protected:
  // ws event trampoline
  static void ws_event_trampoline(void *arg, esp_event_base_t base, int32_t event_id, void *event_data);

  void do_connect_();
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
    std::optional<std::string> src;
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
  bool running_{false};
  bool connecting_{false};      // prevent double connects
  bool pending_connect_{false};
  std::function<void()> on_connected_{};

  // outputs
  struct OutCfg {
    int w{-1}, h{-1};                 // -1 means "auto from canvas"
    std::optional<std::string> src_const;
    std::optional<int>         pace;
    std::optional<float>       ema;
    std::optional<int>         expand;        // 0/1/2
    std::optional<bool>        loop;
    std::optional<std::string> hw_const;      // "auto","none","cuda","qsv","vaapi","videotoolbox","d3d11va"
    std::optional<std::string> format_const;  // "rgb888","rgb565","rgb565le","rgb565be"
  };
  std::map<uint8_t, OutCfg> outputs_;
  std::map<uint8_t, bool>   active_;
};

}}  // namespace esphome::ws_ddp_control
