// © Copyright 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT

#pragma once
#include "esphome/core/component.h"
#include "esphome/components/ddp_stream/ddp_stream.h"

#include <map>
#include <functional>
#include <string>

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
  void add_output(uint8_t id, int w, int h, const std::string &src_const,
                  int pace, float ema, int expand, bool loop, const std::string &hw_const);

  void start(uint8_t out);
  void stop(uint8_t out);

  void set_src(uint8_t out, const std::string &s);
  void set_pace(uint8_t out, int pace);
  void set_ema(uint8_t out, float ema);
  void set_expand(uint8_t out, int expand);
  void set_loop(uint8_t out, bool loop);
  void set_hw(uint8_t out, const std::string &hw);

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

  // control helpers
  void start_(uint8_t out);
  void send_update_(uint8_t out);
  void stop_(uint8_t out);

  static std::string json_escape_(const std::string &s);

  // resolve width/height: explicit override or auto from ddp canvas
  void resolve_size_(uint8_t out, int *w, int *h) const;

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
  bool pending_connect_{false};
  std::function<void()> on_connected_{};

  // outputs
  struct OutCfg {
    int w{-1}, h{-1};                 // -1 means "auto from canvas"
    std::string src_const{""};
    int pace{0};
    float ema{0.0f};
    int expand{1};
    bool loop{true};
    std::string hw_const{"auto"};
  };
  std::map<uint8_t, OutCfg> outputs_;
  std::map<uint8_t, bool>   active_;
  // shadow overrides (runtime)
  std::map<uint8_t, std::string> shadow_src_;
  std::map<uint8_t, std::string> shadow_hw_;
  std::map<uint8_t, int>         shadow_pace_;
  std::map<uint8_t, float>       shadow_ema_;
  std::map<uint8_t, int>         shadow_expand_;
  std::map<uint8_t, bool>        shadow_loop_;
};

}}  // namespace esphome::ws_ddp_control
