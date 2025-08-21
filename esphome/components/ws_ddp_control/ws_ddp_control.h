// © Copyright 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT

#pragma once
#include "esphome/core/component.h"
#include <functional>
#include <string>

namespace esphome { namespace ws_ddp_control {

class WsDdpControl : public Component {
 public:
  // existing
  void set_url(const std::string &url) { url_const_ = url; }
  void set_url(std::function<std::string()> fn) { url_fn_ = std::move(fn); }

  // new structured config
  void set_ws_host(const std::string &h) { ws_host_const_ = h; }
  void set_ws_host(std::function<std::string()> fn) { ws_host_fn_ = std::move(fn); }
  void set_ws_port(int v){ ws_port_ = v; }
  void set_width(int v){ w_ = v; }
  void set_height(int v){ h_ = v; }
  void set_out_id(int v){ out_ = v; }
  void set_src(const std::string &s){ src_const_ = s; }
  void set_src(std::function<std::string()> fn){ src_fn_ = std::move(fn); }
  void set_pace(int v){ pace_ = v; }
  void set_ema(float v){ ema_ = v; }
  void set_expand(int v){ expand_ = v; }      // 0=never,1=auto,2=force
  void set_loop(bool v){ loop_ = v; }
  void set_ddp_port(int v){ ddp_port_ = v; }
  void set_hw(const std::string &s){ hw_const_ = s; }
  void set_hw(std::function<std::string()> fn){ hw_fn_ = std::move(fn); }

  // lifecycle
  void start();
  void stop();
  void request_restart();

  void setup() override {}
  void dump_config() override;
  void loop() override {}

 protected:
  void do_start_();
  std::string build_url_() const;  // NEW

  // existing
  std::string url_const_;
  std::function<std::string()> url_fn_;
  std::string evaluated_;
  void *client_{nullptr};
  bool running_{false};
  bool pending_start_{false};

  // structured fields
  std::string ws_host_const_;
  std::function<std::string()> ws_host_fn_;
  int ws_port_{8788};
  int w_{64}, h_{64}, out_{1};
  std::string src_const_;
  std::function<std::string()> src_fn_;
  int pace_{0};
  float ema_{0.0f};
  int expand_{1};
  bool loop_{true};
  int ddp_port_{4048};
  std::string hw_const_{"auto"};
  std::function<std::string()> hw_fn_;
};

}} // ns
