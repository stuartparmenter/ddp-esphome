// © Copyright 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT

#pragma once

#include "esphome/core/automation.h"
#include "media_proxy_control.h"

namespace esphome {
namespace media_proxy_control {

template<typename... Ts> class SetSourceAction : public Action<Ts...> {
 public:
  SetSourceAction(MediaProxyOutput* output) : output_(output) {}

  TEMPLATABLE_VALUE(std::string, src)

  void play(Ts... x) override {
    std::string src = this->src_.value(x...);
    if (output_) {
      output_->set_source(src);
    }
  }

 protected:
  MediaProxyOutput* output_;
};

template<typename... Ts> class StartAction : public Action<Ts...> {
 public:
  StartAction(MediaProxyOutput* output) : output_(output) {}

  void play(Ts... x) override {
    if (output_) {
      output_->start();
    }
  }

 protected:
  MediaProxyOutput* output_;
};

template<typename... Ts> class StopAction : public Action<Ts...> {
 public:
  StopAction(MediaProxyOutput* output) : output_(output) {}

  void play(Ts... x) override {
    if (output_) {
      output_->stop();
    }
  }

 protected:
  MediaProxyOutput* output_;
};

}  // namespace media_proxy_control
}  // namespace esphome
