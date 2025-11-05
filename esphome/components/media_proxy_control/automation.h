// © Copyright 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT

#pragma once

#include "esphome/core/automation.h"
#include "media_proxy_control.h"
#include "esphome/core/version.h"

namespace esphome {
namespace media_proxy_control {

template<typename... Ts> class SetSourceAction : public Action<Ts...> {
 public:
  SetSourceAction(MediaProxyOutput* output) : output_(output) {}

  TEMPLATABLE_VALUE(std::string, src)

#if ESPHOME_VERSION_CODE >= VERSION_CODE(2025, 11, 0)
  void play(const Ts&... x) override {
    std::string src = this->src_.value(x...);
    if (output_) {
      output_->set_src(src);
    }
  }
#else
  void play(Ts... x) override {
    std::string src = this->src_.value(x...);
    if (output_) {
      output_->set_src(src);
    }
  }
#endif

 protected:
  MediaProxyOutput* output_;
};

template<typename... Ts> class StartAction : public Action<Ts...> {
 public:
  StartAction(MediaProxyOutput* output) : output_(output) {}

#if ESPHOME_VERSION_CODE >= VERSION_CODE(2025, 11, 0)
  void play(const Ts&... x) override {
    if (output_) {
      output_->start();
    }
  }
#else
  void play(Ts... x) override {
    if (output_) {
      output_->start();
    }
  }
#endif

 protected:
  MediaProxyOutput* output_;
};

template<typename... Ts> class StopAction : public Action<Ts...> {
 public:
  StopAction(MediaProxyOutput* output) : output_(output) {}

#if ESPHOME_VERSION_CODE >= VERSION_CODE(2025, 11, 0)
  void play(const Ts&... x) override {
    if (output_) {
      output_->stop();
    }
  }
#else
  void play(Ts... x) override {
    if (output_) {
      output_->stop();
    }
  }
#endif

 protected:
  MediaProxyOutput* output_;
};

}  // namespace media_proxy_control
}  // namespace esphome
