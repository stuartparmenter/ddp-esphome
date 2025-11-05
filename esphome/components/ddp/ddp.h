// © Copyright 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT

#pragma once

#include "esphome/core/component.h"
#include "ddp_renderer.h"

#ifdef USE_ESP_IDF
#include <mdns.h>
#endif

#include <map>
#include <set>
#include <vector>
#include <cstdint>
#include <atomic>

#include <lwip/sockets.h>
#include <lwip/inet.h>
#include <lwip/netdb.h>
#include <errno.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// DDP_METRICS is set via build flag in __init__.py if metrics: true

namespace esphome {
namespace ddp {

// DDP packet header (10 bytes)
#pragma pack(push, 1)
struct DdpHeader {
  uint8_t  flags;        // bit6=DDP(0x40), bit0=PUSH(0x01)
  uint8_t  seq;          // low 4 bits commonly used in DDP
  uint8_t  pixcfg;       // pixel format (see constants below)
  uint8_t  id;           // stream/destination id
  uint32_t offset_be;    // big-endian byte offset
  uint16_t length_be;    // big-endian payload length
};
#pragma pack(pop)

// Pixel config constants (DDP 'pixcfg' byte - byte 2 in header)
// Standard DDP data types (per DDP spec byte 2: bits C R TTT SSS)
// TTT=001 (RGB), SSS=011 (8-bit) = 0x0B
static constexpr uint8_t DDP_PIXCFG_RGB888     = 0x0B;
// Legacy/simplified RGB type (some senders use TTT=001 only)
static constexpr uint8_t DDP_PIXCFG_RGB_LEGACY = 0x01;
// Custom extension values for RGB565 format (16 bits per pixel: 5R+6G+5B)
static constexpr uint8_t DDP_PIXCFG_RGB565_BE  = 0x61;
static constexpr uint8_t DDP_PIXCFG_RGB565_LE  = 0x62;
// Standard DDP RGBW format (per DDP spec)
// TTT=011 (RGBW), SSS=011 (8-bit) = 0x1B
static constexpr uint8_t DDP_PIXCFG_RGBW       = 0x1B;

// Main DDP component - handles UDP reception, protocol parsing, and frame dispatch
class DdpComponent : public Component {
 public:
  // Configuration
  void set_port(uint16_t port) { port_ = port; }
  uint16_t get_port() const { return port_; }

  // Renderer registration (called by renderers during setup())
  void register_renderer(DdpRenderer* renderer);
  void unregister_renderer(DdpRenderer* renderer);

  // Query stream dimensions (for media_proxy_control)
  // Finds first renderer with known dimensions for the given stream ID
  // Returns false if no renderer found or dimensions unknown
  bool get_stream_dimensions(uint8_t stream_id, int* width, int* height) const;

  // ESPHome lifecycle
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::AFTER_CONNECTION; }

 protected:
  // Socket management
  void ensure_socket_();
  void open_socket_();
  void close_socket_();

  // Receive task - runs on dedicated FreeRTOS task for low-latency processing
  static void recv_task_trampoline(void* arg);
  void recv_task();

  // Packet handling - dispatches to registered renderers
  void handle_packet_(const uint8_t* buf, size_t len);
  void handle_push_(uint8_t stream_id, const DdpHeader* hdr);
  void handle_pixel_data_(uint8_t stream_id, const DdpHeader* hdr,
                          const uint8_t* payload, size_t len,
                          size_t bytes_per_pixel, PixelFormat format);

  // Helper to find renderers for a stream
  std::set<DdpRenderer*>* find_renderers_(uint8_t stream_id);

  // Network
  uint16_t port_{4048};
  int sock_{-1};
  TaskHandle_t task_{nullptr};
  std::atomic<bool> task_should_exit_{false};
  std::atomic<bool> task_stopping_{false};
  bool udp_opened_{false};

  // Renderer registry: stream_id → set<DdpRenderer*>
  std::map<uint8_t, std::set<DdpRenderer*>> renderers_;

  // mDNS service registration (uses ESP-IDF mDNS API directly)
  bool mdns_registered_{false};
  void register_mdns_service_();
  std::vector<std::string> txt_storage_;  // Persistent storage for mDNS TXT record strings

#if DDP_METRICS
  // Per-stream metrics
  struct StreamMetrics {
    // Totals
    uint64_t rx_pkts{0};
    uint64_t rx_bytes{0};         // DDP pixel payload bytes (header excluded)
    uint64_t rx_wire_bytes{0};    // UDP payload bytes (header included)

    // Frame assembly
    size_t frame_bytes_accum{0};
    size_t frame_px_accum{0};
    int64_t frame_first_pkt_us{0};
    bool frame_seen_any{false};

    // Frame accounting
    uint32_t frames_started{0};
    uint32_t frames_push{0};

    // EWMA
    double dispatch_lat_us_ewma{0.0};  // Time from first packet to dispatch
    double build_ms_ewma{0.0};          // Frame build time (first to last packet)

    // RX slice behavior
    uint32_t rx_wakeups{0};
    uint64_t rx_pkts_in_slices{0};

    // Windowed (2s) counters
    int64_t log_t0_us{0};
    uint32_t win_frames_push{0};
    uint64_t win_rx_bytes{0};        // payload only
    uint64_t win_rx_wire_bytes{0};   // UDP payload (wire)
    uint32_t win_pkt_gap{0};

    // Max inter-packet gap within a frame
    double intra_ms_max{0.0};
    int64_t last_pkt_us{0};
  };
  std::map<uint8_t, StreamMetrics> metrics_;
#endif

  // Loop optimization: disable when idle
  std::atomic<bool> loop_is_disabled_{false};
  int64_t last_activity_us_{0};
  static constexpr int64_t IDLE_TIMEOUT_US = 1'000'000;  // 1 second
};

}  // namespace ddp
}  // namespace esphome
