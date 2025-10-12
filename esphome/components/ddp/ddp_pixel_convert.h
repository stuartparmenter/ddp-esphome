// © Copyright 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT
//
// Shared pixel format conversion utilities for DDP renderers
// These functions are designed to be called from the UDP receive task
// (pure math operations, no LVGL/ESPHome API calls, no allocations)

#pragma once

#include <cstdint>
#include <cstddef>

namespace esphome {
namespace ddp {

// RGB565 conversion lookup tables (precomputed inline constexpr arrays)
// Used for fast RGB888 → RGB565 conversion
// These are fully declared in the header so any component can include them

// Helper to generate LUT at compile time
namespace detail {
  template<size_t N>
  struct RGB565_LUT_Generator {
    uint16_t r5[256];
    uint16_t g6[256];
    uint16_t b5[256];

    constexpr RGB565_LUT_Generator() : r5{}, g6{}, b5{} {
      for (int i = 0; i < 256; ++i) {
        r5[i] = (uint16_t)((i & 0xF8) << 8);  // 5 bits red, shifted to MSB
        g6[i] = (uint16_t)((i & 0xFC) << 3);  // 6 bits green, middle
        b5[i] = (uint16_t)(i >> 3);            // 5 bits blue, LSB
      }
    }
  };
}

inline constexpr detail::RGB565_LUT_Generator<256> RGB565_LUTS{};
inline constexpr const uint16_t* LUT_R5 = RGB565_LUTS.r5;
inline constexpr const uint16_t* LUT_G6 = RGB565_LUTS.g6;
inline constexpr const uint16_t* LUT_B5 = RGB565_LUTS.b5;

// Convert RGB888 to RGB565 with optional byte swap
// Safe to call from UDP task (pure math, no allocations or API calls)
// Inline so it can be used across components without linkage issues
//
// dst: Destination buffer for RGB565 pixels (must be pre-allocated)
// src: Source RGB888 data (3 bytes per pixel: R, G, B)
// pixel_count: Number of pixels to convert
// swap_bytes: If true, swap byte order (for LV_COLOR_16_SWAP compatibility)
inline void convert_rgb888_to_rgb565(uint16_t* dst, const uint8_t* src,
                                      size_t pixel_count, bool swap_bytes) {
  if (!dst || !src) return;

  const uint8_t* sp = src;

  if (swap_bytes) {
    // Convert and swap bytes for LV_COLOR_16_SWAP
    for (size_t i = 0; i < pixel_count; ++i) {
      uint16_t c = LUT_R5[sp[0]] | LUT_G6[sp[1]] | LUT_B5[sp[2]];
      dst[i] = (uint16_t)((c >> 8) | (c << 8));  // Swap bytes
      sp += 3;
    }
  } else {
    // Convert without swap
    for (size_t i = 0; i < pixel_count; ++i) {
      dst[i] = (uint16_t)(LUT_R5[sp[0]] | LUT_G6[sp[1]] | LUT_B5[sp[2]]);
      sp += 3;
    }
  }
}

// Swap RGB565 byte order in-place
// Safe to call from UDP task (pure math, no allocations or API calls)
// Inline so it can be used across components without linkage issues
//
// buf: RGB565 buffer (modified in-place)
// pixel_count: Number of pixels
inline void swap_rgb565_bytes(uint16_t* buf, size_t pixel_count) {
  if (!buf) return;

  for (size_t i = 0; i < pixel_count; ++i) {
    uint16_t c = buf[i];
    buf[i] = (uint16_t)((c >> 8) | (c << 8));
  }
}

// Convert RGB888 to RGB32 (LVGL 32-bit mode)
// Safe to call from UDP task (pure math, no allocations or API calls)
// Inline so it can be used across components without linkage issues
//
// dst: Destination buffer for RGB32 pixels (must be pre-allocated)
// src: Source RGB888 data (3 bytes per pixel: R, G, B)
// pixel_count: Number of pixels to convert
inline void convert_rgb888_to_rgb32(uint32_t* dst, const uint8_t* src,
                                     size_t pixel_count) {
  if (!dst || !src) return;

  const uint8_t* sp = src;

  for (size_t i = 0; i < pixel_count; ++i) {
    // LVGL RGB32 format: R, G, B, A (alpha = 0xFF)
    uint32_t c = ((uint32_t)sp[0]) |           // Red
                 ((uint32_t)sp[1] << 8) |      // Green
                 ((uint32_t)sp[2] << 16) |     // Blue
                 0xFF000000;                   // Alpha
    dst[i] = c;
    sp += 3;
  }
}

// Convert RGBW to RGB565 (drop W channel) with optional byte swap
// Safe to call from UDP task (pure math, no allocations or API calls)
// Inline so it can be used across components without linkage issues
//
// dst: Destination buffer for RGB565 pixels (must be pre-allocated)
// src: Source RGBW data (4 bytes per pixel: R, G, B, W)
// pixel_count: Number of pixels to convert
// swap_bytes: If true, swap byte order (for LV_COLOR_16_SWAP compatibility)
inline void convert_rgbw_to_rgb565(uint16_t* dst, const uint8_t* src,
                                    size_t pixel_count, bool swap_bytes) {
  if (!dst || !src) return;

  const uint8_t* sp = src;

  if (swap_bytes) {
    // Convert and swap bytes for LV_COLOR_16_SWAP
    for (size_t i = 0; i < pixel_count; ++i) {
      uint16_t c = LUT_R5[sp[0]] | LUT_G6[sp[1]] | LUT_B5[sp[2]];
      dst[i] = (uint16_t)((c >> 8) | (c << 8));  // Swap bytes
      sp += 4;  // Skip W channel
    }
  } else {
    // Convert without swap
    for (size_t i = 0; i < pixel_count; ++i) {
      dst[i] = (uint16_t)(LUT_R5[sp[0]] | LUT_G6[sp[1]] | LUT_B5[sp[2]]);
      sp += 4;  // Skip W channel
    }
  }
}

// Convert RGBW to RGB32 (drop W channel, use 0xFF for alpha)
// Safe to call from UDP task (pure math, no allocations or API calls)
// Inline so it can be used across components without linkage issues
//
// dst: Destination buffer for RGB32 pixels (must be pre-allocated)
// src: Source RGBW data (4 bytes per pixel: R, G, B, W)
// pixel_count: Number of pixels to convert
inline void convert_rgbw_to_rgb32(uint32_t* dst, const uint8_t* src,
                                   size_t pixel_count) {
  if (!dst || !src) return;

  const uint8_t* sp = src;

  for (size_t i = 0; i < pixel_count; ++i) {
    // LVGL RGB32 format: R, G, B, A (alpha = 0xFF, W ignored)
    uint32_t c = ((uint32_t)sp[0]) |           // Red
                 ((uint32_t)sp[1] << 8) |      // Green
                 ((uint32_t)sp[2] << 16) |     // Blue
                 0xFF000000;                   // Alpha (sp[3] = W ignored)
    dst[i] = c;
    sp += 4;
  }
}

}  // namespace ddp
}  // namespace esphome
