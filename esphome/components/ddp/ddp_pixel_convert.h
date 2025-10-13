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

// RGB to RGBW white channel conversion modes (internal use only)
enum class RGBWMode : uint8_t {
  NONE = 0,      // RGB only, W=0 (for RGB strips)
  ACCURATE = 1,  // Extract white, reduce RGB (for RGBW strips)
};

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

// Convert RGB888 to RGBW with configurable white channel mode
// Safe to call from UDP task (pure math, no allocations or API calls)
// Inline so it can be used across components without linkage issues
//
// Modes:
// - NONE: RGB passthrough, W=0 (for RGB-only strips)
// - ACCURATE: Extract white and reduce RGB (for RGBW strips, accurate color)
//   - Pure colors stay pure: (255,0,0) → R=255, G=0, B=0, W=0
//   - White is efficient: (255,255,255) → R=0, G=0, B=0, W=255
//   - Mixed colors accurate: (255,200,200) → R=55, G=0, B=0, W=200 (pink = red + white)
//
// dst: Destination buffer for RGBW pixels (must be pre-allocated, 4 bytes per pixel)
// src: Source RGB888 data (3 bytes per pixel: R, G, B)
// pixel_count: Number of pixels to convert
// mode: White channel conversion mode
inline void convert_rgb888_to_rgbw(uint8_t* dst, const uint8_t* src,
                                    size_t pixel_count, RGBWMode mode) {
  if (!dst || !src) return;

  const uint8_t* sp = src;

  if (mode == RGBWMode::ACCURATE) {
    // Extract white and reduce RGB (accurate color)
    for (size_t i = 0; i < pixel_count; ++i) {
      uint8_t r = sp[0];
      uint8_t g = sp[1];
      uint8_t b = sp[2];

      // Extract common white component (minimum of RGB)
      uint8_t w = r < g ? (r < b ? r : b) : (g < b ? g : b);

      // Store chromatic residuals + white
      dst[i * 4 + 0] = r - w;  // R residual (chromatic)
      dst[i * 4 + 1] = g - w;  // G residual (chromatic)
      dst[i * 4 + 2] = b - w;  // B residual (chromatic)
      dst[i * 4 + 3] = w;      // W = common brightness
      sp += 3;
    }
  } else {  // RGBWMode::NONE
    // RGB passthrough, W=0
    for (size_t i = 0; i < pixel_count; ++i) {
      dst[i * 4 + 0] = sp[0];  // R
      dst[i * 4 + 1] = sp[1];  // G
      dst[i * 4 + 2] = sp[2];  // B
      dst[i * 4 + 3] = 0;      // W = 0
      sp += 3;
    }
  }
}

// Convert RGB565 to RGBW with configurable white channel mode
// Safe to call from UDP task (pure math, no allocations or API calls)
// Inline so it can be used across components without linkage issues
//
// Algorithm: Expand RGB565 to RGB888, then apply white channel mode
//
// dst: Destination buffer for RGBW pixels (must be pre-allocated, 4 bytes per pixel)
// src: Source RGB565 data (2 bytes per pixel)
// pixel_count: Number of pixels to convert
// src_big_endian: If true, source is RGB565 big-endian; if false, little-endian
// mode: White channel conversion mode
inline void convert_rgb565_to_rgbw(uint8_t* dst, const uint8_t* src,
                                    size_t pixel_count, bool src_big_endian,
                                    RGBWMode mode) {
  if (!dst || !src) return;

  const uint8_t* sp = src;

  for (size_t i = 0; i < pixel_count; ++i) {
    // Extract RGB565 value (handle endianness)
    uint16_t v = src_big_endian ? (uint16_t)((sp[0] << 8) | sp[1])
                                : (uint16_t)((sp[1] << 8) | sp[0]);

    // Extract 5/6/5 bit components
    uint8_t r5 = (uint8_t)((v >> 11) & 0x1F);
    uint8_t g6 = (uint8_t)((v >> 5)  & 0x3F);
    uint8_t b5 = (uint8_t)( v        & 0x1F);

    // Expand to 8-bit (scale + replicate MSBs into LSBs)
    uint8_t r = (uint8_t)((r5 << 3) | (r5 >> 2));
    uint8_t g = (uint8_t)((g6 << 2) | (g6 >> 4));
    uint8_t b = (uint8_t)((b5 << 3) | (b5 >> 2));

    if (mode == RGBWMode::ACCURATE) {
      // Extract white and reduce RGB
      uint8_t w = r < g ? (r < b ? r : b) : (g < b ? g : b);
      dst[i * 4 + 0] = r - w;  // R residual (chromatic)
      dst[i * 4 + 1] = g - w;  // G residual (chromatic)
      dst[i * 4 + 2] = b - w;  // B residual (chromatic)
      dst[i * 4 + 3] = w;      // W = common brightness
    } else {  // RGBWMode::NONE
      // RGB passthrough, W=0
      dst[i * 4 + 0] = r;  // R
      dst[i * 4 + 1] = g;  // G
      dst[i * 4 + 2] = b;  // B
      dst[i * 4 + 3] = 0;  // W = 0
    }
    sp += 2;
  }
}

}  // namespace ddp
}  // namespace esphome
