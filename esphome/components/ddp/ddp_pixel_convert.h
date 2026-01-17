// © Copyright 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT
//
// Shared pixel format conversion utilities for DDP renderers
// These functions are designed to be called from the UDP receive task
// (pure math operations, no LVGL/ESPHome API calls, no allocations)

#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstddef>

namespace esphome {
namespace ddp {

// RGB to RGBW white channel conversion modes (internal use only)
enum class RGBWMode : uint8_t {
  NONE = 0,      // RGB only, W=0 (for RGB strips)
  ACCURATE = 1,  // Extract white, reduce RGB (for RGBW strips)
};

// RGB to brightness conversion methods (for monochromatic lights)
enum class BrightnessMethod : uint8_t {
  MAX = 0,        // max(R, G, B) - preserves peak brightness
  AVERAGE = 1,    // (R + G + B) / 3 - simple average
  LUMINANCE = 2,  // 0.299R + 0.587G + 0.114B - perceptual brightness (Rec. 601)
};

// RGB565 conversion lookup tables (compile-time generated, C++20 constexpr)
// Used for fast RGB888 → RGB565 conversion
//
// Implementation pattern based on hub75 color_lut.h optimization:
// - Constexpr generator functions return std::array (better constexpr guarantees)
// - Static constexpr storage ensures flash placement via .rodata section
// - Compile-time validation with static_assert
// - Zero runtime overhead, all computation at compile time
//
// Memory: 3 × 256 × 2 bytes = 1.5 KB in flash (not RAM)

namespace detail {
// Generate R5 lookup table: 8-bit red → 5-bit red in RGB565 format
// Maps [0..255] → [0x0000..0xF800] (bits 15-11)
// consteval forces compile-time evaluation only (C++20)
consteval std::array<uint16_t, 256> generate_r5_lut() {
  std::array<uint16_t, 256> lut{};
  for (int i = 0; i < 256; ++i) {
    lut[i] = static_cast<uint16_t>((i & 0xF8) << 8);  // 5 bits red, shifted to MSB
  }
  return lut;
}

// Generate G6 lookup table: 8-bit green → 6-bit green in RGB565 format
// Maps [0..255] → [0x0000..0x07E0] (bits 10-5)
// consteval forces compile-time evaluation only (C++20)
consteval std::array<uint16_t, 256> generate_g6_lut() {
  std::array<uint16_t, 256> lut{};
  for (int i = 0; i < 256; ++i) {
    lut[i] = static_cast<uint16_t>((i & 0xFC) << 3);  // 6 bits green, middle
  }
  return lut;
}

// Generate B5 lookup table: 8-bit blue → 5-bit blue in RGB565 format
// Maps [0..255] → [0x0000..0x001F] (bits 4-0)
// consteval forces compile-time evaluation only (C++20)
consteval std::array<uint16_t, 256> generate_b5_lut() {
  std::array<uint16_t, 256> lut{};
  for (int i = 0; i < 256; ++i) {
    lut[i] = static_cast<uint16_t>(i >> 3);  // 5 bits blue, LSB
  }
  return lut;
}

// Static constexpr storage - guaranteed compile-time evaluation and flash placement
static constexpr auto LUT_R5_ARRAY = generate_r5_lut();
static constexpr auto LUT_G6_ARRAY = generate_g6_lut();
static constexpr auto LUT_B5_ARRAY = generate_b5_lut();
}  // namespace detail

// Public LUT pointers (fully const-correct: const pointers to const data)
inline constexpr const uint16_t *const LUT_R5 = detail::LUT_R5_ARRAY.data();
inline constexpr const uint16_t *const LUT_G6 = detail::LUT_G6_ARRAY.data();
inline constexpr const uint16_t *const LUT_B5 = detail::LUT_B5_ARRAY.data();

// Compile-time validation: verify critical LUT values
// These static_assert statements ensure the LUTs are correctly generated
static_assert(detail::LUT_R5_ARRAY[0x00] == 0x0000, "R5 LUT: black incorrect");
static_assert(detail::LUT_R5_ARRAY[0xF8] == 0xF800, "R5 LUT: max red incorrect");
static_assert(detail::LUT_G6_ARRAY[0x00] == 0x0000, "G6 LUT: black incorrect");
static_assert(detail::LUT_G6_ARRAY[0xFC] == 0x07E0, "G6 LUT: max green incorrect");
static_assert(detail::LUT_B5_ARRAY[0x00] == 0x0000, "B5 LUT: black incorrect");
static_assert(detail::LUT_B5_ARRAY[0xF8] == 0x001F, "B5 LUT: max blue incorrect");

// RGB565 to RGB888 expansion helper
// Reads a 16-bit RGB565 value with endianness handling and expands to 8-bit RGB
// Safe to call from UDP task (pure math, no allocations or API calls)
//
// sp: Pointer to 2 bytes of RGB565 data
// big_endian: If true, source is RGB565 big-endian; if false, little-endian
// r, g, b: Output RGB888 values (modified in-place)
constexpr void expand_rgb565_to_rgb888(const uint8_t *sp, bool big_endian, uint8_t &r, uint8_t &g, uint8_t &b) noexcept {
  // Read RGB565 value (handle endianness)
  uint16_t v = big_endian ? (uint16_t) ((sp[0] << 8) | sp[1]) : (uint16_t) ((sp[1] << 8) | sp[0]);

  // Extract 5/6/5 bit components
  uint8_t r5 = (uint8_t) ((v >> 11) & 0x1F);
  uint8_t g6 = (uint8_t) ((v >> 5) & 0x3F);
  uint8_t b5 = (uint8_t) (v & 0x1F);

  // Expand to 8-bit (scale + replicate MSBs into LSBs)
  r = (uint8_t) ((r5 << 3) | (r5 >> 2));
  g = (uint8_t) ((g6 << 2) | (g6 >> 4));
  b = (uint8_t) ((b5 << 3) | (b5 >> 2));
}

// Convert RGB888 to RGB565 with optional byte swap
// Safe to call from UDP task (pure math, no allocations or API calls)
// Inline so it can be used across components without linkage issues
//
// dst: Destination buffer for RGB565 pixels (must be pre-allocated)
// src: Source RGB888 data (3 bytes per pixel: R, G, B)
// pixel_count: Number of pixels to convert
// swap_bytes: If true, swap byte order (for LV_COLOR_16_SWAP compatibility)
inline void convert_rgb888_to_rgb565(uint16_t *dst, const uint8_t *src, size_t pixel_count, bool swap_bytes) noexcept {
  if (!dst || !src)
    return;

  const uint8_t *sp = src;

  if (swap_bytes) {
    // Convert and swap bytes for LV_COLOR_16_SWAP
    for (size_t i = 0; i < pixel_count; ++i) {
      uint16_t c = LUT_R5[sp[0]] | LUT_G6[sp[1]] | LUT_B5[sp[2]];
      dst[i] = (uint16_t) ((c >> 8) | (c << 8));  // Swap bytes
      sp += 3;
    }
  } else {
    // Convert without swap
    for (size_t i = 0; i < pixel_count; ++i) {
      dst[i] = (uint16_t) (LUT_R5[sp[0]] | LUT_G6[sp[1]] | LUT_B5[sp[2]]);
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
inline void swap_rgb565_bytes(uint16_t *buf, size_t pixel_count) noexcept {
  if (!buf)
    return;

  for (size_t i = 0; i < pixel_count; ++i) {
    uint16_t c = buf[i];
    buf[i] = (uint16_t) ((c >> 8) | (c << 8));
  }
}

// Convert RGB888 to RGB32 (LVGL 32-bit mode)
// Safe to call from UDP task (pure math, no allocations or API calls)
// Inline so it can be used across components without linkage issues
//
// dst: Destination buffer for RGB32 pixels (must be pre-allocated)
// src: Source RGB888 data (3 bytes per pixel: R, G, B)
// pixel_count: Number of pixels to convert
//
// Note: LVGL lv_color32_t is always BGRA byte order in memory:
//   struct { uint8_t blue; uint8_t green; uint8_t red; uint8_t alpha; }
inline void convert_rgb888_to_rgb32(uint32_t *dst, const uint8_t *src, size_t pixel_count) noexcept {
  if (!dst || !src)
    return;

  const uint8_t *sp = src;

  for (size_t i = 0; i < pixel_count; ++i) {
    // lv_color32_t is {blue, green, red, alpha} in memory
    uint32_t c = ((uint32_t) sp[2]) |        // B (byte 0, from src[2])
                 ((uint32_t) sp[1] << 8) |   // G (byte 1, from src[1])
                 ((uint32_t) sp[0] << 16) |  // R (byte 2, from src[0])
                 0xFF000000;                 // A (byte 3)
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
inline void convert_rgbw_to_rgb565(uint16_t *dst, const uint8_t *src, size_t pixel_count, bool swap_bytes) noexcept {
  if (!dst || !src)
    return;

  const uint8_t *sp = src;

  if (swap_bytes) {
    // Convert and swap bytes for LV_COLOR_16_SWAP
    for (size_t i = 0; i < pixel_count; ++i) {
      uint16_t c = LUT_R5[sp[0]] | LUT_G6[sp[1]] | LUT_B5[sp[2]];
      dst[i] = (uint16_t) ((c >> 8) | (c << 8));  // Swap bytes
      sp += 4;                                    // Skip W channel
    }
  } else {
    // Convert without swap
    for (size_t i = 0; i < pixel_count; ++i) {
      dst[i] = (uint16_t) (LUT_R5[sp[0]] | LUT_G6[sp[1]] | LUT_B5[sp[2]]);
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
//
// Note: LVGL lv_color32_t is always BGRA byte order in memory:
//   struct { uint8_t blue; uint8_t green; uint8_t red; uint8_t alpha; }
inline void convert_rgbw_to_rgb32(uint32_t *dst, const uint8_t *src, size_t pixel_count) noexcept {
  if (!dst || !src)
    return;

  const uint8_t *sp = src;

  for (size_t i = 0; i < pixel_count; ++i) {
    // lv_color32_t is {blue, green, red, alpha} in memory
    uint32_t c = ((uint32_t) sp[2]) |        // B (byte 0, from src[2])
                 ((uint32_t) sp[1] << 8) |   // G (byte 1, from src[1])
                 ((uint32_t) sp[0] << 16) |  // R (byte 2, from src[0])
                 0xFF000000;                 // A (byte 3, src[3]=W ignored)
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
inline void convert_rgb888_to_rgbw(uint8_t *dst, const uint8_t *src, size_t pixel_count, RGBWMode mode) noexcept {
  if (!dst || !src)
    return;

  const uint8_t *sp = src;

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
inline void convert_rgb565_to_rgbw(uint8_t *dst, const uint8_t *src, size_t pixel_count, bool src_big_endian,
                                   RGBWMode mode) noexcept {
  if (!dst || !src)
    return;

  const uint8_t *sp = src;

  for (size_t i = 0; i < pixel_count; ++i) {
    uint8_t r, g, b;
    expand_rgb565_to_rgb888(sp, src_big_endian, r, g, b);

    if (mode == RGBWMode::ACCURATE) {
      // Extract white and reduce RGB
      uint8_t w = r < g ? (r < b ? r : b) : (g < b ? g : b);
      dst[i * 4 + 0] = r - w;  // R residual (chromatic)
      dst[i * 4 + 1] = g - w;  // G residual (chromatic)
      dst[i * 4 + 2] = b - w;  // B residual (chromatic)
      dst[i * 4 + 3] = w;      // W = common brightness
    } else {                   // RGBWMode::NONE
      // RGB passthrough, W=0
      dst[i * 4 + 0] = r;  // R
      dst[i * 4 + 1] = g;  // G
      dst[i * 4 + 2] = b;  // B
      dst[i * 4 + 3] = 0;  // W = 0
    }
    sp += 2;
  }
}

// Convert a single RGB pixel to brightness using the specified method
// Safe to call from UDP task (pure math, no allocations or API calls)
// constexpr: allows compile-time evaluation when inputs are known
// [[gnu::const]]: result depends only on arguments, enabling aggressive optimization
[[gnu::const]] constexpr uint8_t rgb_to_brightness(uint8_t r, uint8_t g, uint8_t b, BrightnessMethod method) noexcept {
  switch (method) {
    case BrightnessMethod::MAX:
      return std::max({r, g, b});
    case BrightnessMethod::AVERAGE:
      return static_cast<uint8_t>((static_cast<uint16_t>(r) + g + b) / 3);
    case BrightnessMethod::LUMINANCE:
    default:
      // Rec. 601 luminance: 0.299R + 0.587G + 0.114B
      // Using fixed-point: (77*R + 150*G + 29*B) >> 8
      return static_cast<uint8_t>((77 * r + 150 * g + 29 * b) >> 8);
  }
}

// Convert RGB888 to brightness (monochromatic output as grayscale RGBW)
// Safe to call from UDP task (pure math, no allocations or API calls)
//
// dst: Destination buffer for RGBW pixels (must be pre-allocated, 4 bytes per pixel)
// src: Source RGB888 data (3 bytes per pixel: R, G, B)
// pixel_count: Number of pixels to convert
// method: Brightness conversion method
//
// Output: R=G=B=brightness, W=0 (grayscale)
inline void convert_rgb888_to_brightness(uint8_t *dst, const uint8_t *src, size_t pixel_count,
                                         BrightnessMethod method) noexcept {
  if (!dst || !src)
    return;

  const uint8_t *sp = src;

  for (size_t i = 0; i < pixel_count; ++i) {
    uint8_t brightness = rgb_to_brightness(sp[0], sp[1], sp[2], method);
    dst[i * 4 + 0] = brightness;  // R
    dst[i * 4 + 1] = brightness;  // G
    dst[i * 4 + 2] = brightness;  // B
    dst[i * 4 + 3] = 0;           // W = 0
    sp += 3;
  }
}

// Convert RGB565 to brightness (monochromatic output as grayscale RGBW)
// Safe to call from UDP task (pure math, no allocations or API calls)
//
// dst: Destination buffer for RGBW pixels (must be pre-allocated, 4 bytes per pixel)
// src: Source RGB565 data (2 bytes per pixel)
// pixel_count: Number of pixels to convert
// src_big_endian: If true, source is RGB565 big-endian; if false, little-endian
// method: Brightness conversion method
//
// Output: R=G=B=brightness, W=0 (grayscale)
inline void convert_rgb565_to_brightness(uint8_t *dst, const uint8_t *src, size_t pixel_count, bool src_big_endian,
                                         BrightnessMethod method) noexcept {
  if (!dst || !src)
    return;

  const uint8_t *sp = src;

  for (size_t i = 0; i < pixel_count; ++i) {
    uint8_t r, g, b;
    expand_rgb565_to_rgb888(sp, src_big_endian, r, g, b);

    uint8_t brightness = rgb_to_brightness(r, g, b, method);
    dst[i * 4 + 0] = brightness;  // R
    dst[i * 4 + 1] = brightness;  // G
    dst[i * 4 + 2] = brightness;  // B
    dst[i * 4 + 3] = 0;           // W = 0
    sp += 2;
  }
}

// Convert RGBW to brightness (monochromatic output as grayscale RGBW)
// Safe to call from UDP task (pure math, no allocations or API calls)
//
// dst: Destination buffer for RGBW pixels (must be pre-allocated, 4 bytes per pixel)
// src: Source RGBW data (4 bytes per pixel: R, G, B, W)
// pixel_count: Number of pixels to convert
// method: Brightness conversion method
//
// Output: R=G=B=brightness, W=0 (grayscale)
// Note: The W channel from input is added to the computed RGB brightness
inline void convert_rgbw_to_brightness(uint8_t *dst, const uint8_t *src, size_t pixel_count,
                                       BrightnessMethod method) noexcept {
  if (!dst || !src)
    return;

  const uint8_t *sp = src;

  for (size_t i = 0; i < pixel_count; ++i) {
    // Compute brightness from RGB channels
    uint8_t rgb_brightness = rgb_to_brightness(sp[0], sp[1], sp[2], method);
    // Add white channel contribution (clamped to 255)
    uint16_t total = static_cast<uint16_t>(rgb_brightness) + sp[3];
    uint8_t brightness = total > 255 ? 255 : static_cast<uint8_t>(total);

    dst[i * 4 + 0] = brightness;  // R
    dst[i * 4 + 1] = brightness;  // G
    dst[i * 4 + 2] = brightness;  // B
    dst[i * 4 + 3] = 0;           // W = 0
    sp += 4;
  }
}

}  // namespace ddp
}  // namespace esphome
