#pragma once

#include <Arduino.h>

// 主要功能：限制浮点数在给定区间内，防止参数越界。
// 使用方法：传入原始值与上下限，返回裁剪后的值。
inline float ClampF(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

// 主要功能：将 0.0~1.0 的比例转换为 0~100 百分比。
// 使用方法：用于功率、占空比等显示与日志输出。
inline uint8_t RatioToPct(float ratio01) {
  float clamped = ClampF(ratio01, 0.0f, 1.0f);
  return static_cast<uint8_t>(clamped * 100.0f + 0.5f);
}

// 主要功能：将秒数格式化为 HH:MM:SS 字符串。
// 使用方法：传入秒数与输出缓冲区，常用于 OLED 显示剩余时间。
inline void FormatDuration(uint32_t sec, char* out, size_t outSize) {
  uint32_t h = sec / 3600UL;
  uint32_t m = (sec % 3600UL) / 60UL;
  uint32_t s = sec % 60UL;
  snprintf(out, outSize, "%02lu:%02lu:%02lu", static_cast<unsigned long>(h),
           static_cast<unsigned long>(m), static_cast<unsigned long>(s));
}

// 主要功能：带上限的安全加法，避免无符号整数超范围。
// 使用方法：用于菜单调整时长等需要饱和加法的场景。
inline uint32_t SaturatingAddU32(uint32_t base, uint32_t delta, uint32_t upper) {
  uint32_t v = base + delta;
  return (v > upper) ? upper : v;
}

// 主要功能：带下限的安全减法，避免无符号整数下溢。
// 使用方法：用于菜单调整时长等需要饱和减法的场景。
inline uint32_t SaturatingSubU32(uint32_t base, uint32_t delta, uint32_t lower) {
  return (base > lower + delta) ? (base - delta) : lower;
}

