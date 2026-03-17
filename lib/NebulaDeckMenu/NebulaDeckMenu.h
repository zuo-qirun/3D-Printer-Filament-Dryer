#pragma once

#include <Arduino.h>
#include <U8g2lib.h>

class NebulaDeckMenu {
 public:
  void begin(const char* const* items, uint8_t count);
  void setSelection(uint8_t index, bool immediate);
  void startEnter();
  void startExit();
  bool isExiting() const;
  bool isExitFinished() const;
  void nudge(int8_t direction);
  bool tick(uint32_t nowMs);
  void render(U8G2& oled) const;
  int16_t menuOffsetY() const;

 private:
  const char* const* items_ = nullptr;
  uint8_t count_ = 0;
  uint8_t selectedIndex_ = 0;

  float selectAnim_ = 0.0f;
  float bounceX_ = 0.0f;
  float bounceV_ = 0.0f;
  float swipeX_ = 0.0f;
  float menuShiftY_ = 64.0f;
  float targetShiftY_ = 64.0f;
  bool exiting_ = false;
  uint32_t lastTickMs_ = 0;

  void drawIcon(U8G2& oled, uint8_t idx, int16_t cx, int16_t cy, uint8_t size, bool selected) const;
  int8_t offsetToIndex(int8_t offset) const;
};
