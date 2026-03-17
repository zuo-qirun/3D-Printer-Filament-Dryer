#include "NebulaDeckMenu.h"

#include <math.h>

namespace {
static float smoothStep(float current, float target, float speedPerSec, float dtSec) {
  float alpha = 1.0f - expf(-speedPerSec * dtSec);
  return current + (target - current) * alpha;
}
}  // namespace

void NebulaDeckMenu::begin(const char* const* items, uint8_t count) {
  items_ = items;
  count_ = count;
  selectedIndex_ = 0;
  selectAnim_ = 0.0f;
  bounceX_ = 0.0f;
  bounceV_ = 0.0f;
  swipeX_ = 0.0f;
  menuShiftY_ = 64.0f;
  targetShiftY_ = 64.0f;
  exiting_ = false;
  lastTickMs_ = 0;
}

void NebulaDeckMenu::setSelection(uint8_t index, bool immediate) {
  if (count_ == 0) return;
  if (index >= count_) index = static_cast<uint8_t>(count_ - 1);
  selectedIndex_ = index;
  if (immediate) {
    selectAnim_ = static_cast<float>(selectedIndex_);
  }
}

void NebulaDeckMenu::startEnter() {
  exiting_ = false;
  menuShiftY_ = 64.0f;
  targetShiftY_ = 0.0f;
}

void NebulaDeckMenu::startExit() {
  exiting_ = true;
  targetShiftY_ = 64.0f;
}

bool NebulaDeckMenu::isExiting() const { return exiting_; }

bool NebulaDeckMenu::isExitFinished() const {
  return exiting_ && fabsf(menuShiftY_ - targetShiftY_) < 0.2f;
}

void NebulaDeckMenu::nudge(int8_t direction) {
  float dir = (direction < 0) ? -1.0f : 1.0f;
  swipeX_ += dir * 22.0f;
  bounceV_ += dir * 12.0f;
}

bool NebulaDeckMenu::tick(uint32_t nowMs) {
  if (count_ == 0) return false;

  if (lastTickMs_ == 0) {
    lastTickMs_ = nowMs;
    selectAnim_ = static_cast<float>(selectedIndex_);
    return true;
  }

  float dtSec = static_cast<float>(nowMs - lastTickMs_) / 1000.0f;
  lastTickMs_ = nowMs;
  if (dtSec < 0.001f) dtSec = 0.001f;
  if (dtSec > 0.05f) dtSec = 0.05f;

  float prevSel = selectAnim_;
  float prevX = bounceX_;
  float prevSwipe = swipeX_;
  float prevY = menuShiftY_;

  selectAnim_ = smoothStep(selectAnim_, static_cast<float>(selectedIndex_), 16.0f, dtSec);
  float shiftSpeed = exiting_ ? 8.0f : 11.0f;
  menuShiftY_ = smoothStep(menuShiftY_, targetShiftY_, shiftSpeed, dtSec);
  swipeX_ = smoothStep(swipeX_, 0.0f, 18.0f, dtSec);

  float accel = (-36.0f * bounceX_) - (10.0f * bounceV_);
  bounceV_ += accel * dtSec;
  bounceX_ += bounceV_ * dtSec;
  if (fabsf(bounceX_) < 0.1f && fabsf(bounceV_) < 0.15f) {
    bounceX_ = 0.0f;
    bounceV_ = 0.0f;
  }

  if (fabsf(selectAnim_ - selectedIndex_) < 0.01f) {
    selectAnim_ = static_cast<float>(selectedIndex_);
  }

  return fabsf(prevSel - selectAnim_) > 0.001f || fabsf(prevX - bounceX_) > 0.001f ||
         fabsf(prevSwipe - swipeX_) > 0.001f || fabsf(prevY - menuShiftY_) > 0.001f;
}

void NebulaDeckMenu::render(U8G2& oled) const {
  if (count_ == 0 || items_ == nullptr) {
    oled.setFont(u8g2_font_6x12_tf);
    oled.drawStr(20, 36, "NO MENU ITEMS");
    return;
  }

  int16_t yBase = 34 + static_cast<int16_t>(menuShiftY_);
  int16_t bounce = static_cast<int16_t>(bounceX_ + swipeX_);

  // Center highlight panel.
  oled.setDrawColor(1);
  oled.drawRBox(36 + bounce, yBase - 25, 56, 50, 7);

  // Left card.
  int8_t leftIdx = offsetToIndex(-1);
  int16_t leftCx = 24 + bounce;
  oled.setDrawColor(1);
  drawIcon(oled, static_cast<uint8_t>(leftIdx), leftCx, yBase - 8, 12, false);
  oled.setFont(u8g2_font_wqy12_t_gb2312);
  int16_t lw = oled.getUTF8Width(items_[leftIdx]);
  // Keep icon/text aligned while forcing partial off-screen clipping.
  int16_t lx = leftCx - (lw / 2) - 18;
  oled.drawUTF8(lx, yBase + 14, items_[leftIdx]);

  // Center card.
  int8_t centerIdx = offsetToIndex(0);
  int16_t centerCx = 64 + bounce;
  oled.setDrawColor(0);
  drawIcon(oled, static_cast<uint8_t>(centerIdx), centerCx, yBase - 8, 18, true);
  oled.setFont(u8g2_font_wqy12_t_gb2312);
  int16_t cw = oled.getUTF8Width(items_[centerIdx]);
  int16_t cx = centerCx - cw / 2;
  if (cx < 0) cx = 0;
  if (cx > 127 - cw) cx = 127 - cw;
  oled.drawUTF8(cx, yBase + 17, items_[centerIdx]);

  // Right card.
  oled.setDrawColor(1);
  int8_t rightIdx = offsetToIndex(+1);
  int16_t rightCx = 104 + bounce;
  drawIcon(oled, static_cast<uint8_t>(rightIdx), rightCx, yBase - 8, 12, false);
  oled.setFont(u8g2_font_wqy12_t_gb2312);
  int16_t rw = oled.getUTF8Width(items_[rightIdx]);
  int16_t rx = rightCx - (rw / 2) + 18;
  oled.drawUTF8(rx, yBase + 14, items_[rightIdx]);

  oled.setDrawColor(1);
}

int16_t NebulaDeckMenu::menuOffsetY() const { return static_cast<int16_t>(menuShiftY_); }

int8_t NebulaDeckMenu::offsetToIndex(int8_t offset) const {
  int16_t idx = static_cast<int16_t>(selectedIndex_) + offset;
  while (idx < 0) idx += count_;
  while (idx >= count_) idx -= count_;
  return static_cast<int8_t>(idx);
}

void NebulaDeckMenu::drawIcon(U8G2& oled, uint8_t idx, int16_t cx, int16_t cy, uint8_t size,
                              bool selected) const {
  int16_t r = size / 2;
  int16_t x = cx - r;
  int16_t y = cy - r;

  switch (idx % 6) {
    case 0:  // power
      oled.drawCircle(cx, cy, r - 1);
      oled.drawVLine(cx, y + 1, r);
      break;
    case 1:  // thermometer
      oled.drawFrame(cx - 2, y + 1, 4, size - 6);
      oled.drawDisc(cx, y + size - 3, 3);
      break;
    case 2:  // timer
      oled.drawCircle(cx, cy, r - 1);
      oled.drawVLine(cx, cy - 1, r - 2);
      oled.drawHLine(cx, cy, r - 2);
      break;
    case 3:  // flame
      oled.drawTriangle(cx - (r - 1), y + size - 1, cx, y + 1, cx + (r - 1), y + size - 1);
      break;
    case 4:  // fan
      oled.drawCircle(cx, cy, r - 1);
      oled.drawDisc(cx, cy, 1);
      oled.drawLine(cx, cy, cx, y + 1);
      oled.drawLine(cx, cy, x + size - 1, cy);
      oled.drawLine(cx, cy, cx, y + size - 1);
      oled.drawLine(cx, cy, x + 1, cy);
      break;
    default:  // settings
      oled.drawFrame(x + 1, y + 1, size - 2, size - 2);
      oled.drawPixel(cx, y);
      oled.drawPixel(cx, y + size - 1);
      oled.drawPixel(x, cy);
      oled.drawPixel(x + size - 1, cy);
      break;
  }

  if (selected) {
    oled.drawRFrame(x - 2, y - 2, size + 4, size + 4, 3);
  }
}
