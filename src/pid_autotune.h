#pragma once

#include <Arduino.h>
#include <math.h>

enum class PidAutoTuneStepResult : uint8_t {
  Running = 0,
  Done,
  Fail,
};

struct PidAutoTuneState {
  bool active = false;
  uint32_t startMs = 0;
  float targetC = 0.0f;
  bool relayHigh = true;
  float peakHigh = -1000.0f;
  float peakLow = 1000.0f;
  uint32_t lastRiseMs = 0;
  float ampSum = 0.0f;
  uint8_t ampCount = 0;
  float periodSum = 0.0f;
  uint8_t periodCount = 0;
};

struct PidAutoTuneConfig {
  float bandC = 1.0f;
  uint32_t minTimeMs = 180000;
  uint32_t timeoutMs = 1200000;
  float relayAmplitude = 0.5f;
  float minAmplitudeC = 0.02f;
};

// 主要功能：初始化 PID 继电自整定状态机。
// 使用方法：进入自动校准时调用一次。
inline void StartPidAutoTune(PidAutoTuneState& s, float targetC, float currentTempC, uint32_t nowMs) {
  s.active = true;
  s.startMs = nowMs;
  s.targetC = targetC;
  s.relayHigh = true;
  s.peakHigh = currentTempC;
  s.peakLow = currentTempC;
  s.lastRiseMs = 0;
  s.ampSum = 0.0f;
  s.ampCount = 0;
  s.periodSum = 0.0f;
  s.periodCount = 0;
}

// 主要功能：执行一次 PID 继电自整定迭代。
// 使用方法：控制周期内调用；运行中持续输出 demand01；完成时写出 kp/ki/kd。
inline PidAutoTuneStepResult StepPidAutoTune(PidAutoTuneState& s, const PidAutoTuneConfig& cfg,
                                             uint32_t nowMs, float measuredTempC, bool sensorOk,
                                             bool hasFault, float& demand01, float& kp, float& ki,
                                             float& kd, float kpMin, float kpMax, float kiMin,
                                             float kiMax, float kdMin, float kdMax) {
  if (!s.active) return PidAutoTuneStepResult::Fail;

  float upper = s.targetC + cfg.bandC;
  float lower = s.targetC - cfg.bandC;

  if (!sensorOk || hasFault) {
    s.active = false;
    demand01 = 0.0f;
    return PidAutoTuneStepResult::Fail;
  }

  if (nowMs - s.startMs > cfg.timeoutMs) {
    s.active = false;
    demand01 = 0.0f;
    return PidAutoTuneStepResult::Fail;
  }

  if (s.relayHigh) {
    demand01 = 1.0f;
    if (measuredTempC > s.peakHigh) s.peakHigh = measuredTempC;
    if (measuredTempC >= upper) {
      s.relayHigh = false;
      s.peakLow = measuredTempC;
    }
  } else {
    demand01 = 0.0f;
    if (measuredTempC < s.peakLow) s.peakLow = measuredTempC;
    if (measuredTempC <= lower) {
      float amp = (s.peakHigh - s.peakLow) * 0.5f;
      if (amp > cfg.minAmplitudeC && amp < 30.0f) {
        s.ampSum += amp;
        if (s.ampCount < 200) s.ampCount++;
      }

      if (s.lastRiseMs > 0) {
        float tu = (nowMs - s.lastRiseMs) / 1000.0f;
        if (tu > 2.0f && tu < 600.0f) {
          s.periodSum += tu;
          if (s.periodCount < 200) s.periodCount++;
        }
      }

      s.lastRiseMs = nowMs;
      s.relayHigh = true;
      s.peakHigh = measuredTempC;
    }
  }

  if ((nowMs - s.startMs) < cfg.minTimeMs || s.ampCount < 4 || s.periodCount < 3) {
    return PidAutoTuneStepResult::Running;
  }

  float a = s.ampSum / s.ampCount;
  float tu = s.periodSum / s.periodCount;
  if (a <= cfg.minAmplitudeC || tu <= 0.0f) {
    s.active = false;
    demand01 = 0.0f;
    return PidAutoTuneStepResult::Fail;
  }

  float ku = (4.0f * cfg.relayAmplitude) / (PI * a);
  float newKp = 0.6f * ku;
  float newKi = (1.2f * ku) / tu;
  float newKd = 0.075f * ku * tu;

  if (newKp < kpMin) newKp = kpMin;
  if (newKp > kpMax) newKp = kpMax;
  if (newKi < kiMin) newKi = kiMin;
  if (newKi > kiMax) newKi = kiMax;
  if (newKd < kdMin) newKd = kdMin;
  if (newKd > kdMax) newKd = kdMax;

  kp = newKp;
  ki = newKi;
  kd = newKd;
  s.active = false;
  demand01 = 0.0f;
  return PidAutoTuneStepResult::Done;
}

