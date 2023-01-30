/**
 * GNU General Public License v3.0+
 *
 * Copyright © 2023 Ayan Shafqat <ayan@shafq.at>
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program. If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef DISTORTION_HPP_
#define DISTORTION_HPP_

#include <algorithm>
#include <cmath>

#include "DspMath.hpp"
#include "FxBlock.hpp"

/**
 * @brief Emulate a class A style distortion, similar to a diode
 *  clipping circuit.
 *
 * @param x Input value
 *
 * @return float Input value distorted/wave-shaped
 */
inline float distortClassA(float x) { return tanhFast(x); }

/**
 * @brief Emulate a class B style distortion, similar to class B
 *  push-pull amplifier
 *
 * @param x Input value
 *
 * @return float Input value distorted/wave-shaped
 */
inline float distortClassB(float x) {
  constexpr float MAXVAL = 1.0F;
  constexpr float MINVAL = -1.0F;
  constexpr float HALF = 0.5F;
  constexpr float TWO = 2.0F;

  // Hard clip value [-1, 1]
  x = std::clamp(x, MINVAL, MAXVAL);

  auto u = std::abs(x);

  // Class B style distortion
  auto v = HALF * tanhFast(TWO * (u - HALF)) + HALF;

  return std::copysign(v, x);
}

class Distortion : public FxBlockSimpleMono {
public:
  static constexpr size_t NPARAM = 3U;
  enum Parameter : uint16_t {
    GAIN = 0,
    TONE = 1,
    ABSEL = 2,
  };

  Distortion(float calibGainActive = 1.0F, float calibGainBypass = 1.0F)
      : FxBlockSimpleMono(calibGainActive, calibGainBypass) {}

  void updateParameter(uint16_t id, uint16_t value) {
    switch (id) {
    case GAIN: {
      // Translate gain value to linear value from normalized [0,1)
      float dbValue = (60.0F * 0x1.0p-10F) * static_cast<float>(value);
      float linValue = undb20Fast(dbValue);
      fxParam_[GAIN].setTarget(linValue);
    } break;
    case TONE: {
      // Just arbitrary coefficient for setting the tone knob
      // TODO: Find a good translation value
      float fc = ((0.4995F * 0x1.0p-10F) * static_cast<float>(value));
      fxParam_[TONE].setTarget(fc);
    } break;
    case ABSEL: {
      // For knob position at 0, class A style distortion is achieved.
      // For knob position at max, class B style distortion is achieved.
      // Then any position in the middle is a blend between class A and
      // class B.
      float abGainValue = (0x1.0p-10F) * static_cast<float>(value);
      fxParam_[ABSEL].setTarget(abGainValue);
    } break;
    default:
      break;
    }
  }

  bool parameterChanged() const {
    for (size_t i = 0; i < NPARAM; ++i) {
      if (!fxParam_[i].isStable()) {
        return true;
      }
    }
    return false;
  }

  inline void run(float *pOut, const float *pIn, size_t size, float gain,
                  float blend, float tone) {
    for (size_t i = 0U; i < size; ++i) {
      float in = pIn[i];
      float dIn = gain * in;

      // The distortion effect
      float dOutA = distortClassA(dIn);
      float dOutB = distortClassB(dIn);

      // Distortion blend
      float blendOut = (blend * dOutA) + ((1.0F - blend) * dOutB);

      // Tone control
      float toneOut = toneState_;
      toneState_ = lowpass1(blendOut, tone, toneState_);

      // Write output sample
      pOut[i] = toneOut;
    }
  }

  // Audio process callback function
  virtual void processAudio() {
    auto frameSize = getFrameSize();
    auto *pIn = getInputBuffer(0);
    auto *pOut = getOutputBuffer(0);

    // If parameters did not change, then only load the parameters once
    if (parameterChanged()) {
      auto gain = fxParam_[GAIN].update();
      auto abBlend = fxParam_[ABSEL].update();
      auto toneCoeff = fxParam_[TONE].update();
      run(pOut, pIn, frameSize, gain, abBlend, toneCoeff);
    } else {
      for (auto j = 0U; j < frameSize; ++j) {
        auto gain = fxParam_[GAIN].update();
        auto abBlend = fxParam_[ABSEL].update();
        auto toneCoeff = fxParam_[TONE].update();
        run(pOut + j, pIn + j, 1, gain, abBlend, toneCoeff);
      }
    }

    // In case filter becomes unstable, set the state back to zero
    if (!std::isfinite(toneState_)) {
      toneState_ = 0.0F;
    }
  }

private:
  FxSmoothParameter fxParam_[NPARAM]{};
  float toneState_{0.0F};
};

#endif // DISTORTION_HPP_
