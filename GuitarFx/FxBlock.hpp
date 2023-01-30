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

#ifndef EX_AUDIO_FX_BLOCK_HPP_
#define EX_AUDIO_FX_BLOCK_HPP_

#include <cassert>
#include <cmath>
#include <cstdint>

#include "arm_math.h"

#include "Arduino.h"
#include "AudioStream.h"

#include "DspMath.hpp"

/**
 * @brief Smooth parameter removes parameter jitter from audio controls
 */
class FxSmoothParameter {
public:
  static constexpr float THRESHOLD = 1.0e-3F;
  static constexpr float COEFF = 0.02662641944419386F; // 1/256 exp tau

#if 0
  // TODO: Consider constexpr methods for computing coefficients
  static constexpr float TARGET_RATIO =
      (1.0F + THRESHOLD) / THRESHOLD;

  static float computeCoeff(float tau) {
    return 1.0F - std::pow(TARGET_RATIO, -1.0F / (tau));
  }
#endif

  FxSmoothParameter() = default;

  FxSmoothParameter(float init) : target_{init}, state_{init} {
    assert(init >= 0.0F);
  }

  bool isStable() const { return isClose(target_, state_, THRESHOLD); }

  float get() const { return state_; }

  void setTarget(float value) {
    assert(value >= 0.0F);
    target_ = value;
  }

  float update() {
    float retVal = 0.0F;

    if (isStable()) {
      state_ = target_;
      retVal = state_;
    } else {
      retVal = state_;
      state_ = lowpass1(target_, COEFF, state_);
    }

    return retVal;
  }

private:
  float target_{0.0F};
  float state_{0.0F};
};

template <size_t numChannel = 1, size_t frameSize = AUDIO_BLOCK_SAMPLES>
class FxBlock : public AudioStream {
public:
  FxBlock(float calGainActive = 1.0F, float calGainBypass = 1.0F)
      : AudioStream(numChannel, channelBuffer_),
        fxVolume_(0.0F), frameCounter_{0}, // clang-format: next line
        frameCount_{frameSize / AUDIO_BLOCK_SAMPLES},
        activeGain_{calGainActive}, bypassGain_{calGainBypass} {
    static_assert(numChannel >= 1, "Number of channel is not greater than 1");
    static_assert(frameSize >= AUDIO_BLOCK_SAMPLES,
                  "Frame size needs to be greater than audio block size");
    static_assert(frameSize % AUDIO_BLOCK_SAMPLES == 0,
                  "Frame size needs to be a multiple of audio block size");
  }

  // This method needs to be implemented by the derived class
  virtual void processAudio(void) = 0;

  float *getInputBuffer(size_t channel) {
    assert(channel < numChannel);
    return audioInBuffer_ + (channel * frameSize);
  }

  float *getOutputBuffer(size_t channel) {
    assert(channel < numChannel);
    return audioOutBuffer_ + (channel * frameSize);
  }

  size_t getFrameSize() const { return frameSize; }

  size_t getNumChannel() const { return numChannel; }

  void fxEnable() { fxVolume_.setTarget(1.0F); }

  void fxDisable() { fxVolume_.setTarget(0.0F); }

private:
  void readAudioBockData(audio_block_t *block, size_t chan, size_t offset) {
    // Convert data block from int16 to float
    float *pin = getInputBuffer(chan) + offset;
    arm_q15_to_float(block->data, pin, AUDIO_BLOCK_SAMPLES);
  }

  void writeAudioBockData(audio_block_t *block, size_t chan, size_t offset) {
    // Convert data from float to int16
    float *pout = getOutputBuffer(chan) + offset;
    // There are not dithering steps in this routine. After conducting
    // several measurements and spectrum analysis, the ADC provides more
    // than enough noise floor and dithering is not necessary.
    arm_float_to_q15(pout, block->data, AUDIO_BLOCK_SAMPLES);
  }

  void processAudioStateMachine(void) {
    // If volume is stable, then process a single gain value
    if (fxVolume_.isStable()) {
      float fxVol = fxVolume_.get();              // Volume for effect
      float byVol = bypassGain_ * (1.0F - fxVol); // Volume for bypassed
      fxVol *= activeGain_;

      // Call the audio process callback if required
      if (fxVol > 0.0F) {
        processAudio();
      }

      for (auto i = 0U; i < numChannel; ++i) {
        auto *ibuf = getInputBuffer(i);  // Input vector
        auto *obuf = getOutputBuffer(i); // Output vector

        for (auto j = 0U; j < frameSize; ++j) {
          obuf[j] = (fxVol * obuf[j]) + (byVol * ibuf[j]);
        }
      }
    } else {
      // Call the audio process callback
      processAudio();

      // Update gain sample by sample
      for (auto j = 0U; j < frameSize; ++j) {
        float fxVol = fxVolume_.update(); // Volume for effect
        float byVol = (1.0F - fxVol);     // Volume for bypassed

        for (auto i = 0U; i < numChannel; ++i) {
          auto *ibuf = getInputBuffer(i);  // Input vector
          auto *obuf = getOutputBuffer(i); // Output vector
          obuf[j] = (fxVol * obuf[j]) + (byVol * ibuf[j]);
        }
      }
    }
  }

  // Callback routine from AudioStream class
  virtual void update(void) {
    // No requeing necessary if frameSize is equal to AUDIO_BLOCK_SAMPLES
    if (frameSize == AUDIO_BLOCK_SAMPLES) {
      audio_block_t *block[numChannel];

      // Read audio block data
      for (size_t chan = 0; chan < numChannel; ++chan) {
        block[chan] = receiveWritable(chan);
        if (block[chan]) {
          readAudioBockData(block[chan], chan, 0);
        }
      }

      // Process audio callback
      processAudioStateMachine();

      // Write the output block to output buffer in the framework
      for (size_t chan = 0; chan < numChannel; ++chan) {
        if (block[chan]) {
          writeAudioBockData(block[chan], chan, 0);
          transmit(block[chan], chan);
          release(block[chan]);
        }
      }
    } else {
      // Probably counter intuitive, but the zero-th set of frames should be
      // zeros or fully filled queue if frameCounter == 0. Call the virtual
      // process audio function if frameCounter is zero.
      if (frameCounter_ == 0) {
        processAudioStateMachine();
      }

      // Write data block in framework
      size_t offset = frameCounter_ * AUDIO_BLOCK_SAMPLES;

      // Write the output block to output buffer in the framework
      for (size_t chan = 0; chan < numChannel; ++chan) {
        audio_block_t *block = receiveWritable(chan);
        if (block) {
          readAudioBockData(block, chan, offset);
          writeAudioBockData(block, chan, offset);
          transmit(block, chan);
          release(block);
        }
      }

      // Update frame counter circularly
      ++frameCounter_;

      if (frameCounter_ >= frameCount_) {
        frameCounter_ = 0;
      }
    }
  }

private:
  FxSmoothParameter fxVolume_{0.0F};
  audio_block_t *channelBuffer_[numChannel]{};
  float audioInBuffer_[numChannel * frameSize];
  float audioOutBuffer_[numChannel * frameSize];
  size_t frameCounter_{0};
  size_t frameCount_{0};
  float activeGain_{1.0F};
  float bypassGain_{1.0F};
};

// Some easy to use block types, like 1 channel with same frame size
using FxBlockSimpleMono = FxBlock<1, AUDIO_BLOCK_SAMPLES>;
using FxBlockSimpleStereo = FxBlock<2, AUDIO_BLOCK_SAMPLES>;

#endif // EX_AUDIO_FX_BLOCK_HPP_
