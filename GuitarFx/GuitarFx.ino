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

#include <Audio.h>
#include <Bounce.h>
#include <SD.h>
#include <SPI.h>
#include <SerialFlash.h>
#include <Wire.h>

#include "Distortion.hpp"
#include "DspMath.hpp"

// Audio Blocks
AudioInputI2S i2sIn;
AudioOutputI2S i2sOut;
AudioAmplifier calibGainIn;
AudioAmplifier calibGainOut;
Distortion fx1;

// Wiring
AudioConnection patchCord0(i2sIn, 0, calibGainIn, 0);
AudioConnection patchCord1(calibGainIn, 0, fx1, 0);
AudioConnection patchCord2(fx1, 0, calibGainOut, 0);
AudioConnection patchCord3(calibGainOut, 0, i2sOut, 0);

// Control blocks for Fx toggle buttons
constexpr int LED = 13;
constexpr int BUTTONPIN = 18;
Bounce pushbutton = Bounce(BUTTONPIN, 20); // 20ms debounce
static bool fxEnable = false;

// Control blocks for analog knobs
IntervalTimer analogKnobTimer;
constexpr int analogKnobInterval = 25'000; // 25ms
constexpr int analogKnobPin1 = A1;
constexpr int analogKnobPin2 = A2;
constexpr int analogKnobPin3 = A3;

constexpr int vSmoothThreshold = 16;
constexpr int vSmoothCoeff = 198;
constexpr int vSmoothQ = 9;

static int v1State = 0;
static int v2State = 0;
static int v3State = 0;

// Invert value, since the knobs were wired backwards (-_-)
constexpr int analogKnobBias = 1 << 10;

void analogKnobHandler() {
  int v1 = analogKnobBias - analogRead(analogKnobPin1);
  int v2 = analogKnobBias - analogRead(analogKnobPin2);
  int v3 = analogKnobBias - analogRead(analogKnobPin3);

  if (abs(v1 - v1State) >= vSmoothThreshold) {
    fx1.updateParameter(0, v1State);
    v1State = lowpass1Fix(v1, vSmoothCoeff, v1State, vSmoothQ);
  }

  if (abs(v2 - v2State) >= vSmoothThreshold) {
    fx1.updateParameter(1, v2State);
    v2State = lowpass1Fix(v2, vSmoothCoeff, v2State, vSmoothQ);
  }

  if (abs(v3 - v3State) >= vSmoothThreshold) {
    fx1.updateParameter(2, v3State);
    v3State = lowpass1Fix(v3, vSmoothCoeff, v3State, vSmoothQ);
  }

#ifdef GFX_DEBUG
  Serial.print("analogKnob: ");
  Serial.print(v1State);
  Serial.print("\t");
  Serial.print(v2State);
  Serial.print("\t");
  Serial.println(v3State);
#endif
}

void setup() {
  // initialize the digital pin as an output.
  pinMode(LED, OUTPUT);
  pinMode(BUTTONPIN, INPUT_PULLDOWN);
  AudioMemory(64); // Preallocate audio buffers for processing

  // TODO: Set calibration gain
  calibGainIn.gain(12.0F);
  calibGainOut.gain(0.05F);

  // Set up analog knob timer interrupt
  analogKnobTimer.begin(analogKnobHandler, analogKnobInterval);

  // UI to show system has started
  // Blink LED for 1 second and then blink twice for 250 ms
  digitalWrite(LED, HIGH);
  delay(1000);
  digitalWrite(LED, LOW);
  delay(250);

  for (int i = 0; i < 2; ++i) {
    digitalWrite(LED, HIGH);
    delay(250);
    digitalWrite(LED, LOW);
    delay(250);
  }
}

void loop() {
  if (pushbutton.update()) {
    if (pushbutton.fallingEdge()) {
      fxEnable = !fxEnable;
      if (fxEnable) {
        fx1.fxEnable();
        digitalWrite(LED, HIGH);
      } else {
        fx1.fxDisable();
        digitalWrite(LED, LOW);
      }
    }
  }
}
