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

// Calibration gain
constexpr float CALIBRATION_BYPASS = 12.0F;
constexpr float CALIBRATION_ACTIVE = 1.0F;

// Audio Blocks
AudioInputI2S i2sIn;
AudioOutputI2S i2sOut;
Distortion fx1(CALIBRATION_ACTIVE, CALIBRATION_BYPASS);

// Audio library Wiring
AudioConnection patchCord0(i2sIn, 0, fx1, 0);
AudioConnection patchCord2(fx1, 0, i2sOut, 0);

// Pins and other constants
constexpr int LED_PIN = 13;
constexpr int BUTTON_PIN = 18;
constexpr int BUTTON_DEBOUNCE = 25; // Miliseconds debounce time

#ifdef GFX_DEBUG
constexpr int ANALOG_KNOB_INTERVAL = 250'000; // ms
#else
constexpr int ANALOG_KNOB_INTERVAL = 25'000; // ms
#endif

constexpr int ANALOG_KNOB_PIN1 = A1;
constexpr int ANALOG_KNOB_PIN2 = A2;
constexpr int ANALOG_KNOB_PIN3 = A3;

// Control blocks for debouncer and timers
Bounce pushbutton = Bounce(BUTTON_PIN, BUTTON_DEBOUNCE);
IntervalTimer analogKnobTimer;

// Timer handler for analog knobs
static int vState[3] = {0,0,0};
void analogKnobHandler() {
  constexpr int vSmoothThreshold = 16; // Threshold for analog input difference
  constexpr int vSmoothCoeff = 298; // Smooth LPF filter coefficient
  constexpr int vSmoothQ = 9; // Smooth LPF filter shift factor

  // Invert value, since the knobs were wired backwards (-_-)
  constexpr int analogKnobBias = 1 << 10;

  int v1 = analogKnobBias - analogRead(ANALOG_KNOB_PIN1);
  int v2 = analogKnobBias - analogRead(ANALOG_KNOB_PIN2);
  int v3 = analogKnobBias - analogRead(ANALOG_KNOB_PIN3);

  if (abs(v1 - vState[0]) >= vSmoothThreshold) {
    fx1.updateParameter(0, vState[0]);
    vState[0] = lowpass1Fix(v1, vSmoothCoeff, vState[0], vSmoothQ);
  }

  if (abs(v2 - vState[1]) >= vSmoothThreshold) {
    fx1.updateParameter(1, vState[1]);
    vState[1] = lowpass1Fix(v2, vSmoothCoeff, vState[1], vSmoothQ);
  }

  if (abs(v3 - vState[2]) >= vSmoothThreshold) {
    fx1.updateParameter(2, vState[2]);
    vState[2] = lowpass1Fix(v3, vSmoothCoeff, vState[2], vSmoothQ);
  }

#ifdef GFX_DEBUG
  Serial.print("analogKnob: ");
  for (int i = 0; i < 3; ++i) {
  Serial.print(vState[i]);
  Serial.print("\t");
  }
  Serial.print('\n');
#endif
}

void setup() {
  // initialize the digital pin as an output.
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLDOWN);
  AudioMemory(64); // Preallocate audio buffers for processing

  // Set up analog knob timer interrupt
  analogKnobTimer.begin(analogKnobHandler, ANALOG_KNOB_INTERVAL);

  // UI to show system has started
  // Blink LED_PIN for 1 second and then blink twice for 250 ms
  digitalWrite(LED_PIN, HIGH);
  delay(1000);
  digitalWrite(LED_PIN, LOW);
  delay(250);

  for (int i = 0; i < 2; ++i) {
    digitalWrite(LED_PIN, HIGH);
    delay(250);
    digitalWrite(LED_PIN, LOW);
    delay(250);
  }

#ifdef GFX_DBUG
  Serial.begin(115200);
#endif
}

static bool fxEnable = false;
void loop() {
  if (pushbutton.update()) {
    if (pushbutton.fallingEdge()) {
      // Toggle fx enable state
      fxEnable = !fxEnable;
      if (fxEnable) {
        fx1.fxEnable();
        digitalWrite(LED_PIN, HIGH);
      } else {
        fx1.fxDisable();
        digitalWrite(LED_PIN, LOW);
      }
    }
  }
}
