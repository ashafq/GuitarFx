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

#ifndef DSP_MATH_HPP_
#define DSP_MATH_HPP_

#include <algorithm>
#include <cmath>

#if __cplusplus < 201703L
namespace std {
// std::clamp was introduced in C++17. This is a workaround for compilers
// that do not support C++17, like Teensy's 1.x SDK.
template <typename T>
constexpr const T &clamp(const T &x, const T &minval, const T &maxval) {
  return std::max(std::min(x, maxval), minval);
}
} // namespace std
#endif

/**
 * @brief Fast approximation to hyperbolic tangent function @p tanh
 * @param x Input value
 * @return float @p tanh(x)
 */
inline float fastTanh(float x) {
  x = std::clamp(x, -1.0F, 1.0F);
  return x * (1.5F - 0.5F * x * x);
}

/**
 * @brief Compare two floating point values and return true if they
 *   are close to each other
 *
 * @param a A floating point value
 * @param b A floating point value
 * @param threshold Threshold of tolerance
 *
 * @return true If @p a and @p b are close
 * @return false If @p a and @p b are _not_ close
 */
inline bool isClose(float a, float b, float threshold = 0x1.0p6F) {
  return std::abs(a - b) <= threshold;
}

/**
 * @brief Convert decibel respect to full scale (1.0) to linear value
 * @param[in] x A decibel value
 * @return @p x converted to linear
 */
float dbToLinear20(float x) {
  constexpr float D2L = 0x1.542a5a12e1c5ap-3F; // log2(10) / 20
  return std::exp2(D2L * x);
}

/**
 * @brief One pole low pass filter
 *
 * @details Implementation of a single order low pass filter with unity gain
 *
 * H(z) = b0 / (1 + a1 * z^-1), b0 = 1 - a1
 *
 * Y(n) = b0 * X(n) + a1 * Y(n - 1)
 *
 * @param in Input sample, X(n)
 * @param b0 Tuning parameter [numerator coefficient: b0, a1 = (1 - b0)]
 * @param state Current filter state, Y(n-1)
 * @return float Output value, Y(n)
 */
float lowpass1(float in, float b0, float state) {
  return (b0 * (in - state)) + state;
}

/**
 * @brief One pole low pass filter in fixedpoint
 *
 * @details Implementation of a single order low pass filter with unity gain
 *
 * H(z) = b0 / (1 + a1 * z^-1), b0 = 1 - a1
 *
 * Y(n) = b0 * X(n) + a1 * Y(n - 1)
 *
 * @param in Input sample, X(n)
 * @param b0 Tuning parameter [numerator coefficient: b0, a1 = (1 - b0)]
 * @param state Current filter state, Y(n-1)
 * @param q Is the amount to shift back due to fixed point multiply
 *    and it is dependent on the Q value of the coefficient
 *
 * @return float Output value, Y(n)
 */
int32_t lowpass1Fix(int32_t in, int32_t b0, int32_t state, int32_t q) {
  return ((b0 * (in - state)) >> q) + state;
}

#endif // DSP_MATH_HPP_
