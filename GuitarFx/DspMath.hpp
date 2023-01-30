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

namespace {
/**
 * @brief Fast approximation to hyperbolic tangent function @p tanh
 * @param x Input value
 * @return float @p tanh(x)
 */
inline float tanhFast(float x) noexcept {
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
inline bool isClose(float a, float b, float threshold = 0x1.0p6F) noexcept {
  return std::abs(a - b) <= threshold;
}

namespace impl {
union ieee_bits {
  float f;
  int32_t i;
  explicit ieee_bits(float f_) : f{f_} {
    static_assert(sizeof(float) == sizeof(int32_t),
                  "Float and int32_t must have the same size");
  }
  explicit ieee_bits(int32_t i_) : i{i_} {}
};
} // namespace impl

/**
 * @brief Convert a linear value to decibel value
 * @param[in] x A decibel value
 * @return @p x converted to linear
 */
inline float db20Fast(float x) noexcept {
  impl::ieee_bits z{x};
  z.i -= 0x3f7893f5;
  return static_cast<float>(z.i) * 0x1.815182p-21F;
}

/**
 * @brief Convert decibel respect to full scale (1.0) to linear value
 * @param[in] x A decibel value
 * @return @p x converted to linear
 */
inline float undb20Fast(float x) noexcept {
  impl::ieee_bits z{static_cast<int32_t>(0x1.542a5a56619bcp+20F * x) +
                    0x3f7893f5};
  return z.f;
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
float lowpass1(float in, float b0, float state) noexcept {
  return (b0 * (in - state)) + state;
}

/**
 * @brief One stage biquad filter
 *
 * @details Implementation of a single stage biqad filter as shown below
 *
 * H(z) = (b0 + b1 * z^-1 + b2 * z^-2) / (1 + b1 * z^-1 + b2 * z^-2)
 *
 * @param[in] coeff Filter coefficients in [b0, b1, b2, a1, a2] form
 * @param[in, out] state Filter states [w1, w2]
 * @param in Input value
 * @return float Output of the filter
 */
inline float biquadFilter(const float coeff[5], float state[2], float in) noexcept{
  float out = (coeff[0] * in) + state[0];
  state[0] = (coeff[1] * in) - (coeff[3] * out) + state[1];
  state[1] = (coeff[2] * in) - (coeff[4] * out);
  return out;
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

} // namespace
#endif // DSP_MATH_HPP_
