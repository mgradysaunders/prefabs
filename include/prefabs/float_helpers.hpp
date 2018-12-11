/* Copyright (c) 2018 M. Grady Saunders
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 
 *   1. Redistributions of source code must retain the above
 *      copyright notice, this list of conditions and the following
 *      disclaimer.
 * 
 *   2. Redistributions in binary form must reproduce the above
 *      copyright notice, this list of conditions and the following
 *      disclaimer in the documentation and/or other materials
 *      provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 * OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/*+-+*/
#pragma once
#ifndef PREFABS_FLOAT_HELPERS_HPP
#define PREFABS_FLOAT_HELPERS_HPP

// for assert
#include <cassert>

// for std::remquo, std::nextafter, ...
#include <cmath>

// for std::numeric_limits
#include <limits>

// for std::enable_if, std::is_floating_point
#include <type_traits>

namespace pr {

/**
 * @defgroup float_helpers Float helpers
 */
/**@{*/

/**
 * @brief Enable type `U` if type `T` is floating point.
 */
template <typename T, typename U = T>
using enable_float = std::enable_if<std::is_floating_point<T>::value, U>;

/**
 * @brief Increment to next representable value.
 */
template <typename T>
inline typename enable_float<T>::type finc(T x)
{
    return std::nextafter(x, +std::numeric_limits<T>::infinity());
}

/**
 * @brief Decrement to next representable value.
 */
template <typename T>
inline typename enable_float<T>::type fdec(T x)
{
    return std::nextafter(x, -std::numeric_limits<T>::infinity());
}

/**
 * @brief Clamp.
 *
 * @param[in] x
 * Value.
 *
 * @param[in] a
 * Minimum value.
 *
 * @param[in] b
 * Maximum value.
 */
template <typename T>
inline typename enable_float<T>::type fclamp(T x, T a, T b)
{
    assert(a <= b);
    return std::fmax(a, std::fmin(x, b));
}

/**
 * @brief Repeat.
 *
 * Wrap value @f$ x @f$ in range @f$ [a, b) @f$.
 *
 * @param[in] x
 * Value.
 *
 * @param[in] a
 * Minimum value.
 *
 * @param[in] b
 * Maximum value.
 */
template <typename T>
inline typename enable_float<T>::type frepeat(T x, T a, T b)
{
    if (a > b) {
        std::swap(a, b);
        x = -x;
    }
    T x0 = x - a;
    T b0 = b - a;
    T r0 = std::remainder(x0, b0);
    if (r0 < T(0)) {
        r0 += b0;
    }
    return r0 + a;
}

/**
 * @brief Mirror.
 *
 * Wrap value @f$ x @f$ in range @f$ [a, b) @f$, with mirroring.
 *
 * @param[in] x
 * Value.
 *
 * @param[in] a
 * Minimum value.
 *
 * @param[in] b
 * Maximum value.
 */
template <typename T>
inline typename enable_float<T>::type fmirror(T x, T a, T b)
{
    if (a > b) {
        std::swap(a, b);
        x = -x;
    }
    T x0 = x - a;
    T b0 = b - a;
    int q0;
    T r0 = std::remquo(x0, b0, &q0);
    if (r0 < T(0)) {
        r0 += b0;
        q0++;
    }
    if (q0 & 1) {
        r0 = b0 - r0;
    }
    return r0 + a;
}

/**@}*/

} // namespace pr

#endif // #ifndef PREFABS_FLOAT_HELPERS_HPP
