/* Copyright (c) 2018-19 M. Grady Saunders
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
#if !DOXYGEN
#if !(__cplusplus >= 201703L)
#error "preform/float_helpers.hpp requires >=C++17"
#endif // #if !(__cplusplus >= 201703L)
#endif // #if !DOXYGEN
#pragma once
#ifndef PREFORM_FLOAT_HELPERS_HPP
#define PREFORM_FLOAT_HELPERS_HPP

// for assert
#include <cassert>

// for std::swap
#include <utility>

// for std::enable_if, std::is_floating_point
#include <type_traits>

#include <preform/math.hpp>

namespace pr {

/**
 * @defgroup float_helpers Float helpers
 *
 * `<preform/float_helpers.hpp>`
 *
 * __C++ version__: >=C++17
 */
/**@{*/

/**
 * @brief Increment to next representable value.
 */
template <typename T>
inline std::enable_if_t<
       std::is_floating_point<T>::value, T> finc(T x)
{
    return pr::nextafter(x, +pr::numeric_limits<T>::infinity());
}

/**
 * @brief Decrement to next representable value.
 */
template <typename T>
inline std::enable_if_t<
       std::is_floating_point<T>::value, T> fdec(T x)
{
    return pr::nextafter(x, -pr::numeric_limits<T>::infinity());
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
inline std::enable_if_t<
       std::is_floating_point<T>::value, T> fclamp(T x, T a, T b)
{
    if (a > b) {
        std::swap(a, b);
    }
    return pr::fmax(a, 
           pr::fmin(x, b));
}

/**
 * @brief Repeat.
 *
 * @image html frepeat.svg
 *
 * Wrap value @f$ x @f$ in range @f$ [a, b) @f$.
 *
 * @note
 * If @f$ a > b @f$, the implementation is equivalent to
 * to @f$ \operatorname{frepeat}(-x; b, a) @f$.
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
inline std::enable_if_t<
       std::is_floating_point<T>::value, T> frepeat(T x, T a, T b)
{
    if (a > b) {
        std::swap(a, b);
        x = -x;
    }
    T x0 = x - a;
    T b0 = b - a;
    T r0 = pr::remainder(x0, b0);
    if (r0 < T(0)) {
        r0 += b0;
    }
    return r0 + a;
}

/**
 * @brief Mirror.
 *
 * @image html fmirror.svg
 *
 * Wrap value @f$ x @f$ in range @f$ [a, b) @f$, with mirroring.
 *
 * @note
 * If @f$ a > b @f$, the implementation is equivalent
 * to @f$ \operatorname{fmirror}(-x; b, a) @f$.
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
inline std::enable_if_t<
       std::is_floating_point<T>::value, T> fmirror(T x, T a, T b)
{
    if (a > b) {
        std::swap(a, b);
        x = -x;
    }
    T x0 = x - a;
    T b0 = b - a;
    int q0;
    T r0 = pr::remquo(x0, b0, &q0);
    if (r0 < T(0)) {
        r0 += b0;
        q0++;
    }
    if (q0 & 1) {
        r0 = b0 - r0;
    }
    return r0 + a;
}

/**
 * @brief Cycle mode.
 */
enum class cycle_mode
{
    /**
     * @brief Use `fclamp()`.
     */
    clamp,

    /**
     * @brief Use `frepeat()`.
     */
    repeat,

    /**
     * @brief Use `fmirror()`.
     */
    mirror
};

/**
 * @brief Cycle.
 *
 * @param[in] x
 * Value
 *
 * @param[in] a
 * Minimum value.
 *
 * @param[in] b
 * Maximum value.
 *
 * @param[in] mode
 * Cycle mode.
 */
template <typename T>
inline std::enable_if_t<
       std::is_floating_point<T>::value, T> fcycle(T x, T a, T b,
                                             cycle_mode mode)
{
    switch (mode) {
        default:
        case cycle_mode::clamp: return fclamp(x, a, b);
        case cycle_mode::repeat: return frepeat(x, a, b);
        case cycle_mode::mirror: return fmirror(x, a, b);
    }

    // Unreachable.
    return T();
}

/**
 * @brief Stretch floating point values to signed/unsigned 
 * normalized integer values.
 */
template <typename U, typename T>
inline std::enable_if_t<
       std::is_arithmetic<T>::value &&
       std::is_arithmetic<U>::value, U> fstretch(T x)
{
    if constexpr (std::is_floating_point<T>::value &&
                  std::is_floating_point<U>::value) {
        // Float -> Float.
        return static_cast<U>(x);
    }
    else if constexpr (std::is_floating_point<T>::value &&
                       std::is_unsigned<U>::value) {
        // Float -> Unorm.
        return 
            static_cast<U>(pr::round(
            static_cast<double>(pr::numeric_limits<U>::max()) * 
            static_cast<double>(pr::fclamp(x, T(0), T(1)))));
    }
    else if constexpr (std::is_floating_point<T>::value &&
                       std::is_integral<U>::value) {
        // Float -> Snorm.
        return
            static_cast<U>(pr::round(
            static_cast<double>(pr::numeric_limits<U>::max()) * 
            static_cast<double>(pr::fclamp(x, T(-1), T(1) - 
                                pr::numeric_limits<T>::machine_epsilon()))));
    }
    else if constexpr (std::is_unsigned<T>::value &&
                       std::is_floating_point<U>::value) {
        // Unorm -> Float.
        return
            static_cast<U>(
            static_cast<double>(x) /
            static_cast<double>(pr::numeric_limits<T>::max()));
    }
    else if constexpr (std::is_integral<T>::value &&
                       std::is_floating_point<U>::value) {
        // Snorm -> Float.
        return
            static_cast<U>(
            static_cast<double>(x) /
            static_cast<double>(pr::numeric_limits<T>::max()));
    }
    else if constexpr (std::is_integral<T>::value &&
                       std::is_integral<U>::value) {
        // Xnorm -> Xnorm.
        return 
            fstretch<double, U>(
            fstretch<T, double>(x));
    }
    else {
        // Unreachable.
        return U();
    }
}

/**@}*/

} // namespace pr

#endif // #ifndef PREFORM_FLOAT_HELPERS_HPP
