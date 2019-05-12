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
#if !(__cplusplus >= 201402L)
#error "pr/quadmath.hpp requires >=C++14"
#endif // #if !(__cplusplus >= 201402L)
#if !(__GNUC__ && !__clang__)
#error "pr/quadmath.hpp requires GCC"
#endif // #if !(__GNUC__ && !__clang__)
#endif // #if !DOXYGEN
#pragma once
#ifndef PR_QUADMATH_HPP
#define PR_QUADMATH_HPP

// for std::memcpy
#include <cstring>

// for quadmath
#include <quadmath.h>

#include <pr/math.hpp>

namespace pr {

/**
 * @defgroup quadmath Quadmath
 *
 * `<pr/quadmath.hpp>`
 *
 * __C++ version__: >=C++14 (`-std=gnu++14`, `-lquadmath`)
 */
/**@{*/

#if !DOXYGEN

template <>
struct is_complex<__complex128> : std::true_type
{
};

#endif // #if !DOXYGEN

/**
 * @brief Specialize `__float128`.
 */
template <>
struct numeric_limits<__float128>
{
    /**
     * @brief Set `has_infinity`.
     */
    static constexpr bool has_infinity = true;

    /**
     * @brief Set `has_quiet_NaN`.
     */
    static constexpr bool has_quiet_NaN = true;

    /**
     * @brief Set `has_signaling_NaN`.
     */
    static constexpr bool has_signaling_NaN = true;

    /**
     * @brief Set `has_denorm`.
     */
    static constexpr std::float_denorm_style has_denorm =
                     std::float_denorm_style::denorm_present;

    /**
     * @brief Wrap `FLT128_MIN_EXP`.
     */
    static constexpr int min_exponent = FLT128_MIN_EXP;

    /**
     * @brief Wrap `FLT128_MAX_EXP`.
     */
    static constexpr int max_exponent = FLT128_MAX_EXP;

    /**
     * @brief Wrap `FLT128_MIN_10_EXP`.
     */
    static constexpr int min_exponent10 = FLT128_MIN_10_EXP;

    /**
     * @brief Wrap `FLT128_MAX_10_EXP`.
     */
    static constexpr int max_exponent10 = FLT128_MAX_10_EXP;

    /**
     * @brief Wrap `FLT128_MANT_DIG = 113`.
     */
    static constexpr int digits = FLT128_MANT_DIG;

    /**
     * @brief Wrap `FLT128_DIG = 33`.
     */
    static constexpr int digits10 = FLT128_DIG;

    /**
     * @brief From `FLT128_MANT_DIG`, infer `max_digits10 = 36`.
     */
    static constexpr int max_digits10 = 36;

    /**
     * @brief Analogous to `pr::numeric_limits::min_squarable()`.
     */
    static constexpr __float128 min_squarable() noexcept
    {
        // 2^(-8247.5)
        return 0x1.6a09e667f3bcc908b2fb1366ea96p-8248q;
    }

    /**
     * @brief Analogous to `pr::numeric_limits::min_invertible()`.
     */
    static constexpr __float128 min_invertible() noexcept
    {
        return FLT128_MIN / 4 + FLT128_DENORM_MIN;
    }

    /**
     * @brief Wrap `FLT128_MIN`.
     */
    static constexpr __float128 min() noexcept
    {
        return FLT128_MIN;
    }

    /**
     * @brief Wrap `FLT128_MAX`.
     */
    static constexpr __float128 max() noexcept
    {
        return FLT128_MAX;
    }

    /**
     * @brief Wrap `FLT128_EPSILON`.
     */
    static constexpr __float128 epsilon() noexcept
    {
        return FLT128_EPSILON;
    }

    /**
     * @brief For floating point types, machine echelon.
     */
    static constexpr __float128 machine_epsilon() noexcept
    {
        return epsilon() / 2;
    }

    /**
     * @brief For floating point types, echelon.
     */
    static constexpr __float128 echelon(unsigned n) noexcept
    {
        return machine_epsilon() * n / (1 - machine_epsilon() * n);
    }

    /**
     * @brief Wrap `HUGE_VALQ`.
     */
    static __float128 infinity() noexcept
    {
        return HUGE_VALQ;
    }

    /**
     * @brief Wrap `nanq()`.
     */
    static __float128 quiet_NaN() noexcept
    {
        return ::nanq("1");
    }

    /**
     * @brief Wrap `nanq()`, flip signaling bit.
     */
    static __float128 signaling_NaN() noexcept
    {
        __float128 f = ::nanq("1");
        __int128 i;
        std::memcpy(&i, &f, sizeof(f));
        i ^= __int128(1) << 111;
        std::memcpy(&f, &i, sizeof(i));
        return f;
    }

    /**
     * @brief Wrap `FLT128_DENORM_MIN`.
     */
    static constexpr __float128 denorm_min() noexcept
    {
        return FLT128_DENORM_MIN;
    }
};

/**
 * @brief Specialize `__float128`.
 */
template <>
struct numeric_constants<__float128>
{
    /**
     * @brief Wrap `M_Eq`.
     */
    static constexpr __float128 M_e() noexcept
    {
        return M_Eq;
    }

    /**
     * @brief Wrap `M_LOG2Eq`.
     */
    static constexpr __float128 M_log2e() noexcept
    {
        return M_LOG2Eq;
    }

    /**
     * @brief Wrap `M_LOG10Eq`.
     */
    static constexpr __float128 M_log10e() noexcept
    {
        return M_LOG10Eq;
    }

    /**
     * @brief Wrap `M_LN2q`.
     */
    static constexpr __float128 M_ln2() noexcept
    {
        return M_LN2q;
    }

    /**
     * @brief Wrap `M_LN10q`.
     */
    static constexpr __float128 M_ln10() noexcept
    {
        return M_LN10q;
    }

    /**
     * @brief Wrap `M_PIq`.
     */
    static constexpr __float128 M_pi() noexcept
    {
        return M_PIq;
    }

    /**
     * @brief Wrap `M_PI_2q`.
     */
    static constexpr __float128 M_pi_2() noexcept
    {
        return M_PI_2q;
    }

    /**
     * @brief Wrap `M_PI_4q`.
     */
    static constexpr __float128 M_pi_4() noexcept
    {
        return M_PI_4q;
    }

    /**
     * @brief Wrap `M_1_PIq`.
     */
    static constexpr __float128 M_1_pi() noexcept
    {
        return M_1_PIq;
    }

    /**
     * @brief Wrap `M_2_PIq`.
     */
    static constexpr __float128 M_2_pi() noexcept
    {
        return M_2_PIq;
    }

    /**
     * @brief Wrap `M_2_SQRTPIq`.
     */
    static constexpr __float128 M_2_sqrtpi() noexcept
    {
        return M_2_SQRTPIq;
    }

    /**
     * @brief Wrap `M_SQRT2q`.
     */
    static constexpr __float128 M_sqrt2() noexcept
    {
        return M_SQRT2q;
    }

    /**
     * @brief Wrap `M_SQRT1_2q`.
     */
    static constexpr __float128 M_sqrt1_2() noexcept
    {
        return M_SQRT1_2q;
    }

    /**
     * @brief @f$ \gamma @f$ (Euler's constant).
     */
    static constexpr __float128 M_gamma() noexcept
    {
        return 0.57721566490153286060651209008240243104215q;
    }

    /**
     * @brief @f$ h @f$ (Planck's constant).
     *
     * @f[
     *      h = 6.62607015\times10^{-34}\,\mathrm{J}\cdot\mathrm{s}
     * @f]
     */
    static constexpr __float128 M_h() noexcept
    {
        return 6.62607015e-34q;
    }

    /**
     * @brief @f$ c @f$ (light speed).
     *
     * @f[
     *      c = 299792458\,\mathrm{m}/\mathrm{s}
     * @f]
     */
    static constexpr __float128 M_c() noexcept
    {
        return 299792458.0q;
    }
};

/**
 * @brief Specialize `__complex128`.
 */
template <>
struct numeric_constants<__complex128> : numeric_constants<__float128>
{
};

/**@}*/

} // namespace pr

#if !DOXYGEN
#include "quadmath.inl"
#endif // #if !DOXYGEN

namespace pr {

/**
 * @addtogroup quadmath
 */
/**@{*/

/**
 * @name Sign/step functions (__float128)
 */
/**@{*/

/**
 * @brief Sign function.
 *
 * @f[
 *      \operatorname{sign}(x) =
 *      \begin{cases}
 *          -1 & x < 0
 *      \\  +1 & x > 0
 *      \end{cases}
 * @f]
 *
 * @note
 * Uses `::copysignq()`. Hence,
 * - `pr::sign(-0.0) = -1.0` and
 * - `pr::sign(+0.0) = +1.0`.
 */
__attribute__((always_inline))
inline __float128 sign(__float128 x)
{
    return ::copysignq(1.0q, x);
}

/**
 * @brief Step function.
 *
 * @f[
 *      \operatorname{step}(x) =
 *      \frac{1}{2} \operatorname{sign}(x) +
 *      \frac{1}{2} =
 *      \begin{cases}
 *          0 & x < 0
 *      \\  1 & x > 0
 *      \end{cases}
 * @f]
 *
 * @note
 * Uses `::signbitq()`. Hence,
 * - `pr::step(-0.0) = 0.0` and
 * - `pr::step(+0.0) = 1.0`.
 */
__attribute__((always_inline))
inline __float128 step(__float128 x)
{
    if (::signbitq(x)) {
        return 0.0q;
    }
    else {
        return 1.0q;
    }
}

/**@}*/

/**
 * @name Sign/step functions (__complex128)
 */
/**@{*/

/**
 * @brief Sign function.
 *
 * @f[
 *      \operatorname{sign}(x) =
 *      \begin{cases}
 *          1     & x =   0
 *      \\  x/|x| & x \ne 0
 *      \end{cases}
 * @f]
 *
 * @note
 * If `pr::imag(x) == 0`, computes `pr::sign(pr::real(x))` and
 * preserves sign of `pr::imag(x)`.
 */
__attribute__((always_inline))
inline __complex128 sign(const __complex128& x)
{
    if (pr::imag(x) == 0.0q) {
        return {
            pr::sign(pr::real(x)),
            pr::imag(x)
        };
    }
    else {
        return x / pr::abs(x);
    }
}

/**
 * @brief Step function.
 *
 * @f[
 *      \operatorname{step}(x) =
 *      \frac{1}{2} \operatorname{sign}(x) +
 *      \frac{1}{2}
 * @f]
 *
 * @note
 * If `pr::imag(x) == 0`, computes `pr::step(pr::real(x))` and
 * preserves sign of `pr::imag(x)`.
 */
__attribute__((always_inline))
inline __complex128 step(const __complex128& x)
{
    if (pr::imag(x) == 0.0q) {
        return {
            pr::step(pr::real(x)),
            pr::imag(x)
        };
    }
    else {
        return pr::sign(x) * 0.5q + 0.5q;
    }
}

/**@}*/

/**@}*/

} // namespace pr

#endif // #ifndef PR_QUADMATH_HPP
