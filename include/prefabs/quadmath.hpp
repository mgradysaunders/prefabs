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
#ifndef PREFABS_QUADMATH_HPP
#define PREFABS_QUADMATH_HPP

#if !(__GNUC__ && !__clang__)
#error "GCC necessary for quadmath"
#endif // #if !(__GNUC__ && !__clang__)

// for std::memcpy
#include <cstring>

// for quadmath
#include <quadmath.h>

#include <prefabs/math.hpp>

namespace pr {

/**
 * @defgroup quadmath Quadmath
 *
 * `<prefabs/quadmath.hpp>`
 */
/**@{*/

/**
 * @brief Enable type `U` if type `T` is `__float128`.
 */
template <typename T, typename U = T>
using enable_float128 = 
            std::enable_if<std::is_same<T, __float128>::value, U>;

/**
 * @brief Enable type `U` if type `T` is `__complex128`.
 */
template <typename T, typename U = T>
using enable_complex128 = 
            std::enable_if<std::is_same<T, __complex128>::value, U>;

/**@}*/

#include <prefabs/quadmath.inl>

/**
 * @addtogroup quadmath
 */
/**@{*/

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

/**@}*/

} // namespace pr

#endif // #ifndef PREFABS_QUADMATH_HPP
