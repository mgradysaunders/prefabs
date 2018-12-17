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
#ifndef PREFABS_MATH_HPP
#define PREFABS_MATH_HPP

// for std::fabs, std::sqrt, ...
#include <cmath>

// for std::complex
#include <complex>

// for std::numeric_limits
#include <limits>

namespace pr {

/**
 * @defgroup math Math
 *
 * `<prefabs/math.hpp>`
 */
/**@{*/

/**
 * @brief Wrap `std::numeric_limits`.
 */
template <typename T>
struct numeric_limits : std::numeric_limits<T>
{
    /**
     * @brief For floating point types, machine epsilon.
     */
    static constexpr T machine_epsilon() noexcept
    {
        return numeric_limits::epsilon() / 2;
    }

    /**
     * @brief For floating point types, echelon.
     */
    static constexpr T echelon(unsigned n) noexcept
    {
        return machine_epsilon() * n / (1 - machine_epsilon() * n);
    }
};

/**
 * @brief Useful numeric constants.
 */
template <typename T>
struct numeric_constants
{
    /**
     * @brief @f$ e @f$.
     */
    static constexpr T M_e() noexcept
    {
        return T(2.7182818284590452353602874713526625L);
    }

    /**
     * @brief @f$ \log_2(e) @f$.
     */
    static constexpr T M_log2e() noexcept
    {
        return T(1.4426950408889634073599246810018921L);
    }

    /**
     * @brief @f$ \log_{10}(e) @f$.
     */
    static constexpr T M_log10e() noexcept
    {
        return T(0.4342944819032518276511289189166051L);
    }

    /**
     * @brief @f$ \log_e(2) @f$.
     */
    static constexpr T M_ln2() noexcept
    {
        return T(0.6931471805599453094172321214581766L);
    }

    /**
     * @brief @f$ \log_e(10) @f$.
     */
    static constexpr T M_ln10() noexcept
    {
        return T(2.3025850929940456840179914546843642L);
    }

    /**
     * @brief @f$ \pi @f$.
     */
    static constexpr T M_pi() noexcept
    {
        return T(3.1415926535897932384626433832795029L);
    }

    /**
     * @brief @f$ \pi/2 @f$.
     */
    static constexpr T M_pi_2() noexcept
    {
        return T(1.5707963267948966192313216916397514L);
    }

    /**
     * @brief @f$ \pi/4 @f$.
     */
    static constexpr T M_pi_4() noexcept
    {
        return T(0.7853981633974483096156608458198757L);
    }

    /**
     * @brief @f$ 1/\pi @f$.
     */
    static constexpr T M_1_pi() noexcept
    {
        return T(0.3183098861837906715377675267450287L);
    }

    /**
     * @brief @f$ 2/\pi @f$.
     */
    static constexpr T M_2_pi() noexcept
    {
        return T(0.6366197723675813430755350534900574L);
    }

    /**
     * @brief @f$ 2/\sqrt{\pi} @f$.
     */
    static constexpr T M_2_sqrtpi() noexcept
    {
        return T(1.1283791670955125738961589031215452L);
    }

    /**
     * @brief @f$ \sqrt{2} @f$.
     */
    static constexpr T M_sqrt2() noexcept
    {
        return T(1.4142135623730950488016887242096981L);
    }

    /**
     * @brief @f$ \sqrt{1/2} @f$.
     */
    static constexpr T M_sqrt1_2() noexcept
    {
        return T(0.7071067811865475244008443621048490L);
    }
};

/**
 * @brief Specialize `std::complex<T>`.
 */
template <typename T>
struct numeric_constants<std::complex<T>> : numeric_constants<T>
{
};

/**
 * @name Complex accessors (arithmetic)
 */
/**@{*/

/**
 * @brief Analogous to `std::real()`, except do not promote to floating point.
 */
template <typename T>
constexpr std::enable_if_t<std::is_arithmetic<T>::value, T> real(T x)
{
    return x;
}

/**
 * @brief Analogous to `std::imag()`, except do not promote to floating point.
 */
template <typename T>
constexpr std::enable_if_t<std::is_arithmetic<T>::value, T> imag(T x)
{
    (void) x;
    return 0;
}

/**
 * @brief Analogous to `std::norm()`, except do not promote to floating point.
 */
template <typename T>
constexpr std::enable_if_t<std::is_arithmetic<T>::value, T> norm(T x)
{
    return x * x;
}

/**
 * @brief Analogous to `std::conj()`, except do not promote to `std::complex`.
 */
template <typename T>
constexpr std::enable_if_t<std::is_arithmetic<T>::value, T> conj(T x)
{
    return x;
}

/**@}*/

/**
 * @name Complex accessors (complex)
 */
/**@{*/

/**
 * @brief Wrap `std::real()`.
 */
template <typename T> constexpr T real(const std::complex<T>& x)
{
    return std::real(x);
}

/**
 * @brief Wrap `std::imag()`.
 */
template <typename T> constexpr T imag(const std::complex<T>& x)
{
    return std::imag(x);
}

/**
 * @brief Analgous to `std:norm()`, except constexpr.
 */
template <typename T> constexpr T norm(const std::complex<T>& x)
{
    return std::real(x) * std::real(x) + std::imag(x) * std::imag(x);
}

/**
 * @brief Analgous to `std:conj()`, except constexpr.
 */
template <typename T> constexpr std::complex<T> conj(const std::complex<T>& x)
{
    return {std::real(x), -std::imag(x)};
}

/**@}*/

/**
 * @name Misc
 */
/**@{*/

/**
 * @brief Raise to integer power.
 */
template <typename T>
constexpr T nthpow(T x, int n)
{
    if (n < 0) {
        return T(1) / nthpow(x, -n);
    }
    else {
        T y(1);
        while (n-- > 0) y *= x;
        return y;
    }
}

/**@}*/

/**@}*/

#include "math.inl"

} // namespace pr

#endif // #ifndef PREFABS_MATH_HPP
