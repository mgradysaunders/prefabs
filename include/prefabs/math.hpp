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
#if !DOXYGEN
#if !(__cplusplus >= 201402L)
#error "prefabs/math.hpp requires >=C++14"
#endif // #if !(__cplusplus >= 201402L)
#endif // #if !DOXYGEN
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
 *
 * __C++ version__: >=C++14
 */
/**@{*/

#if !DOXYGEN

template <typename T>
struct numeric_limits_min_squarable;

template <>
struct numeric_limits_min_squarable<float> 
{
    static constexpr float min_squarable() noexcept
    {
        // 2^(-75)
        return 0x1.000002p-75f;
    }
};

template <>
struct numeric_limits_min_squarable<double> 
{
    static constexpr double min_squarable() noexcept
    {
        // 2^(-537.5)
        return 0x1.6a09e667f3bcdp-538;
    }
};

template <>
struct numeric_limits_min_squarable<long double> 
{
    static constexpr long double min_squarable() noexcept
    {
        // 2^(-8223)
        return 0x8.000000000000001p-8226L;
    }
};

#endif // #if !DOXYGEN

/**
 * @brief Wrap `std::numeric_limits`.
 */
template <typename T>
struct numeric_limits : std::numeric_limits<T>
{

    /**
     * @brief For floating point types, minimum squarable value.
     *
     * The minimum squarable value @f$ \varepsilon_{\text{sqr}} @f$ is
     * the minimum positive value satisfying @f$ x \otimes x > 0 @f$.
     */
    template <bool B = numeric_limits::is_iec559>
    static constexpr std::enable_if_t<B, T> min_squarable() noexcept
    {
        return numeric_limits_min_squarable<T>::min_squarable();
    }

    /**
     * @brief For floating point types, minimum invertible value.
     *
     * The minimum invertible value @f$ \varepsilon_{\text{inv}} @f$ is
     * the minimum positive value satisfying @f$ 1 \oslash x < \infty @f$.
     *
     * For IEEE floating point types (and possibly others?),  
     * @f$ \varepsilon_{\text{inv}} =
     *     \varepsilon_{\text{min}} / 4 + \Delta_{\text{denormal}} @f$
     * where @f$ \varepsilon_{\text{min}} @f$ is the minimum 
     * positive normal value and @f$ \Delta_{\text{denormal}} @f$ is 
     * the denormal spacing.
     */
    template <bool B = numeric_limits::is_iec559>
    static constexpr std::enable_if_t<B, T> min_invertible() noexcept
    {
        return numeric_limits::min() / 4 + 
               numeric_limits::denorm_min();
    }

    /**
     * @brief For floating point types, machine epsilon.
     *
     * Machine epsilon @f$ \varepsilon_{\mathrm{m}} @f$, also known 
     * as unit roundoff, is the maximum representable value satisfying 
     * @f$ 1 \oplus x = 1 @f$.
     *
     * For IEEE floating point types (and possibly others?),
     * @f$ \varepsilon_{\text{m}} =
     *     \varepsilon_{\text{std}} / 2 @f$
     * where @f$ \varepsilon_{\text{std}} @f$ is standard epsilon, the
     * difference between 1 and the next representable value.
     */
    template <bool B = numeric_limits::is_iec559>
    static constexpr std::enable_if_t<B, T> machine_epsilon() noexcept
    {
        return numeric_limits::epsilon() / 2;
    }

    /**
     * @brief For floating point types, echelon.
     */
    template <bool B = numeric_limits::is_iec559>
    static constexpr std::enable_if_t<B, T> echelon(unsigned n) noexcept
    {
        return machine_epsilon() * n / (1 - machine_epsilon() * n);
    }
};

/**
 * @brief Numeric constants.
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

#if !DOXYGEN

template <typename T>
struct is_complex : std::false_type
{
};

template <typename T>
struct is_complex<std::complex<T>> : std::true_type
{
};

#endif // #if !DOXYGEN

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
 * @brief Analogous to `std:norm()`, except constexpr.
 */
template <typename T> constexpr T norm(const std::complex<T>& x)
{
    return std::real(x) * std::real(x) + std::imag(x) * std::imag(x);
}

/**
 * @brief Analogous to `std:conj()`, except constexpr.
 */
template <typename T> constexpr std::complex<T> conj(const std::complex<T>& x)
{
    return {std::real(x), -std::imag(x)};
}

/**@}*/

#if !DOXYGEN
#if (__cplusplus >= 201703L)
using std::void_t;
#else
template <typename...>
using void_t = void;
#endif // #if (__cplusplus >= 201703L)
#endif // #if !DOXYGEN

#if !DOXYGEN

template <typename T, typename = void_t<>>
struct conj_traits_has_symm : std::false_type 
{
};

template <typename T>
struct conj_traits_has_symm<T, 
            void_t<decltype(T::symm(
            std::declval<typename T::value_type>()))>> : 
            std::true_type 
{
};

template <typename T, typename = void_t<>>
struct conj_traits_has_skew : std::false_type 
{
};

template <typename T>
struct conj_traits_has_skew<T, 
            void_t<decltype(T::skew(
            std::declval<typename T::value_type>()))>> : 
            std::true_type 
{
};

template <typename T, typename = void_t<>>
struct conj_traits_has_norm : std::false_type 
{
};

template <typename T>
struct conj_traits_has_norm<T, 
            void_t<decltype(T::norm(
            std::declval<typename T::value_type>()))>> : 
            std::true_type 
{
};

template <typename T, typename = void_t<>>
struct conj_traits_norm_type
{
    typedef typename T::value_type type;
};

template <typename T>
struct conj_traits_norm_type<T, void_t<typename T::norm_type>>
{
    typedef typename T::norm_type type;
};

#endif // #if !DOXYGEN

/**
 * @brief Type traits for conjugate policy.
 *
 * A conjugate policy provides at least
 * - a public member type `value_type` and
 * - a public static member function `conj()` with signature equivalent 
 * to `value_type(const value_type&)`.
 *
 * By default, the implementation provides additionally
 * - a public member type `norm_type` equivalent to `value_type`,
 * - a public static member function `symm()` with signature equivalent
 * to `value_type(const value_type&)`,
 * - a public static member function `skew()` with signature equivalent
 * to `value_type(const value_type&)`, and
 * - a public static member function `norm()` with signature equivalent
 * to `norm_type(const value_type&)`.
 *
 * A conjugate policy may override `symm()`, `skew()`,
 * or `norm()` for efficiency.
 */
template <typename T>
struct conj_traits
{
    /**
     * @brief Value type.
     */
    typedef typename T::value_type value_type;

    /**
     * @brief Norm type.
     */
    typedef typename conj_traits_norm_type<T>::type norm_type;

    /**
     * @brief Conjugate.
     */
    __attribute__((always_inline))
    static constexpr value_type conj(const value_type& x)
    {
        return T::conj(x);
    }

#if !DOXYGEN 

    // If traits provides symm, delegate.
    template <bool B = conj_traits_has_symm<T>::value>
    __attribute__((always_inline))
    static constexpr std::enable_if_t<B, value_type> symm(const value_type& x)
    {
        return T::symm(x);
    }

    // If traits provides skew, delegate.
    template <bool B = conj_traits_has_skew<T>::value>
    __attribute__((always_inline))
    static constexpr std::enable_if_t<B, value_type> skew(const value_type& x)
    {
        return T::skew(x);
    }

    // If traits provides norm, delegate.
    template <bool B = conj_traits_has_norm<T>::value>
    __attribute__((always_inline))
    static constexpr std::enable_if_t<B, norm_type> norm(const value_type& x)
    {
        return T::norm(x);
    }

#endif // #if !DOXYGEN

    /**
     * @brief Symmetric part.
     *
     * @f[
     *      \symm(x) = \frac{x + x^*}{2}
     * @f]
     */
    template <bool B = conj_traits_has_symm<T>::value>
    __attribute__((always_inline))
    static constexpr std::enable_if_t<!B, value_type> symm(const value_type& x)
    {
        return value_type(x + T::conj(x)) / value_type(2);
    }

    /**
     * @brief Skew-symmetric part.
     *
     * @f[
     *      \skew(x) = \frac{x - x^*}{2}
     * @f]
     */
    template <bool B = conj_traits_has_skew<T>::value>
    __attribute__((always_inline))
    static constexpr std::enable_if_t<!B, value_type> skew(const value_type& x)
    {
        return value_type(x - T::conj(x)) / value_type(2);
    }

    /**
     * @brief Norm square.
     *
     * @f[
     *      x x^*
     * @f]
     */
    template <bool B = conj_traits_has_norm<T>::value>
    __attribute__((always_inline))
    static constexpr std::enable_if_t<!B, norm_type> norm(const value_type& x)
    {
        return x * T::conj(x);
    }
};

/**
 * @brief Null conjugate policy.
 */
template <typename T>
struct conj_null
{
    /**
     * @brief Value type.
     */
    typedef T value_type;

    /**
     * @brief Conjugate.
     *
     * @f[
     *      x^* = x
     * @f]
     */
    __attribute__((always_inline))
    static constexpr value_type conj(const value_type& x)
    {
        return x;
    }

    /**
     * @brief Symmetric part.
     *
     * @f[
     *      \symm(x) = x
     * @f]
     */
    __attribute__((always_inline))
    static constexpr value_type symm(const value_type& x)
    {
        return x;
    }

    /**
     * @brief Skew-symmetric part.
     *
     * @f[
     *      \skew(x) = 0
     * @f]
     */
    __attribute__((always_inline))
    static constexpr value_type skew(const value_type& x)
    {
        (void) x;
        return 0;
    }

    /**
     * @brief Norm square.
     *
     * @f[
     *      x x^* = x^2
     * @f]
     */
    __attribute__((always_inline))
    static constexpr value_type norm(const value_type& x)
    {
        return x * x;
    }
};

/**
 * @brief Imag conjugate policy.
 */
template <typename T, typename = void_t<>>
struct conj_imag;

/**
 * @brief Imag conjugate policy (arithmetic).
 */
template <typename T>
struct conj_imag<T, 
            void_t<
                std::enable_if_t<
                std::is_arithmetic<T>::value, T>>> : conj_null<T>
{
};

/**
 * @brief Imag conjugate policy (complex).
 */
template <typename T>
struct conj_imag<std::complex<T>>
{
    /**
     * @brief Value type.
     */
    typedef std::complex<T> value_type;

    /**
     * @brief Norm type.
     */
    typedef T norm_type;

    /**
     * @brief Conjugate.
     *
     * @f[
     *      (a + i b)^* = a - i b
     * @f]
     */
    __attribute__((always_inline))
    static constexpr value_type conj(const value_type& x)
    {
        return {+x.real(), -x.imag()};
    }

    /**
     * @brief Symmetric part.
     *
     * @f[
     *      \symm(a + i b) = a
     * @f]
     */
    __attribute__((always_inline))
    static constexpr value_type symm(const value_type& x)
    {
        return {+x.real(), 0};
    }

    /**
     * @brief Skew-symmetric part.
     *
     * @f[
     *
     *      \skew(a + i b) = i b
     *
     * @f]
     */
    __attribute__((always_inline))
    static constexpr value_type skew(const value_type& x)
    {
        return {0, +x.imag()};
    }

    /**
     * @brief Norm square.
     *
     * @f[
     *      (a + i b)
     *      (a + i b)^* = a^2 + b^2
     * @f]
     */
    __attribute__((always_inline))
    static constexpr norm_type norm(const value_type& x)
    {
        return x.real() * x.real() + x.imag() * x.imag();
    }
};

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

} // namespace pr

#if !DOXYGEN
#include "math.inl"
#endif // #if !DOXYGEN

#endif // #ifndef PREFABS_MATH_HPP
