/* Copyright (c) 2018-20 M. Grady Saunders
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
#error "preform/math.hpp requires >=C++14"
#endif // #if !(__cplusplus >= 201402L)
#endif // #if !DOXYGEN
#pragma once
#ifndef PREFORM_MATH_HPP
#define PREFORM_MATH_HPP

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
 * `<preform/math.hpp>`
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
    #if (__cplusplus >= 201703L)
        // 2^(-75)
        return 0x1.000002p-75f;
    #else
        return 2.646978276e-23f;
    #endif // #if (__cplusplus >= 201703L)
    }
};

template <>
struct numeric_limits_min_squarable<double>
{
    static constexpr double min_squarable() noexcept
    {
    #if (__cplusplus >= 201703L)
        // 2^(-537.5)
        return 0x1.6a09e667f3bcdp-538;
    #else
        return 1.57172778470262880e-162;
    #endif // #if (__cplusplus >= 201703L)
    }
};

template <>
struct numeric_limits_min_squarable<long double>
{
    static constexpr long double min_squarable() noexcept
    {
    #if (__cplusplus >= 201703L)
        // 2^(-8223)
        return 0x8.000000000000001p-8226L;
    #else
        return 4.269191686890197838238e-2476L;
    #endif // #if (__cplusplus >= 201703L)
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

    /**
     * @brief @f$ \gamma @f$ (Euler's constant).
     *
     * @f[
     *      \gamma =
     *          \lim_{n\to\infty}
     *          \left(-\log(n) + \sum_{k=1}^{n}\frac{1}{k}\right)
     * @f]
     */
    static constexpr T M_gamma() noexcept
    {
        return T(0.5772156649015328606065120900824024L);
    }

    /**
     * @brief @f$ h @f$ (Planck's constant).
     *
     * @f[
     *      h = 6.62607015\times10^{-34}\,\mathrm{J}\cdot\mathrm{s}
     * @f]
     */
    static constexpr T M_h() noexcept
    {
        return T(6.62607015e-34L);
    }

    /**
     * @brief @f$ c @f$ (light speed).
     *
     * @f[
     *      c = 299792458\,\mathrm{m}/\mathrm{s}
     * @f]
     */
    static constexpr T M_c() noexcept
    {
        return T(299792458);
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

/**
 * @name Float checks (complex)
 */
/**@{*/

/**
 * @brief Any Inf?
 */
template <typename T>
__attribute__((always_inline))
inline bool isinf(const std::complex<T>& x)
{
    return pr::isinf(x.real()) || pr::isinf(x.imag());
}

/**
 * @brief Any NaN?
 */
template <typename T>
__attribute__((always_inline))
inline bool isnan(const std::complex<T>& x)
{
    return pr::isnan(x.real()) || pr::isnan(x.imag());
}

/**
 * @brief All finite?
 */
template <typename T>
__attribute__((always_inline))
inline bool isfinite(const std::complex<T>& x)
{
    return pr::isfinite(x.real()) && pr::isfinite(x.imag());
}

/**
 * @brief All normal?
 */
template <typename T>
__attribute__((always_inline))
inline bool isnormal(const std::complex<T>& x)
{
    return pr::isnormal(x.real()) && pr::isnormal(x.imag());
}

/**@}*/

/**
 * @name Sign/step functions (arithmetic)
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
 * Uses `std::copysign()`. Hence,
 * - `pr::sign(-0.0) = -1.0` and
 * - `pr::sign(+0.0) = +1.0`.
 */
template <typename T>
__attribute__((always_inline))
inline auto sign(T x) -> decltype(std::copysign(T(1), x))
{
    return std::copysign(T(1), x);
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
 * Uses `std::signbit()`. Hence,
 * - `pr::step(-0.0) = 0.0` and
 * - `pr::step(+0.0) = 1.0`.
 */
template <typename T>
__attribute__((always_inline))
inline auto step(T x) -> decltype(std::signbit(x) ? T(0) : T(1))
{
    return std::signbit(x) ? T(0) : T(1);
}

/**@}*/

/**
 * @name Sign/step functions (complex)
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
template <typename T>
__attribute__((always_inline))
inline std::complex<T> sign(const std::complex<T>& x)
{
    if (pr::imag(x) == T(0)) {
        return {
            pr::sign(pr::real(x)),
            pr::imag(x)
        };
    }
    else {
        return x / std::abs(x);
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
template <typename T>
__attribute__((always_inline))
inline std::complex<T> step(const std::complex<T>& x)
{
    if (pr::imag(x) == T(0)) {
        return {
            pr::step(pr::real(x)),
            pr::imag(x)
        };
    }
    else {
        return pr::sign(x) * T(0.5) + T(0.5);
    }
}

/**@}*/

/**@}*/

} // namespace pr

#if !DOXYGEN
#include "math.inl"
#endif // #if !DOXYGEN

namespace pr {

/**
 * @addtogroup math
 */
/**@{*/

/**
 * @name Misc functions
 */
/**@{*/

/**
 * @brief Analogous to `std::min()`, except only for numbers.
 */
template <typename T>
constexpr std::enable_if_t<
          std::is_arithmetic<T>::value, T> min(const T& x, const T& y)
{
    return x < y ? x : y;
}

/**
 * @brief Analogous to `std::max()`, except only for numbers.
 */
template <typename T>
constexpr std::enable_if_t<
          std::is_arithmetic<T>::value, T> max(const T& x, const T& y)
{
    return x < y ? y : x;
}

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

/**
 * @name Misc special functions
 */
/**@{*/

/**
 * @brief Log beta function.
 *
 * @f[
 *      \log\beta(p, q) =
 *      \log\frac{\Gamma(p)\Gamma(q)}{\Gamma(p + q)}
 * @f]
 */
template <typename T>
__attribute__((always_inline))
inline std::enable_if_t<
       std::is_floating_point<T>::value, T> lbeta(T p, T q)
{
    if (p == 0 ||
        q == 0) {
        return 0;
    }
    else {
        return pr::lgamma(p) + pr::lgamma(q) - pr::lgamma(p + q);
    }
}

/**
 * @brief Beta function.
 *
 * @f[
 *      \beta(p, q) =
 *      \frac{\Gamma(p)\Gamma(q)}{\Gamma(p + q)}
 * @f]
 */
template <typename T>
__attribute__((always_inline))
inline std::enable_if_t<
       std::is_floating_point<T>::value, T> beta(T p, T q)
{
    return pr::exp(pr::lbeta(p, q));
}

/**
 * @brief Regularized incomplete beta function.
 *
 * @f[
 *      I(x;p,q) = \frac{1}{\beta(p, q)}
 *                 \int_0^x t^{p-1} (1 - t)^{q-1}\, dt
 * @f]
 * for
 * - @f$ x \in [0, 1] @f$
 * - @f$ p \in (0, \infty) @f$
 * - @f$ q \in (0, \infty) @f$
 *
 * @throw std::runtime_error
 * If implementation fails to converge.
 *
 * @see
 * Codeplea's [article][1].
 * [1]: https://codeplea.com/incomplete-beta-function-c
 */
template <typename T>
inline std::enable_if_t<
       std::is_floating_point<T>::value, T> betai(T x, T p, T q)
{
    // Limit case?
    if (x == T(0) ||
        x == T(1)) {
        return x;
    }
    // Invalid?
    else if (!(x > T(0) && x < T(1) &&
               p > T(0) && q > T(0))) {
        return pr::numeric_limits<T>::quiet_NaN();
    }
    // Simplify.
    else if (q == T(1)) {
        return pr::pow(x, p);
    }
    // Simplify.
    else if (p == T(1)) {
        return 1 - pr::pow(1 - x, q);
    }
    else {
        // Lentz's continued fraction convergence condition.
        bool flip = x > (p + 1) / (p + q + 2);
        if (flip) {
            x = 1 - x;
            std::swap(p, q);
        }

        // Lentz's continued fraction.
        T f = 2;
        T c = 2, d = 1;
        int k = 1;
        int m = 0;
        for (; k < 200; k++) {

            T a;
            m = k / 2;
            if (k % 2 == 0) {
                a = +x * ((m * (q - m)) /
                         ((p + (2 * m - 1)) * (p + 2 * m)));
            }
            else {
                a = -x * ((p + m) * (p + q + m) /
                         ((p + (2 * m + 1)) * (p + 2 * m)));
            }

            d = 1 + a * d;
            if (pr::fabs(d) < // TODO Is this ever negative?
                    pr::numeric_limits<T>::min_invertible()) {
                d = pr::numeric_limits<T>::min_invertible();
            }
            d = 1 / d;
            if (pr::fabs(c) < // TODO Is this ever negative?
                    pr::fabs(a) * pr::numeric_limits<T>::min_invertible()) {
                c = pr::fabs(a) * pr::numeric_limits<T>::min_invertible();
            }

            c = 1 + a / c;
            f = f * c * d;
            if (pr::fabs(1 - c * d) < T(1e-8)) {
                break;
            }
        }

        if (k >= 200) {
            // Failure.
            throw std::runtime_error(__PRETTY_FUNCTION__);
        }

        // Success.
        T y =
            pr::exp(p * pr::log(x) +
                    q * pr::log1p(-x) - pr::lbeta(p, q)) / p * (f - 1);
        return flip ? 1 - y : y;
    }
}

// TODO Unnecessary?
#if 0
#if !DOXYGEN

namespace detail {

// Binomial coefficient for unsigned integers.
template <typename T>
inline std::enable_if_t<
       std::is_integral<T>::value &&
       std::is_unsigned<T>::value, T> binom(T n, T k)
{
    // Special case.
    if (k > n) {
        return 0;
    }

    // Reduce with symmetry.
    if (k > n / 2) {
        k = n - k;
    }

    // Special case.
    if (k == 0) {
        return 1;
    }

    // Special case.
    if (k == 1) {
        return n;
    }

    // Find greatest common divisor.
    auto find_gcd = [](T a, T b) {
        while (a) {
            T r = b % a;
            b = a;
            a = r;
        }
        return b;
    };

    T r = 1; // Result.
    T x = 0; // Temporary numerator.
    T y = 0; // Temporary denominator.
    T g = 0; // Temporary divisor.
    for (T j = 1; j <= k; ++j, --n) {
        if (r >= pr::numeric_limits<T>::max() / n) {

            // Reduce.
            g = find_gcd(n, j), x = n / g, y = j / g;
            g = find_gcd(r, y), r = r / g, y = y / g;
            if (r >= pr::numeric_limits<T>::max() / x) {
                // Overflow.
                throw std::runtime_error(__PRETTY_FUNCTION__);
            }

            // Update.
            r = r * x;
            r = r / y;
        }
        else {

            // Update.
            r = r * n;
            r = r / j;
        }
    }
    return r;
}

// Binomial coefficient for signed integers.
template <typename T>
inline std::enable_if_t<
       std::is_integral<T>::value &&
       std::is_signed<T>::value, T> binom(T n, T k)
{
    if (n >= 0 &&
        k >= 0) {
        // Delegate.
        T r = T(binom(
                std::make_unsigned_t<T>(n),
                std::make_unsigned_t<T>(k)));
        // Overflow in unsigned to signed conversion?
        if (r < 0) {
            throw std::runtime_error(__PRETTY_FUNCTION__);
        }
        return r;
    }
    else if (n < 0) {
        if (k >= 0) {
            // Delegate.
            T r = T(binom(
                    std::make_unsigned_t<T>(k - n - 1),
                    std::make_unsigned_t<T>(k)));
            // Overflow in unsigned to signed conversion?
            if (r < 0) {
                throw std::runtime_error(__PRETTY_FUNCTION__);
            }
            if (k & 1) {
                return -r; // (-1)^k
            }
            else {
                return +r;
            }
        }
        else if (k <= n) {
            // Delegate.
            T r = T(binom(
                    std::make_unsigned_t<T>(-k - 1),
                    std::make_unsigned_t<T>(-k + n)));
            // Overflow in unsigned to signed conversion?
            if (r < 0) {
                throw std::runtime_error(__PRETTY_FUNCTION__);
            }
            if ((n - k) & 1) {
                return -r; // (-1)^(n - k)
            }
            else {
                return +r;
            }
        }
        else {
            return 0;
        }
    }
    else {
        return 0;
    }
}

// Binomial coefficient for floating point numbers.
template <typename T>
inline std::enable_if_t<
       std::is_floating_point<T>::value, T> binom(T n, T k)
{
    if (n == pr::trunc(n)) {
        if (k == pr::trunc(k)) {
            try {
                // Delegate.
                return binom(
                        pr::llrint(n),
                        pr::llrint(k));
            }
            catch (const std::runtime_error&) {
                // Overflow.
                return pr::numeric_limits<T>::infinity();
            }
        }

        // Negative integer n and non-integer k?
        if (pr::signbit(n)) {
            return pr::numeric_limits<T>::infinity();
        }
    }

    // Evaluate with gamma functions.
    T a = n + 1;
    T b = k + 1;
    T c = n - k + 1;
    T r = pr::exp(
            pr::lgamma(a) -
            pr::lgamma(b) -
            pr::lgamma(c));
    // Evaluate sign.
    int m =
        int(pr::signbit(a) && (pr::llrint(pr::floor(a)) & 1)) ^
        int(pr::signbit(b) && (pr::llrint(pr::floor(b)) & 1)) ^
        int(pr::signbit(c) && (pr::llrint(pr::floor(c)) & 1));
    if (m & 1) {
        return -r;
    }
    else {
        return +r;
    }
}

} // namespace detail

#endif// #if !DOXYGEN

/**
 * @brief Binomial coefficient.
 *
 * @f[
 *      {n \choose k} =
 *         \frac{n!}{k!(n - k)!} =
 *         \frac{\Gamma(n + 1)}{\Gamma(k + 1)\Gamma(n - k + 1)}
 * @f]
 *
 * @throw std::runtime_error
 * If `T` is integral and the calculation overflows.
 *
 * @note
 * If `T` is floating point, but both `n` and `k` are exactly representable
 * as integers, delegates to integral implementation. If, in turn, the integral
 * calculation overflows, returns infinity (and does not throw).
 *
 * @note
 * If `T` is floating point and calculation overflows, returns
 * infinity (and does not throw).
 */
template <typename T>
__attribute__((always_inline))
inline std::enable_if_t<
       std::is_arithmetic<T>::value, T> binom(T n, T k)
{
    return detail::binom(n, k);
}
#endif // #if 0

/**
 * @brief Error function inverse.
 */
template <typename T>
inline std::enable_if_t<
       std::is_floating_point<T>::value, T> erfinv(T y)
{
    T w = -pr::log((1 - y) * (1 + y));
    T p;
    if (w < T(5)) {
        w = w - T(2.5);
        p = pr::fma(w, T(+2.81022636e-08), T(+3.43273939e-7));
        p = pr::fma(w, p, T(-3.52338770e-6));
        p = pr::fma(w, p, T(-4.39150654e-6));
        p = pr::fma(w, p, T(+2.18580870e-4));
        p = pr::fma(w, p, T(-1.25372503e-3));
        p = pr::fma(w, p, T(-4.17768164e-3));
        p = pr::fma(w, p, T(+2.46640727e-1));
        p = pr::fma(w, p, T(+1.50140941));
    }
    else {
        w = pr::sqrt(w) - 3;
        p = pr::fma(w, T(-2.00214257e-4), T(+1.00950558e-4));
        p = pr::fma(w, p, T(+1.34934322e-3));
        p = pr::fma(w, p, T(-3.67342844e-3));
        p = pr::fma(w, p, T(+5.73950773e-3));
        p = pr::fma(w, p, T(-7.62246130e-3));
        p = pr::fma(w, p, T(+9.43887047e-3));
        p = pr::fma(w, p, T(+1.00167406));
        p = pr::fma(w, p, T(+2.83297682));
    }
    return p * y;
}

/**@}*/

/**
 * @name Misc interpolation functions
 */
/**@{*/

/**
 * @brief Linear interpolation.
 *
 * @param [in] t
 * Factor.
 *
 * @param [in] p0
 * Control point at @f$ t = 0 @f$.
 *
 * @param [in] p1
 * Control point at @f$ t = 1 @f$.
 */
template <
    typename Tvalue,
    typename Tcontrol
    >
__attribute__((always_inline))
constexpr auto lerp(
            Tvalue t,
            const Tcontrol& p0,
            const Tcontrol& p1)
{
    return (1 - t) * p0 + t * p1;
}

/**
 * @brief Hermite interpolation.
 *
 * @param [in] t
 * Factor.
 *
 * @param [in] p0
 * Control point at @f$ t = 0 @f$.
 *
 * @param [in] m0
 * Slope at @f$ t = 0 @f$.
 *
 * @param [in] m1
 * Slope at @f$ t = 1 @f$.
 *
 * @param [in] p1
 * Control point at @f$ t = 1 @f$.
 *
 * @see Wikipedia's article for [Cubic Hermite spline][1].
 * [1]: https://en.wikipedia.org/wiki/Cubic_Hermite_spline
 */
template <
    typename Tvalue,
    typename Tcontrol
    >
constexpr auto hermite(
            Tvalue t,
            const Tcontrol& p0,
            const Tcontrol& m0,
            const Tcontrol& m1,
            const Tcontrol& p1)
{
    Tvalue s = t - 1;
    Tvalue h00 = s * s * (1 + 2 * t), h10 = s * s * t;
    Tvalue h01 = t * t * (3 - 2 * t), h11 = t * t * s;
    return (h00 * p0 + h10 * m0) +
           (h01 * p1 + h11 * m1);
}

/**
 * @brief Hermite interpolation deriviatve.
 *
 * @param [in] t
 * Factor.
 *
 * @param [in] p0
 * Control point at @f$ t = 0 @f$.
 *
 * @param [in] m0
 * Slope at @f$ t = 0 @f$.
 *
 * @param [in] m1
 * Slope at @f$ t = 1 @f$.
 *
 * @param [in] p1
 * Control point at @f$ t = 1 @f$.
 */
template <
    typename Tvalue,
    typename Tcontrol
    >
constexpr auto hermite_deriv(
            Tvalue t,
            const Tcontrol& p0,
            const Tcontrol& m0,
            const Tcontrol& m1,
            const Tcontrol& p1)
{
    Tvalue g00 = 6 * t * (t - 1);
    Tvalue g10 = 3 * t * t - 4 * t + 1;
    Tvalue g11 = 3 * t * t - 2 * t;
    return g00 * (p0 - p1) + g10 * m0 + g11 * m1;
}

/**
 * @brief Catmull-Rom interpolation.
 *
 * @param [in] t
 * Factor.
 *
 * @param [in] pprev
 * Previous control point.
 *
 * @param [in] p0
 * Control point at @f$ t = 0 @f$.
 *
 * @param [in] p1
 * Control point at @f$ t = 1 @f$.
 *
 * @param [in] pnext
 * Next control point.
 *
 * @see Wikipedia's article for [Cubic Hermite spline][1].
 * [1]: https://en.wikipedia.org/wiki/Cubic_Hermite_spline#Catmull–Rom_spline
 */
template <
    typename Tvalue,
    typename Tcontrol
    >
__attribute__((always_inline))
constexpr auto catmull(
            Tvalue t,
            const Tcontrol& pprev,
            const Tcontrol& p0,
            const Tcontrol& p1,
            const Tcontrol& pnext)
{
    return hermite(t, p0, p1 - pprev, pnext - p0, p1);
}

/**
 * @brief Catmull-Rom interpolation derivative.
 *
 * @param [in] t
 * Factor.
 *
 * @param [in] pprev
 * Previous control point.
 *
 * @param [in] p0
 * Control point at @f$ t = 0 @f$.
 *
 * @param [in] p1
 * Control point at @f$ t = 1 @f$.
 *
 * @param [in] pnext
 * Next control point.
 *
 * @see Wikipedia's article for [Cubic Hermite spline][1].
 * [1]: https://en.wikipedia.org/wiki/Cubic_Hermite_spline#Catmull–Rom_spline
 */
template <
    typename Tvalue,
    typename Tcontrol
    >
__attribute__((always_inline))
constexpr auto catmull_deriv(
            Tvalue t,
            const Tcontrol& pprev,
            const Tcontrol& p0,
            const Tcontrol& p1,
            const Tcontrol& pnext)
{
    return hermite_deriv(t, p0, p1 - pprev, pnext - p0, p1);
}

/**@}*/

/**@}*/

} // namespace pr

#endif // #ifndef PREFORM_MATH_HPP
