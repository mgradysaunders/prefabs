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
// A ruby script generates this file, DO NOT EDIT

namespace pre {

/**
 * @addtogroup math
 */
/**@{*/

/**
 * @name Math wrappers
 */
/**@{*/

/**
 * @brief Wrap `std::abs()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto abs(T x) -> decltype(std::abs(x))
{
    return std::abs(x);
}

/**
 * @brief Wrap `std::arg()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto arg(T x) -> decltype(std::arg(x))
{
    return std::arg(x);
}

/**
 * @brief Wrap `std::fabs()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto fabs(T x) -> decltype(std::fabs(x))
{
    return std::fabs(x);
}

/**
 * @brief Wrap `std::fma()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto fma(T x, T y, T z) -> decltype(std::fma(x, y, z))
{
    return std::fma(x, y, z);
}

/**
 * @brief Wrap `std::fmin()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto fmin(T x, T y) -> decltype(std::fmin(x, y))
{
    return std::fmin(x, y);
}

/**
 * @brief Wrap `std::fmax()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto fmax(T x, T y) -> decltype(std::fmax(x, y))
{
    return std::fmax(x, y);
}

/**
 * @brief Wrap `std::fdim()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto fdim(T x, T y) -> decltype(std::fdim(x, y))
{
    return std::fdim(x, y);
}

/**
 * @brief Wrap `std::fmod()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto fmod(T x, T y) -> decltype(std::fmod(x, y))
{
    return std::fmod(x, y);
}

/**
 * @brief Wrap `std::remquo()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto remquo(T x, T y, int* q) -> decltype(std::remquo(x, y, q))
{
    return std::remquo(x, y, q);
}

/**
 * @brief Wrap `std::remainder()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto remainder(T x, T y) -> decltype(std::remainder(x, y))
{
    return std::remainder(x, y);
}

/**
 * @brief Wrap `std::nearbyint()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto nearbyint(T x) -> decltype(std::nearbyint(x))
{
    return std::nearbyint(x);
}

/**
 * @brief Wrap `std::floor()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto floor(T x) -> decltype(std::floor(x))
{
    return std::floor(x);
}

/**
 * @brief Wrap `std::ceil()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto ceil(T x) -> decltype(std::ceil(x))
{
    return std::ceil(x);
}

/**
 * @brief Wrap `std::trunc()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto trunc(T x) -> decltype(std::trunc(x))
{
    return std::trunc(x);
}

/**
 * @brief Wrap `std::round()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto round(T x) -> decltype(std::round(x))
{
    return std::round(x);
}

/**
 * @brief Wrap `std::rint()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto rint(T x) -> decltype(std::rint(x))
{
    return std::rint(x);
}

/**
 * @brief Wrap `std::lrint()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto lrint(T x) -> decltype(std::lrint(x))
{
    return std::lrint(x);
}

/**
 * @brief Wrap `std::llrint()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto llrint(T x) -> decltype(std::llrint(x))
{
    return std::llrint(x);
}

/**
 * @brief Wrap `std::lround()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto lround(T x) -> decltype(std::lround(x))
{
    return std::lround(x);
}

/**
 * @brief Wrap `std::llround()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto llround(T x) -> decltype(std::llround(x))
{
    return std::llround(x);
}

/**
 * @brief Wrap `std::frexp()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto frexp(T x, int* p) -> decltype(std::frexp(x, p))
{
    return std::frexp(x, p);
}

/**
 * @brief Wrap `std::ldexp()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto ldexp(T x, int p) -> decltype(std::ldexp(x, p))
{
    return std::ldexp(x, p);
}

/**
 * @brief Wrap `std::logb()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto logb(T x) -> decltype(std::logb(x))
{
    return std::logb(x);
}

/**
 * @brief Wrap `std::ilogb()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto ilogb(T x) -> decltype(std::ilogb(x))
{
    return std::ilogb(x);
}

/**
 * @brief Wrap `std::scalbn()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto scalbn(T x, int p) -> decltype(std::scalbn(x, p))
{
    return std::scalbn(x, p);
}

/**
 * @brief Wrap `std::scalbln()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto scalbln(T x, long p) -> decltype(std::scalbln(x, p))
{
    return std::scalbln(x, p);
}

/**
 * @brief Wrap `std::modf()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto modf(T x, T* p) -> decltype(std::modf(x, p))
{
    return std::modf(x, p);
}

/**
 * @brief Wrap `std::nextafter()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto nextafter(T x, T y) -> decltype(std::nextafter(x, y))
{
    return std::nextafter(x, y);
}

/**
 * @brief Wrap `std::nexttoward()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto nexttoward(T x, long double y) -> decltype(std::nexttoward(x, y))
{
    return std::nexttoward(x, y);
}

/**
 * @brief Wrap `std::copysign()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto copysign(T x, T y) -> decltype(std::copysign(x, y))
{
    return std::copysign(x, y);
}

/**
 * @brief Wrap `std::signbit()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto signbit(T x) -> decltype(std::signbit(x))
{
    return std::signbit(x);
}

/**
 * @brief Wrap `std::isnan()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto isnan(T x) -> decltype(std::isnan(x))
{
    return std::isnan(x);
}

/**
 * @brief Wrap `std::isinf()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto isinf(T x) -> decltype(std::isinf(x))
{
    return std::isinf(x);
}

/**
 * @brief Wrap `std::isfinite()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto isfinite(T x) -> decltype(std::isfinite(x))
{
    return std::isfinite(x);
}

/**
 * @brief Wrap `std::isnormal()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto isnormal(T x) -> decltype(std::isnormal(x))
{
    return std::isnormal(x);
}

/**
 * @brief Wrap `std::exp()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto exp(T x) -> decltype(std::exp(x))
{
    return std::exp(x);
}

/**
 * @brief Wrap `std::log()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto log(T x) -> decltype(std::log(x))
{
    return std::log(x);
}

/**
 * @brief Wrap `std::exp2()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto exp2(T x) -> decltype(std::exp2(x))
{
    return std::exp2(x);
}

/**
 * @brief Wrap `std::log2()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto log2(T x) -> decltype(std::log2(x))
{
    return std::log2(x);
}

/**
 * @brief Wrap `std::log10()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto log10(T x) -> decltype(std::log10(x))
{
    return std::log10(x);
}

/**
 * @brief Wrap `std::expm1()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto expm1(T x) -> decltype(std::expm1(x))
{
    return std::expm1(x);
}

/**
 * @brief Wrap `std::log1p()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto log1p(T x) -> decltype(std::log1p(x))
{
    return std::log1p(x);
}

/**
 * @brief Wrap `std::pow()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto pow(T x, T y) -> decltype(std::pow(x, y))
{
    return std::pow(x, y);
}

/**
 * @brief Wrap `std::sqrt()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto sqrt(T x) -> decltype(std::sqrt(x))
{
    return std::sqrt(x);
}

/**
 * @brief Wrap `std::cbrt()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto cbrt(T x) -> decltype(std::cbrt(x))
{
    return std::cbrt(x);
}

/**
 * @brief Wrap `std::hypot()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto hypot(T x, T y) -> decltype(std::hypot(x, y))
{
    return std::hypot(x, y);
}

/**
 * @brief Wrap `std::erf()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto erf(T x) -> decltype(std::erf(x))
{
    return std::erf(x);
}

/**
 * @brief Wrap `std::erfc()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto erfc(T x) -> decltype(std::erfc(x))
{
    return std::erfc(x);
}

/**
 * @brief Wrap `std::lgamma()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto lgamma(T x) -> decltype(std::lgamma(x))
{
    return std::lgamma(x);
}

/**
 * @brief Wrap `std::tgamma()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto tgamma(T x) -> decltype(std::tgamma(x))
{
    return std::tgamma(x);
}

/**
 * @brief Wrap `std::sin()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto sin(T x) -> decltype(std::sin(x))
{
    return std::sin(x);
}

/**
 * @brief Wrap `std::cos()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto cos(T x) -> decltype(std::cos(x))
{
    return std::cos(x);
}

/**
 * @brief Wrap `std::tan()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto tan(T x) -> decltype(std::tan(x))
{
    return std::tan(x);
}

/**
 * @brief Wrap `std::asin()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto asin(T x) -> decltype(std::asin(x))
{
    return std::asin(x);
}

/**
 * @brief Wrap `std::acos()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto acos(T x) -> decltype(std::acos(x))
{
    return std::acos(x);
}

/**
 * @brief Wrap `std::atan()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto atan(T x) -> decltype(std::atan(x))
{
    return std::atan(x);
}

/**
 * @brief Wrap `std::atan2()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto atan2(T y, T x) -> decltype(std::atan2(y, x))
{
    return std::atan2(y, x);
}

/**
 * @brief Wrap `std::sinh()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto sinh(T x) -> decltype(std::sinh(x))
{
    return std::sinh(x);
}

/**
 * @brief Wrap `std::cosh()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto cosh(T x) -> decltype(std::cosh(x))
{
    return std::cosh(x);
}

/**
 * @brief Wrap `std::tanh()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto tanh(T x) -> decltype(std::tanh(x))
{
    return std::tanh(x);
}

/**
 * @brief Wrap `std::asinh()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto asinh(T x) -> decltype(std::asinh(x))
{
    return std::asinh(x);
}

/**
 * @brief Wrap `std::acosh()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto acosh(T x) -> decltype(std::acosh(x))
{
    return std::acosh(x);
}

/**
 * @brief Wrap `std::atanh()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto atanh(T x) -> decltype(std::atanh(x))
{
    return std::atanh(x);
}

/**@}*/

/**
 * @name Reciprocal trigonometric functions
 */
/**@{*/

/**
 * @brief Reciprocal of `pre::sin()`.
 */
template <typename T>
__attribute__((always_inline))
inline auto csc(T x) -> decltype(pre::sin(x))
{
    return static_cast<decltype(pre::sin(x))>(1) / pre::sin(x);
}

/**
 * @brief Reciprocal of `pre::cos()`.
 */
template <typename T>
__attribute__((always_inline))
inline auto sec(T x) -> decltype(pre::cos(x))
{
    return static_cast<decltype(pre::cos(x))>(1) / pre::cos(x);
}

/**
 * @brief Reciprocal of `pre::tan()`.
 */
template <typename T>
__attribute__((always_inline))
inline auto cot(T x) -> decltype(pre::tan(x))
{
    return static_cast<decltype(pre::tan(x))>(1) / pre::tan(x);
}

/**
 * @brief Reciprocal of `pre::sinh()`.
 */
template <typename T>
__attribute__((always_inline))
inline auto csch(T x) -> decltype(pre::sinh(x))
{
    return static_cast<decltype(pre::sinh(x))>(1) / pre::sinh(x);
}

/**
 * @brief Reciprocal of `pre::cosh()`.
 */
template <typename T>
__attribute__((always_inline))
inline auto sech(T x) -> decltype(pre::cosh(x))
{
    return static_cast<decltype(pre::cosh(x))>(1) / pre::cosh(x);
}

/**
 * @brief Reciprocal of `pre::tanh()`.
 */
template <typename T>
__attribute__((always_inline))
inline auto coth(T x) -> decltype(pre::tanh(x))
{
    return static_cast<decltype(pre::tanh(x))>(1) / pre::tanh(x);
}

/**
 * @brief Inverse of `pre::csc()`.
 */
template <typename T>
__attribute__((always_inline))
inline auto acsc(T x) -> decltype(pre::asin(x))
{
    return pre::asin(static_cast<decltype(pre::asin(x))>(1) / x);
}

/**
 * @brief Inverse of `pre::sec()`.
 */
template <typename T>
__attribute__((always_inline))
inline auto asec(T x) -> decltype(pre::acos(x))
{
    return pre::acos(static_cast<decltype(pre::acos(x))>(1) / x);
}

/**
 * @brief Inverse of `pre::cot()`.
 */
template <typename T>
__attribute__((always_inline))
inline auto acot(T x) -> decltype(pre::atan(x))
{
    return pre::atan(static_cast<decltype(pre::atan(x))>(1) / x);
}

/**
 * @brief Inverse of `pre::csch()`.
 */
template <typename T>
__attribute__((always_inline))
inline auto acsch(T x) -> decltype(pre::asinh(x))
{
    return pre::asinh(static_cast<decltype(pre::asinh(x))>(1) / x);
}

/**
 * @brief Inverse of `pre::sech()`.
 */
template <typename T>
__attribute__((always_inline))
inline auto asech(T x) -> decltype(pre::acosh(x))
{
    return pre::acosh(static_cast<decltype(pre::acosh(x))>(1) / x);
}

/**
 * @brief Inverse of `pre::coth()`.
 */
template <typename T>
__attribute__((always_inline))
inline auto acoth(T x) -> decltype(pre::atanh(x))
{
    return pre::atanh(static_cast<decltype(pre::atanh(x))>(1) / x);
}

/**@}*/

/**@}*/

} // namespace pre

