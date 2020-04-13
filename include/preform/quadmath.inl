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
 * @addtogroup quadmath
 */
/**@{*/

/**
 * @name Quadmath wrappers (__float128)
 */
/**@{*/

/**
 * @brief Wrap `fabsq()`.
 */
template <typename T>
__attribute__((always_inline))
inline std::enable_if_t<std::is_same<T, __float128>::value, T> fabs(T x)
{
    return ::fabsq(x);
}

/**
 * @brief Wrap `fmaq()`.
 */
template <typename T>
__attribute__((always_inline))
inline std::enable_if_t<std::is_same<T, __float128>::value, T> fma(T x, T y, T z)
{
    return ::fmaq(x, y, z);
}

/**
 * @brief Wrap `fminq()`.
 */
template <typename T>
__attribute__((always_inline))
inline std::enable_if_t<std::is_same<T, __float128>::value, T> fmin(T x, T y)
{
    return ::fminq(x, y);
}

/**
 * @brief Wrap `fmaxq()`.
 */
template <typename T>
__attribute__((always_inline))
inline std::enable_if_t<std::is_same<T, __float128>::value, T> fmax(T x, T y)
{
    return ::fmaxq(x, y);
}

/**
 * @brief Wrap `fdimq()`.
 */
template <typename T>
__attribute__((always_inline))
inline std::enable_if_t<std::is_same<T, __float128>::value, T> fdim(T x, T y)
{
    return ::fdimq(x, y);
}

/**
 * @brief Wrap `fmodq()`.
 */
template <typename T>
__attribute__((always_inline))
inline std::enable_if_t<std::is_same<T, __float128>::value, T> fmod(T x, T y)
{
    return ::fmodq(x, y);
}

/**
 * @brief Wrap `remquoq()`.
 */
template <typename T>
__attribute__((always_inline))
inline std::enable_if_t<std::is_same<T, __float128>::value, T> remquo(T x, T y, int* q)
{
    return ::remquoq(x, y, q);
}

/**
 * @brief Wrap `remainderq()`.
 */
template <typename T>
__attribute__((always_inline))
inline std::enable_if_t<std::is_same<T, __float128>::value, T> remainder(T x, T y)
{
    return ::remainderq(x, y);
}

/**
 * @brief Wrap `nearbyintq()`.
 */
template <typename T>
__attribute__((always_inline))
inline std::enable_if_t<std::is_same<T, __float128>::value, T> nearbyint(T x)
{
    return ::nearbyintq(x);
}

/**
 * @brief Wrap `floorq()`.
 */
template <typename T>
__attribute__((always_inline))
inline std::enable_if_t<std::is_same<T, __float128>::value, T> floor(T x)
{
    return ::floorq(x);
}

/**
 * @brief Wrap `ceilq()`.
 */
template <typename T>
__attribute__((always_inline))
inline std::enable_if_t<std::is_same<T, __float128>::value, T> ceil(T x)
{
    return ::ceilq(x);
}

/**
 * @brief Wrap `truncq()`.
 */
template <typename T>
__attribute__((always_inline))
inline std::enable_if_t<std::is_same<T, __float128>::value, T> trunc(T x)
{
    return ::truncq(x);
}

/**
 * @brief Wrap `roundq()`.
 */
template <typename T>
__attribute__((always_inline))
inline std::enable_if_t<std::is_same<T, __float128>::value, T> round(T x)
{
    return ::roundq(x);
}

/**
 * @brief Wrap `rintq()`.
 */
template <typename T>
__attribute__((always_inline))
inline std::enable_if_t<std::is_same<T, __float128>::value, T> rint(T x)
{
    return ::rintq(x);
}

/**
 * @brief Wrap `lrintq()`.
 */
template <typename T>
__attribute__((always_inline))
inline std::enable_if_t<std::is_same<T, __float128>::value, long> lrint(T x)
{
    return ::lrintq(x);
}

/**
 * @brief Wrap `llrintq()`.
 */
template <typename T>
__attribute__((always_inline))
inline std::enable_if_t<std::is_same<T, __float128>::value, long long> llrint(T x)
{
    return ::llrintq(x);
}

/**
 * @brief Wrap `lroundq()`.
 */
template <typename T>
__attribute__((always_inline))
inline std::enable_if_t<std::is_same<T, __float128>::value, long> lround(T x)
{
    return ::lroundq(x);
}

/**
 * @brief Wrap `llroundq()`.
 */
template <typename T>
__attribute__((always_inline))
inline std::enable_if_t<std::is_same<T, __float128>::value, long long> llround(T x)
{
    return ::llroundq(x);
}

/**
 * @brief Wrap `frexpq()`.
 */
template <typename T>
__attribute__((always_inline))
inline std::enable_if_t<std::is_same<T, __float128>::value, T> frexp(T x, int* p)
{
    return ::frexpq(x, p);
}

/**
 * @brief Wrap `ldexpq()`.
 */
template <typename T>
__attribute__((always_inline))
inline std::enable_if_t<std::is_same<T, __float128>::value, T> ldexp(T x, int p)
{
    return ::ldexpq(x, p);
}

/**
 * @brief Wrap `logbq()`.
 */
template <typename T>
__attribute__((always_inline))
inline std::enable_if_t<std::is_same<T, __float128>::value, T> logb(T x)
{
    return ::logbq(x);
}

/**
 * @brief Wrap `ilogbq()`.
 */
template <typename T>
__attribute__((always_inline))
inline std::enable_if_t<std::is_same<T, __float128>::value, int> ilogb(T x)
{
    return ::ilogbq(x);
}

/**
 * @brief Wrap `scalbnq()`.
 */
template <typename T>
__attribute__((always_inline))
inline std::enable_if_t<std::is_same<T, __float128>::value, T> scalbn(T x, int p)
{
    return ::scalbnq(x, p);
}

/**
 * @brief Wrap `scalblnq()`.
 */
template <typename T>
__attribute__((always_inline))
inline std::enable_if_t<std::is_same<T, __float128>::value, T> scalbln(T x, long p)
{
    return ::scalblnq(x, p);
}

/**
 * @brief Wrap `modfq()`.
 */
template <typename T>
__attribute__((always_inline))
inline std::enable_if_t<std::is_same<T, __float128>::value, T> modf(T x, T* p)
{
    return ::modfq(x, p);
}

/**
 * @brief Wrap `nextafterq()`.
 */
template <typename T>
__attribute__((always_inline))
inline std::enable_if_t<std::is_same<T, __float128>::value, T> nextafter(T x, T y)
{
    return ::nextafterq(x, y);
}

/**
 * @brief Wrap `copysignq()`.
 */
template <typename T>
__attribute__((always_inline))
inline std::enable_if_t<std::is_same<T, __float128>::value, T> copysign(T x, T y)
{
    return ::copysignq(x, y);
}

/**
 * @brief Wrap `signbitq()`.
 */
template <typename T>
__attribute__((always_inline))
inline std::enable_if_t<std::is_same<T, __float128>::value, bool> signbit(T x)
{
    return ::signbitq(x);
}

/**
 * @brief Wrap `isnanq()`.
 */
template <typename T>
__attribute__((always_inline))
inline std::enable_if_t<std::is_same<T, __float128>::value, bool> isnan(T x)
{
    return ::isnanq(x);
}

/**
 * @brief Wrap `isinfq()`.
 */
template <typename T>
__attribute__((always_inline))
inline std::enable_if_t<std::is_same<T, __float128>::value, bool> isinf(T x)
{
    return ::isinfq(x);
}

/**
 * @brief Wrap `finiteq()`.
 */
template <typename T>
__attribute__((always_inline))
inline std::enable_if_t<std::is_same<T, __float128>::value, bool> isfinite(T x)
{
    return ::finiteq(x);
}

/**
 * @brief Wrap `expq()`.
 */
template <typename T>
__attribute__((always_inline))
inline std::enable_if_t<std::is_same<T, __float128>::value, T> exp(T x)
{
    return ::expq(x);
}

/**
 * @brief Wrap `logq()`.
 */
template <typename T>
__attribute__((always_inline))
inline std::enable_if_t<std::is_same<T, __float128>::value, T> log(T x)
{
    return ::logq(x);
}

/**
 * @brief Wrap `log2q()`.
 */
template <typename T>
__attribute__((always_inline))
inline std::enable_if_t<std::is_same<T, __float128>::value, T> log2(T x)
{
    return ::log2q(x);
}

/**
 * @brief Wrap `log10q()`.
 */
template <typename T>
__attribute__((always_inline))
inline std::enable_if_t<std::is_same<T, __float128>::value, T> log10(T x)
{
    return ::log10q(x);
}

/**
 * @brief Wrap `expm1q()`.
 */
template <typename T>
__attribute__((always_inline))
inline std::enable_if_t<std::is_same<T, __float128>::value, T> expm1(T x)
{
    return ::expm1q(x);
}

/**
 * @brief Wrap `log1pq()`.
 */
template <typename T>
__attribute__((always_inline))
inline std::enable_if_t<std::is_same<T, __float128>::value, T> log1p(T x)
{
    return ::log1pq(x);
}

/**
 * @brief Wrap `powq()`.
 */
template <typename T>
__attribute__((always_inline))
inline std::enable_if_t<std::is_same<T, __float128>::value, T> pow(T x, T y)
{
    return ::powq(x, y);
}

/**
 * @brief Wrap `sqrtq()`.
 */
template <typename T>
__attribute__((always_inline))
inline std::enable_if_t<std::is_same<T, __float128>::value, T> sqrt(T x)
{
    return ::sqrtq(x);
}

/**
 * @brief Wrap `cbrtq()`.
 */
template <typename T>
__attribute__((always_inline))
inline std::enable_if_t<std::is_same<T, __float128>::value, T> cbrt(T x)
{
    return ::cbrtq(x);
}

/**
 * @brief Wrap `hypotq()`.
 */
template <typename T>
__attribute__((always_inline))
inline std::enable_if_t<std::is_same<T, __float128>::value, T> hypot(T x, T y)
{
    return ::hypotq(x, y);
}

/**
 * @brief Wrap `erfq()`.
 */
template <typename T>
__attribute__((always_inline))
inline std::enable_if_t<std::is_same<T, __float128>::value, T> erf(T x)
{
    return ::erfq(x);
}

/**
 * @brief Wrap `erfcq()`.
 */
template <typename T>
__attribute__((always_inline))
inline std::enable_if_t<std::is_same<T, __float128>::value, T> erfc(T x)
{
    return ::erfcq(x);
}

/**
 * @brief Wrap `lgammaq()`.
 */
template <typename T>
__attribute__((always_inline))
inline std::enable_if_t<std::is_same<T, __float128>::value, T> lgamma(T x)
{
    return ::lgammaq(x);
}

/**
 * @brief Wrap `tgammaq()`.
 */
template <typename T>
__attribute__((always_inline))
inline std::enable_if_t<std::is_same<T, __float128>::value, T> tgamma(T x)
{
    return ::tgammaq(x);
}

/**
 * @brief Wrap `sinq()`.
 */
template <typename T>
__attribute__((always_inline))
inline std::enable_if_t<std::is_same<T, __float128>::value, T> sin(T x)
{
    return ::sinq(x);
}

/**
 * @brief Wrap `cosq()`.
 */
template <typename T>
__attribute__((always_inline))
inline std::enable_if_t<std::is_same<T, __float128>::value, T> cos(T x)
{
    return ::cosq(x);
}

/**
 * @brief Wrap `tanq()`.
 */
template <typename T>
__attribute__((always_inline))
inline std::enable_if_t<std::is_same<T, __float128>::value, T> tan(T x)
{
    return ::tanq(x);
}

/**
 * @brief Wrap `asinq()`.
 */
template <typename T>
__attribute__((always_inline))
inline std::enable_if_t<std::is_same<T, __float128>::value, T> asin(T x)
{
    return ::asinq(x);
}

/**
 * @brief Wrap `acosq()`.
 */
template <typename T>
__attribute__((always_inline))
inline std::enable_if_t<std::is_same<T, __float128>::value, T> acos(T x)
{
    return ::acosq(x);
}

/**
 * @brief Wrap `atanq()`.
 */
template <typename T>
__attribute__((always_inline))
inline std::enable_if_t<std::is_same<T, __float128>::value, T> atan(T x)
{
    return ::atanq(x);
}

/**
 * @brief Wrap `atan2q()`.
 */
template <typename T>
__attribute__((always_inline))
inline std::enable_if_t<std::is_same<T, __float128>::value, T> atan2(T y, T x)
{
    return ::atan2q(y, x);
}

/**
 * @brief Wrap `sinhq()`.
 */
template <typename T>
__attribute__((always_inline))
inline std::enable_if_t<std::is_same<T, __float128>::value, T> sinh(T x)
{
    return ::sinhq(x);
}

/**
 * @brief Wrap `coshq()`.
 */
template <typename T>
__attribute__((always_inline))
inline std::enable_if_t<std::is_same<T, __float128>::value, T> cosh(T x)
{
    return ::coshq(x);
}

/**
 * @brief Wrap `tanhq()`.
 */
template <typename T>
__attribute__((always_inline))
inline std::enable_if_t<std::is_same<T, __float128>::value, T> tanh(T x)
{
    return ::tanhq(x);
}

/**
 * @brief Wrap `asinhq()`.
 */
template <typename T>
__attribute__((always_inline))
inline std::enable_if_t<std::is_same<T, __float128>::value, T> asinh(T x)
{
    return ::asinhq(x);
}

/**
 * @brief Wrap `acoshq()`.
 */
template <typename T>
__attribute__((always_inline))
inline std::enable_if_t<std::is_same<T, __float128>::value, T> acosh(T x)
{
    return ::acoshq(x);
}

/**
 * @brief Wrap `atanhq()`.
 */
template <typename T>
__attribute__((always_inline))
inline std::enable_if_t<std::is_same<T, __float128>::value, T> atanh(T x)
{
    return ::atanhq(x);
}

/**@}*/

/**
 * @name Quadmath wrappers (__complex128)
 */
/**@{*/

/**
 * @brief Wrap `cabsq()`.
 */
template <typename U>
__attribute__((always_inline))
inline std::enable_if_t<std::is_same<U, __complex128>::value, __float128> abs(U x)
{
    return ::cabsq(x);
}

/**
 * @brief Wrap `cargq()`.
 */
template <typename U>
__attribute__((always_inline))
inline std::enable_if_t<std::is_same<U, __complex128>::value, __float128> arg(U x)
{
    return ::cargq(x);
}

/**
 * @brief Wrap `crealq()`.
 */
template <typename U>
__attribute__((always_inline))
inline std::enable_if_t<std::is_same<U, __complex128>::value, __float128> real(U x)
{
    return ::crealq(x);
}

/**
 * @brief Wrap `cimagq()`.
 */
template <typename U>
__attribute__((always_inline))
inline std::enable_if_t<std::is_same<U, __complex128>::value, __float128> imag(U x)
{
    return ::cimagq(x);
}

/**
 * @brief Wrap `conjq()`.
 */
template <typename U>
__attribute__((always_inline))
inline std::enable_if_t<std::is_same<U, __complex128>::value, U> conj(U x)
{
    return ::conjq(x);
}

/**
 * @brief Wrap `cprojq()`.
 */
template <typename U>
__attribute__((always_inline))
inline std::enable_if_t<std::is_same<U, __complex128>::value, U> proj(U x)
{
    return ::cprojq(x);
}

/**
 * @brief Wrap `cexpq()`.
 */
template <typename U>
__attribute__((always_inline))
inline std::enable_if_t<std::is_same<U, __complex128>::value, U> exp(U x)
{
    return ::cexpq(x);
}

/**
 * @brief Wrap `clogq()`.
 */
template <typename U>
__attribute__((always_inline))
inline std::enable_if_t<std::is_same<U, __complex128>::value, U> log(U x)
{
    return ::clogq(x);
}

/**
 * @brief Wrap `clog10q()`.
 */
template <typename U>
__attribute__((always_inline))
inline std::enable_if_t<std::is_same<U, __complex128>::value, U> log10(U x)
{
    return ::clog10q(x);
}

/**
 * @brief Wrap `cpowq()`.
 */
template <typename U>
__attribute__((always_inline))
inline std::enable_if_t<std::is_same<U, __complex128>::value, U> pow(U x, U y)
{
    return ::cpowq(x, y);
}

/**
 * @brief Wrap `csqrtq()`.
 */
template <typename U>
__attribute__((always_inline))
inline std::enable_if_t<std::is_same<U, __complex128>::value, U> sqrt(U x)
{
    return ::csqrtq(x);
}

/**
 * @brief Wrap `csinq()`.
 */
template <typename U>
__attribute__((always_inline))
inline std::enable_if_t<std::is_same<U, __complex128>::value, U> sin(U x)
{
    return ::csinq(x);
}

/**
 * @brief Wrap `ccosq()`.
 */
template <typename U>
__attribute__((always_inline))
inline std::enable_if_t<std::is_same<U, __complex128>::value, U> cos(U x)
{
    return ::ccosq(x);
}

/**
 * @brief Wrap `ctanq()`.
 */
template <typename U>
__attribute__((always_inline))
inline std::enable_if_t<std::is_same<U, __complex128>::value, U> tan(U x)
{
    return ::ctanq(x);
}

/**
 * @brief Wrap `casinq()`.
 */
template <typename U>
__attribute__((always_inline))
inline std::enable_if_t<std::is_same<U, __complex128>::value, U> asin(U x)
{
    return ::casinq(x);
}

/**
 * @brief Wrap `cacosq()`.
 */
template <typename U>
__attribute__((always_inline))
inline std::enable_if_t<std::is_same<U, __complex128>::value, U> acos(U x)
{
    return ::cacosq(x);
}

/**
 * @brief Wrap `catanq()`.
 */
template <typename U>
__attribute__((always_inline))
inline std::enable_if_t<std::is_same<U, __complex128>::value, U> atan(U x)
{
    return ::catanq(x);
}

/**
 * @brief Wrap `csinhq()`.
 */
template <typename U>
__attribute__((always_inline))
inline std::enable_if_t<std::is_same<U, __complex128>::value, U> sinh(U x)
{
    return ::csinhq(x);
}

/**
 * @brief Wrap `ccoshq()`.
 */
template <typename U>
__attribute__((always_inline))
inline std::enable_if_t<std::is_same<U, __complex128>::value, U> cosh(U x)
{
    return ::ccoshq(x);
}

/**
 * @brief Wrap `ctanhq()`.
 */
template <typename U>
__attribute__((always_inline))
inline std::enable_if_t<std::is_same<U, __complex128>::value, U> tanh(U x)
{
    return ::ctanhq(x);
}

/**
 * @brief Wrap `casinhq()`.
 */
template <typename U>
__attribute__((always_inline))
inline std::enable_if_t<std::is_same<U, __complex128>::value, U> asinh(U x)
{
    return ::casinhq(x);
}

/**
 * @brief Wrap `cacoshq()`.
 */
template <typename U>
__attribute__((always_inline))
inline std::enable_if_t<std::is_same<U, __complex128>::value, U> acosh(U x)
{
    return ::cacoshq(x);
}

/**
 * @brief Wrap `catanhq()`.
 */
template <typename U>
__attribute__((always_inline))
inline std::enable_if_t<std::is_same<U, __complex128>::value, U> atanh(U x)
{
    return ::catanhq(x);
}

/**@}*/

/**
 * @name Reciprocal trigonometric functions (__float128/__complex128)
 */
/**@{*/

/**
 * @brief Reciprocal of `pre::sin()`.
 */
template <typename T>
__attribute__((always_inline))
inline std::enable_if_t<
                std::is_same<T, __float128>::value ||
                std::is_same<T, __complex128>::value, T> csc(T x)
{
    return T(1) / pre::sin(x);
}

/**
 * @brief Reciprocal of `pre::cos()`.
 */
template <typename T>
__attribute__((always_inline))
inline std::enable_if_t<
                std::is_same<T, __float128>::value ||
                std::is_same<T, __complex128>::value, T> sec(T x)
{
    return T(1) / pre::cos(x);
}

/**
 * @brief Reciprocal of `pre::tan()`.
 */
template <typename T>
__attribute__((always_inline))
inline std::enable_if_t<
                std::is_same<T, __float128>::value ||
                std::is_same<T, __complex128>::value, T> cot(T x)
{
    return T(1) / pre::tan(x);
}

/**
 * @brief Reciprocal of `pre::sinh()`.
 */
template <typename T>
__attribute__((always_inline))
inline std::enable_if_t<
                std::is_same<T, __float128>::value ||
                std::is_same<T, __complex128>::value, T> csch(T x)
{
    return T(1) / pre::sinh(x);
}

/**
 * @brief Reciprocal of `pre::cosh()`.
 */
template <typename T>
__attribute__((always_inline))
inline std::enable_if_t<
                std::is_same<T, __float128>::value ||
                std::is_same<T, __complex128>::value, T> sech(T x)
{
    return T(1) / pre::cosh(x);
}

/**
 * @brief Reciprocal of `pre::tanh()`.
 */
template <typename T>
__attribute__((always_inline))
inline std::enable_if_t<
                std::is_same<T, __float128>::value ||
                std::is_same<T, __complex128>::value, T> coth(T x)
{
    return T(1) / pre::tanh(x);
}

/**
 * @brief Inverse of `pre::csc()`.
 */
template <typename T>
__attribute__((always_inline))
inline std::enable_if_t<
                std::is_same<T, __float128>::value ||
                std::is_same<T, __complex128>::value, T> acsc(T x)
{
    return pre::asin(T(1) / x);
}

/**
 * @brief Inverse of `pre::sec()`.
 */
template <typename T>
__attribute__((always_inline))
inline std::enable_if_t<
                std::is_same<T, __float128>::value ||
                std::is_same<T, __complex128>::value, T> asec(T x)
{
    return pre::acos(T(1) / x);
}

/**
 * @brief Inverse of `pre::cot()`.
 */
template <typename T>
__attribute__((always_inline))
inline std::enable_if_t<
                std::is_same<T, __float128>::value ||
                std::is_same<T, __complex128>::value, T> acot(T x)
{
    return pre::atan(T(1) / x);
}

/**
 * @brief Inverse of `pre::csch()`.
 */
template <typename T>
__attribute__((always_inline))
inline std::enable_if_t<
                std::is_same<T, __float128>::value ||
                std::is_same<T, __complex128>::value, T> acsch(T x)
{
    return pre::asinh(T(1) / x);
}

/**
 * @brief Inverse of `pre::sech()`.
 */
template <typename T>
__attribute__((always_inline))
inline std::enable_if_t<
                std::is_same<T, __float128>::value ||
                std::is_same<T, __complex128>::value, T> asech(T x)
{
    return pre::acosh(T(1) / x);
}

/**
 * @brief Inverse of `pre::coth()`.
 */
template <typename T>
__attribute__((always_inline))
inline std::enable_if_t<
                std::is_same<T, __float128>::value ||
                std::is_same<T, __complex128>::value, T> acoth(T x)
{
    return pre::atanh(T(1) / x);
}

/**@}*/

/**@}*/

} // namespace pre

