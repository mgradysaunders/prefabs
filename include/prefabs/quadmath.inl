/**
 * @addtogroup quadmath
 */
/**@{*/

/**
 * @name Quadmath wrappers (`__float128`)
 */
/**@{*/

/**
 * @brief Wrap `fabsq()`.
 */
template <typename T>
__attribute__((always_inline))
inline typename enable_float128<T>::type fabs(T x)
{
    return ::fabsq(x);
}

/**
 * @brief Wrap `fmaq()`.
 */
template <typename T>
__attribute__((always_inline))
inline typename enable_float128<T>::type fma(T x, T y, T z)
{
    return ::fmaq(x, y, z);
}

/**
 * @brief Wrap `fminq()`.
 */
template <typename T>
__attribute__((always_inline))
inline typename enable_float128<T>::type fmin(T x, T y)
{
    return ::fminq(x, y);
}

/**
 * @brief Wrap `fmaxq()`.
 */
template <typename T>
__attribute__((always_inline))
inline typename enable_float128<T>::type fmax(T x, T y)
{
    return ::fmaxq(x, y);
}

/**
 * @brief Wrap `fdimq()`.
 */
template <typename T>
__attribute__((always_inline))
inline typename enable_float128<T>::type fdim(T x, T y)
{
    return ::fdimq(x, y);
}

/**
 * @brief Wrap `fmodq()`.
 */
template <typename T>
__attribute__((always_inline))
inline typename enable_float128<T>::type fmod(T x, T y)
{
    return ::fmodq(x, y);
}

/**
 * @brief Wrap `remquoq()`.
 */
template <typename T>
__attribute__((always_inline))
inline typename enable_float128<T>::type remquo(T x, T y, int* q)
{
    return ::remquoq(x, y, q);
}

/**
 * @brief Wrap `remainderq()`.
 */
template <typename T>
__attribute__((always_inline))
inline typename enable_float128<T>::type remainder(T x, T y)
{
    return ::remainderq(x, y);
}

/**
 * @brief Wrap `nearbyintq()`.
 */
template <typename T>
__attribute__((always_inline))
inline typename enable_float128<T>::type nearbyint(T x)
{
    return ::nearbyintq(x);
}

/**
 * @brief Wrap `floorq()`.
 */
template <typename T>
__attribute__((always_inline))
inline typename enable_float128<T>::type floor(T x)
{
    return ::floorq(x);
}

/**
 * @brief Wrap `ceilq()`.
 */
template <typename T>
__attribute__((always_inline))
inline typename enable_float128<T>::type ceil(T x)
{
    return ::ceilq(x);
}

/**
 * @brief Wrap `truncq()`.
 */
template <typename T>
__attribute__((always_inline))
inline typename enable_float128<T>::type trunc(T x)
{
    return ::truncq(x);
}

/**
 * @brief Wrap `roundq()`.
 */
template <typename T>
__attribute__((always_inline))
inline typename enable_float128<T>::type round(T x)
{
    return ::roundq(x);
}

/**
 * @brief Wrap `rintq()`.
 */
template <typename T>
__attribute__((always_inline))
inline typename enable_float128<T>::type rint(T x)
{
    return ::rintq(x);
}

/**
 * @brief Wrap `lrintq()`.
 */
template <typename T>
__attribute__((always_inline))
inline typename enable_float128<T, long>::type lrint(T x)
{
    return ::lrintq(x);
}

/**
 * @brief Wrap `llrintq()`.
 */
template <typename T>
__attribute__((always_inline))
inline typename enable_float128<T, long long>::type llrint(T x)
{
    return ::llrintq(x);
}

/**
 * @brief Wrap `lroundq()`.
 */
template <typename T>
__attribute__((always_inline))
inline typename enable_float128<T, long>::type lround(T x)
{
    return ::lroundq(x);
}

/**
 * @brief Wrap `llroundq()`.
 */
template <typename T>
__attribute__((always_inline))
inline typename enable_float128<T, long long>::type llround(T x)
{
    return ::llroundq(x);
}

/**
 * @brief Wrap `frexpq()`.
 */
template <typename T>
__attribute__((always_inline))
inline typename enable_float128<T>::type frexp(T x, int* p)
{
    return ::frexpq(x, p);
}

/**
 * @brief Wrap `ldexpq()`.
 */
template <typename T>
__attribute__((always_inline))
inline typename enable_float128<T>::type ldexp(T x, int p)
{
    return ::ldexpq(x, p);
}

/**
 * @brief Wrap `logbq()`.
 */
template <typename T>
__attribute__((always_inline))
inline typename enable_float128<T>::type logb(T x)
{
    return ::logbq(x);
}

/**
 * @brief Wrap `ilogbq()`.
 */
template <typename T>
__attribute__((always_inline))
inline typename enable_float128<T, int>::type ilogb(T x)
{
    return ::ilogbq(x);
}

/**
 * @brief Wrap `scalbnq()`.
 */
template <typename T>
__attribute__((always_inline))
inline typename enable_float128<T>::type scalbn(T x, int p)
{
    return ::scalbnq(x, p);
}

/**
 * @brief Wrap `scalblnq()`.
 */
template <typename T>
__attribute__((always_inline))
inline typename enable_float128<T>::type scalbln(T x, long p)
{
    return ::scalblnq(x, p);
}

/**
 * @brief Wrap `modfq()`.
 */
template <typename T>
__attribute__((always_inline))
inline typename enable_float128<T>::type modf(T x, T* p)
{
    return ::modfq(x, p);
}

/**
 * @brief Wrap `nextafterq()`.
 */
template <typename T>
__attribute__((always_inline))
inline typename enable_float128<T>::type nextafter(T x, T y)
{
    return ::nextafterq(x, y);
}

/**
 * @brief Wrap `copysignq()`.
 */
template <typename T>
__attribute__((always_inline))
inline typename enable_float128<T>::type copysign(T x, T y)
{
    return ::copysignq(x, y);
}

/**
 * @brief Wrap `signbitq()`.
 */
template <typename T>
__attribute__((always_inline))
inline typename enable_float128<T, bool>::type signbit(T x)
{
    return ::signbitq(x);
}

/**
 * @brief Wrap `isnanq()`.
 */
template <typename T>
__attribute__((always_inline))
inline typename enable_float128<T, bool>::type isnan(T x)
{
    return ::isnanq(x);
}

/**
 * @brief Wrap `isinfq()`.
 */
template <typename T>
__attribute__((always_inline))
inline typename enable_float128<T, bool>::type isinf(T x)
{
    return ::isinfq(x);
}

/**
 * @brief Wrap `finiteq()`.
 */
template <typename T>
__attribute__((always_inline))
inline typename enable_float128<T, bool>::type isfinite(T x)
{
    return ::finiteq(x);
}

/**
 * @brief Wrap `expq()`.
 */
template <typename T>
__attribute__((always_inline))
inline typename enable_float128<T>::type exp(T x)
{
    return ::expq(x);
}

/**
 * @brief Wrap `logq()`.
 */
template <typename T>
__attribute__((always_inline))
inline typename enable_float128<T>::type log(T x)
{
    return ::logq(x);
}

/**
 * @brief Wrap `log2q()`.
 */
template <typename T>
__attribute__((always_inline))
inline typename enable_float128<T>::type log2(T x)
{
    return ::log2q(x);
}

/**
 * @brief Wrap `log10q()`.
 */
template <typename T>
__attribute__((always_inline))
inline typename enable_float128<T>::type log10(T x)
{
    return ::log10q(x);
}

/**
 * @brief Wrap `expm1q()`.
 */
template <typename T>
__attribute__((always_inline))
inline typename enable_float128<T>::type expm1(T x)
{
    return ::expm1q(x);
}

/**
 * @brief Wrap `log1pq()`.
 */
template <typename T>
__attribute__((always_inline))
inline typename enable_float128<T>::type log1p(T x)
{
    return ::log1pq(x);
}

/**
 * @brief Wrap `powq()`.
 */
template <typename T>
__attribute__((always_inline))
inline typename enable_float128<T>::type pow(T x, T y)
{
    return ::powq(x, y);
}

/**
 * @brief Wrap `sqrtq()`.
 */
template <typename T>
__attribute__((always_inline))
inline typename enable_float128<T>::type sqrt(T x)
{
    return ::sqrtq(x);
}

/**
 * @brief Wrap `cbrtq()`.
 */
template <typename T>
__attribute__((always_inline))
inline typename enable_float128<T>::type cbrt(T x)
{
    return ::cbrtq(x);
}

/**
 * @brief Wrap `hypotq()`.
 */
template <typename T>
__attribute__((always_inline))
inline typename enable_float128<T>::type hypot(T x, T y)
{
    return ::hypotq(x, y);
}

/**
 * @brief Wrap `erfq()`.
 */
template <typename T>
__attribute__((always_inline))
inline typename enable_float128<T>::type erf(T x)
{
    return ::erfq(x);
}

/**
 * @brief Wrap `erfcq()`.
 */
template <typename T>
__attribute__((always_inline))
inline typename enable_float128<T>::type erfc(T x)
{
    return ::erfcq(x);
}

/**
 * @brief Wrap `lgammaq()`.
 */
template <typename T>
__attribute__((always_inline))
inline typename enable_float128<T>::type lgamma(T x)
{
    return ::lgammaq(x);
}

/**
 * @brief Wrap `tgammaq()`.
 */
template <typename T>
__attribute__((always_inline))
inline typename enable_float128<T>::type tgamma(T x)
{
    return ::tgammaq(x);
}

/**
 * @brief Wrap `sinq()`.
 */
template <typename T>
__attribute__((always_inline))
inline typename enable_float128<T>::type sin(T x)
{
    return ::sinq(x);
}

/**
 * @brief Wrap `cosq()`.
 */
template <typename T>
__attribute__((always_inline))
inline typename enable_float128<T>::type cos(T x)
{
    return ::cosq(x);
}

/**
 * @brief Wrap `tanq()`.
 */
template <typename T>
__attribute__((always_inline))
inline typename enable_float128<T>::type tan(T x)
{
    return ::tanq(x);
}

/**
 * @brief Wrap `asinq()`.
 */
template <typename T>
__attribute__((always_inline))
inline typename enable_float128<T>::type asin(T x)
{
    return ::asinq(x);
}

/**
 * @brief Wrap `acosq()`.
 */
template <typename T>
__attribute__((always_inline))
inline typename enable_float128<T>::type acos(T x)
{
    return ::acosq(x);
}

/**
 * @brief Wrap `atanq()`.
 */
template <typename T>
__attribute__((always_inline))
inline typename enable_float128<T>::type atan(T x)
{
    return ::atanq(x);
}

/**
 * @brief Wrap `atan2q()`.
 */
template <typename T>
__attribute__((always_inline))
inline typename enable_float128<T>::type atan2(T y, T x)
{
    return ::atan2q(y, x);
}

/**
 * @brief Wrap `sinhq()`.
 */
template <typename T>
__attribute__((always_inline))
inline typename enable_float128<T>::type sinh(T x)
{
    return ::sinhq(x);
}

/**
 * @brief Wrap `coshq()`.
 */
template <typename T>
__attribute__((always_inline))
inline typename enable_float128<T>::type cosh(T x)
{
    return ::coshq(x);
}

/**
 * @brief Wrap `tanhq()`.
 */
template <typename T>
__attribute__((always_inline))
inline typename enable_float128<T>::type tanh(T x)
{
    return ::tanhq(x);
}

/**
 * @brief Wrap `asinhq()`.
 */
template <typename T>
__attribute__((always_inline))
inline typename enable_float128<T>::type asinh(T x)
{
    return ::asinhq(x);
}

/**
 * @brief Wrap `acoshq()`.
 */
template <typename T>
__attribute__((always_inline))
inline typename enable_float128<T>::type acosh(T x)
{
    return ::acoshq(x);
}

/**
 * @brief Wrap `atanhq()`.
 */
template <typename T>
__attribute__((always_inline))
inline typename enable_float128<T>::type atanh(T x)
{
    return ::atanhq(x);
}

/**@}*/

/**
 * @name Quadmath wrappers (`__complex128`)
 */
/**@{*/

/**
 * @brief Wrap `cabsq()`.
 */
template <typename U>
__attribute__((always_inline))
inline typename enable_complex128<U, __float128>::type abs(U x)
{
    return ::cabsq(x);
}

/**
 * @brief Wrap `cargq()`.
 */
template <typename U>
__attribute__((always_inline))
inline typename enable_complex128<U, __float128>::type arg(U x)
{
    return ::cargq(x);
}

/**
 * @brief Wrap `crealq()`.
 */
template <typename U>
__attribute__((always_inline))
inline typename enable_complex128<U, __float128>::type real(U x)
{
    return ::crealq(x);
}

/**
 * @brief Wrap `cimagq()`.
 */
template <typename U>
__attribute__((always_inline))
inline typename enable_complex128<U, __float128>::type imag(U x)
{
    return ::cimagq(x);
}

/**
 * @brief Wrap `conjq()`.
 */
template <typename U>
__attribute__((always_inline))
inline typename enable_complex128<U>::type conj(U x)
{
    return ::conjq(x);
}

/**
 * @brief Wrap `cprojq()`.
 */
template <typename U>
__attribute__((always_inline))
inline typename enable_complex128<U>::type proj(U x)
{
    return ::cprojq(x);
}

/**
 * @brief Wrap `cexpq()`.
 */
template <typename U>
__attribute__((always_inline))
inline typename enable_complex128<U>::type exp(U x)
{
    return ::cexpq(x);
}

/**
 * @brief Wrap `clogq()`.
 */
template <typename U>
__attribute__((always_inline))
inline typename enable_complex128<U>::type log(U x)
{
    return ::clogq(x);
}

/**
 * @brief Wrap `clog10q()`.
 */
template <typename U>
__attribute__((always_inline))
inline typename enable_complex128<U>::type log10(U x)
{
    return ::clog10q(x);
}

/**
 * @brief Wrap `cpowq()`.
 */
template <typename U>
__attribute__((always_inline))
inline typename enable_complex128<U>::type pow(U x, U y)
{
    return ::cpowq(x, y);
}

/**
 * @brief Wrap `csqrtq()`.
 */
template <typename U>
__attribute__((always_inline))
inline typename enable_complex128<U>::type sqrt(U x)
{
    return ::csqrtq(x);
}

/**
 * @brief Wrap `csinq()`.
 */
template <typename U>
__attribute__((always_inline))
inline typename enable_complex128<U>::type sin(U x)
{
    return ::csinq(x);
}

/**
 * @brief Wrap `ccosq()`.
 */
template <typename U>
__attribute__((always_inline))
inline typename enable_complex128<U>::type cos(U x)
{
    return ::ccosq(x);
}

/**
 * @brief Wrap `ctanq()`.
 */
template <typename U>
__attribute__((always_inline))
inline typename enable_complex128<U>::type tan(U x)
{
    return ::ctanq(x);
}

/**
 * @brief Wrap `casinq()`.
 */
template <typename U>
__attribute__((always_inline))
inline typename enable_complex128<U>::type asin(U x)
{
    return ::casinq(x);
}

/**
 * @brief Wrap `cacosq()`.
 */
template <typename U>
__attribute__((always_inline))
inline typename enable_complex128<U>::type acos(U x)
{
    return ::cacosq(x);
}

/**
 * @brief Wrap `catanq()`.
 */
template <typename U>
__attribute__((always_inline))
inline typename enable_complex128<U>::type atan(U x)
{
    return ::catanq(x);
}

/**
 * @brief Wrap `csinhq()`.
 */
template <typename U>
__attribute__((always_inline))
inline typename enable_complex128<U>::type sinh(U x)
{
    return ::csinhq(x);
}

/**
 * @brief Wrap `ccoshq()`.
 */
template <typename U>
__attribute__((always_inline))
inline typename enable_complex128<U>::type cosh(U x)
{
    return ::ccoshq(x);
}

/**
 * @brief Wrap `ctanhq()`.
 */
template <typename U>
__attribute__((always_inline))
inline typename enable_complex128<U>::type tanh(U x)
{
    return ::ctanhq(x);
}

/**
 * @brief Wrap `casinhq()`.
 */
template <typename U>
__attribute__((always_inline))
inline typename enable_complex128<U>::type asinh(U x)
{
    return ::casinhq(x);
}

/**
 * @brief Wrap `cacoshq()`.
 */
template <typename U>
__attribute__((always_inline))
inline typename enable_complex128<U>::type acosh(U x)
{
    return ::cacoshq(x);
}

/**
 * @brief Wrap `catanhq()`.
 */
template <typename U>
__attribute__((always_inline))
inline typename enable_complex128<U>::type atanh(U x)
{
    return ::catanhq(x);
}

/**@}*/

/**@}*/

