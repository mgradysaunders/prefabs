#if !DOXYGEN
#define DECLTYPE(func, ...) -> decltype(std::func(__VA_ARGS__))
#endif // #if !DOXYGEN

/**
 * @addtogroup math
 */
/**@{*/

/**
 * @name Math wrappers
 */
/**@{*/

/**
 * @brief Wrap `std::fabs()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto fabs(T x) DECLTYPE(fabs, x)
{
    return std::fabs(x);
}

/**
 * @brief Wrap `std::fma()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto fma(T x, T y, T z) DECLTYPE(fma, x, y, z)
{
    return std::fma(x, y, z);
}

/**
 * @brief Wrap `std::fmin()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto fmin(T x, T y) DECLTYPE(fmin, x, y)
{
    return std::fmin(x, y);
}

/**
 * @brief Wrap `std::fmax()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto fmax(T x, T y) DECLTYPE(fmax, x, y)
{
    return std::fmax(x, y);
}

/**
 * @brief Wrap `std::fdim()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto fdim(T x, T y) DECLTYPE(fdim, x, y)
{
    return std::fdim(x, y);
}

/**
 * @brief Wrap `std::fmod()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto fmod(T x, T y) DECLTYPE(fmod, x, y)
{
    return std::fmod(x, y);
}

/**
 * @brief Wrap `std::remquo()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto remquo(T x, T y, int* q) DECLTYPE(remquo, x, y, q)
{
    return std::remquo(x, y, q);
}

/**
 * @brief Wrap `std::remainder()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto remainder(T x, T y) DECLTYPE(remainder, x, y)
{
    return std::remainder(x, y);
}

/**
 * @brief Wrap `std::nearbyint()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto nearbyint(T x) DECLTYPE(nearbyint, x)
{
    return std::nearbyint(x);
}

/**
 * @brief Wrap `std::floor()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto floor(T x) DECLTYPE(floor, x)
{
    return std::floor(x);
}

/**
 * @brief Wrap `std::ceil()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto ceil(T x) DECLTYPE(ceil, x)
{
    return std::ceil(x);
}

/**
 * @brief Wrap `std::trunc()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto trunc(T x) DECLTYPE(trunc, x)
{
    return std::trunc(x);
}

/**
 * @brief Wrap `std::round()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto round(T x) DECLTYPE(round, x)
{
    return std::round(x);
}

/**
 * @brief Wrap `std::rint()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto rint(T x) DECLTYPE(rint, x)
{
    return std::rint(x);
}

/**
 * @brief Wrap `std::lrint()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto lrint(T x) DECLTYPE(lrint, x)
{
    return std::lrint(x);
}

/**
 * @brief Wrap `std::llrint()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto llrint(T x) DECLTYPE(llrint, x)
{
    return std::llrint(x);
}

/**
 * @brief Wrap `std::lround()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto lround(T x) DECLTYPE(lround, x)
{
    return std::lround(x);
}

/**
 * @brief Wrap `std::llround()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto llround(T x) DECLTYPE(llround, x)
{
    return std::llround(x);
}

/**
 * @brief Wrap `std::frexp()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto frexp(T x, int* p) DECLTYPE(frexp, x, p)
{
    return std::frexp(x, p);
}

/**
 * @brief Wrap `std::ldexp()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto ldexp(T x, int p) DECLTYPE(ldexp, x, p)
{
    return std::ldexp(x, p);
}

/**
 * @brief Wrap `std::logb()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto logb(T x) DECLTYPE(logb, x)
{
    return std::logb(x);
}

/**
 * @brief Wrap `std::ilogb()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto ilogb(T x) DECLTYPE(ilogb, x)
{
    return std::ilogb(x);
}

/**
 * @brief Wrap `std::scalbn()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto scalbn(T x, int p) DECLTYPE(scalbn, x, p)
{
    return std::scalbn(x, p);
}

/**
 * @brief Wrap `std::scalbln()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto scalbln(T x, long p) DECLTYPE(scalbln, x, p)
{
    return std::scalbln(x, p);
}

/**
 * @brief Wrap `std::modf()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto modf(T x, T* p) DECLTYPE(modf, x, p)
{
    return std::modf(x, p);
}

/**
 * @brief Wrap `std::nextafter()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto nextafter(T x, T y) DECLTYPE(nextafter, x, y)
{
    return std::nextafter(x, y);
}

/**
 * @brief Wrap `std::nexttoward()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto nexttoward(T x, long double y) DECLTYPE(nexttoward, x, y)
{
    return std::nexttoward(x, y);
}

/**
 * @brief Wrap `std::copysign()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto copysign(T x, T y) DECLTYPE(copysign, x, y)
{
    return std::copysign(x, y);
}

/**
 * @brief Wrap `std::signbit()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto signbit(T x) DECLTYPE(signbit, x)
{
    return std::signbit(x);
}

/**
 * @brief Wrap `std::isnan()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto isnan(T x) DECLTYPE(isnan, x)
{
    return std::isnan(x);
}

/**
 * @brief Wrap `std::isinf()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto isinf(T x) DECLTYPE(isinf, x)
{
    return std::isinf(x);
}

/**
 * @brief Wrap `std::isfinite()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto isfinite(T x) DECLTYPE(isfinite, x)
{
    return std::isfinite(x);
}

/**
 * @brief Wrap `std::isnormal()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto isnormal(T x) DECLTYPE(isnormal, x)
{
    return std::isnormal(x);
}

/**
 * @brief Wrap `std::exp()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto exp(T x) DECLTYPE(exp, x)
{
    return std::exp(x);
}

/**
 * @brief Wrap `std::log()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto log(T x) DECLTYPE(log, x)
{
    return std::log(x);
}

/**
 * @brief Wrap `std::exp2()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto exp2(T x) DECLTYPE(exp2, x)
{
    return std::exp2(x);
}

/**
 * @brief Wrap `std::log2()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto log2(T x) DECLTYPE(log2, x)
{
    return std::log2(x);
}

/**
 * @brief Wrap `std::log10()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto log10(T x) DECLTYPE(log10, x)
{
    return std::log10(x);
}

/**
 * @brief Wrap `std::expm1()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto expm1(T x) DECLTYPE(expm1, x)
{
    return std::expm1(x);
}

/**
 * @brief Wrap `std::log1p()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto log1p(T x) DECLTYPE(log1p, x)
{
    return std::log1p(x);
}

/**
 * @brief Wrap `std::pow()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto pow(T x, T y) DECLTYPE(pow, x, y)
{
    return std::pow(x, y);
}

/**
 * @brief Wrap `std::sqrt()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto sqrt(T x) DECLTYPE(sqrt, x)
{
    return std::sqrt(x);
}

/**
 * @brief Wrap `std::cbrt()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto cbrt(T x) DECLTYPE(cbrt, x)
{
    return std::cbrt(x);
}

/**
 * @brief Wrap `std::hypot()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto hypot(T x, T y) DECLTYPE(hypot, x, y)
{
    return std::hypot(x, y);
}

/**
 * @brief Wrap `std::erf()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto erf(T x) DECLTYPE(erf, x)
{
    return std::erf(x);
}

/**
 * @brief Wrap `std::erfc()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto erfc(T x) DECLTYPE(erfc, x)
{
    return std::erfc(x);
}

/**
 * @brief Wrap `std::lgamma()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto lgamma(T x) DECLTYPE(lgamma, x)
{
    return std::lgamma(x);
}

/**
 * @brief Wrap `std::tgamma()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto tgamma(T x) DECLTYPE(tgamma, x)
{
    return std::tgamma(x);
}

/**
 * @brief Wrap `std::sin()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto sin(T x) DECLTYPE(sin, x)
{
    return std::sin(x);
}

/**
 * @brief Wrap `std::cos()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto cos(T x) DECLTYPE(cos, x)
{
    return std::cos(x);
}

/**
 * @brief Wrap `std::tan()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto tan(T x) DECLTYPE(tan, x)
{
    return std::tan(x);
}

/**
 * @brief Wrap `std::asin()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto asin(T x) DECLTYPE(asin, x)
{
    return std::asin(x);
}

/**
 * @brief Wrap `std::acos()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto acos(T x) DECLTYPE(acos, x)
{
    return std::acos(x);
}

/**
 * @brief Wrap `std::atan()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto atan(T x) DECLTYPE(atan, x)
{
    return std::atan(x);
}

/**
 * @brief Wrap `std::atan2()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto atan2(T y, T x) DECLTYPE(atan2, y, x)
{
    return std::atan2(y, x);
}

/**
 * @brief Wrap `std::sinh()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto sinh(T x) DECLTYPE(sinh, x)
{
    return std::sinh(x);
}

/**
 * @brief Wrap `std::cosh()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto cosh(T x) DECLTYPE(cosh, x)
{
    return std::cosh(x);
}

/**
 * @brief Wrap `std::tanh()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto tanh(T x) DECLTYPE(tanh, x)
{
    return std::tanh(x);
}

/**
 * @brief Wrap `std::asinh()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto asinh(T x) DECLTYPE(asinh, x)
{
    return std::asinh(x);
}

/**
 * @brief Wrap `std::acosh()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto acosh(T x) DECLTYPE(acosh, x)
{
    return std::acosh(x);
}

/**
 * @brief Wrap `std::atanh()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto atanh(T x) DECLTYPE(atanh, x)
{
    return std::atanh(x);
}

/**@}*/

/**@}*/

#if !DOXYGEN
#undef DECLTYPE
#endif // #if !DOXYGEN

