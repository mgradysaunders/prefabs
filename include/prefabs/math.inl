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
inline auto fabs(T x)
#if !DOXYGEN
    -> decltype(std::fabs(x))
#endif // #if !DOXYGEN
{
    return std::fabs(x);
}

/**
 * @brief Wrap `std::fma()`.
 */
template <typename T>
__attribute__((always_inline))
inline auto fma(T x, T y, T z)
#if !DOXYGEN
    -> decltype(std::fma(x, y, z))
#endif // #if !DOXYGEN
{
    return std::fma(x, y, z);
}

/**
 * @brief Wrap `std::fmin()`.
 */
template <typename T>
__attribute__((always_inline))
inline auto fmin(T x, T y)
#if !DOXYGEN
    -> decltype(std::fmin(x, y))
#endif // #if !DOXYGEN
{
    return std::fmin(x, y);
}

/**
 * @brief Wrap `std::fmax()`.
 */
template <typename T>
__attribute__((always_inline))
inline auto fmax(T x, T y)
#if !DOXYGEN
    -> decltype(std::fmax(x, y))
#endif // #if !DOXYGEN
{
    return std::fmax(x, y);
}

/**
 * @brief Wrap `std::fdim()`.
 */
template <typename T>
__attribute__((always_inline))
inline auto fdim(T x, T y)
#if !DOXYGEN
    -> decltype(std::fdim(x, y))
#endif // #if !DOXYGEN
{
    return std::fdim(x, y);
}

/**
 * @brief Wrap `std::fmod()`.
 */
template <typename T>
__attribute__((always_inline))
inline auto fmod(T x, T y)
#if !DOXYGEN
    -> decltype(std::fmod(x, y))
#endif // #if !DOXYGEN
{
    return std::fmod(x, y);
}

/**
 * @brief Wrap `std::remquo()`.
 */
template <typename T>
__attribute__((always_inline))
inline auto remquo(T x, T y, int* q)
#if !DOXYGEN
    -> decltype(std::remquo(x, y, q))
#endif // #if !DOXYGEN
{
    return std::remquo(x, y, q);
}

/**
 * @brief Wrap `std::remainder()`.
 */
template <typename T>
__attribute__((always_inline))
inline auto remainder(T x, T y)
#if !DOXYGEN
    -> decltype(std::remainder(x, y))
#endif // #if !DOXYGEN
{
    return std::remainder(x, y);
}

/**
 * @brief Wrap `std::nearbyint()`.
 */
template <typename T>
__attribute__((always_inline))
inline auto nearbyint(T x)
#if !DOXYGEN
    -> decltype(std::nearbyint(x))
#endif // #if !DOXYGEN
{
    return std::nearbyint(x);
}

/**
 * @brief Wrap `std::floor()`.
 */
template <typename T>
__attribute__((always_inline))
inline auto floor(T x)
#if !DOXYGEN
    -> decltype(std::floor(x))
#endif // #if !DOXYGEN
{
    return std::floor(x);
}

/**
 * @brief Wrap `std::ceil()`.
 */
template <typename T>
__attribute__((always_inline))
inline auto ceil(T x)
#if !DOXYGEN
    -> decltype(std::ceil(x))
#endif // #if !DOXYGEN
{
    return std::ceil(x);
}

/**
 * @brief Wrap `std::trunc()`.
 */
template <typename T>
__attribute__((always_inline))
inline auto trunc(T x)
#if !DOXYGEN
    -> decltype(std::trunc(x))
#endif // #if !DOXYGEN
{
    return std::trunc(x);
}

/**
 * @brief Wrap `std::round()`.
 */
template <typename T>
__attribute__((always_inline))
inline auto round(T x)
#if !DOXYGEN
    -> decltype(std::round(x))
#endif // #if !DOXYGEN
{
    return std::round(x);
}

/**
 * @brief Wrap `std::rint()`.
 */
template <typename T>
__attribute__((always_inline))
inline auto rint(T x)
#if !DOXYGEN
    -> decltype(std::rint(x))
#endif // #if !DOXYGEN
{
    return std::rint(x);
}

/**
 * @brief Wrap `std::lrint()`.
 */
template <typename T>
__attribute__((always_inline))
inline auto lrint(T x)
#if !DOXYGEN
    -> decltype(std::lrint(x))
#endif // #if !DOXYGEN
{
    return std::lrint(x);
}

/**
 * @brief Wrap `std::llrint()`.
 */
template <typename T>
__attribute__((always_inline))
inline auto llrint(T x)
#if !DOXYGEN
    -> decltype(std::llrint(x))
#endif // #if !DOXYGEN
{
    return std::llrint(x);
}

/**
 * @brief Wrap `std::lround()`.
 */
template <typename T>
__attribute__((always_inline))
inline auto lround(T x)
#if !DOXYGEN
    -> decltype(std::lround(x))
#endif // #if !DOXYGEN
{
    return std::lround(x);
}

/**
 * @brief Wrap `std::llround()`.
 */
template <typename T>
__attribute__((always_inline))
inline auto llround(T x)
#if !DOXYGEN
    -> decltype(std::llround(x))
#endif // #if !DOXYGEN
{
    return std::llround(x);
}

/**
 * @brief Wrap `std::frexp()`.
 */
template <typename T>
__attribute__((always_inline))
inline auto frexp(T x, int* p)
#if !DOXYGEN
    -> decltype(std::frexp(x, p))
#endif // #if !DOXYGEN
{
    return std::frexp(x, p);
}

/**
 * @brief Wrap `std::ldexp()`.
 */
template <typename T>
__attribute__((always_inline))
inline auto ldexp(T x, int p)
#if !DOXYGEN
    -> decltype(std::ldexp(x, p))
#endif // #if !DOXYGEN
{
    return std::ldexp(x, p);
}

/**
 * @brief Wrap `std::logb()`.
 */
template <typename T>
__attribute__((always_inline))
inline auto logb(T x)
#if !DOXYGEN
    -> decltype(std::logb(x))
#endif // #if !DOXYGEN
{
    return std::logb(x);
}

/**
 * @brief Wrap `std::ilogb()`.
 */
template <typename T>
__attribute__((always_inline))
inline auto ilogb(T x)
#if !DOXYGEN
    -> decltype(std::ilogb(x))
#endif // #if !DOXYGEN
{
    return std::ilogb(x);
}

/**
 * @brief Wrap `std::scalbn()`.
 */
template <typename T>
__attribute__((always_inline))
inline auto scalbn(T x, int p)
#if !DOXYGEN
    -> decltype(std::scalbn(x, p))
#endif // #if !DOXYGEN
{
    return std::scalbn(x, p);
}

/**
 * @brief Wrap `std::scalbln()`.
 */
template <typename T>
__attribute__((always_inline))
inline auto scalbln(T x, long p)
#if !DOXYGEN
    -> decltype(std::scalbln(x, p))
#endif // #if !DOXYGEN
{
    return std::scalbln(x, p);
}

/**
 * @brief Wrap `std::modf()`.
 */
template <typename T>
__attribute__((always_inline))
inline auto modf(T x, T* p)
#if !DOXYGEN
    -> decltype(std::modf(x, p))
#endif // #if !DOXYGEN
{
    return std::modf(x, p);
}

/**
 * @brief Wrap `std::nextafter()`.
 */
template <typename T>
__attribute__((always_inline))
inline auto nextafter(T x, T y)
#if !DOXYGEN
    -> decltype(std::nextafter(x, y))
#endif // #if !DOXYGEN
{
    return std::nextafter(x, y);
}

/**
 * @brief Wrap `std::nexttoward()`.
 */
template <typename T>
__attribute__((always_inline))
inline auto nexttoward(T x, long double y)
#if !DOXYGEN
    -> decltype(std::nexttoward(x, y))
#endif // #if !DOXYGEN
{
    return std::nexttoward(x, y);
}

/**
 * @brief Wrap `std::copysign()`.
 */
template <typename T>
__attribute__((always_inline))
inline auto copysign(T x, T y)
#if !DOXYGEN
    -> decltype(std::copysign(x, y))
#endif // #if !DOXYGEN
{
    return std::copysign(x, y);
}

/**
 * @brief Wrap `std::signbit()`.
 */
template <typename T>
__attribute__((always_inline))
inline auto signbit(T x)
#if !DOXYGEN
    -> decltype(std::signbit(x))
#endif // #if !DOXYGEN
{
    return std::signbit(x);
}

/**
 * @brief Wrap `std::isnan()`.
 */
template <typename T>
__attribute__((always_inline))
inline auto isnan(T x)
#if !DOXYGEN
    -> decltype(std::isnan(x))
#endif // #if !DOXYGEN
{
    return std::isnan(x);
}

/**
 * @brief Wrap `std::isinf()`.
 */
template <typename T>
__attribute__((always_inline))
inline auto isinf(T x)
#if !DOXYGEN
    -> decltype(std::isinf(x))
#endif // #if !DOXYGEN
{
    return std::isinf(x);
}

/**
 * @brief Wrap `std::isfinite()`.
 */
template <typename T>
__attribute__((always_inline))
inline auto isfinite(T x)
#if !DOXYGEN
    -> decltype(std::isfinite(x))
#endif // #if !DOXYGEN
{
    return std::isfinite(x);
}

/**
 * @brief Wrap `std::isnormal()`.
 */
template <typename T>
__attribute__((always_inline))
inline auto isnormal(T x)
#if !DOXYGEN
    -> decltype(std::isnormal(x))
#endif // #if !DOXYGEN
{
    return std::isnormal(x);
}

/**
 * @brief Wrap `std::exp()`.
 */
template <typename T>
__attribute__((always_inline))
inline auto exp(T x)
#if !DOXYGEN
    -> decltype(std::exp(x))
#endif // #if !DOXYGEN
{
    return std::exp(x);
}

/**
 * @brief Wrap `std::log()`.
 */
template <typename T>
__attribute__((always_inline))
inline auto log(T x)
#if !DOXYGEN
    -> decltype(std::log(x))
#endif // #if !DOXYGEN
{
    return std::log(x);
}

/**
 * @brief Wrap `std::exp2()`.
 */
template <typename T>
__attribute__((always_inline))
inline auto exp2(T x)
#if !DOXYGEN
    -> decltype(std::exp2(x))
#endif // #if !DOXYGEN
{
    return std::exp2(x);
}

/**
 * @brief Wrap `std::log2()`.
 */
template <typename T>
__attribute__((always_inline))
inline auto log2(T x)
#if !DOXYGEN
    -> decltype(std::log2(x))
#endif // #if !DOXYGEN
{
    return std::log2(x);
}

/**
 * @brief Wrap `std::log10()`.
 */
template <typename T>
__attribute__((always_inline))
inline auto log10(T x)
#if !DOXYGEN
    -> decltype(std::log10(x))
#endif // #if !DOXYGEN
{
    return std::log10(x);
}

/**
 * @brief Wrap `std::expm1()`.
 */
template <typename T>
__attribute__((always_inline))
inline auto expm1(T x)
#if !DOXYGEN
    -> decltype(std::expm1(x))
#endif // #if !DOXYGEN
{
    return std::expm1(x);
}

/**
 * @brief Wrap `std::log1p()`.
 */
template <typename T>
__attribute__((always_inline))
inline auto log1p(T x)
#if !DOXYGEN
    -> decltype(std::log1p(x))
#endif // #if !DOXYGEN
{
    return std::log1p(x);
}

/**
 * @brief Wrap `std::pow()`.
 */
template <typename T>
__attribute__((always_inline))
inline auto pow(T x, T y)
#if !DOXYGEN
    -> decltype(std::pow(x, y))
#endif // #if !DOXYGEN
{
    return std::pow(x, y);
}

/**
 * @brief Wrap `std::sqrt()`.
 */
template <typename T>
__attribute__((always_inline))
inline auto sqrt(T x)
#if !DOXYGEN
    -> decltype(std::sqrt(x))
#endif // #if !DOXYGEN
{
    return std::sqrt(x);
}

/**
 * @brief Wrap `std::cbrt()`.
 */
template <typename T>
__attribute__((always_inline))
inline auto cbrt(T x)
#if !DOXYGEN
    -> decltype(std::cbrt(x))
#endif // #if !DOXYGEN
{
    return std::cbrt(x);
}

/**
 * @brief Wrap `std::hypot()`.
 */
template <typename T>
__attribute__((always_inline))
inline auto hypot(T x, T y)
#if !DOXYGEN
    -> decltype(std::hypot(x, y))
#endif // #if !DOXYGEN
{
    return std::hypot(x, y);
}

/**
 * @brief Wrap `std::erf()`.
 */
template <typename T>
__attribute__((always_inline))
inline auto erf(T x)
#if !DOXYGEN
    -> decltype(std::erf(x))
#endif // #if !DOXYGEN
{
    return std::erf(x);
}

/**
 * @brief Wrap `std::erfc()`.
 */
template <typename T>
__attribute__((always_inline))
inline auto erfc(T x)
#if !DOXYGEN
    -> decltype(std::erfc(x))
#endif // #if !DOXYGEN
{
    return std::erfc(x);
}

/**
 * @brief Wrap `std::lgamma()`.
 */
template <typename T>
__attribute__((always_inline))
inline auto lgamma(T x)
#if !DOXYGEN
    -> decltype(std::lgamma(x))
#endif // #if !DOXYGEN
{
    return std::lgamma(x);
}

/**
 * @brief Wrap `std::tgamma()`.
 */
template <typename T>
__attribute__((always_inline))
inline auto tgamma(T x)
#if !DOXYGEN
    -> decltype(std::tgamma(x))
#endif // #if !DOXYGEN
{
    return std::tgamma(x);
}

/**
 * @brief Wrap `std::sin()`.
 */
template <typename T>
__attribute__((always_inline))
inline auto sin(T x)
#if !DOXYGEN
    -> decltype(std::sin(x))
#endif // #if !DOXYGEN
{
    return std::sin(x);
}

/**
 * @brief Wrap `std::cos()`.
 */
template <typename T>
__attribute__((always_inline))
inline auto cos(T x)
#if !DOXYGEN
    -> decltype(std::cos(x))
#endif // #if !DOXYGEN
{
    return std::cos(x);
}

/**
 * @brief Wrap `std::tan()`.
 */
template <typename T>
__attribute__((always_inline))
inline auto tan(T x)
#if !DOXYGEN
    -> decltype(std::tan(x))
#endif // #if !DOXYGEN
{
    return std::tan(x);
}

/**
 * @brief Wrap `std::asin()`.
 */
template <typename T>
__attribute__((always_inline))
inline auto asin(T x)
#if !DOXYGEN
    -> decltype(std::asin(x))
#endif // #if !DOXYGEN
{
    return std::asin(x);
}

/**
 * @brief Wrap `std::acos()`.
 */
template <typename T>
__attribute__((always_inline))
inline auto acos(T x)
#if !DOXYGEN
    -> decltype(std::acos(x))
#endif // #if !DOXYGEN
{
    return std::acos(x);
}

/**
 * @brief Wrap `std::atan()`.
 */
template <typename T>
__attribute__((always_inline))
inline auto atan(T x)
#if !DOXYGEN
    -> decltype(std::atan(x))
#endif // #if !DOXYGEN
{
    return std::atan(x);
}

/**
 * @brief Wrap `std::atan2()`.
 */
template <typename T>
__attribute__((always_inline))
inline auto atan2(T y, T x)
#if !DOXYGEN
    -> decltype(std::atan2(y, x))
#endif // #if !DOXYGEN
{
    return std::atan2(y, x);
}

/**
 * @brief Wrap `std::sinh()`.
 */
template <typename T>
__attribute__((always_inline))
inline auto sinh(T x)
#if !DOXYGEN
    -> decltype(std::sinh(x))
#endif // #if !DOXYGEN
{
    return std::sinh(x);
}

/**
 * @brief Wrap `std::cosh()`.
 */
template <typename T>
__attribute__((always_inline))
inline auto cosh(T x)
#if !DOXYGEN
    -> decltype(std::cosh(x))
#endif // #if !DOXYGEN
{
    return std::cosh(x);
}

/**
 * @brief Wrap `std::tanh()`.
 */
template <typename T>
__attribute__((always_inline))
inline auto tanh(T x)
#if !DOXYGEN
    -> decltype(std::tanh(x))
#endif // #if !DOXYGEN
{
    return std::tanh(x);
}

/**
 * @brief Wrap `std::asinh()`.
 */
template <typename T>
__attribute__((always_inline))
inline auto asinh(T x)
#if !DOXYGEN
    -> decltype(std::asinh(x))
#endif // #if !DOXYGEN
{
    return std::asinh(x);
}

/**
 * @brief Wrap `std::acosh()`.
 */
template <typename T>
__attribute__((always_inline))
inline auto acosh(T x)
#if !DOXYGEN
    -> decltype(std::acosh(x))
#endif // #if !DOXYGEN
{
    return std::acosh(x);
}

/**
 * @brief Wrap `std::atanh()`.
 */
template <typename T>
__attribute__((always_inline))
inline auto atanh(T x)
#if !DOXYGEN
    -> decltype(std::atanh(x))
#endif // #if !DOXYGEN
{
    return std::atanh(x);
}

/**@}*/

/**@}*/

