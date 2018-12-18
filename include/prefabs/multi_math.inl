/**
 * @addtogroup multi_math
 */
/**@{*/

/**
 * @name Math wrappers (multi)
 */
/**@{*/

/**
 * @brief Wrap `pr::abs()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline multi<decltype(pr::abs(T())), N...> abs(const multi<T, N...>& x)
{
    multi<decltype(pr::abs(T())), N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pr::abs(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pr::arg()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline multi<decltype(pr::arg(T())), N...> arg(const multi<T, N...>& x)
{
    multi<decltype(pr::arg(T())), N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pr::arg(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pr::min()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline multi<decltype(pr::min(T(), T())), N...> min(const multi<T, N...>& x, const multi<T, N...>& y)
{
    multi<decltype(pr::min(T(), T())), N...> res;
    auto itrx = x.begin();
    auto itry = y.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itry, ++itrres) {
        *itrres = pr::min(*itrx, *itry);
    }
    return res;
}

/**
 * @brief Wrap `pr::max()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline multi<decltype(pr::max(T(), T())), N...> max(const multi<T, N...>& x, const multi<T, N...>& y)
{
    multi<decltype(pr::max(T(), T())), N...> res;
    auto itrx = x.begin();
    auto itry = y.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itry, ++itrres) {
        *itrres = pr::max(*itrx, *itry);
    }
    return res;
}

/**
 * @brief Wrap `pr::fabs()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline multi<decltype(pr::fabs(T())), N...> fabs(const multi<T, N...>& x)
{
    multi<decltype(pr::fabs(T())), N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pr::fabs(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pr::fma()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline multi<decltype(pr::fma(T(), T(), T())), N...> fma(const multi<T, N...>& x, const multi<T, N...>& y, const multi<T, N...>& z)
{
    multi<decltype(pr::fma(T(), T(), T())), N...> res;
    auto itrx = x.begin();
    auto itry = y.begin();
    auto itrz = z.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itry, ++itrz, ++itrres) {
        *itrres = pr::fma(*itrx, *itry, *itrz);
    }
    return res;
}

/**
 * @brief Wrap `pr::fmin()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline multi<decltype(pr::fmin(T(), T())), N...> fmin(const multi<T, N...>& x, const multi<T, N...>& y)
{
    multi<decltype(pr::fmin(T(), T())), N...> res;
    auto itrx = x.begin();
    auto itry = y.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itry, ++itrres) {
        *itrres = pr::fmin(*itrx, *itry);
    }
    return res;
}

/**
 * @brief Wrap `pr::fmax()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline multi<decltype(pr::fmax(T(), T())), N...> fmax(const multi<T, N...>& x, const multi<T, N...>& y)
{
    multi<decltype(pr::fmax(T(), T())), N...> res;
    auto itrx = x.begin();
    auto itry = y.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itry, ++itrres) {
        *itrres = pr::fmax(*itrx, *itry);
    }
    return res;
}

/**
 * @brief Wrap `pr::fdim()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline multi<decltype(pr::fdim(T(), T())), N...> fdim(const multi<T, N...>& x, const multi<T, N...>& y)
{
    multi<decltype(pr::fdim(T(), T())), N...> res;
    auto itrx = x.begin();
    auto itry = y.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itry, ++itrres) {
        *itrres = pr::fdim(*itrx, *itry);
    }
    return res;
}

/**
 * @brief Wrap `pr::fmod()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline multi<decltype(pr::fmod(T(), T())), N...> fmod(const multi<T, N...>& x, const multi<T, N...>& y)
{
    multi<decltype(pr::fmod(T(), T())), N...> res;
    auto itrx = x.begin();
    auto itry = y.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itry, ++itrres) {
        *itrres = pr::fmod(*itrx, *itry);
    }
    return res;
}

/**
 * @brief Wrap `pr::remquo()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline multi<decltype(pr::remquo(T(), T(), std::declval<int*>())), N...> remquo(const multi<T, N...>& x, const multi<T, N...>& y, multi<int, N...>* q)
{
    multi<decltype(pr::remquo(T(), T(), std::declval<int*>())), N...> res;
    auto itrx = x.begin();
    auto itry = y.begin();
    auto itrq = q->begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itry, ++itrq, ++itrres) {
        *itrres = pr::remquo(*itrx, *itry, &(*itrq));
    }
    return res;
}

/**
 * @brief Wrap `pr::remainder()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline multi<decltype(pr::remainder(T(), T())), N...> remainder(const multi<T, N...>& x, const multi<T, N...>& y)
{
    multi<decltype(pr::remainder(T(), T())), N...> res;
    auto itrx = x.begin();
    auto itry = y.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itry, ++itrres) {
        *itrres = pr::remainder(*itrx, *itry);
    }
    return res;
}

/**
 * @brief Wrap `pr::nearbyint()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline multi<decltype(pr::nearbyint(T())), N...> nearbyint(const multi<T, N...>& x)
{
    multi<decltype(pr::nearbyint(T())), N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pr::nearbyint(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pr::floor()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline multi<decltype(pr::floor(T())), N...> floor(const multi<T, N...>& x)
{
    multi<decltype(pr::floor(T())), N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pr::floor(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pr::ceil()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline multi<decltype(pr::ceil(T())), N...> ceil(const multi<T, N...>& x)
{
    multi<decltype(pr::ceil(T())), N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pr::ceil(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pr::trunc()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline multi<decltype(pr::trunc(T())), N...> trunc(const multi<T, N...>& x)
{
    multi<decltype(pr::trunc(T())), N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pr::trunc(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pr::round()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline multi<decltype(pr::round(T())), N...> round(const multi<T, N...>& x)
{
    multi<decltype(pr::round(T())), N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pr::round(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pr::rint()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline multi<decltype(pr::rint(T())), N...> rint(const multi<T, N...>& x)
{
    multi<decltype(pr::rint(T())), N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pr::rint(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pr::lrint()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline multi<decltype(pr::lrint(T())), N...> lrint(const multi<T, N...>& x)
{
    multi<decltype(pr::lrint(T())), N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pr::lrint(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pr::llrint()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline multi<decltype(pr::llrint(T())), N...> llrint(const multi<T, N...>& x)
{
    multi<decltype(pr::llrint(T())), N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pr::llrint(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pr::lround()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline multi<decltype(pr::lround(T())), N...> lround(const multi<T, N...>& x)
{
    multi<decltype(pr::lround(T())), N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pr::lround(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pr::llround()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline multi<decltype(pr::llround(T())), N...> llround(const multi<T, N...>& x)
{
    multi<decltype(pr::llround(T())), N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pr::llround(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pr::frexp()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline multi<decltype(pr::frexp(T(), std::declval<int*>())), N...> frexp(const multi<T, N...>& x, multi<int, N...>* p)
{
    multi<decltype(pr::frexp(T(), std::declval<int*>())), N...> res;
    auto itrx = x.begin();
    auto itrp = p->begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrp, ++itrres) {
        *itrres = pr::frexp(*itrx, &(*itrp));
    }
    return res;
}

/**
 * @brief Wrap `pr::ldexp()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline multi<decltype(pr::ldexp(T(), std::declval<int>())), N...> ldexp(const multi<T, N...>& x, const multi<int, N...>& p)
{
    multi<decltype(pr::ldexp(T(), std::declval<int>())), N...> res;
    auto itrx = x.begin();
    auto itrp = p.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrp, ++itrres) {
        *itrres = pr::ldexp(*itrx, *itrp);
    }
    return res;
}

/**
 * @brief Wrap `pr::logb()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline multi<decltype(pr::logb(T())), N...> logb(const multi<T, N...>& x)
{
    multi<decltype(pr::logb(T())), N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pr::logb(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pr::ilogb()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline multi<decltype(pr::ilogb(T())), N...> ilogb(const multi<T, N...>& x)
{
    multi<decltype(pr::ilogb(T())), N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pr::ilogb(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pr::scalbn()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline multi<decltype(pr::scalbn(T(), std::declval<int>())), N...> scalbn(const multi<T, N...>& x, const multi<int, N...>& p)
{
    multi<decltype(pr::scalbn(T(), std::declval<int>())), N...> res;
    auto itrx = x.begin();
    auto itrp = p.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrp, ++itrres) {
        *itrres = pr::scalbn(*itrx, *itrp);
    }
    return res;
}

/**
 * @brief Wrap `pr::scalbln()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline multi<decltype(pr::scalbln(T(), std::declval<long>())), N...> scalbln(const multi<T, N...>& x, const multi<long, N...>& p)
{
    multi<decltype(pr::scalbln(T(), std::declval<long>())), N...> res;
    auto itrx = x.begin();
    auto itrp = p.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrp, ++itrres) {
        *itrres = pr::scalbln(*itrx, *itrp);
    }
    return res;
}

/**
 * @brief Wrap `pr::modf()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline multi<decltype(pr::modf(T(), std::declval<T*>())), N...> modf(const multi<T, N...>& x, multi<T, N...>* p)
{
    multi<decltype(pr::modf(T(), std::declval<T*>())), N...> res;
    auto itrx = x.begin();
    auto itrp = p->begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrp, ++itrres) {
        *itrres = pr::modf(*itrx, &(*itrp));
    }
    return res;
}

/**
 * @brief Wrap `pr::nextafter()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline multi<decltype(pr::nextafter(T(), T())), N...> nextafter(const multi<T, N...>& x, const multi<T, N...>& y)
{
    multi<decltype(pr::nextafter(T(), T())), N...> res;
    auto itrx = x.begin();
    auto itry = y.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itry, ++itrres) {
        *itrres = pr::nextafter(*itrx, *itry);
    }
    return res;
}

/**
 * @brief Wrap `pr::nexttoward()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline multi<decltype(pr::nexttoward(T(), std::declval<long double>())), N...> nexttoward(const multi<T, N...>& x, const multi<long double, N...>& y)
{
    multi<decltype(pr::nexttoward(T(), std::declval<long double>())), N...> res;
    auto itrx = x.begin();
    auto itry = y.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itry, ++itrres) {
        *itrres = pr::nexttoward(*itrx, *itry);
    }
    return res;
}

/**
 * @brief Wrap `pr::copysign()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline multi<decltype(pr::copysign(T(), T())), N...> copysign(const multi<T, N...>& x, const multi<T, N...>& y)
{
    multi<decltype(pr::copysign(T(), T())), N...> res;
    auto itrx = x.begin();
    auto itry = y.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itry, ++itrres) {
        *itrres = pr::copysign(*itrx, *itry);
    }
    return res;
}

/**
 * @brief Wrap `pr::signbit()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline multi<decltype(pr::signbit(T())), N...> signbit(const multi<T, N...>& x)
{
    multi<decltype(pr::signbit(T())), N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pr::signbit(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pr::isnan()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline multi<decltype(pr::isnan(T())), N...> isnan(const multi<T, N...>& x)
{
    multi<decltype(pr::isnan(T())), N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pr::isnan(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pr::isinf()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline multi<decltype(pr::isinf(T())), N...> isinf(const multi<T, N...>& x)
{
    multi<decltype(pr::isinf(T())), N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pr::isinf(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pr::isfinite()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline multi<decltype(pr::isfinite(T())), N...> isfinite(const multi<T, N...>& x)
{
    multi<decltype(pr::isfinite(T())), N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pr::isfinite(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pr::isnormal()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline multi<decltype(pr::isnormal(T())), N...> isnormal(const multi<T, N...>& x)
{
    multi<decltype(pr::isnormal(T())), N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pr::isnormal(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pr::exp()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline multi<decltype(pr::exp(T())), N...> exp(const multi<T, N...>& x)
{
    multi<decltype(pr::exp(T())), N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pr::exp(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pr::log()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline multi<decltype(pr::log(T())), N...> log(const multi<T, N...>& x)
{
    multi<decltype(pr::log(T())), N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pr::log(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pr::exp2()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline multi<decltype(pr::exp2(T())), N...> exp2(const multi<T, N...>& x)
{
    multi<decltype(pr::exp2(T())), N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pr::exp2(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pr::log2()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline multi<decltype(pr::log2(T())), N...> log2(const multi<T, N...>& x)
{
    multi<decltype(pr::log2(T())), N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pr::log2(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pr::log10()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline multi<decltype(pr::log10(T())), N...> log10(const multi<T, N...>& x)
{
    multi<decltype(pr::log10(T())), N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pr::log10(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pr::expm1()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline multi<decltype(pr::expm1(T())), N...> expm1(const multi<T, N...>& x)
{
    multi<decltype(pr::expm1(T())), N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pr::expm1(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pr::log1p()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline multi<decltype(pr::log1p(T())), N...> log1p(const multi<T, N...>& x)
{
    multi<decltype(pr::log1p(T())), N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pr::log1p(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pr::pow()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline multi<decltype(pr::pow(T(), T())), N...> pow(const multi<T, N...>& x, const multi<T, N...>& y)
{
    multi<decltype(pr::pow(T(), T())), N...> res;
    auto itrx = x.begin();
    auto itry = y.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itry, ++itrres) {
        *itrres = pr::pow(*itrx, *itry);
    }
    return res;
}

/**
 * @brief Wrap `pr::sqrt()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline multi<decltype(pr::sqrt(T())), N...> sqrt(const multi<T, N...>& x)
{
    multi<decltype(pr::sqrt(T())), N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pr::sqrt(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pr::cbrt()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline multi<decltype(pr::cbrt(T())), N...> cbrt(const multi<T, N...>& x)
{
    multi<decltype(pr::cbrt(T())), N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pr::cbrt(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pr::hypot()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline multi<decltype(pr::hypot(T(), T())), N...> hypot(const multi<T, N...>& x, const multi<T, N...>& y)
{
    multi<decltype(pr::hypot(T(), T())), N...> res;
    auto itrx = x.begin();
    auto itry = y.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itry, ++itrres) {
        *itrres = pr::hypot(*itrx, *itry);
    }
    return res;
}

/**
 * @brief Wrap `pr::erf()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline multi<decltype(pr::erf(T())), N...> erf(const multi<T, N...>& x)
{
    multi<decltype(pr::erf(T())), N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pr::erf(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pr::erfc()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline multi<decltype(pr::erfc(T())), N...> erfc(const multi<T, N...>& x)
{
    multi<decltype(pr::erfc(T())), N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pr::erfc(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pr::lgamma()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline multi<decltype(pr::lgamma(T())), N...> lgamma(const multi<T, N...>& x)
{
    multi<decltype(pr::lgamma(T())), N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pr::lgamma(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pr::tgamma()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline multi<decltype(pr::tgamma(T())), N...> tgamma(const multi<T, N...>& x)
{
    multi<decltype(pr::tgamma(T())), N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pr::tgamma(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pr::sin()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline multi<decltype(pr::sin(T())), N...> sin(const multi<T, N...>& x)
{
    multi<decltype(pr::sin(T())), N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pr::sin(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pr::cos()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline multi<decltype(pr::cos(T())), N...> cos(const multi<T, N...>& x)
{
    multi<decltype(pr::cos(T())), N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pr::cos(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pr::tan()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline multi<decltype(pr::tan(T())), N...> tan(const multi<T, N...>& x)
{
    multi<decltype(pr::tan(T())), N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pr::tan(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pr::asin()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline multi<decltype(pr::asin(T())), N...> asin(const multi<T, N...>& x)
{
    multi<decltype(pr::asin(T())), N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pr::asin(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pr::acos()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline multi<decltype(pr::acos(T())), N...> acos(const multi<T, N...>& x)
{
    multi<decltype(pr::acos(T())), N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pr::acos(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pr::atan()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline multi<decltype(pr::atan(T())), N...> atan(const multi<T, N...>& x)
{
    multi<decltype(pr::atan(T())), N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pr::atan(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pr::atan2()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline multi<decltype(pr::atan2(T(), T())), N...> atan2(const multi<T, N...>& y, const multi<T, N...>& x)
{
    multi<decltype(pr::atan2(T(), T())), N...> res;
    auto itry = y.begin();
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itry, ++itrx, ++itrres) {
        *itrres = pr::atan2(*itry, *itrx);
    }
    return res;
}

/**
 * @brief Wrap `pr::sinh()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline multi<decltype(pr::sinh(T())), N...> sinh(const multi<T, N...>& x)
{
    multi<decltype(pr::sinh(T())), N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pr::sinh(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pr::cosh()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline multi<decltype(pr::cosh(T())), N...> cosh(const multi<T, N...>& x)
{
    multi<decltype(pr::cosh(T())), N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pr::cosh(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pr::tanh()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline multi<decltype(pr::tanh(T())), N...> tanh(const multi<T, N...>& x)
{
    multi<decltype(pr::tanh(T())), N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pr::tanh(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pr::asinh()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline multi<decltype(pr::asinh(T())), N...> asinh(const multi<T, N...>& x)
{
    multi<decltype(pr::asinh(T())), N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pr::asinh(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pr::acosh()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline multi<decltype(pr::acosh(T())), N...> acosh(const multi<T, N...>& x)
{
    multi<decltype(pr::acosh(T())), N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pr::acosh(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pr::atanh()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline multi<decltype(pr::atanh(T())), N...> atanh(const multi<T, N...>& x)
{
    multi<decltype(pr::atanh(T())), N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pr::atanh(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pr::csc()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline multi<decltype(pr::csc(T())), N...> csc(const multi<T, N...>& x)
{
    multi<decltype(pr::csc(T())), N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pr::csc(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pr::sec()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline multi<decltype(pr::sec(T())), N...> sec(const multi<T, N...>& x)
{
    multi<decltype(pr::sec(T())), N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pr::sec(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pr::cot()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline multi<decltype(pr::cot(T())), N...> cot(const multi<T, N...>& x)
{
    multi<decltype(pr::cot(T())), N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pr::cot(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pr::csch()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline multi<decltype(pr::csch(T())), N...> csch(const multi<T, N...>& x)
{
    multi<decltype(pr::csch(T())), N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pr::csch(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pr::sech()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline multi<decltype(pr::sech(T())), N...> sech(const multi<T, N...>& x)
{
    multi<decltype(pr::sech(T())), N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pr::sech(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pr::coth()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline multi<decltype(pr::coth(T())), N...> coth(const multi<T, N...>& x)
{
    multi<decltype(pr::coth(T())), N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pr::coth(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pr::acsc()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline multi<decltype(pr::acsc(T())), N...> acsc(const multi<T, N...>& x)
{
    multi<decltype(pr::acsc(T())), N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pr::acsc(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pr::asec()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline multi<decltype(pr::asec(T())), N...> asec(const multi<T, N...>& x)
{
    multi<decltype(pr::asec(T())), N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pr::asec(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pr::acot()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline multi<decltype(pr::acot(T())), N...> acot(const multi<T, N...>& x)
{
    multi<decltype(pr::acot(T())), N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pr::acot(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pr::acsch()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline multi<decltype(pr::acsch(T())), N...> acsch(const multi<T, N...>& x)
{
    multi<decltype(pr::acsch(T())), N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pr::acsch(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pr::asech()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline multi<decltype(pr::asech(T())), N...> asech(const multi<T, N...>& x)
{
    multi<decltype(pr::asech(T())), N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pr::asech(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pr::acoth()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline multi<decltype(pr::acoth(T())), N...> acoth(const multi<T, N...>& x)
{
    multi<decltype(pr::acoth(T())), N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pr::acoth(*itrx);
    }
    return res;
}

/**@}*/

/**@}*/

