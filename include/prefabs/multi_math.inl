namespace pr {

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
inline auto abs(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pr::abs(
        std::declval<T>()))>, N...> res;
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
inline auto arg(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pr::arg(
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pr::arg(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pr::real()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto real(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pr::real(
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pr::real(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pr::imag()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto imag(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pr::imag(
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pr::imag(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pr::conj()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto conj(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pr::conj(
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pr::conj(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pr::norm()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto norm(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pr::norm(
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pr::norm(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pr::min()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto min(
            const multi<T, N...>& x,
            const multi<T, N...>& y)
{
    multi<
        std::decay_t<decltype(pr::min(
        std::declval<T>(),
        std::declval<T>()))>, N...> res;
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
inline auto max(
            const multi<T, N...>& x,
            const multi<T, N...>& y)
{
    multi<
        std::decay_t<decltype(pr::max(
        std::declval<T>(),
        std::declval<T>()))>, N...> res;
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
inline auto fabs(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pr::fabs(
        std::declval<T>()))>, N...> res;
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
inline auto fma(
            const multi<T, N...>& x,
            const multi<T, N...>& y,
            const multi<T, N...>& z)
{
    multi<
        std::decay_t<decltype(pr::fma(
        std::declval<T>(),
        std::declval<T>(),
        std::declval<T>()))>, N...> res;
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
inline auto fmin(
            const multi<T, N...>& x,
            const multi<T, N...>& y)
{
    multi<
        std::decay_t<decltype(pr::fmin(
        std::declval<T>(),
        std::declval<T>()))>, N...> res;
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
inline auto fmax(
            const multi<T, N...>& x,
            const multi<T, N...>& y)
{
    multi<
        std::decay_t<decltype(pr::fmax(
        std::declval<T>(),
        std::declval<T>()))>, N...> res;
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
inline auto fdim(
            const multi<T, N...>& x,
            const multi<T, N...>& y)
{
    multi<
        std::decay_t<decltype(pr::fdim(
        std::declval<T>(),
        std::declval<T>()))>, N...> res;
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
inline auto fmod(
            const multi<T, N...>& x,
            const multi<T, N...>& y)
{
    multi<
        std::decay_t<decltype(pr::fmod(
        std::declval<T>(),
        std::declval<T>()))>, N...> res;
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
inline auto remquo(
            const multi<T, N...>& x,
            const multi<T, N...>& y,
            multi<int, N...>* q)
{
    multi<
        std::decay_t<decltype(pr::remquo(
        std::declval<T>(),
        std::declval<T>(),
        std::declval<int*>()))>, N...> res;
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
inline auto remainder(
            const multi<T, N...>& x,
            const multi<T, N...>& y)
{
    multi<
        std::decay_t<decltype(pr::remainder(
        std::declval<T>(),
        std::declval<T>()))>, N...> res;
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
inline auto nearbyint(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pr::nearbyint(
        std::declval<T>()))>, N...> res;
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
inline auto floor(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pr::floor(
        std::declval<T>()))>, N...> res;
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
inline auto ceil(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pr::ceil(
        std::declval<T>()))>, N...> res;
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
inline auto trunc(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pr::trunc(
        std::declval<T>()))>, N...> res;
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
inline auto round(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pr::round(
        std::declval<T>()))>, N...> res;
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
inline auto rint(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pr::rint(
        std::declval<T>()))>, N...> res;
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
inline auto lrint(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pr::lrint(
        std::declval<T>()))>, N...> res;
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
inline auto llrint(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pr::llrint(
        std::declval<T>()))>, N...> res;
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
inline auto lround(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pr::lround(
        std::declval<T>()))>, N...> res;
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
inline auto llround(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pr::llround(
        std::declval<T>()))>, N...> res;
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
inline auto frexp(
            const multi<T, N...>& x,
            multi<int, N...>* p)
{
    multi<
        std::decay_t<decltype(pr::frexp(
        std::declval<T>(),
        std::declval<int*>()))>, N...> res;
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
inline auto ldexp(
            const multi<T, N...>& x,
            const multi<int, N...>& p)
{
    multi<
        std::decay_t<decltype(pr::ldexp(
        std::declval<T>(),
        std::declval<int>()))>, N...> res;
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
inline auto logb(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pr::logb(
        std::declval<T>()))>, N...> res;
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
inline auto ilogb(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pr::ilogb(
        std::declval<T>()))>, N...> res;
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
inline auto scalbn(
            const multi<T, N...>& x,
            const multi<int, N...>& p)
{
    multi<
        std::decay_t<decltype(pr::scalbn(
        std::declval<T>(),
        std::declval<int>()))>, N...> res;
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
inline auto scalbln(
            const multi<T, N...>& x,
            const multi<long, N...>& p)
{
    multi<
        std::decay_t<decltype(pr::scalbln(
        std::declval<T>(),
        std::declval<long>()))>, N...> res;
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
inline auto modf(
            const multi<T, N...>& x,
            multi<T, N...>* p)
{
    multi<
        std::decay_t<decltype(pr::modf(
        std::declval<T>(),
        std::declval<T*>()))>, N...> res;
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
inline auto nextafter(
            const multi<T, N...>& x,
            const multi<T, N...>& y)
{
    multi<
        std::decay_t<decltype(pr::nextafter(
        std::declval<T>(),
        std::declval<T>()))>, N...> res;
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
inline auto nexttoward(
            const multi<T, N...>& x,
            const multi<long double, N...>& y)
{
    multi<
        std::decay_t<decltype(pr::nexttoward(
        std::declval<T>(),
        std::declval<long double>()))>, N...> res;
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
inline auto copysign(
            const multi<T, N...>& x,
            const multi<T, N...>& y)
{
    multi<
        std::decay_t<decltype(pr::copysign(
        std::declval<T>(),
        std::declval<T>()))>, N...> res;
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
inline auto signbit(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pr::signbit(
        std::declval<T>()))>, N...> res;
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
inline auto isnan(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pr::isnan(
        std::declval<T>()))>, N...> res;
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
inline auto isinf(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pr::isinf(
        std::declval<T>()))>, N...> res;
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
inline auto isfinite(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pr::isfinite(
        std::declval<T>()))>, N...> res;
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
inline auto isnormal(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pr::isnormal(
        std::declval<T>()))>, N...> res;
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
inline auto exp(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pr::exp(
        std::declval<T>()))>, N...> res;
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
inline auto log(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pr::log(
        std::declval<T>()))>, N...> res;
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
inline auto exp2(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pr::exp2(
        std::declval<T>()))>, N...> res;
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
inline auto log2(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pr::log2(
        std::declval<T>()))>, N...> res;
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
inline auto log10(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pr::log10(
        std::declval<T>()))>, N...> res;
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
inline auto expm1(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pr::expm1(
        std::declval<T>()))>, N...> res;
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
inline auto log1p(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pr::log1p(
        std::declval<T>()))>, N...> res;
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
inline auto pow(
            const multi<T, N...>& x,
            const multi<T, N...>& y)
{
    multi<
        std::decay_t<decltype(pr::pow(
        std::declval<T>(),
        std::declval<T>()))>, N...> res;
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
inline auto sqrt(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pr::sqrt(
        std::declval<T>()))>, N...> res;
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
inline auto cbrt(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pr::cbrt(
        std::declval<T>()))>, N...> res;
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
inline auto hypot(
            const multi<T, N...>& x,
            const multi<T, N...>& y)
{
    multi<
        std::decay_t<decltype(pr::hypot(
        std::declval<T>(),
        std::declval<T>()))>, N...> res;
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
inline auto erf(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pr::erf(
        std::declval<T>()))>, N...> res;
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
inline auto erfc(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pr::erfc(
        std::declval<T>()))>, N...> res;
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
inline auto lgamma(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pr::lgamma(
        std::declval<T>()))>, N...> res;
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
inline auto tgamma(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pr::tgamma(
        std::declval<T>()))>, N...> res;
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
inline auto sin(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pr::sin(
        std::declval<T>()))>, N...> res;
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
inline auto cos(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pr::cos(
        std::declval<T>()))>, N...> res;
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
inline auto tan(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pr::tan(
        std::declval<T>()))>, N...> res;
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
inline auto asin(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pr::asin(
        std::declval<T>()))>, N...> res;
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
inline auto acos(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pr::acos(
        std::declval<T>()))>, N...> res;
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
inline auto atan(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pr::atan(
        std::declval<T>()))>, N...> res;
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
inline auto atan2(
            const multi<T, N...>& y,
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pr::atan2(
        std::declval<T>(),
        std::declval<T>()))>, N...> res;
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
inline auto sinh(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pr::sinh(
        std::declval<T>()))>, N...> res;
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
inline auto cosh(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pr::cosh(
        std::declval<T>()))>, N...> res;
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
inline auto tanh(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pr::tanh(
        std::declval<T>()))>, N...> res;
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
inline auto asinh(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pr::asinh(
        std::declval<T>()))>, N...> res;
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
inline auto acosh(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pr::acosh(
        std::declval<T>()))>, N...> res;
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
inline auto atanh(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pr::atanh(
        std::declval<T>()))>, N...> res;
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
inline auto csc(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pr::csc(
        std::declval<T>()))>, N...> res;
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
inline auto sec(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pr::sec(
        std::declval<T>()))>, N...> res;
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
inline auto cot(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pr::cot(
        std::declval<T>()))>, N...> res;
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
inline auto csch(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pr::csch(
        std::declval<T>()))>, N...> res;
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
inline auto sech(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pr::sech(
        std::declval<T>()))>, N...> res;
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
inline auto coth(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pr::coth(
        std::declval<T>()))>, N...> res;
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
inline auto acsc(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pr::acsc(
        std::declval<T>()))>, N...> res;
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
inline auto asec(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pr::asec(
        std::declval<T>()))>, N...> res;
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
inline auto acot(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pr::acot(
        std::declval<T>()))>, N...> res;
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
inline auto acsch(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pr::acsch(
        std::declval<T>()))>, N...> res;
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
inline auto asech(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pr::asech(
        std::declval<T>()))>, N...> res;
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
inline auto acoth(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pr::acoth(
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pr::acoth(*itrx);
    }
    return res;
}

/**@}*/

/**@}*/

} // namespace pr

