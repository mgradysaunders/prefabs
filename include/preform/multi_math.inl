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
 * @addtogroup multi_math
 */
/**@{*/

/**
 * @name Math wrappers (multi)
 */
/**@{*/

/**
 * @brief Wrap `pre::abs()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto abs(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pre::abs(
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pre::abs(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pre::arg()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto arg(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pre::arg(
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pre::arg(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pre::real()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto real(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pre::real(
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pre::real(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pre::imag()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto imag(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pre::imag(
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pre::imag(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pre::conj()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto conj(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pre::conj(
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pre::conj(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pre::norm()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto norm(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pre::norm(
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pre::norm(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pre::min()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto min(
            const multi<T, N...>& x,
            const multi<T, N...>& y)
{
    multi<
        std::decay_t<decltype(pre::min(
        std::declval<T>(),
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itry = y.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itry, ++itrres) {
        *itrres = pre::min(*itrx, *itry);
    }
    return res;
}

/**
 * @brief Wrap `pre::max()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto max(
            const multi<T, N...>& x,
            const multi<T, N...>& y)
{
    multi<
        std::decay_t<decltype(pre::max(
        std::declval<T>(),
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itry = y.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itry, ++itrres) {
        *itrres = pre::max(*itrx, *itry);
    }
    return res;
}

/**
 * @brief Wrap `pre::fabs()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto fabs(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pre::fabs(
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pre::fabs(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pre::fma()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto fma(
            const multi<T, N...>& x,
            const multi<T, N...>& y,
            const multi<T, N...>& z)
{
    multi<
        std::decay_t<decltype(pre::fma(
        std::declval<T>(),
        std::declval<T>(),
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itry = y.begin();
    auto itrz = z.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itry, ++itrz, ++itrres) {
        *itrres = pre::fma(*itrx, *itry, *itrz);
    }
    return res;
}

/**
 * @brief Wrap `pre::fmin()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto fmin(
            const multi<T, N...>& x,
            const multi<T, N...>& y)
{
    multi<
        std::decay_t<decltype(pre::fmin(
        std::declval<T>(),
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itry = y.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itry, ++itrres) {
        *itrres = pre::fmin(*itrx, *itry);
    }
    return res;
}

/**
 * @brief Wrap `pre::fmax()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto fmax(
            const multi<T, N...>& x,
            const multi<T, N...>& y)
{
    multi<
        std::decay_t<decltype(pre::fmax(
        std::declval<T>(),
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itry = y.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itry, ++itrres) {
        *itrres = pre::fmax(*itrx, *itry);
    }
    return res;
}

/**
 * @brief Wrap `pre::fdim()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto fdim(
            const multi<T, N...>& x,
            const multi<T, N...>& y)
{
    multi<
        std::decay_t<decltype(pre::fdim(
        std::declval<T>(),
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itry = y.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itry, ++itrres) {
        *itrres = pre::fdim(*itrx, *itry);
    }
    return res;
}

/**
 * @brief Wrap `pre::fmod()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto fmod(
            const multi<T, N...>& x,
            const multi<T, N...>& y)
{
    multi<
        std::decay_t<decltype(pre::fmod(
        std::declval<T>(),
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itry = y.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itry, ++itrres) {
        *itrres = pre::fmod(*itrx, *itry);
    }
    return res;
}

/**
 * @brief Wrap `pre::remquo()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto remquo(
            const multi<T, N...>& x,
            const multi<T, N...>& y,
            multi<int, N...>* q)
{
    multi<
        std::decay_t<decltype(pre::remquo(
        std::declval<T>(),
        std::declval<T>(),
        std::declval<int*>()))>, N...> res;
    auto itrx = x.begin();
    auto itry = y.begin();
    auto itrq = q->begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itry, ++itrq, ++itrres) {
        *itrres = pre::remquo(*itrx, *itry, &(*itrq));
    }
    return res;
}

/**
 * @brief Wrap `pre::remainder()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto remainder(
            const multi<T, N...>& x,
            const multi<T, N...>& y)
{
    multi<
        std::decay_t<decltype(pre::remainder(
        std::declval<T>(),
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itry = y.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itry, ++itrres) {
        *itrres = pre::remainder(*itrx, *itry);
    }
    return res;
}

/**
 * @brief Wrap `pre::nearbyint()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto nearbyint(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pre::nearbyint(
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pre::nearbyint(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pre::floor()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto floor(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pre::floor(
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pre::floor(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pre::ceil()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto ceil(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pre::ceil(
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pre::ceil(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pre::trunc()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto trunc(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pre::trunc(
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pre::trunc(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pre::round()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto round(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pre::round(
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pre::round(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pre::rint()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto rint(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pre::rint(
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pre::rint(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pre::lrint()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto lrint(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pre::lrint(
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pre::lrint(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pre::llrint()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto llrint(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pre::llrint(
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pre::llrint(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pre::lround()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto lround(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pre::lround(
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pre::lround(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pre::llround()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto llround(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pre::llround(
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pre::llround(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pre::frexp()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto frexp(
            const multi<T, N...>& x,
            multi<int, N...>* p)
{
    multi<
        std::decay_t<decltype(pre::frexp(
        std::declval<T>(),
        std::declval<int*>()))>, N...> res;
    auto itrx = x.begin();
    auto itrp = p->begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrp, ++itrres) {
        *itrres = pre::frexp(*itrx, &(*itrp));
    }
    return res;
}

/**
 * @brief Wrap `pre::ldexp()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto ldexp(
            const multi<T, N...>& x,
            const multi<int, N...>& p)
{
    multi<
        std::decay_t<decltype(pre::ldexp(
        std::declval<T>(),
        std::declval<int>()))>, N...> res;
    auto itrx = x.begin();
    auto itrp = p.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrp, ++itrres) {
        *itrres = pre::ldexp(*itrx, *itrp);
    }
    return res;
}

/**
 * @brief Wrap `pre::logb()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto logb(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pre::logb(
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pre::logb(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pre::ilogb()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto ilogb(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pre::ilogb(
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pre::ilogb(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pre::scalbn()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto scalbn(
            const multi<T, N...>& x,
            const multi<int, N...>& p)
{
    multi<
        std::decay_t<decltype(pre::scalbn(
        std::declval<T>(),
        std::declval<int>()))>, N...> res;
    auto itrx = x.begin();
    auto itrp = p.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrp, ++itrres) {
        *itrres = pre::scalbn(*itrx, *itrp);
    }
    return res;
}

/**
 * @brief Wrap `pre::scalbln()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto scalbln(
            const multi<T, N...>& x,
            const multi<long, N...>& p)
{
    multi<
        std::decay_t<decltype(pre::scalbln(
        std::declval<T>(),
        std::declval<long>()))>, N...> res;
    auto itrx = x.begin();
    auto itrp = p.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrp, ++itrres) {
        *itrres = pre::scalbln(*itrx, *itrp);
    }
    return res;
}

/**
 * @brief Wrap `pre::modf()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto modf(
            const multi<T, N...>& x,
            multi<T, N...>* p)
{
    multi<
        std::decay_t<decltype(pre::modf(
        std::declval<T>(),
        std::declval<T*>()))>, N...> res;
    auto itrx = x.begin();
    auto itrp = p->begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrp, ++itrres) {
        *itrres = pre::modf(*itrx, &(*itrp));
    }
    return res;
}

/**
 * @brief Wrap `pre::nextafter()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto nextafter(
            const multi<T, N...>& x,
            const multi<T, N...>& y)
{
    multi<
        std::decay_t<decltype(pre::nextafter(
        std::declval<T>(),
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itry = y.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itry, ++itrres) {
        *itrres = pre::nextafter(*itrx, *itry);
    }
    return res;
}

/**
 * @brief Wrap `pre::nexttoward()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto nexttoward(
            const multi<T, N...>& x,
            const multi<long double, N...>& y)
{
    multi<
        std::decay_t<decltype(pre::nexttoward(
        std::declval<T>(),
        std::declval<long double>()))>, N...> res;
    auto itrx = x.begin();
    auto itry = y.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itry, ++itrres) {
        *itrres = pre::nexttoward(*itrx, *itry);
    }
    return res;
}

/**
 * @brief Wrap `pre::copysign()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto copysign(
            const multi<T, N...>& x,
            const multi<T, N...>& y)
{
    multi<
        std::decay_t<decltype(pre::copysign(
        std::declval<T>(),
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itry = y.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itry, ++itrres) {
        *itrres = pre::copysign(*itrx, *itry);
    }
    return res;
}

/**
 * @brief Wrap `pre::signbit()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto signbit(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pre::signbit(
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pre::signbit(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pre::isnan()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto isnan(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pre::isnan(
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pre::isnan(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pre::isinf()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto isinf(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pre::isinf(
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pre::isinf(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pre::isfinite()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto isfinite(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pre::isfinite(
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pre::isfinite(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pre::isnormal()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto isnormal(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pre::isnormal(
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pre::isnormal(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pre::exp()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto exp(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pre::exp(
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pre::exp(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pre::log()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto log(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pre::log(
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pre::log(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pre::exp2()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto exp2(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pre::exp2(
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pre::exp2(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pre::log2()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto log2(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pre::log2(
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pre::log2(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pre::log10()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto log10(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pre::log10(
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pre::log10(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pre::expm1()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto expm1(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pre::expm1(
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pre::expm1(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pre::log1p()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto log1p(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pre::log1p(
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pre::log1p(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pre::pow()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto pow(
            const multi<T, N...>& x,
            const multi<T, N...>& y)
{
    multi<
        std::decay_t<decltype(pre::pow(
        std::declval<T>(),
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itry = y.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itry, ++itrres) {
        *itrres = pre::pow(*itrx, *itry);
    }
    return res;
}

/**
 * @brief Wrap `pre::sqrt()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto sqrt(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pre::sqrt(
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pre::sqrt(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pre::cbrt()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto cbrt(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pre::cbrt(
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pre::cbrt(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pre::hypot()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto hypot(
            const multi<T, N...>& x,
            const multi<T, N...>& y)
{
    multi<
        std::decay_t<decltype(pre::hypot(
        std::declval<T>(),
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itry = y.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itry, ++itrres) {
        *itrres = pre::hypot(*itrx, *itry);
    }
    return res;
}

/**
 * @brief Wrap `pre::erf()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto erf(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pre::erf(
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pre::erf(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pre::erfc()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto erfc(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pre::erfc(
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pre::erfc(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pre::lgamma()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto lgamma(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pre::lgamma(
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pre::lgamma(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pre::tgamma()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto tgamma(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pre::tgamma(
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pre::tgamma(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pre::sin()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto sin(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pre::sin(
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pre::sin(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pre::cos()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto cos(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pre::cos(
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pre::cos(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pre::tan()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto tan(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pre::tan(
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pre::tan(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pre::asin()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto asin(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pre::asin(
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pre::asin(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pre::acos()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto acos(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pre::acos(
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pre::acos(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pre::atan()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto atan(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pre::atan(
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pre::atan(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pre::atan2()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto atan2(
            const multi<T, N...>& y,
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pre::atan2(
        std::declval<T>(),
        std::declval<T>()))>, N...> res;
    auto itry = y.begin();
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itry, ++itrx, ++itrres) {
        *itrres = pre::atan2(*itry, *itrx);
    }
    return res;
}

/**
 * @brief Wrap `pre::sinh()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto sinh(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pre::sinh(
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pre::sinh(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pre::cosh()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto cosh(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pre::cosh(
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pre::cosh(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pre::tanh()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto tanh(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pre::tanh(
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pre::tanh(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pre::asinh()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto asinh(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pre::asinh(
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pre::asinh(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pre::acosh()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto acosh(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pre::acosh(
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pre::acosh(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pre::atanh()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto atanh(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pre::atanh(
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pre::atanh(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pre::csc()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto csc(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pre::csc(
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pre::csc(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pre::sec()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto sec(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pre::sec(
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pre::sec(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pre::cot()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto cot(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pre::cot(
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pre::cot(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pre::csch()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto csch(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pre::csch(
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pre::csch(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pre::sech()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto sech(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pre::sech(
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pre::sech(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pre::coth()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto coth(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pre::coth(
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pre::coth(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pre::acsc()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto acsc(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pre::acsc(
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pre::acsc(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pre::asec()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto asec(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pre::asec(
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pre::asec(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pre::acot()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto acot(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pre::acot(
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pre::acot(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pre::acsch()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto acsch(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pre::acsch(
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pre::acsch(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pre::asech()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto asech(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pre::asech(
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pre::asech(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pre::acoth()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto acoth(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pre::acoth(
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pre::acoth(*itrx);
    }
    return res;
}

/**@}*/

/**@}*/

} // namespace pre

