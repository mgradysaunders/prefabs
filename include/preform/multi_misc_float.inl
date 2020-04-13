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
 * @addtogroup multi_misc_float
 */
/**@{*/

/**
 * @name Float helpers (multi)
 */
/**@{*/

/**
 * @brief Wrap `pre::finc()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto finc(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pre::finc(
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pre::finc(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pre::fdec()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto fdec(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pre::fdec(
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pre::fdec(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pre::fclamp()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto fclamp(
            const multi<T, N...>& x,
            const multi<T, N...>& a,
            const multi<T, N...>& b)
{
    multi<
        std::decay_t<decltype(pre::fclamp(
        std::declval<T>(),
        std::declval<T>(),
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itra = a.begin();
    auto itrb = b.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itra, ++itrb, ++itrres) {
        *itrres = pre::fclamp(*itrx, *itra, *itrb);
    }
    return res;
}

/**
 * @brief Wrap `pre::frepeat()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto frepeat(
            const multi<T, N...>& x,
            const multi<T, N...>& a,
            const multi<T, N...>& b)
{
    multi<
        std::decay_t<decltype(pre::frepeat(
        std::declval<T>(),
        std::declval<T>(),
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itra = a.begin();
    auto itrb = b.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itra, ++itrb, ++itrres) {
        *itrres = pre::frepeat(*itrx, *itra, *itrb);
    }
    return res;
}

/**
 * @brief Wrap `pre::fmirror()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto fmirror(
            const multi<T, N...>& x,
            const multi<T, N...>& a,
            const multi<T, N...>& b)
{
    multi<
        std::decay_t<decltype(pre::fmirror(
        std::declval<T>(),
        std::declval<T>(),
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itra = a.begin();
    auto itrb = b.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itra, ++itrb, ++itrres) {
        *itrres = pre::fmirror(*itrx, *itra, *itrb);
    }
    return res;
}

/**
 * @brief Wrap `pre::fastfloor()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto fastfloor(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pre::fastfloor(
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pre::fastfloor(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pre::fastceil()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto fastceil(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pre::fastceil(
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pre::fastceil(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pre::fastround()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto fastround(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pre::fastround(
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pre::fastround(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pre::fasttrunc()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto fasttrunc(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pre::fasttrunc(
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pre::fasttrunc(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pre::sinpi()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto sinpi(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pre::sinpi(
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pre::sinpi(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pre::cospi()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto cospi(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pre::cospi(
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pre::cospi(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pre::fstretch()`.
 */
template <typename U, typename T, std::size_t... N>
__attribute__((always_inline))
inline auto fstretch(const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pre::fstretch<U>(
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pre::fstretch<U>(*itrx);
    }
    return res;
}

/**@}*/

/**@}*/

} // namespace pre

