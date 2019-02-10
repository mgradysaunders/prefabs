namespace pr {

/**
 * @addtogroup multi_float_helpers
 */
/**@{*/

/**
 * @name Float helpers (multi)
 */
/**@{*/

/**
 * @brief Wrap `pr::finc()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto finc(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pr::finc(
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pr::finc(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pr::fdec()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto fdec(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pr::fdec(
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pr::fdec(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pr::fclamp()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto fclamp(
            const multi<T, N...>& x,
            const multi<T, N...>& a,
            const multi<T, N...>& b)
{
    multi<
        std::decay_t<decltype(pr::fclamp(
        std::declval<T>(),
        std::declval<T>(),
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itra = a.begin();
    auto itrb = b.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itra, ++itrb, ++itrres) {
        *itrres = pr::fclamp(*itrx, *itra, *itrb);
    }
    return res;
}

/**
 * @brief Wrap `pr::frepeat()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto frepeat(
            const multi<T, N...>& x,
            const multi<T, N...>& a,
            const multi<T, N...>& b)
{
    multi<
        std::decay_t<decltype(pr::frepeat(
        std::declval<T>(),
        std::declval<T>(),
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itra = a.begin();
    auto itrb = b.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itra, ++itrb, ++itrres) {
        *itrres = pr::frepeat(*itrx, *itra, *itrb);
    }
    return res;
}

/**
 * @brief Wrap `pr::fmirror()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto fmirror(
            const multi<T, N...>& x,
            const multi<T, N...>& a,
            const multi<T, N...>& b)
{
    multi<
        std::decay_t<decltype(pr::fmirror(
        std::declval<T>(),
        std::declval<T>(),
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itra = a.begin();
    auto itrb = b.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itra, ++itrb, ++itrres) {
        *itrres = pr::fmirror(*itrx, *itra, *itrb);
    }
    return res;
}

/**
 * @brief Wrap `pr::fastfloor()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto fastfloor(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pr::fastfloor(
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pr::fastfloor(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pr::fastceil()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto fastceil(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pr::fastceil(
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pr::fastceil(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pr::fastround()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto fastround(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pr::fastround(
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pr::fastround(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pr::fasttrunc()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline auto fasttrunc(
            const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pr::fasttrunc(
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pr::fasttrunc(*itrx);
    }
    return res;
}

/**
 * @brief Wrap `pr::fstretch()`.
 */
template <typename U, typename T, std::size_t... N>
__attribute__((always_inline))
inline auto fstretch(const multi<T, N...>& x)
{
    multi<
        std::decay_t<decltype(pr::fstretch<U>(
        std::declval<T>()))>, N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pr::fstretch<U>(*itrx);
    }
    return res;
}

/**@}*/

/**@}*/

} // namespace pr

