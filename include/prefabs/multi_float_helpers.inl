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
inline multi<std::decay_t<decltype(pr::finc(T()))>, N...> finc(const multi<T, N...>& x)
{
    multi<std::decay_t<decltype(pr::finc(T()))>, N...> res;
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
inline multi<std::decay_t<decltype(pr::fdec(T()))>, N...> fdec(const multi<T, N...>& x)
{
    multi<std::decay_t<decltype(pr::fdec(T()))>, N...> res;
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
inline multi<std::decay_t<decltype(pr::fclamp(T(), T(), T()))>, N...> fclamp(const multi<T, N...>& x, const multi<T, N...>& a, const multi<T, N...>& b)
{
    multi<std::decay_t<decltype(pr::fclamp(T(), T(), T()))>, N...> res;
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
inline multi<std::decay_t<decltype(pr::frepeat(T(), T(), T()))>, N...> frepeat(const multi<T, N...>& x, const multi<T, N...>& a, const multi<T, N...>& b)
{
    multi<std::decay_t<decltype(pr::frepeat(T(), T(), T()))>, N...> res;
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
inline multi<std::decay_t<decltype(pr::fmirror(T(), T(), T()))>, N...> fmirror(const multi<T, N...>& x, const multi<T, N...>& a, const multi<T, N...>& b)
{
    multi<std::decay_t<decltype(pr::fmirror(T(), T(), T()))>, N...> res;
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
 * @brief Wrap `pr::fcycle()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline multi<std::decay_t<decltype(pr::fcycle(T(), T(), T(), std::declval<cycle_mode>()))>, N...> fcycle(const multi<T, N...>& x, const multi<T, N...>& a, const multi<T, N...>& b, const multi<cycle_mode, N...>& mode)
{
    multi<std::decay_t<decltype(pr::fcycle(T(), T(), T(), std::declval<cycle_mode>()))>, N...> res;
    auto itrx = x.begin();
    auto itra = a.begin();
    auto itrb = b.begin();
    auto itrmode = mode.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itra, ++itrb, ++itrmode, ++itrres) {
        *itrres = pr::fcycle(*itrx, *itra, *itrb, *itrmode);
    }
    return res;
}

/**
 * @brief Wrap `pr::fstretch()`.
 */
template <typename U, typename T, std::size_t... N>
__attribute__((always_inline))
inline multi<std::decay_t<decltype(pr::fstretch<U>(T()))>, N...> fstretch(const multi<T, N...>& x)
{
    multi<std::decay_t<decltype(pr::fstretch<U>(T()))>, N...> res;
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

