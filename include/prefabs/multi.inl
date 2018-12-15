/**
 * @addtogroup multi
 */
/**@{*/

/**
 * @name Unary operators (multi)
 */
/**@{*/

/**
 * @brief Distribute `operator+`.
 */
template <typename T, std::size_t... N>
constexpr multi<decltype(+T()), N...> 
                operator+(const multi<T, N...>& arr)
{
    multi<decltype(+T()), N...> res;
    auto itrarr = arr.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrarr, ++itrres) {
        *itrres = +*itrarr;
    }
    return res;
}

/**
 * @brief Distribute `operator-`.
 */
template <typename T, std::size_t... N>
constexpr multi<decltype(-T()), N...> 
                operator-(const multi<T, N...>& arr)
{
    multi<decltype(-T()), N...> res;
    auto itrarr = arr.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrarr, ++itrres) {
        *itrres = -*itrarr;
    }
    return res;
}

/**
 * @brief Distribute `operator~`.
 */
template <typename T, std::size_t... N>
constexpr multi<decltype(~T()), N...> 
                operator~(const multi<T, N...>& arr)
{
    multi<decltype(~T()), N...> res;
    auto itrarr = arr.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrarr, ++itrres) {
        *itrres = ~*itrarr;
    }
    return res;
}

/**
 * @brief Distribute `operator!`.
 */
template <typename T, std::size_t... N>
constexpr multi<decltype(!T()), N...> 
                operator!(const multi<T, N...>& arr)
{
    multi<decltype(!T()), N...> res;
    auto itrarr = arr.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrarr, ++itrres) {
        *itrres = !*itrarr;
    }
    return res;
}

/**@}*/

/**
 * @name Binary operators (multi/multi)
 */
/**@{*/

/**
 * @brief Distribute `operator+`.
 */
template <typename T, typename U, std::size_t... N>
constexpr multi<decltype(T() + U()), N...> 
                operator+(
                    const multi<T, N...>& arr0, 
                    const multi<U, N...>& arr1)
{
    multi<decltype(T() + U()), N...> res;
    auto itrarr0 = arr0.begin();
    auto itrarr1 = arr1.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrarr0, ++itrarr1, ++itrres) {
        *itrres = *itrarr0 + *itrarr1;
    }
    return res;
}

/**
 * @brief Distribute `operator-`.
 */
template <typename T, typename U, std::size_t... N>
constexpr multi<decltype(T() - U()), N...> 
                operator-(
                    const multi<T, N...>& arr0, 
                    const multi<U, N...>& arr1)
{
    multi<decltype(T() - U()), N...> res;
    auto itrarr0 = arr0.begin();
    auto itrarr1 = arr1.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrarr0, ++itrarr1, ++itrres) {
        *itrres = *itrarr0 - *itrarr1;
    }
    return res;
}

/**
 * @brief Distribute `operator*`.
 */
template <typename T, typename U, std::size_t... N>
constexpr multi<decltype(T() * U()), N...> 
                operator*(
                    const multi<T, N...>& arr0, 
                    const multi<U, N...>& arr1)
{
    multi<decltype(T() * U()), N...> res;
    auto itrarr0 = arr0.begin();
    auto itrarr1 = arr1.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrarr0, ++itrarr1, ++itrres) {
        *itrres = *itrarr0 * *itrarr1;
    }
    return res;
}

/**
 * @brief Distribute `operator/`.
 */
template <typename T, typename U, std::size_t... N>
constexpr multi<decltype(T() / U()), N...> 
                operator/(
                    const multi<T, N...>& arr0, 
                    const multi<U, N...>& arr1)
{
    multi<decltype(T() / U()), N...> res;
    auto itrarr0 = arr0.begin();
    auto itrarr1 = arr1.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrarr0, ++itrarr1, ++itrres) {
        *itrres = *itrarr0 / *itrarr1;
    }
    return res;
}

/**
 * @brief Distribute `operator%`.
 */
template <typename T, typename U, std::size_t... N>
constexpr multi<decltype(T() % U()), N...> 
                operator%(
                    const multi<T, N...>& arr0, 
                    const multi<U, N...>& arr1)
{
    multi<decltype(T() % U()), N...> res;
    auto itrarr0 = arr0.begin();
    auto itrarr1 = arr1.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrarr0, ++itrarr1, ++itrres) {
        *itrres = *itrarr0 % *itrarr1;
    }
    return res;
}

/**
 * @brief Distribute `operator&`.
 */
template <typename T, typename U, std::size_t... N>
constexpr multi<decltype(T() & U()), N...> 
                operator&(
                    const multi<T, N...>& arr0, 
                    const multi<U, N...>& arr1)
{
    multi<decltype(T() & U()), N...> res;
    auto itrarr0 = arr0.begin();
    auto itrarr1 = arr1.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrarr0, ++itrarr1, ++itrres) {
        *itrres = *itrarr0 & *itrarr1;
    }
    return res;
}

/**
 * @brief Distribute `operator|`.
 */
template <typename T, typename U, std::size_t... N>
constexpr multi<decltype(T() | U()), N...> 
                operator|(
                    const multi<T, N...>& arr0, 
                    const multi<U, N...>& arr1)
{
    multi<decltype(T() | U()), N...> res;
    auto itrarr0 = arr0.begin();
    auto itrarr1 = arr1.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrarr0, ++itrarr1, ++itrres) {
        *itrres = *itrarr0 | *itrarr1;
    }
    return res;
}

/**
 * @brief Distribute `operator^`.
 */
template <typename T, typename U, std::size_t... N>
constexpr multi<decltype(T() ^ U()), N...> 
                operator^(
                    const multi<T, N...>& arr0, 
                    const multi<U, N...>& arr1)
{
    multi<decltype(T() ^ U()), N...> res;
    auto itrarr0 = arr0.begin();
    auto itrarr1 = arr1.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrarr0, ++itrarr1, ++itrres) {
        *itrres = *itrarr0 ^ *itrarr1;
    }
    return res;
}

/**
 * @brief Distribute `operator>>`.
 */
template <typename T, typename U, std::size_t... N>
constexpr multi<decltype(T() >> U()), N...> 
                operator>>(
                    const multi<T, N...>& arr0, 
                    const multi<U, N...>& arr1)
{
    multi<decltype(T() >> U()), N...> res;
    auto itrarr0 = arr0.begin();
    auto itrarr1 = arr1.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrarr0, ++itrarr1, ++itrres) {
        *itrres = *itrarr0 >> *itrarr1;
    }
    return res;
}

/**
 * @brief Distribute `operator<<`.
 */
template <typename T, typename U, std::size_t... N>
constexpr multi<decltype(T() << U()), N...> 
                operator<<(
                    const multi<T, N...>& arr0, 
                    const multi<U, N...>& arr1)
{
    multi<decltype(T() << U()), N...> res;
    auto itrarr0 = arr0.begin();
    auto itrarr1 = arr1.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrarr0, ++itrarr1, ++itrres) {
        *itrres = *itrarr0 << *itrarr1;
    }
    return res;
}

/**
 * @brief Distribute `operator+=`.
 */
template <typename T, typename U, std::size_t... N>
constexpr multi<T, N...>& operator+=(
                    multi<T, N...>& arr, const multi<U, N...>& oth)
{
    auto itrarr = arr.begin();
    auto itroth = oth.begin();
    for (; itroth < oth.end(); ++itrarr, ++itroth) {
        *itrarr += *itroth;
    }
    return arr;
}

/**
 * @brief Distribute `operator-=`.
 */
template <typename T, typename U, std::size_t... N>
constexpr multi<T, N...>& operator-=(
                    multi<T, N...>& arr, const multi<U, N...>& oth)
{
    auto itrarr = arr.begin();
    auto itroth = oth.begin();
    for (; itroth < oth.end(); ++itrarr, ++itroth) {
        *itrarr -= *itroth;
    }
    return arr;
}

/**
 * @brief Distribute `operator*=`.
 */
template <typename T, typename U, std::size_t... N>
constexpr multi<T, N...>& operator*=(
                    multi<T, N...>& arr, const multi<U, N...>& oth)
{
    auto itrarr = arr.begin();
    auto itroth = oth.begin();
    for (; itroth < oth.end(); ++itrarr, ++itroth) {
        *itrarr *= *itroth;
    }
    return arr;
}

/**
 * @brief Distribute `operator/=`.
 */
template <typename T, typename U, std::size_t... N>
constexpr multi<T, N...>& operator/=(
                    multi<T, N...>& arr, const multi<U, N...>& oth)
{
    auto itrarr = arr.begin();
    auto itroth = oth.begin();
    for (; itroth < oth.end(); ++itrarr, ++itroth) {
        *itrarr /= *itroth;
    }
    return arr;
}

/**
 * @brief Distribute `operator%=`.
 */
template <typename T, typename U, std::size_t... N>
constexpr multi<T, N...>& operator%=(
                    multi<T, N...>& arr, const multi<U, N...>& oth)
{
    auto itrarr = arr.begin();
    auto itroth = oth.begin();
    for (; itroth < oth.end(); ++itrarr, ++itroth) {
        *itrarr %= *itroth;
    }
    return arr;
}

/**
 * @brief Distribute `operator&=`.
 */
template <typename T, typename U, std::size_t... N>
constexpr multi<T, N...>& operator&=(
                    multi<T, N...>& arr, const multi<U, N...>& oth)
{
    auto itrarr = arr.begin();
    auto itroth = oth.begin();
    for (; itroth < oth.end(); ++itrarr, ++itroth) {
        *itrarr &= *itroth;
    }
    return arr;
}

/**
 * @brief Distribute `operator|=`.
 */
template <typename T, typename U, std::size_t... N>
constexpr multi<T, N...>& operator|=(
                    multi<T, N...>& arr, const multi<U, N...>& oth)
{
    auto itrarr = arr.begin();
    auto itroth = oth.begin();
    for (; itroth < oth.end(); ++itrarr, ++itroth) {
        *itrarr |= *itroth;
    }
    return arr;
}

/**
 * @brief Distribute `operator^=`.
 */
template <typename T, typename U, std::size_t... N>
constexpr multi<T, N...>& operator^=(
                    multi<T, N...>& arr, const multi<U, N...>& oth)
{
    auto itrarr = arr.begin();
    auto itroth = oth.begin();
    for (; itroth < oth.end(); ++itrarr, ++itroth) {
        *itrarr ^= *itroth;
    }
    return arr;
}

/**
 * @brief Distribute `operator>>=`.
 */
template <typename T, typename U, std::size_t... N>
constexpr multi<T, N...>& operator>>=(
                    multi<T, N...>& arr, const multi<U, N...>& oth)
{
    auto itrarr = arr.begin();
    auto itroth = oth.begin();
    for (; itroth < oth.end(); ++itrarr, ++itroth) {
        *itrarr >>= *itroth;
    }
    return arr;
}

/**
 * @brief Distribute `operator<<=`.
 */
template <typename T, typename U, std::size_t... N>
constexpr multi<T, N...>& operator<<=(
                    multi<T, N...>& arr, const multi<U, N...>& oth)
{
    auto itrarr = arr.begin();
    auto itroth = oth.begin();
    for (; itroth < oth.end(); ++itrarr, ++itroth) {
        *itrarr <<= *itroth;
    }
    return arr;
}

/**@}*/

/**
 * @name Binary operators (multi/entry)
 */
/**@{*/

/**
 * @brief Distribute `operator+`.
 */
template <typename T, typename U, std::size_t... N>
constexpr std::enable_if_t<!is_multi<U>::value, 
    multi<decltype(T() + U()), N...>> operator+(
                        const multi<T, N...>& arr, const U& val) 
{
    multi<decltype(T() + U()), N...> res;
    auto itrarr = arr.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrarr, ++itrres) {
        *itrres = *itrarr + val;
    }
    return res;
}

/**
 * @brief Distribute `operator-`.
 */
template <typename T, typename U, std::size_t... N>
constexpr std::enable_if_t<!is_multi<U>::value, 
    multi<decltype(T() - U()), N...>> operator-(
                        const multi<T, N...>& arr, const U& val) 
{
    multi<decltype(T() - U()), N...> res;
    auto itrarr = arr.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrarr, ++itrres) {
        *itrres = *itrarr - val;
    }
    return res;
}

/**
 * @brief Distribute `operator*`.
 */
template <typename T, typename U, std::size_t... N>
constexpr std::enable_if_t<!is_multi<U>::value, 
    multi<decltype(T() * U()), N...>> operator*(
                        const multi<T, N...>& arr, const U& val) 
{
    multi<decltype(T() * U()), N...> res;
    auto itrarr = arr.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrarr, ++itrres) {
        *itrres = *itrarr * val;
    }
    return res;
}

/**
 * @brief Distribute `operator/`.
 */
template <typename T, typename U, std::size_t... N>
constexpr std::enable_if_t<!is_multi<U>::value, 
    multi<decltype(T() / U()), N...>> operator/(
                        const multi<T, N...>& arr, const U& val) 
{
    multi<decltype(T() / U()), N...> res;
    auto itrarr = arr.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrarr, ++itrres) {
        *itrres = *itrarr / val;
    }
    return res;
}

/**
 * @brief Distribute `operator%`.
 */
template <typename T, typename U, std::size_t... N>
constexpr std::enable_if_t<!is_multi<U>::value, 
    multi<decltype(T() % U()), N...>> operator%(
                        const multi<T, N...>& arr, const U& val) 
{
    multi<decltype(T() % U()), N...> res;
    auto itrarr = arr.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrarr, ++itrres) {
        *itrres = *itrarr % val;
    }
    return res;
}

/**
 * @brief Distribute `operator&`.
 */
template <typename T, typename U, std::size_t... N>
constexpr std::enable_if_t<!is_multi<U>::value, 
    multi<decltype(T() & U()), N...>> operator&(
                        const multi<T, N...>& arr, const U& val) 
{
    multi<decltype(T() & U()), N...> res;
    auto itrarr = arr.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrarr, ++itrres) {
        *itrres = *itrarr & val;
    }
    return res;
}

/**
 * @brief Distribute `operator|`.
 */
template <typename T, typename U, std::size_t... N>
constexpr std::enable_if_t<!is_multi<U>::value, 
    multi<decltype(T() | U()), N...>> operator|(
                        const multi<T, N...>& arr, const U& val) 
{
    multi<decltype(T() | U()), N...> res;
    auto itrarr = arr.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrarr, ++itrres) {
        *itrres = *itrarr | val;
    }
    return res;
}

/**
 * @brief Distribute `operator^`.
 */
template <typename T, typename U, std::size_t... N>
constexpr std::enable_if_t<!is_multi<U>::value, 
    multi<decltype(T() ^ U()), N...>> operator^(
                        const multi<T, N...>& arr, const U& val) 
{
    multi<decltype(T() ^ U()), N...> res;
    auto itrarr = arr.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrarr, ++itrres) {
        *itrres = *itrarr ^ val;
    }
    return res;
}

/**
 * @brief Distribute `operator>>`.
 */
template <typename T, typename U, std::size_t... N>
constexpr std::enable_if_t<!is_multi<U>::value, 
    multi<decltype(T() >> U()), N...>> operator>>(
                        const multi<T, N...>& arr, const U& val) 
{
    multi<decltype(T() >> U()), N...> res;
    auto itrarr = arr.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrarr, ++itrres) {
        *itrres = *itrarr >> val;
    }
    return res;
}

/**
 * @brief Distribute `operator<<`.
 */
template <typename T, typename U, std::size_t... N>
constexpr std::enable_if_t<!is_multi<U>::value, 
    multi<decltype(T() << U()), N...>> operator<<(
                        const multi<T, N...>& arr, const U& val) 
{
    multi<decltype(T() << U()), N...> res;
    auto itrarr = arr.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrarr, ++itrres) {
        *itrres = *itrarr << val;
    }
    return res;
}

/**
 * @brief Distribute `operator+=`.
 */
template <typename T, typename U, std::size_t... N>
constexpr std::enable_if_t<!is_multi<U>::value, 
    multi<T, N...>&> operator+=(multi<T, N...>& arr, const U& val)
{
    for (auto itrarr = arr.begin(); 
              itrarr < arr.end(); ++itrarr) {
        *itrarr += val;
    }
    return arr;
}

/**
 * @brief Distribute `operator-=`.
 */
template <typename T, typename U, std::size_t... N>
constexpr std::enable_if_t<!is_multi<U>::value, 
    multi<T, N...>&> operator-=(multi<T, N...>& arr, const U& val)
{
    for (auto itrarr = arr.begin(); 
              itrarr < arr.end(); ++itrarr) {
        *itrarr -= val;
    }
    return arr;
}

/**
 * @brief Distribute `operator*=`.
 */
template <typename T, typename U, std::size_t... N>
constexpr std::enable_if_t<!is_multi<U>::value, 
    multi<T, N...>&> operator*=(multi<T, N...>& arr, const U& val)
{
    for (auto itrarr = arr.begin(); 
              itrarr < arr.end(); ++itrarr) {
        *itrarr *= val;
    }
    return arr;
}

/**
 * @brief Distribute `operator/=`.
 */
template <typename T, typename U, std::size_t... N>
constexpr std::enable_if_t<!is_multi<U>::value, 
    multi<T, N...>&> operator/=(multi<T, N...>& arr, const U& val)
{
    for (auto itrarr = arr.begin(); 
              itrarr < arr.end(); ++itrarr) {
        *itrarr /= val;
    }
    return arr;
}

/**
 * @brief Distribute `operator%=`.
 */
template <typename T, typename U, std::size_t... N>
constexpr std::enable_if_t<!is_multi<U>::value, 
    multi<T, N...>&> operator%=(multi<T, N...>& arr, const U& val)
{
    for (auto itrarr = arr.begin(); 
              itrarr < arr.end(); ++itrarr) {
        *itrarr %= val;
    }
    return arr;
}

/**
 * @brief Distribute `operator&=`.
 */
template <typename T, typename U, std::size_t... N>
constexpr std::enable_if_t<!is_multi<U>::value, 
    multi<T, N...>&> operator&=(multi<T, N...>& arr, const U& val)
{
    for (auto itrarr = arr.begin(); 
              itrarr < arr.end(); ++itrarr) {
        *itrarr &= val;
    }
    return arr;
}

/**
 * @brief Distribute `operator|=`.
 */
template <typename T, typename U, std::size_t... N>
constexpr std::enable_if_t<!is_multi<U>::value, 
    multi<T, N...>&> operator|=(multi<T, N...>& arr, const U& val)
{
    for (auto itrarr = arr.begin(); 
              itrarr < arr.end(); ++itrarr) {
        *itrarr |= val;
    }
    return arr;
}

/**
 * @brief Distribute `operator^=`.
 */
template <typename T, typename U, std::size_t... N>
constexpr std::enable_if_t<!is_multi<U>::value, 
    multi<T, N...>&> operator^=(multi<T, N...>& arr, const U& val)
{
    for (auto itrarr = arr.begin(); 
              itrarr < arr.end(); ++itrarr) {
        *itrarr ^= val;
    }
    return arr;
}

/**
 * @brief Distribute `operator>>=`.
 */
template <typename T, typename U, std::size_t... N>
constexpr std::enable_if_t<!is_multi<U>::value, 
    multi<T, N...>&> operator>>=(multi<T, N...>& arr, const U& val)
{
    for (auto itrarr = arr.begin(); 
              itrarr < arr.end(); ++itrarr) {
        *itrarr >>= val;
    }
    return arr;
}

/**
 * @brief Distribute `operator<<=`.
 */
template <typename T, typename U, std::size_t... N>
constexpr std::enable_if_t<!is_multi<U>::value, 
    multi<T, N...>&> operator<<=(multi<T, N...>& arr, const U& val)
{
    for (auto itrarr = arr.begin(); 
              itrarr < arr.end(); ++itrarr) {
        *itrarr <<= val;
    }
    return arr;
}

/**@}*/

/**
 * @name Binary operators (entry/multi)
 */
/**@{*/

/**
 * @brief Distribute `operator+`.
 */
template <typename T, typename U, std::size_t... N>
constexpr std::enable_if_t<!is_multi<T>::value, 
    multi<decltype(T() + U()), N...>> operator+(
                        const T& val, const multi<U, N...>& arr) 
{
    multi<decltype(T() + U()), N...> res;
    auto itrarr = arr.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrarr, ++itrres) {
        *itrres = val + *itrarr;
    }
    return res;
}

/**
 * @brief Distribute `operator-`.
 */
template <typename T, typename U, std::size_t... N>
constexpr std::enable_if_t<!is_multi<T>::value, 
    multi<decltype(T() - U()), N...>> operator-(
                        const T& val, const multi<U, N...>& arr) 
{
    multi<decltype(T() - U()), N...> res;
    auto itrarr = arr.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrarr, ++itrres) {
        *itrres = val - *itrarr;
    }
    return res;
}

/**
 * @brief Distribute `operator*`.
 */
template <typename T, typename U, std::size_t... N>
constexpr std::enable_if_t<!is_multi<T>::value, 
    multi<decltype(T() * U()), N...>> operator*(
                        const T& val, const multi<U, N...>& arr) 
{
    multi<decltype(T() * U()), N...> res;
    auto itrarr = arr.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrarr, ++itrres) {
        *itrres = val * *itrarr;
    }
    return res;
}

/**
 * @brief Distribute `operator/`.
 */
template <typename T, typename U, std::size_t... N>
constexpr std::enable_if_t<!is_multi<T>::value, 
    multi<decltype(T() / U()), N...>> operator/(
                        const T& val, const multi<U, N...>& arr) 
{
    multi<decltype(T() / U()), N...> res;
    auto itrarr = arr.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrarr, ++itrres) {
        *itrres = val / *itrarr;
    }
    return res;
}

/**
 * @brief Distribute `operator%`.
 */
template <typename T, typename U, std::size_t... N>
constexpr std::enable_if_t<!is_multi<T>::value, 
    multi<decltype(T() % U()), N...>> operator%(
                        const T& val, const multi<U, N...>& arr) 
{
    multi<decltype(T() % U()), N...> res;
    auto itrarr = arr.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrarr, ++itrres) {
        *itrres = val % *itrarr;
    }
    return res;
}

/**
 * @brief Distribute `operator&`.
 */
template <typename T, typename U, std::size_t... N>
constexpr std::enable_if_t<!is_multi<T>::value, 
    multi<decltype(T() & U()), N...>> operator&(
                        const T& val, const multi<U, N...>& arr) 
{
    multi<decltype(T() & U()), N...> res;
    auto itrarr = arr.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrarr, ++itrres) {
        *itrres = val & *itrarr;
    }
    return res;
}

/**
 * @brief Distribute `operator|`.
 */
template <typename T, typename U, std::size_t... N>
constexpr std::enable_if_t<!is_multi<T>::value, 
    multi<decltype(T() | U()), N...>> operator|(
                        const T& val, const multi<U, N...>& arr) 
{
    multi<decltype(T() | U()), N...> res;
    auto itrarr = arr.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrarr, ++itrres) {
        *itrres = val | *itrarr;
    }
    return res;
}

/**
 * @brief Distribute `operator^`.
 */
template <typename T, typename U, std::size_t... N>
constexpr std::enable_if_t<!is_multi<T>::value, 
    multi<decltype(T() ^ U()), N...>> operator^(
                        const T& val, const multi<U, N...>& arr) 
{
    multi<decltype(T() ^ U()), N...> res;
    auto itrarr = arr.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrarr, ++itrres) {
        *itrres = val ^ *itrarr;
    }
    return res;
}

/**
 * @brief Distribute `operator>>`.
 */
template <typename T, typename U, std::size_t... N>
constexpr std::enable_if_t<!is_multi<T>::value, 
    multi<decltype(T() >> U()), N...>> operator>>(
                        const T& val, const multi<U, N...>& arr) 
{
    multi<decltype(T() >> U()), N...> res;
    auto itrarr = arr.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrarr, ++itrres) {
        *itrres = val >> *itrarr;
    }
    return res;
}

/**
 * @brief Distribute `operator<<`.
 */
template <typename T, typename U, std::size_t... N>
constexpr std::enable_if_t<!is_multi<T>::value, 
    multi<decltype(T() << U()), N...>> operator<<(
                        const T& val, const multi<U, N...>& arr) 
{
    multi<decltype(T() << U()), N...> res;
    auto itrarr = arr.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrarr, ++itrres) {
        *itrres = val << *itrarr;
    }
    return res;
}

/**@}*/

/**@}*/

