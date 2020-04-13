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
__attribute__((always_inline))
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
__attribute__((always_inline))
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
__attribute__((always_inline))
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
__attribute__((always_inline))
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
__attribute__((always_inline))
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
__attribute__((always_inline))
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
__attribute__((always_inline))
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
__attribute__((always_inline))
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
__attribute__((always_inline))
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
__attribute__((always_inline))
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
__attribute__((always_inline))
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
__attribute__((always_inline))
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
__attribute__((always_inline))
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
__attribute__((always_inline))
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
__attribute__((always_inline))
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
__attribute__((always_inline))
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
__attribute__((always_inline))
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
__attribute__((always_inline))
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
__attribute__((always_inline))
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
__attribute__((always_inline))
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
__attribute__((always_inline))
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
__attribute__((always_inline))
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
__attribute__((always_inline))
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
__attribute__((always_inline))
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
__attribute__((always_inline))
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
__attribute__((always_inline))
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
__attribute__((always_inline))
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
__attribute__((always_inline))
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
__attribute__((always_inline))
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
__attribute__((always_inline))
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
__attribute__((always_inline))
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
__attribute__((always_inline))
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
__attribute__((always_inline))
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
__attribute__((always_inline))
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
__attribute__((always_inline))
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
__attribute__((always_inline))
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
__attribute__((always_inline))
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
__attribute__((always_inline))
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
__attribute__((always_inline))
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
__attribute__((always_inline))
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
__attribute__((always_inline))
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
__attribute__((always_inline))
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
__attribute__((always_inline))
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
__attribute__((always_inline))
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
__attribute__((always_inline))
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
__attribute__((always_inline))
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
__attribute__((always_inline))
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
__attribute__((always_inline))
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
__attribute__((always_inline))
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
__attribute__((always_inline))
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
__attribute__((always_inline))
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
__attribute__((always_inline))
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
__attribute__((always_inline))
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
__attribute__((always_inline))
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

/**
 * @name Comparison operators (multi/multi)
 */
/**@{*/

/**
 * @brief Distribute `operator==`.
 */
template <typename T, typename U, std::size_t... N>
__attribute__((always_inline))
constexpr multi<decltype(T() == U()), N...> 
                operator==(
                    const multi<T, N...>& arr0, 
                    const multi<U, N...>& arr1)
{
    multi<decltype(T() == U()), N...> res;
    auto itrarr0 = arr0.begin();
    auto itrarr1 = arr1.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrarr0, ++itrarr1, ++itrres) {
        *itrres = *itrarr0 == *itrarr1;
    }
    return res;
}

/**
 * @brief Distribute `operator!=`.
 */
template <typename T, typename U, std::size_t... N>
__attribute__((always_inline))
constexpr multi<decltype(T() != U()), N...> 
                operator!=(
                    const multi<T, N...>& arr0, 
                    const multi<U, N...>& arr1)
{
    multi<decltype(T() != U()), N...> res;
    auto itrarr0 = arr0.begin();
    auto itrarr1 = arr1.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrarr0, ++itrarr1, ++itrres) {
        *itrres = *itrarr0 != *itrarr1;
    }
    return res;
}

/**
 * @brief Distribute `operator<`.
 */
template <typename T, typename U, std::size_t... N>
__attribute__((always_inline))
constexpr multi<decltype(T() < U()), N...> 
                operator<(
                    const multi<T, N...>& arr0, 
                    const multi<U, N...>& arr1)
{
    multi<decltype(T() < U()), N...> res;
    auto itrarr0 = arr0.begin();
    auto itrarr1 = arr1.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrarr0, ++itrarr1, ++itrres) {
        *itrres = *itrarr0 < *itrarr1;
    }
    return res;
}

/**
 * @brief Distribute `operator>`.
 */
template <typename T, typename U, std::size_t... N>
__attribute__((always_inline))
constexpr multi<decltype(T() > U()), N...> 
                operator>(
                    const multi<T, N...>& arr0, 
                    const multi<U, N...>& arr1)
{
    multi<decltype(T() > U()), N...> res;
    auto itrarr0 = arr0.begin();
    auto itrarr1 = arr1.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrarr0, ++itrarr1, ++itrres) {
        *itrres = *itrarr0 > *itrarr1;
    }
    return res;
}

/**
 * @brief Distribute `operator<=`.
 */
template <typename T, typename U, std::size_t... N>
__attribute__((always_inline))
constexpr multi<decltype(T() <= U()), N...> 
                operator<=(
                    const multi<T, N...>& arr0, 
                    const multi<U, N...>& arr1)
{
    multi<decltype(T() <= U()), N...> res;
    auto itrarr0 = arr0.begin();
    auto itrarr1 = arr1.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrarr0, ++itrarr1, ++itrres) {
        *itrres = *itrarr0 <= *itrarr1;
    }
    return res;
}

/**
 * @brief Distribute `operator>=`.
 */
template <typename T, typename U, std::size_t... N>
__attribute__((always_inline))
constexpr multi<decltype(T() >= U()), N...> 
                operator>=(
                    const multi<T, N...>& arr0, 
                    const multi<U, N...>& arr1)
{
    multi<decltype(T() >= U()), N...> res;
    auto itrarr0 = arr0.begin();
    auto itrarr1 = arr1.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrarr0, ++itrarr1, ++itrres) {
        *itrres = *itrarr0 >= *itrarr1;
    }
    return res;
}

/**
 * @brief Distribute `operator&&`.
 */
template <typename T, typename U, std::size_t... N>
__attribute__((always_inline))
constexpr multi<decltype(T() && U()), N...> 
                operator&&(
                    const multi<T, N...>& arr0, 
                    const multi<U, N...>& arr1)
{
    multi<decltype(T() && U()), N...> res;
    auto itrarr0 = arr0.begin();
    auto itrarr1 = arr1.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrarr0, ++itrarr1, ++itrres) {
        *itrres = *itrarr0 && *itrarr1;
    }
    return res;
}

/**
 * @brief Distribute `operator||`.
 */
template <typename T, typename U, std::size_t... N>
__attribute__((always_inline))
constexpr multi<decltype(T() || U()), N...> 
                operator||(
                    const multi<T, N...>& arr0, 
                    const multi<U, N...>& arr1)
{
    multi<decltype(T() || U()), N...> res;
    auto itrarr0 = arr0.begin();
    auto itrarr1 = arr1.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrarr0, ++itrarr1, ++itrres) {
        *itrres = *itrarr0 || *itrarr1;
    }
    return res;
}

/**@}*/

/**
 * @name Comparison operators (multi/entry)
 */
/**@{*/

/**
 * @brief Distribute `operator<<`.
 */
template <typename T, typename U, std::size_t... N>
__attribute__((always_inline))
constexpr std::enable_if_t<!is_multi<U>::value, 
    multi<decltype(T() == U()), N...>> operator==(
                        const multi<T, N...>& arr, const U& val) 
{
    multi<decltype(T() == U()), N...> res;
    auto itrarr = arr.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrarr, ++itrres) {
        *itrres = *itrarr == val;
    }
    return res;
}

/**
 * @brief Distribute `operator<<`.
 */
template <typename T, typename U, std::size_t... N>
__attribute__((always_inline))
constexpr std::enable_if_t<!is_multi<U>::value, 
    multi<decltype(T() != U()), N...>> operator!=(
                        const multi<T, N...>& arr, const U& val) 
{
    multi<decltype(T() != U()), N...> res;
    auto itrarr = arr.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrarr, ++itrres) {
        *itrres = *itrarr != val;
    }
    return res;
}

/**
 * @brief Distribute `operator<<`.
 */
template <typename T, typename U, std::size_t... N>
__attribute__((always_inline))
constexpr std::enable_if_t<!is_multi<U>::value, 
    multi<decltype(T() < U()), N...>> operator<(
                        const multi<T, N...>& arr, const U& val) 
{
    multi<decltype(T() < U()), N...> res;
    auto itrarr = arr.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrarr, ++itrres) {
        *itrres = *itrarr < val;
    }
    return res;
}

/**
 * @brief Distribute `operator<<`.
 */
template <typename T, typename U, std::size_t... N>
__attribute__((always_inline))
constexpr std::enable_if_t<!is_multi<U>::value, 
    multi<decltype(T() > U()), N...>> operator>(
                        const multi<T, N...>& arr, const U& val) 
{
    multi<decltype(T() > U()), N...> res;
    auto itrarr = arr.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrarr, ++itrres) {
        *itrres = *itrarr > val;
    }
    return res;
}

/**
 * @brief Distribute `operator<<`.
 */
template <typename T, typename U, std::size_t... N>
__attribute__((always_inline))
constexpr std::enable_if_t<!is_multi<U>::value, 
    multi<decltype(T() <= U()), N...>> operator<=(
                        const multi<T, N...>& arr, const U& val) 
{
    multi<decltype(T() <= U()), N...> res;
    auto itrarr = arr.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrarr, ++itrres) {
        *itrres = *itrarr <= val;
    }
    return res;
}

/**
 * @brief Distribute `operator<<`.
 */
template <typename T, typename U, std::size_t... N>
__attribute__((always_inline))
constexpr std::enable_if_t<!is_multi<U>::value, 
    multi<decltype(T() >= U()), N...>> operator>=(
                        const multi<T, N...>& arr, const U& val) 
{
    multi<decltype(T() >= U()), N...> res;
    auto itrarr = arr.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrarr, ++itrres) {
        *itrres = *itrarr >= val;
    }
    return res;
}

/**
 * @brief Distribute `operator<<`.
 */
template <typename T, typename U, std::size_t... N>
__attribute__((always_inline))
constexpr std::enable_if_t<!is_multi<U>::value, 
    multi<decltype(T() && U()), N...>> operator&&(
                        const multi<T, N...>& arr, const U& val) 
{
    multi<decltype(T() && U()), N...> res;
    auto itrarr = arr.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrarr, ++itrres) {
        *itrres = *itrarr && val;
    }
    return res;
}

/**
 * @brief Distribute `operator<<`.
 */
template <typename T, typename U, std::size_t... N>
__attribute__((always_inline))
constexpr std::enable_if_t<!is_multi<U>::value, 
    multi<decltype(T() || U()), N...>> operator||(
                        const multi<T, N...>& arr, const U& val) 
{
    multi<decltype(T() || U()), N...> res;
    auto itrarr = arr.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrarr, ++itrres) {
        *itrres = *itrarr || val;
    }
    return res;
}

/**@}*/

/**
 * @name Comparison operators (entry/multi)
 */
/**@{*/

/**
 * @brief Distribute `operator==`.
 */
template <typename T, typename U, std::size_t... N>
__attribute__((always_inline))
constexpr std::enable_if_t<!is_multi<T>::value, 
    multi<decltype(T() == U()), N...>> operator==(
                        const T& val, const multi<U, N...>& arr) 
{
    multi<decltype(T() == U()), N...> res;
    auto itrarr = arr.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrarr, ++itrres) {
        *itrres = val == *itrarr;
    }
    return res;
}

/**
 * @brief Distribute `operator!=`.
 */
template <typename T, typename U, std::size_t... N>
__attribute__((always_inline))
constexpr std::enable_if_t<!is_multi<T>::value, 
    multi<decltype(T() != U()), N...>> operator!=(
                        const T& val, const multi<U, N...>& arr) 
{
    multi<decltype(T() != U()), N...> res;
    auto itrarr = arr.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrarr, ++itrres) {
        *itrres = val != *itrarr;
    }
    return res;
}

/**
 * @brief Distribute `operator<`.
 */
template <typename T, typename U, std::size_t... N>
__attribute__((always_inline))
constexpr std::enable_if_t<!is_multi<T>::value, 
    multi<decltype(T() < U()), N...>> operator<(
                        const T& val, const multi<U, N...>& arr) 
{
    multi<decltype(T() < U()), N...> res;
    auto itrarr = arr.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrarr, ++itrres) {
        *itrres = val < *itrarr;
    }
    return res;
}

/**
 * @brief Distribute `operator>`.
 */
template <typename T, typename U, std::size_t... N>
__attribute__((always_inline))
constexpr std::enable_if_t<!is_multi<T>::value, 
    multi<decltype(T() > U()), N...>> operator>(
                        const T& val, const multi<U, N...>& arr) 
{
    multi<decltype(T() > U()), N...> res;
    auto itrarr = arr.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrarr, ++itrres) {
        *itrres = val > *itrarr;
    }
    return res;
}

/**
 * @brief Distribute `operator<=`.
 */
template <typename T, typename U, std::size_t... N>
__attribute__((always_inline))
constexpr std::enable_if_t<!is_multi<T>::value, 
    multi<decltype(T() <= U()), N...>> operator<=(
                        const T& val, const multi<U, N...>& arr) 
{
    multi<decltype(T() <= U()), N...> res;
    auto itrarr = arr.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrarr, ++itrres) {
        *itrres = val <= *itrarr;
    }
    return res;
}

/**
 * @brief Distribute `operator>=`.
 */
template <typename T, typename U, std::size_t... N>
__attribute__((always_inline))
constexpr std::enable_if_t<!is_multi<T>::value, 
    multi<decltype(T() >= U()), N...>> operator>=(
                        const T& val, const multi<U, N...>& arr) 
{
    multi<decltype(T() >= U()), N...> res;
    auto itrarr = arr.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrarr, ++itrres) {
        *itrres = val >= *itrarr;
    }
    return res;
}

/**
 * @brief Distribute `operator&&`.
 */
template <typename T, typename U, std::size_t... N>
__attribute__((always_inline))
constexpr std::enable_if_t<!is_multi<T>::value, 
    multi<decltype(T() && U()), N...>> operator&&(
                        const T& val, const multi<U, N...>& arr) 
{
    multi<decltype(T() && U()), N...> res;
    auto itrarr = arr.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrarr, ++itrres) {
        *itrres = val && *itrarr;
    }
    return res;
}

/**
 * @brief Distribute `operator||`.
 */
template <typename T, typename U, std::size_t... N>
__attribute__((always_inline))
constexpr std::enable_if_t<!is_multi<T>::value, 
    multi<decltype(T() || U()), N...>> operator||(
                        const T& val, const multi<U, N...>& arr) 
{
    multi<decltype(T() || U()), N...> res;
    auto itrarr = arr.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrarr, ++itrres) {
        *itrres = val || *itrarr;
    }
    return res;
}

/**@}*/

/**
 * @brief Template alias for convenience.
 */
template <typename T>
using vec1 = multi<T, 1>;

/**
 * @brief Template alias for convenience.
 */
template <typename T>
using vec2 = multi<T, 2>;

/**
 * @brief Template alias for convenience.
 */
template <typename T>
using vec3 = multi<T, 3>;

/**
 * @brief Template alias for convenience.
 */
template <typename T>
using vec4 = multi<T, 4>;

/**
 * @brief Template alias for convenience.
 */
template <typename T>
using mat1 = multi<T, 1, 1>;

/**
 * @brief Template alias for convenience.
 */
template <typename T>
using mat2 = multi<T, 2, 2>;

/**
 * @brief Template alias for convenience.
 */
template <typename T>
using mat3 = multi<T, 3, 3>;

/**
 * @brief Template alias for convenience.
 */
template <typename T>
using mat4 = multi<T, 4, 4>;

/**
 * @brief Template alias for convenience.
 */
template <typename T>
using mat1x1 = multi<T, 1, 1>;

/**
 * @brief Template alias for convenience.
 */
template <typename T>
using mat1x2 = multi<T, 1, 2>;

/**
 * @brief Template alias for convenience.
 */
template <typename T>
using mat1x3 = multi<T, 1, 3>;

/**
 * @brief Template alias for convenience.
 */
template <typename T>
using mat1x4 = multi<T, 1, 4>;

/**
 * @brief Template alias for convenience.
 */
template <typename T>
using mat2x1 = multi<T, 2, 1>;

/**
 * @brief Template alias for convenience.
 */
template <typename T>
using mat2x2 = multi<T, 2, 2>;

/**
 * @brief Template alias for convenience.
 */
template <typename T>
using mat2x3 = multi<T, 2, 3>;

/**
 * @brief Template alias for convenience.
 */
template <typename T>
using mat2x4 = multi<T, 2, 4>;

/**
 * @brief Template alias for convenience.
 */
template <typename T>
using mat3x1 = multi<T, 3, 1>;

/**
 * @brief Template alias for convenience.
 */
template <typename T>
using mat3x2 = multi<T, 3, 2>;

/**
 * @brief Template alias for convenience.
 */
template <typename T>
using mat3x3 = multi<T, 3, 3>;

/**
 * @brief Template alias for convenience.
 */
template <typename T>
using mat3x4 = multi<T, 3, 4>;

/**
 * @brief Template alias for convenience.
 */
template <typename T>
using mat4x1 = multi<T, 4, 1>;

/**
 * @brief Template alias for convenience.
 */
template <typename T>
using mat4x2 = multi<T, 4, 2>;

/**
 * @brief Template alias for convenience.
 */
template <typename T>
using mat4x3 = multi<T, 4, 3>;

/**
 * @brief Template alias for convenience.
 */
template <typename T>
using mat4x4 = multi<T, 4, 4>;

/**@}*/

} // namespace pre

