namespace pr {

/**
 * @addtogroup dense_vector_view
 */
/**@{*/

/**
 * @name Binary operators (dense_vector_view/dense_vector_view)
 */
/**@{*/

/**
 * @brief Distribute `operator+=`.
 *
 * @throw std::invalid_argument
 * Unless `arr.size() == oth.size()`.
 */
template <typename T, typename U>
__attribute__((always_inline))
constexpr dense_vector_view<T>& operator+=(
          dense_vector_view<T>& arr, dense_vector_view<U> oth)
{
    if (arr.size() != oth.size()) {
        throw std::invalid_argument(__PRETTY_FUNCTION__);
    }
    auto itrarr = arr.begin();
    auto itroth = oth.begin();
    for (; itroth != oth.end(); ++itrarr, ++itroth) {
        *itrarr += *itroth;
    }
    return arr;
}

/**
 * @brief Distribute `operator-=`.
 *
 * @throw std::invalid_argument
 * Unless `arr.size() == oth.size()`.
 */
template <typename T, typename U>
__attribute__((always_inline))
constexpr dense_vector_view<T>& operator-=(
          dense_vector_view<T>& arr, dense_vector_view<U> oth)
{
    if (arr.size() != oth.size()) {
        throw std::invalid_argument(__PRETTY_FUNCTION__);
    }
    auto itrarr = arr.begin();
    auto itroth = oth.begin();
    for (; itroth != oth.end(); ++itrarr, ++itroth) {
        *itrarr -= *itroth;
    }
    return arr;
}

/**
 * @brief Distribute `operator*=`.
 *
 * @throw std::invalid_argument
 * Unless `arr.size() == oth.size()`.
 */
template <typename T, typename U>
__attribute__((always_inline))
constexpr dense_vector_view<T>& operator*=(
          dense_vector_view<T>& arr, dense_vector_view<U> oth)
{
    if (arr.size() != oth.size()) {
        throw std::invalid_argument(__PRETTY_FUNCTION__);
    }
    auto itrarr = arr.begin();
    auto itroth = oth.begin();
    for (; itroth != oth.end(); ++itrarr, ++itroth) {
        *itrarr *= *itroth;
    }
    return arr;
}

/**
 * @brief Distribute `operator/=`.
 *
 * @throw std::invalid_argument
 * Unless `arr.size() == oth.size()`.
 */
template <typename T, typename U>
__attribute__((always_inline))
constexpr dense_vector_view<T>& operator/=(
          dense_vector_view<T>& arr, dense_vector_view<U> oth)
{
    if (arr.size() != oth.size()) {
        throw std::invalid_argument(__PRETTY_FUNCTION__);
    }
    auto itrarr = arr.begin();
    auto itroth = oth.begin();
    for (; itroth != oth.end(); ++itrarr, ++itroth) {
        *itrarr /= *itroth;
    }
    return arr;
}

/**
 * @brief Distribute `operator%=`.
 *
 * @throw std::invalid_argument
 * Unless `arr.size() == oth.size()`.
 */
template <typename T, typename U>
__attribute__((always_inline))
constexpr dense_vector_view<T>& operator%=(
          dense_vector_view<T>& arr, dense_vector_view<U> oth)
{
    if (arr.size() != oth.size()) {
        throw std::invalid_argument(__PRETTY_FUNCTION__);
    }
    auto itrarr = arr.begin();
    auto itroth = oth.begin();
    for (; itroth != oth.end(); ++itrarr, ++itroth) {
        *itrarr %= *itroth;
    }
    return arr;
}

/**
 * @brief Distribute `operator&=`.
 *
 * @throw std::invalid_argument
 * Unless `arr.size() == oth.size()`.
 */
template <typename T, typename U>
__attribute__((always_inline))
constexpr dense_vector_view<T>& operator&=(
          dense_vector_view<T>& arr, dense_vector_view<U> oth)
{
    if (arr.size() != oth.size()) {
        throw std::invalid_argument(__PRETTY_FUNCTION__);
    }
    auto itrarr = arr.begin();
    auto itroth = oth.begin();
    for (; itroth != oth.end(); ++itrarr, ++itroth) {
        *itrarr &= *itroth;
    }
    return arr;
}

/**
 * @brief Distribute `operator|=`.
 *
 * @throw std::invalid_argument
 * Unless `arr.size() == oth.size()`.
 */
template <typename T, typename U>
__attribute__((always_inline))
constexpr dense_vector_view<T>& operator|=(
          dense_vector_view<T>& arr, dense_vector_view<U> oth)
{
    if (arr.size() != oth.size()) {
        throw std::invalid_argument(__PRETTY_FUNCTION__);
    }
    auto itrarr = arr.begin();
    auto itroth = oth.begin();
    for (; itroth != oth.end(); ++itrarr, ++itroth) {
        *itrarr |= *itroth;
    }
    return arr;
}

/**
 * @brief Distribute `operator^=`.
 *
 * @throw std::invalid_argument
 * Unless `arr.size() == oth.size()`.
 */
template <typename T, typename U>
__attribute__((always_inline))
constexpr dense_vector_view<T>& operator^=(
          dense_vector_view<T>& arr, dense_vector_view<U> oth)
{
    if (arr.size() != oth.size()) {
        throw std::invalid_argument(__PRETTY_FUNCTION__);
    }
    auto itrarr = arr.begin();
    auto itroth = oth.begin();
    for (; itroth != oth.end(); ++itrarr, ++itroth) {
        *itrarr ^= *itroth;
    }
    return arr;
}

/**
 * @brief Distribute `operator>>=`.
 *
 * @throw std::invalid_argument
 * Unless `arr.size() == oth.size()`.
 */
template <typename T, typename U>
__attribute__((always_inline))
constexpr dense_vector_view<T>& operator>>=(
          dense_vector_view<T>& arr, dense_vector_view<U> oth)
{
    if (arr.size() != oth.size()) {
        throw std::invalid_argument(__PRETTY_FUNCTION__);
    }
    auto itrarr = arr.begin();
    auto itroth = oth.begin();
    for (; itroth != oth.end(); ++itrarr, ++itroth) {
        *itrarr >>= *itroth;
    }
    return arr;
}

/**
 * @brief Distribute `operator<<=`.
 *
 * @throw std::invalid_argument
 * Unless `arr.size() == oth.size()`.
 */
template <typename T, typename U>
__attribute__((always_inline))
constexpr dense_vector_view<T>& operator<<=(
          dense_vector_view<T>& arr, dense_vector_view<U> oth)
{
    if (arr.size() != oth.size()) {
        throw std::invalid_argument(__PRETTY_FUNCTION__);
    }
    auto itrarr = arr.begin();
    auto itroth = oth.begin();
    for (; itroth != oth.end(); ++itrarr, ++itroth) {
        *itrarr <<= *itroth;
    }
    return arr;
}

/**@}*/

/**
 * @name Binary operators (dense_vector_view/value_type)
 */
/**@{*/

/**
 * @brief Distribute `operator+=`.
 */
template <typename T>
__attribute__((always_inline))
constexpr dense_vector_view<T>& operator+=(
          dense_vector_view<T>& arr,
          typename dense_vector_view<T>::value_type val)
{
    for (auto itrarr = arr.begin(); 
              itrarr != arr.end(); ++itrarr) {
        *itrarr += val;
    }
    return arr;
}

/**
 * @brief Distribute `operator-=`.
 */
template <typename T>
__attribute__((always_inline))
constexpr dense_vector_view<T>& operator-=(
          dense_vector_view<T>& arr,
          typename dense_vector_view<T>::value_type val)
{
    for (auto itrarr = arr.begin(); 
              itrarr != arr.end(); ++itrarr) {
        *itrarr -= val;
    }
    return arr;
}

/**
 * @brief Distribute `operator*=`.
 */
template <typename T>
__attribute__((always_inline))
constexpr dense_vector_view<T>& operator*=(
          dense_vector_view<T>& arr,
          typename dense_vector_view<T>::value_type val)
{
    for (auto itrarr = arr.begin(); 
              itrarr != arr.end(); ++itrarr) {
        *itrarr *= val;
    }
    return arr;
}

/**
 * @brief Distribute `operator/=`.
 */
template <typename T>
__attribute__((always_inline))
constexpr dense_vector_view<T>& operator/=(
          dense_vector_view<T>& arr,
          typename dense_vector_view<T>::value_type val)
{
    for (auto itrarr = arr.begin(); 
              itrarr != arr.end(); ++itrarr) {
        *itrarr /= val;
    }
    return arr;
}

/**
 * @brief Distribute `operator%=`.
 */
template <typename T>
__attribute__((always_inline))
constexpr dense_vector_view<T>& operator%=(
          dense_vector_view<T>& arr,
          typename dense_vector_view<T>::value_type val)
{
    for (auto itrarr = arr.begin(); 
              itrarr != arr.end(); ++itrarr) {
        *itrarr %= val;
    }
    return arr;
}

/**
 * @brief Distribute `operator&=`.
 */
template <typename T>
__attribute__((always_inline))
constexpr dense_vector_view<T>& operator&=(
          dense_vector_view<T>& arr,
          typename dense_vector_view<T>::value_type val)
{
    for (auto itrarr = arr.begin(); 
              itrarr != arr.end(); ++itrarr) {
        *itrarr &= val;
    }
    return arr;
}

/**
 * @brief Distribute `operator|=`.
 */
template <typename T>
__attribute__((always_inline))
constexpr dense_vector_view<T>& operator|=(
          dense_vector_view<T>& arr,
          typename dense_vector_view<T>::value_type val)
{
    for (auto itrarr = arr.begin(); 
              itrarr != arr.end(); ++itrarr) {
        *itrarr |= val;
    }
    return arr;
}

/**
 * @brief Distribute `operator^=`.
 */
template <typename T>
__attribute__((always_inline))
constexpr dense_vector_view<T>& operator^=(
          dense_vector_view<T>& arr,
          typename dense_vector_view<T>::value_type val)
{
    for (auto itrarr = arr.begin(); 
              itrarr != arr.end(); ++itrarr) {
        *itrarr ^= val;
    }
    return arr;
}

/**
 * @brief Distribute `operator>>=`.
 */
template <typename T>
__attribute__((always_inline))
constexpr dense_vector_view<T>& operator>>=(
          dense_vector_view<T>& arr,
          typename dense_vector_view<T>::value_type val)
{
    for (auto itrarr = arr.begin(); 
              itrarr != arr.end(); ++itrarr) {
        *itrarr >>= val;
    }
    return arr;
}

/**
 * @brief Distribute `operator<<=`.
 */
template <typename T>
__attribute__((always_inline))
constexpr dense_vector_view<T>& operator<<=(
          dense_vector_view<T>& arr,
          typename dense_vector_view<T>::value_type val)
{
    for (auto itrarr = arr.begin(); 
              itrarr != arr.end(); ++itrarr) {
        *itrarr <<= val;
    }
    return arr;
}

/**@}*/

/**@}*/

} // namespace pr

