namespace pr {

/**
 * @addtogroup dense_matrix_view
 */
/**@{*/

/**
 * @name Binary operators (dense_matrix_view/dense_matrix_view)
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
constexpr dense_matrix_view<T>& operator+=(
          dense_matrix_view<T>& arr, dense_matrix_view<U> oth)
{
    if (arr.size() != oth.size()) {
        throw std::invalid_argument(__PRETTY_FUNCTION__);
    }
    for (typename dense_matrix_view<T>::difference_type i = 0;
                i < arr.size0(); i++) {
         arr[i] += oth[i];
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
constexpr dense_matrix_view<T>& operator-=(
          dense_matrix_view<T>& arr, dense_matrix_view<U> oth)
{
    if (arr.size() != oth.size()) {
        throw std::invalid_argument(__PRETTY_FUNCTION__);
    }
    for (typename dense_matrix_view<T>::difference_type i = 0;
                i < arr.size0(); i++) {
         arr[i] -= oth[i];
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
constexpr dense_matrix_view<T>& operator*=(
          dense_matrix_view<T>& arr, dense_matrix_view<U> oth)
{
    if (arr.size() != oth.size()) {
        throw std::invalid_argument(__PRETTY_FUNCTION__);
    }
    for (typename dense_matrix_view<T>::difference_type i = 0;
                i < arr.size0(); i++) {
         arr[i] *= oth[i];
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
constexpr dense_matrix_view<T>& operator/=(
          dense_matrix_view<T>& arr, dense_matrix_view<U> oth)
{
    if (arr.size() != oth.size()) {
        throw std::invalid_argument(__PRETTY_FUNCTION__);
    }
    for (typename dense_matrix_view<T>::difference_type i = 0;
                i < arr.size0(); i++) {
         arr[i] /= oth[i];
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
constexpr dense_matrix_view<T>& operator%=(
          dense_matrix_view<T>& arr, dense_matrix_view<U> oth)
{
    if (arr.size() != oth.size()) {
        throw std::invalid_argument(__PRETTY_FUNCTION__);
    }
    for (typename dense_matrix_view<T>::difference_type i = 0;
                i < arr.size0(); i++) {
         arr[i] %= oth[i];
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
constexpr dense_matrix_view<T>& operator&=(
          dense_matrix_view<T>& arr, dense_matrix_view<U> oth)
{
    if (arr.size() != oth.size()) {
        throw std::invalid_argument(__PRETTY_FUNCTION__);
    }
    for (typename dense_matrix_view<T>::difference_type i = 0;
                i < arr.size0(); i++) {
         arr[i] &= oth[i];
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
constexpr dense_matrix_view<T>& operator|=(
          dense_matrix_view<T>& arr, dense_matrix_view<U> oth)
{
    if (arr.size() != oth.size()) {
        throw std::invalid_argument(__PRETTY_FUNCTION__);
    }
    for (typename dense_matrix_view<T>::difference_type i = 0;
                i < arr.size0(); i++) {
         arr[i] |= oth[i];
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
constexpr dense_matrix_view<T>& operator^=(
          dense_matrix_view<T>& arr, dense_matrix_view<U> oth)
{
    if (arr.size() != oth.size()) {
        throw std::invalid_argument(__PRETTY_FUNCTION__);
    }
    for (typename dense_matrix_view<T>::difference_type i = 0;
                i < arr.size0(); i++) {
         arr[i] ^= oth[i];
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
constexpr dense_matrix_view<T>& operator>>=(
          dense_matrix_view<T>& arr, dense_matrix_view<U> oth)
{
    if (arr.size() != oth.size()) {
        throw std::invalid_argument(__PRETTY_FUNCTION__);
    }
    for (typename dense_matrix_view<T>::difference_type i = 0;
                i < arr.size0(); i++) {
         arr[i] >>= oth[i];
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
constexpr dense_matrix_view<T>& operator<<=(
          dense_matrix_view<T>& arr, dense_matrix_view<U> oth)
{
    if (arr.size() != oth.size()) {
        throw std::invalid_argument(__PRETTY_FUNCTION__);
    }
    for (typename dense_matrix_view<T>::difference_type i = 0;
                i < arr.size0(); i++) {
         arr[i] <<= oth[i];
    }
    return arr;
}

/**@}*/

/**
 * @name Binary operators (dense_matrix_view/value_type)
 */
/**@{*/

/**
 * @brief Distribute `operator+=`.
 */
template <typename T>
__attribute__((always_inline))
constexpr dense_matrix_view<T>& operator+=(
          dense_matrix_view<T>& arr,
          typename dense_matrix_view<T>::value_type val)
{
    for (typename dense_matrix_view<T>::difference_type i = 0;
                i < arr.size0(); i++) {
        arr[i] += val;
    }
    return arr;
}

/**
 * @brief Distribute `operator-=`.
 */
template <typename T>
__attribute__((always_inline))
constexpr dense_matrix_view<T>& operator-=(
          dense_matrix_view<T>& arr,
          typename dense_matrix_view<T>::value_type val)
{
    for (typename dense_matrix_view<T>::difference_type i = 0;
                i < arr.size0(); i++) {
        arr[i] -= val;
    }
    return arr;
}

/**
 * @brief Distribute `operator*=`.
 */
template <typename T>
__attribute__((always_inline))
constexpr dense_matrix_view<T>& operator*=(
          dense_matrix_view<T>& arr,
          typename dense_matrix_view<T>::value_type val)
{
    for (typename dense_matrix_view<T>::difference_type i = 0;
                i < arr.size0(); i++) {
        arr[i] *= val;
    }
    return arr;
}

/**
 * @brief Distribute `operator/=`.
 */
template <typename T>
__attribute__((always_inline))
constexpr dense_matrix_view<T>& operator/=(
          dense_matrix_view<T>& arr,
          typename dense_matrix_view<T>::value_type val)
{
    for (typename dense_matrix_view<T>::difference_type i = 0;
                i < arr.size0(); i++) {
        arr[i] /= val;
    }
    return arr;
}

/**
 * @brief Distribute `operator%=`.
 */
template <typename T>
__attribute__((always_inline))
constexpr dense_matrix_view<T>& operator%=(
          dense_matrix_view<T>& arr,
          typename dense_matrix_view<T>::value_type val)
{
    for (typename dense_matrix_view<T>::difference_type i = 0;
                i < arr.size0(); i++) {
        arr[i] %= val;
    }
    return arr;
}

/**
 * @brief Distribute `operator&=`.
 */
template <typename T>
__attribute__((always_inline))
constexpr dense_matrix_view<T>& operator&=(
          dense_matrix_view<T>& arr,
          typename dense_matrix_view<T>::value_type val)
{
    for (typename dense_matrix_view<T>::difference_type i = 0;
                i < arr.size0(); i++) {
        arr[i] &= val;
    }
    return arr;
}

/**
 * @brief Distribute `operator|=`.
 */
template <typename T>
__attribute__((always_inline))
constexpr dense_matrix_view<T>& operator|=(
          dense_matrix_view<T>& arr,
          typename dense_matrix_view<T>::value_type val)
{
    for (typename dense_matrix_view<T>::difference_type i = 0;
                i < arr.size0(); i++) {
        arr[i] |= val;
    }
    return arr;
}

/**
 * @brief Distribute `operator^=`.
 */
template <typename T>
__attribute__((always_inline))
constexpr dense_matrix_view<T>& operator^=(
          dense_matrix_view<T>& arr,
          typename dense_matrix_view<T>::value_type val)
{
    for (typename dense_matrix_view<T>::difference_type i = 0;
                i < arr.size0(); i++) {
        arr[i] ^= val;
    }
    return arr;
}

/**
 * @brief Distribute `operator>>=`.
 */
template <typename T>
__attribute__((always_inline))
constexpr dense_matrix_view<T>& operator>>=(
          dense_matrix_view<T>& arr,
          typename dense_matrix_view<T>::value_type val)
{
    for (typename dense_matrix_view<T>::difference_type i = 0;
                i < arr.size0(); i++) {
        arr[i] >>= val;
    }
    return arr;
}

/**
 * @brief Distribute `operator<<=`.
 */
template <typename T>
__attribute__((always_inline))
constexpr dense_matrix_view<T>& operator<<=(
          dense_matrix_view<T>& arr,
          typename dense_matrix_view<T>::value_type val)
{
    for (typename dense_matrix_view<T>::difference_type i = 0;
                i < arr.size0(); i++) {
        arr[i] <<= val;
    }
    return arr;
}

/**@}*/

/**@}*/

} // namespace pr

