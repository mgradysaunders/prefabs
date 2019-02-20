/* Copyright (c) 2018-19 M. Grady Saunders
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

