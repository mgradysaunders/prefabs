OP2 = ['+', '-', '*', '/', '%', '&', '|', '^', '>>', '<<']

puts <<STR
namespace pr {

/**
 * @addtogroup dense_matrix_view
 */
/**@{*/

STR

puts <<STR
/**
 * @name Binary operators (dense_matrix_view/dense_matrix_view)
 */
/**@{*/

STR

for op2 in OP2
    puts <<STR
/**
 * @brief Distribute `operator#{op2}=`.
 *
 * @throw std::invalid_argument
 * Unless `arr.size() == oth.size()`.
 */
template <typename T, typename U>
__attribute__((always_inline))
constexpr dense_matrix_view<T>& operator#{op2}=(
          dense_matrix_view<T>& arr, dense_matrix_view<U> oth)
{
    if (arr.size() != oth.size()) {
        throw std::invalid_argument(__PRETTY_FUNCTION__);
    }
    for (typename dense_matrix_view<T>::difference_type i = 0;
                i < arr.size0(); i++) {
         arr[i] #{op2}= oth[i];
    }
    return arr;
}

STR
end

puts <<STR
/**@}*/

STR

#------------------------------------------------------------------------------

puts <<STR
/**
 * @name Binary operators (dense_matrix_view/value_type)
 */
/**@{*/

STR

for op2 in OP2
    puts <<STR
/**
 * @brief Distribute `operator#{op2}=`.
 */
template <typename T>
__attribute__((always_inline))
constexpr dense_matrix_view<T>& operator#{op2}=(
          dense_matrix_view<T>& arr,
          typename dense_matrix_view<T>::value_type val)
{
    for (typename dense_matrix_view<T>::difference_type i = 0;
                i < arr.size0(); i++) {
        arr[i] #{op2}= val;
    }
    return arr;
}

STR
end

puts <<STR
/**@}*/

STR

#------------------------------------------------------------------------------

puts <<STR
/**@}*/

} // namespace pr

STR
