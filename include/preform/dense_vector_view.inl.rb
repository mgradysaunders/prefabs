OP2 = ['+', '-', '*', '/', '%', '&', '|', '^', '>>', '<<']

puts <<STR
namespace pre {

/**
 * @addtogroup dense_vector_view
 */
/**@{*/

STR

puts <<STR
/**
 * @name Binary operators (dense_vector_view/dense_vector_view)
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
constexpr dense_vector_view<T>& operator#{op2}=(
          dense_vector_view<T>& arr, dense_vector_view<U> oth)
{
    if (arr.size() != oth.size()) {
        throw std::invalid_argument(__PRETTY_FUNCTION__);
    }
    auto itrarr = arr.begin();
    auto itroth = oth.begin();
    for (; itroth != oth.end(); ++itrarr, ++itroth) {
        *itrarr #{op2}= *itroth;
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
 * @name Binary operators (dense_vector_view/value_type)
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
constexpr dense_vector_view<T>& operator#{op2}=(
          dense_vector_view<T>& arr,
          typename dense_vector_view<T>::value_type val)
{
    for (auto itrarr = arr.begin(); 
              itrarr != arr.end(); ++itrarr) {
        *itrarr #{op2}= val;
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

} // namespace pre

STR
