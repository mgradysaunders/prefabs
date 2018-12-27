OP1 = ['+', '-', '~', '!']
OP2 = ['+', '-', '*', '/', '%', '&', '|', '^', '>>', '<<']
OPC = ['==', '!=', '<', '>', '<=', '>=', '&&', '||']

puts <<STR
namespace pr {

/**
 * @addtogroup multi
 */
/**@{*/

STR

#------------------------------------------------------------------------------

puts <<STR
/**
 * @name Unary operators (multi)
 */
/**@{*/

STR

for op1 in OP1
    puts <<STR
/**
 * @brief Distribute `operator#{op1}`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
constexpr multi<decltype(#{op1}T()), N...> 
                operator#{op1}(const multi<T, N...>& arr)
{
    multi<decltype(#{op1}T()), N...> res;
    auto itrarr = arr.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrarr, ++itrres) {
        *itrres = #{op1}*itrarr;
    }
    return res;
}

STR
end

puts <<STR
/**@}*/

STR

#------------------------------------------------------------------------------

puts <<STR
/**
 * @name Binary operators (multi/multi)
 */
/**@{*/

STR

for op2 in OP2
    puts <<STR
/**
 * @brief Distribute `operator#{op2}`.
 */
template <typename T, typename U, std::size_t... N>
__attribute__((always_inline))
constexpr multi<decltype(T() #{op2} U()), N...> 
                operator#{op2}(
                    const multi<T, N...>& arr0, 
                    const multi<U, N...>& arr1)
{
    multi<decltype(T() #{op2} U()), N...> res;
    auto itrarr0 = arr0.begin();
    auto itrarr1 = arr1.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrarr0, ++itrarr1, ++itrres) {
        *itrres = *itrarr0 #{op2} *itrarr1;
    }
    return res;
}

STR
end

for op2 in OP2
    puts <<STR
/**
 * @brief Distribute `operator#{op2}=`.
 */
template <typename T, typename U, std::size_t... N>
__attribute__((always_inline))
constexpr multi<T, N...>& operator#{op2}=(
                    multi<T, N...>& arr, const multi<U, N...>& oth)
{
    auto itrarr = arr.begin();
    auto itroth = oth.begin();
    for (; itroth < oth.end(); ++itrarr, ++itroth) {
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
 * @name Binary operators (multi/entry)
 */
/**@{*/

STR

for op2 in OP2
    puts <<STR
/**
 * @brief Distribute `operator#{op2}`.
 */
template <typename T, typename U, std::size_t... N>
__attribute__((always_inline))
constexpr std::enable_if_t<!is_multi<U>::value, 
    multi<decltype(T() #{op2} U()), N...>> operator#{op2}(
                        const multi<T, N...>& arr, const U& val) 
{
    multi<decltype(T() #{op2} U()), N...> res;
    auto itrarr = arr.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrarr, ++itrres) {
        *itrres = *itrarr #{op2} val;
    }
    return res;
}

STR
end

for op2 in OP2
    puts <<STR
/**
 * @brief Distribute `operator#{op2}=`.
 */
template <typename T, typename U, std::size_t... N>
__attribute__((always_inline))
constexpr std::enable_if_t<!is_multi<U>::value, 
    multi<T, N...>&> operator#{op2}=(multi<T, N...>& arr, const U& val)
{
    for (auto itrarr = arr.begin(); 
              itrarr < arr.end(); ++itrarr) {
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
/**
 * @name Binary operators (entry/multi)
 */
/**@{*/

STR

for op2 in OP2
    puts <<STR
/**
 * @brief Distribute `operator#{op2}`.
 */
template <typename T, typename U, std::size_t... N>
__attribute__((always_inline))
constexpr std::enable_if_t<!is_multi<T>::value, 
    multi<decltype(T() #{op2} U()), N...>> operator#{op2}(
                        const T& val, const multi<U, N...>& arr) 
{
    multi<decltype(T() #{op2} U()), N...> res;
    auto itrarr = arr.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrarr, ++itrres) {
        *itrres = val #{op2} *itrarr;
    }
    return res;
}

STR
end

puts <<STR
/**@}*/

STR

#------------------------------------------------------------------------------

puts <<STR
/**
 * @name Comparison operators (multi/multi)
 */
/**@{*/

STR

for opc in OPC
    puts <<STR
/**
 * @brief Distribute `operator#{opc}`.
 */
template <typename T, typename U, std::size_t... N>
__attribute__((always_inline))
constexpr multi<decltype(T() #{opc} U()), N...> 
                operator#{opc}(
                    const multi<T, N...>& arr0, 
                    const multi<U, N...>& arr1)
{
    multi<decltype(T() #{opc} U()), N...> res;
    auto itrarr0 = arr0.begin();
    auto itrarr1 = arr1.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrarr0, ++itrarr1, ++itrres) {
        *itrres = *itrarr0 #{opc} *itrarr1;
    }
    return res;
}

STR
end

puts <<STR
/**@}*/

STR

#------------------------------------------------------------------------------

puts <<STR
/**
 * @name Comparison operators (multi/entry)
 */
/**@{*/

STR

for opc in OPC
    puts <<STR
/**
 * @brief Distribute `operator#{op2}`.
 */
template <typename T, typename U, std::size_t... N>
__attribute__((always_inline))
constexpr std::enable_if_t<!is_multi<U>::value, 
    multi<decltype(T() #{opc} U()), N...>> operator#{opc}(
                        const multi<T, N...>& arr, const U& val) 
{
    multi<decltype(T() #{opc} U()), N...> res;
    auto itrarr = arr.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrarr, ++itrres) {
        *itrres = *itrarr #{opc} val;
    }
    return res;
}

STR
end

puts <<STR
/**@}*/

STR

#------------------------------------------------------------------------------

puts <<STR
/**
 * @name Comparison operators (entry/multi)
 */
/**@{*/

STR

for opc in OPC
    puts <<STR
/**
 * @brief Distribute `operator#{opc}`.
 */
template <typename T, typename U, std::size_t... N>
__attribute__((always_inline))
constexpr std::enable_if_t<!is_multi<T>::value, 
    multi<decltype(T() #{opc} U()), N...>> operator#{opc}(
                        const T& val, const multi<U, N...>& arr) 
{
    multi<decltype(T() #{opc} U()), N...> res;
    auto itrarr = arr.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrarr, ++itrres) {
        *itrres = val #{opc} *itrarr;
    }
    return res;
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
