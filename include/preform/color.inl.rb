pack = [
    'uint8',
    'uint16',
    'uint32',
    'int8',
    'int16', 
    'int32'
]

unpack = [
    'float',
    'double'
]

puts <<STR
namespace pr {

/**
 * @addtogroup color
 */
/**@{*/

/**
 * @name Color packing
 */
/**@{*/

STR

for elem in pack
    puts <<STR
/**
 * @brief Alias for `fstretch<std::#{elem}_t>()`.
 */
template <typename T>
__attribute__((always_inline))
inline std::enable_if_t<
       std::is_floating_point<T>::value, std::#{elem}_t> pack_#{elem}(T v)
{
    return fstretch<std::#{elem}_t>(v);
}

STR
end

for elem in pack
    puts <<STR
/**
 * @brief Alias for `fstretch<std::#{elem}_t>()`.
 */
template <typename T, std::size_t N>
__attribute__((always_inline))
inline std::enable_if_t<
       std::is_floating_point<T>::value, 
       multi<std::#{elem}_t, N>> pack_#{elem}(const multi<T, N>& v)
{
    return fstretch<std::#{elem}_t>(v);
}

STR
end

for elem in unpack
    puts <<STR
/**
 * @brief Alias for `fstretch<#{elem}>()`.
 */
template <typename T>
__attribute__((always_inline))
inline std::enable_if_t<
       std::is_integral<T>::value, #{elem}> unpack_#{elem}(T v)
{
    return fstretch<#{elem}>(v);
}

STR
end

for elem in unpack
    puts <<STR
/**
 * @brief Alias for `fstretch<#{elem}>()`.
 */
template <typename T, std::size_t N>
__attribute__((always_inline))
inline std::enable_if_t<
       std::is_integral<T>::value, 
       multi<#{elem}, N>> unpack_#{elem}(const multi<T, N>& v)
{
    return fstretch<#{elem}>(v);
}

STR
end

puts <<STR
/**@}*/

/**@}*/

} // namespace pr

STR
