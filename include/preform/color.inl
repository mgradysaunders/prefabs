namespace pr {

/**
 * @addtogroup color
 */
/**@{*/

/**
 * @name Color packing
 */
/**@{*/

/**
 * @brief Alias for `fstretch<std::uint8_t>()`.
 */
template <typename T>
__attribute__((always_inline))
inline std::enable_if_t<
       std::is_floating_point<T>::value, std::uint8_t> pack_uint8(T v)
{
    return fstretch<std::uint8_t>(v);
}

/**
 * @brief Alias for `fstretch<std::uint16_t>()`.
 */
template <typename T>
__attribute__((always_inline))
inline std::enable_if_t<
       std::is_floating_point<T>::value, std::uint16_t> pack_uint16(T v)
{
    return fstretch<std::uint16_t>(v);
}

/**
 * @brief Alias for `fstretch<std::uint32_t>()`.
 */
template <typename T>
__attribute__((always_inline))
inline std::enable_if_t<
       std::is_floating_point<T>::value, std::uint32_t> pack_uint32(T v)
{
    return fstretch<std::uint32_t>(v);
}

/**
 * @brief Alias for `fstretch<std::int8_t>()`.
 */
template <typename T>
__attribute__((always_inline))
inline std::enable_if_t<
       std::is_floating_point<T>::value, std::int8_t> pack_int8(T v)
{
    return fstretch<std::int8_t>(v);
}

/**
 * @brief Alias for `fstretch<std::int16_t>()`.
 */
template <typename T>
__attribute__((always_inline))
inline std::enable_if_t<
       std::is_floating_point<T>::value, std::int16_t> pack_int16(T v)
{
    return fstretch<std::int16_t>(v);
}

/**
 * @brief Alias for `fstretch<std::int32_t>()`.
 */
template <typename T>
__attribute__((always_inline))
inline std::enable_if_t<
       std::is_floating_point<T>::value, std::int32_t> pack_int32(T v)
{
    return fstretch<std::int32_t>(v);
}

/**
 * @brief Alias for `fstretch<std::uint8_t>()`.
 */
template <typename T, std::size_t N>
__attribute__((always_inline))
inline std::enable_if_t<
       std::is_floating_point<T>::value, 
       multi<std::uint8_t, N>> pack_uint8(const multi<T, N>& v)
{
    return fstretch<std::uint8_t>(v);
}

/**
 * @brief Alias for `fstretch<std::uint16_t>()`.
 */
template <typename T, std::size_t N>
__attribute__((always_inline))
inline std::enable_if_t<
       std::is_floating_point<T>::value, 
       multi<std::uint16_t, N>> pack_uint16(const multi<T, N>& v)
{
    return fstretch<std::uint16_t>(v);
}

/**
 * @brief Alias for `fstretch<std::uint32_t>()`.
 */
template <typename T, std::size_t N>
__attribute__((always_inline))
inline std::enable_if_t<
       std::is_floating_point<T>::value, 
       multi<std::uint32_t, N>> pack_uint32(const multi<T, N>& v)
{
    return fstretch<std::uint32_t>(v);
}

/**
 * @brief Alias for `fstretch<std::int8_t>()`.
 */
template <typename T, std::size_t N>
__attribute__((always_inline))
inline std::enable_if_t<
       std::is_floating_point<T>::value, 
       multi<std::int8_t, N>> pack_int8(const multi<T, N>& v)
{
    return fstretch<std::int8_t>(v);
}

/**
 * @brief Alias for `fstretch<std::int16_t>()`.
 */
template <typename T, std::size_t N>
__attribute__((always_inline))
inline std::enable_if_t<
       std::is_floating_point<T>::value, 
       multi<std::int16_t, N>> pack_int16(const multi<T, N>& v)
{
    return fstretch<std::int16_t>(v);
}

/**
 * @brief Alias for `fstretch<std::int32_t>()`.
 */
template <typename T, std::size_t N>
__attribute__((always_inline))
inline std::enable_if_t<
       std::is_floating_point<T>::value, 
       multi<std::int32_t, N>> pack_int32(const multi<T, N>& v)
{
    return fstretch<std::int32_t>(v);
}

/**
 * @brief Alias for `fstretch<float>()`.
 */
template <typename T>
__attribute__((always_inline))
inline std::enable_if_t<
       std::is_integral<T>::value, float> unpack_float(T v)
{
    return fstretch<float>(v);
}

/**
 * @brief Alias for `fstretch<double>()`.
 */
template <typename T>
__attribute__((always_inline))
inline std::enable_if_t<
       std::is_integral<T>::value, double> unpack_double(T v)
{
    return fstretch<double>(v);
}

/**
 * @brief Alias for `fstretch<float>()`.
 */
template <typename T, std::size_t N>
__attribute__((always_inline))
inline std::enable_if_t<
       std::is_integral<T>::value, 
       multi<float, N>> unpack_float(const multi<T, N>& v)
{
    return fstretch<float>(v);
}

/**
 * @brief Alias for `fstretch<double>()`.
 */
template <typename T, std::size_t N>
__attribute__((always_inline))
inline std::enable_if_t<
       std::is_integral<T>::value, 
       multi<double, N>> unpack_double(const multi<T, N>& v)
{
    return fstretch<double>(v);
}

/**@}*/

/**@}*/

} // namespace pr

