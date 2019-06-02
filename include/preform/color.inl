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

