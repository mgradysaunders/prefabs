/* Copyright (c) 2018 M. Grady Saunders
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
#if !DOXYGEN
#if !(__cplusplus >= 201402L)
#error "prefabs/endian.hpp requires >=C++14"
#endif // #if !(__cplusplus >= 201402L)
#endif // #if !DOXYGEN
#pragma once
#ifndef PREFABS_ENDIAN_HPP
#define PREFABS_ENDIAN_HPP

// for std::reverse, std::max
#include <algorithm>

// for std::size_t
#include <cstddef>

// for std::memcpy
#include <cstring>

// for std::is_arithmetic, ...
#include <type_traits>

namespace pr {

/**
 * @defgroup endian Endian
 *
 * `<prefabs/endian.hpp>`
 *
 * __C++ version__: >=C++14
 */
/**@{*/

/**
 * @brief Endianness.
 */
enum class endian {
    little,
    big
};

/**
 * @brief Host endianness.
 */
inline endian host_endian()
{
    char c = 0;
    long l = 1;
    std::memcpy(&c, &l, 1);
    return c == 1 ? endian::little : endian::big;
}

#if !DOXYGEN

template <typename T>
struct endian_format_impl
{
    T* st;    
    endian en;
};

template <typename T, typename U>
inline std::enable_if_t<
       std::is_arithmetic<U>::value,
       endian_format_impl<T>> operator>>(
       endian_format_impl<T> is, U& value)
{
    char chars[sizeof(U)] = {};
    is.st->read(&chars[0], sizeof(U));
    if (is.en != host_endian()) {
        // reverse endian
        std::reverse(
                &chars[0],
                &chars[0] + sizeof(U));
    }
    std::memcpy(&value, &chars[0], sizeof(U));
    return is;
}

template <typename T, typename U>
inline std::enable_if_t<
       std::is_arithmetic<U>::value,
       endian_format_impl<T>> operator<<(
       endian_format_impl<T> os, U value)
{
    char chars[sizeof(U)] = {};
    std::memcpy(&chars[0], &value, sizeof(U));
    if (os.en != host_endian()) {
        // reverse endian
        std::reverse(
                &chars[0],
                &chars[0] + sizeof(U));
    }
    os.st->write(&chars[0], sizeof(U));
    return os;
}

// TODO operator>> std::string&
// TODO operator<< const std::string&
// TODO operator<< const char*

#endif // #if !DOXYGEN

/**
 * @brief Endian format.
 *
 * @param[in] st
 * Stream reference.
 *
 * @param[in] en
 * Endianness.
 */
template <typename T> inline auto endian_format(T& st, endian en) 
{
    return endian_format_impl<T>{&st, en};
}

/**@}*/

} // namespace pr

#endif // #ifndef PREFABS_ENDIAN_HPP
