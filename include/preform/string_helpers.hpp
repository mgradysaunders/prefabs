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
#if !DOXYGEN
#if !(__cplusplus >= 201402L)
#error "preform/string_helpers.hpp requires >= C++14"
#endif // #if !(__cplusplus >= 201402L)
#endif // #if !DOXYGEN
#pragma once
#ifndef PREFORM_STRING_HELPERS_HPP
#define PREFORM_STRING_HELPERS_HPP

// for std::snprintf
#include <cstdio>

// for std::string
#include <string>

// for std::runtime_error
#include <stdexcept>

// for std::forward
#include <utility>

// for std::type_info
#include <typeinfo>

// for abi::__cxa_demangle
#if __GNUC__
#include <cxxabi.h>
#endif // #if __GNUC__

namespace pr {

/**
 * @defgroup string_helpers String helpers
 *
 * `<preform/string_helpers.hpp>`
 *
 * __C++ version__: >=C++14
 */
/**@{*/

/**
 * @brief String from printf.
 */
template <typename... Targs>
inline std::string str_printf(const char* fmt, Targs&&... args)
{
    // Initialize.
    std::string str;
    str.resize(
        1 + 
        std::snprintf(
            NULL, 0, fmt, 
            std::forward<Targs>(args)...));

    // Print.
    std::snprintf(
        &str[0], str.size(), fmt,
        std::forward<Targs>(args)...);
    return str;
}

/**
 * @brief String from typename.
 */
template <typename T>
inline std::string str_typename()
{
#if __GNUC__
    std::string str;
    int status;
    char* chars = abi::__cxa_demangle(typeid(T).name(), 0, 0, &status);
    str = chars;
    free(chars);
    if (status != 0) {
        throw 
            std::runtime_error(
            std::string("abi::__cxa_demangle status = ")
                .append(std::to_string(status)));
    }
    return str;
#else
    return std::string(typeid(T).name());
#endif // #if __GNUC__
}

/**@}*/

} // namespace pr

#endif // #ifndef PREFORM_STRING_HELPERS_HPP
