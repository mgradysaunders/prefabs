/* Copyright (c) 2018-20 M. Grady Saunders
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
#error "preform/misc_string.hpp requires >= C++14"
#endif // #if !(__cplusplus >= 201402L)
#endif // #if !DOXYGEN
#pragma once
#ifndef PREFORM_MISC_STRING_HPP
#define PREFORM_MISC_STRING_HPP

// for std::snprintf
#include <cstdio>

// for std::toupper, std::tolower
#include <cctype>

// for std::towupper, std::towlower
#include <cwctype>

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

namespace pre {

/**
 * @defgroup misc_string Miscellaneous string
 *
 * `<preform/misc_string.hpp>`
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

#if !DOXYGEN

template <typename Tchar>
struct ci_to_upper;

template <>
struct ci_to_upper<char> {
    __attribute__((always_inline))
    static auto to_upper(char c)
    {
        return std::toupper(static_cast<unsigned char>(c));
    }
};

template <>
struct ci_to_upper<wchar_t> {
    __attribute__((always_inline))
    static auto to_upper(wchar_t c)
    {
        return std::towupper(static_cast<std::wint_t>(c));
    }
};

#endif // #if !DOXYGEN

/**
 * @brief Case-insensitive character traits.
 */
template <typename Tchar>
struct ci_char_traits : public std::char_traits<Tchar>
{
public:

    // Sanity check.
    static_assert(
        std::is_same<Tchar, char>::value ||
        std::is_same<Tchar, wchar_t>::value,
        "Tchar must be char or wchar_t");

    /**
     * @brief Compare equal.
     */
    static bool eq(Tchar c1, Tchar c2)
    {
        return ci_to_upper<Tchar>::to_upper(c1) == 
               ci_to_upper<Tchar>::to_upper(c2);
    }

    /**
     * @brief Compare less-than.
     */
    static bool lt(Tchar c1, Tchar c2)
    {
        return ci_to_upper<Tchar>::to_upper(c1) <
               ci_to_upper<Tchar>::to_upper(c2);
    }

    /**
     * @brief Compare lexicographical.
     */
    static int compare(const Tchar* s1, const Tchar* s2, std::size_t n)
    {
        for (; n-- != 0; ++s1, ++s2) {
            auto c1 = ci_to_upper<Tchar>::to_upper(*s1);
            auto c2 = ci_to_upper<Tchar>::to_upper(*s2);
            if (c1 < c2) { return -1; }
            if (c2 < c1) { return +1; }
        }
        return 0;
    }

    /**
     * @brief Find character in string.
     */
    static const Tchar* find(const Tchar* s, int n, Tchar c)
    {
        for (; n-- > 0; ++s) {
            if (eq(*s, c)) {
                return s;
            }
        }
        return nullptr;
    }
};

/**
 * @brief Case-insensitive string.
 */
using ci_string = std::basic_string<char, ci_char_traits<char>>;

/**
 * @brief Case-insensitive wide string.
 */
using ci_wstring = std::basic_string<wchar_t, ci_char_traits<wchar_t>>;

/**@}*/

} // namespace pre

#endif // #ifndef PREFORM_MISC_STRING_HPP
