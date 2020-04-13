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
#error "preform/byte_order.hpp requires >=C++14"
#endif // #if !(__cplusplus >= 201402L)
#endif // #if !DOXYGEN
#pragma once
#ifndef PREFORM_BYTE_ORDER_HPP
#define PREFORM_BYTE_ORDER_HPP

// for std::reverse, std::max
#include <algorithm>

// for std::size_t
#include <cstddef>

// for std::memcpy
#include <cstring>

// for std::basic_string
#include <string>

// for std::reference_wrapper
#include <functional>

// for std::is_arithmetic, ...
#include <type_traits>

namespace pre {

/**
 * @defgroup byte_order Byte order
 *
 * `<preform/byte_order.hpp>`
 *
 * __C++ version__: >=C++14
 */
/**@{*/

/**
 * @brief Byte order.
 */
enum class byte_order {

    /**
     * @brief Little endian.
     */
    little,

    /**
     * @brief Big endian.
     */
    big
};

/**
 * @brief Byte order to string.
 */
constexpr const char* to_string(byte_order order)
{
    switch (order) {
        case byte_order::little: return "little";
        case byte_order::big:    return "big";
        default: break;
    }
    return "unknown";
}

/**
 * @brief Host byte order.
 */
inline byte_order host_byte_order()
{
    unsigned char uc = 0;
    unsigned long ul = 1;
    std::memcpy(&uc, &ul, 1);
    return uc == 1 ? byte_order::little : byte_order::big;
}

/**
 * @brief Byte stream wrapper.
 *
 * @param[in] Tstream
 * Stream type.
 */
template <typename Tstream>
struct byte_stream_wrapper
{
public:

    /**
     * @brief Stream char type.
     */
    typedef typename Tstream::char_type char_type;

    // Sanity check.
    static_assert(
         sizeof(char_type) == 1,
        "sizeof(char_type) must be 1");

    /**
     * @brief Constructor.
     *
     * @param[in] ref
     * Stream reference.
     *
     * @param[in] rev
     * Stream byte order reversed?
     */
    byte_stream_wrapper(Tstream& ref, bool rev) :
            ref_(ref),
            rev_(rev)
    {
    }

public:

    /**
     * @brief Write arithmetic value.
     */
    template <typename Tvalue>
    std::enable_if_t<
    std::is_arithmetic<Tvalue>::value,
            byte_stream_wrapper> operator<<(Tvalue value)
    {
        if (rev_ == false || sizeof(Tvalue) == 1) {
            ref_.get().write(
                static_cast<const char_type*>(
                static_cast<const void*>(&value)), sizeof(Tvalue));
        }
        else {
            // Reverse byte order.
            char_type bytes[sizeof(Tvalue)];
            std::memcpy(
                    &bytes[0],
                    &value,
                    sizeof(Tvalue));
            std::reverse(
                    &bytes[0],
                    &bytes[0] + sizeof(Tvalue));
            ref_.get().write(&bytes[0], sizeof(Tvalue));
        }
        return *this;
    }

    /**
     * @brief Write arithmetic value array.
     */
    template <typename Tvalue, std::size_t N>
    std::enable_if_t<
    std::is_arithmetic<Tvalue>::value,
            byte_stream_wrapper> operator<<(const Tvalue (&values)[N])
    {
        if (rev_ == false || sizeof(Tvalue) == 1) {
            ref_.get().write(
                static_cast<const char_type*>(
                static_cast<const void*>(&values[0])), sizeof(Tvalue) * N);
        }
        else {
            // Reverse byte order.
            char_type bytes[sizeof(Tvalue) * N];
            std::memcpy(
                &bytes[0],
                &values[0],
                sizeof(Tvalue) * N);
            for (std::size_t k = 0; k < N; k++) {
                std::reverse(
                        &bytes[0] + k * sizeof(Tvalue),
                        &bytes[0] + k * sizeof(Tvalue) + sizeof(Tvalue));
            }
            ref_.get().write(&bytes[0], sizeof(Tvalue) * N);
        }
        return *this;
    }

    /**
     * @brief Write arithmetic value array.
     *
     * @throw std::invalid_argument
     * If `values` is null and `count` is non-zero.
     */
    template <typename Tvalue>
    std::enable_if_t<
    std::is_arithmetic<Tvalue>::value,
            byte_stream_wrapper> write(
                        const Tvalue* values, std::size_t count)
    {
        if (values == nullptr) {
            if (count == 0) {
                return *this;
            }
            else {
                throw std::invalid_argument(__PRETTY_FUNCTION__);
            }
        }
        if (rev_ == false || sizeof(Tvalue) == 1) {
            ref_.get().write(
                static_cast<const char_type*>(
                static_cast<const void*>(&values[0])), sizeof(Tvalue) * count);
        }
        else {
            // Reverse byte order.
            char_type bytes[sizeof(Tvalue)];
            for (const Tvalue* value = values;
                               value < values + count; value++) {
                std::memcpy(
                        &bytes[0],
                        &value[0],
                        sizeof(Tvalue));
                std::reverse(
                        &bytes[0],
                        &bytes[0] + sizeof(Tvalue));
                ref_.get().write(&bytes[0], sizeof(Tvalue));
            }
        }
        return *this;
    }

    /**
     * @brief Read arithmetic value.
     */
    template <typename Tvalue>
    std::enable_if_t<
    std::is_arithmetic<Tvalue>::value,
            byte_stream_wrapper> operator>>(Tvalue& value)
    {
        if (rev_ == false || sizeof(Tvalue) == 1) {
            ref_.get().read(
                static_cast<char_type*>(
                static_cast<void*>(&value)), sizeof(Tvalue));
        }
        else {
            // Reverse byte order.
            char_type bytes[sizeof(Tvalue)];
            ref_.get().read(&bytes[0], sizeof(Tvalue));
            std::reverse(
                    &bytes[0],
                    &bytes[0] + sizeof(Tvalue));
            std::memcpy(
                    &value,
                    &bytes[0],
                    sizeof(Tvalue));
        }
        return *this;
    }

    /**
     * @brief Read arithmetic value array.
     */
    template <typename Tvalue, std::size_t N>
    std::enable_if_t<
    std::is_arithmetic<Tvalue>::value,
            byte_stream_wrapper> operator>>(Tvalue (&values)[N])
    {
        if (rev_ == false || sizeof(Tvalue) == 1) {
            ref_.get().read(
                static_cast<char_type*>(
                static_cast<void*>(&values[0])), sizeof(Tvalue) * N);
        }
        else {
            // Reverse byte order.
            char_type bytes[sizeof(Tvalue) * N];
            ref_.get().read(&bytes[0], sizeof(Tvalue) * N);
            for (std::size_t k = 0; k < N; k++) {
                std::reverse(
                        &bytes[0] + k * sizeof(Tvalue),
                        &bytes[0] + k * sizeof(Tvalue) + sizeof(Tvalue));
            }
            std::memcpy(
                    &values[0],
                    &bytes[0],
                    sizeof(Tvalue) * N);
        }
        return *this;
    }

    /**
     * @brief Read arithmetic value array.
     *
     * @throw std::invalid_argument
     * If `values` is null and `count` is non-zero.
     */
    template <typename Tvalue>
    std::enable_if_t<
    std::is_arithmetic<Tvalue>::value,
            byte_stream_wrapper> read(
                        Tvalue* values, std::size_t count)
    {
        if (values == nullptr) {
            if (count == 0) {
                return *this;
            }
            else {
                throw std::invalid_argument(__PRETTY_FUNCTION__);
            }
        }
        if (rev_ == false || sizeof(Tvalue) == 1) {
            ref_.get().read(
                static_cast<char_type*>(
                static_cast<void*>(&values[0])), sizeof(Tvalue) * count);
        }
        else {
            // Reverse byte order.
            char_type bytes[sizeof(Tvalue)];
            for (Tvalue* value = values;
                         value < values + count; value++) {
                ref_.get().read(&bytes[0], sizeof(Tvalue));
                std::reverse(
                        &bytes[0],
                        &bytes[0] + sizeof(Tvalue));
                std::memcpy(
                        &value[0],
                        &bytes[0],
                        sizeof(Tvalue));
            }
        }
        return *this;
    }

    /**
     * @brief Write zero-terminated string.
     *
     * @throw std::invalid_argument
     * If null.
     */
    template <typename Tchar>
    std::enable_if_t<
    std::is_integral<Tchar>::value,
            byte_stream_wrapper> operator<<(const Tchar* cstr)
    {
        if (cstr == nullptr) {
            throw std::invalid_argument(__PRETTY_FUNCTION__);
        }
        for (;;) {
            *this << *cstr;
            if (*cstr++ == Tchar(0)) {
                break;
            }
        }
        return *this;
    }

    /**
     * @brief Write zero-terminated string.
     */
    template <
        typename Tchar,
        typename Tchar_traits,
        typename Tchar_alloc
        >
    byte_stream_wrapper operator<<(
            const std::basic_string<Tchar, Tchar_traits, Tchar_alloc>& str)
    {
        return *this << str.c_str();
    }

    /**
     * @brief Read zero-terminated string.
     */
    template <
        typename Tchar,
        typename Tchar_traits,
        typename Tchar_alloc
        >
    byte_stream_wrapper operator>>(
            std::basic_string<Tchar, Tchar_traits, Tchar_alloc>& str)
    {
        str.clear();
        while (ref_.get().good()) {
            Tchar c;
            *this >> c;
            if (c != Tchar(0)) {
                str.push_back(c);
            }
            else {
                break;
            }
        }
        return *this;
    }

private:

    /**
     * @brief Stream reference.
     */
    std::reference_wrapper<Tstream> ref_;

    /**
     * @brief Stream byte order reversed?
     */
    bool rev_ = false;
};

/**
 * @brief Make byte stream.
 *
 * @param[in] ref
 * Stream reference.
 *
 * @param[in] rev
 * Stream byte order reversed?
 */
template <typename Tstream>
inline byte_stream_wrapper<Tstream> byte_stream(Tstream& ref, bool rev)
{
    return {ref, rev};
}

/**
 * @brief Make byte stream.
 *
 * @param[in] ref
 * Stream reference.
 *
 * @param[in] ord
 * Stream byte order, reversed if not host byte order.
 */
template <typename Tstream>
inline byte_stream_wrapper<Tstream> byte_stream(Tstream& ref, byte_order ord)
{
    return {ref, ord != host_byte_order()};
}

/**@}*/

} // namespace pre

#endif // #ifndef PREFORM_BYTE_ORDER_HPP
