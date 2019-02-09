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
#if !(__cplusplus >= 201703L)
#error "preform/type_traits.hpp requires >=C++17"
#endif // #if !(__cplusplus >= 201703L)
#endif // #if !DOXYGEN
#pragma once
#ifndef PREFORM_TYPE_TRAITS_HPP
#define PREFORM_TYPE_TRAITS_HPP

#include <type_traits>

namespace pr {

/**
 * @defgroup type_traits Type traits
 *
 * `<preform/type_traits.hpp>`
 *
 * __C++ version__: >=C++17
 */
/**@{*/

/**
 * @brief Value type of.
 */
template <typename T, typename = std::void_t<>>
struct value_type_of
{
    using type = T;
};

/**
 * @brief Value type of specialization.
 */
template <typename T>
struct value_type_of<T, std::void_t<typename T::value_type>>
{
    using type = typename T::value_type;
};

/**
 * @brief Inner-most value type of.
 */
template <typename T, typename = std::void_t<>>
struct inner_value_type_of
{
    using type = T;
};

/**
 * @brief Inner-most value type of specialization.
 */
template <typename T>
struct inner_value_type_of<T, std::void_t<typename T::value_type>>
{
    using type =
            typename inner_value_type_of<
            typename T::value_type>::type;
};

/**@}*/

} // namespace pr

#endif // #ifndef PREFORM_TYPE_TRAITS_HPP
