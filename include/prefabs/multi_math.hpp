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
#if !(__cplusplus >= 201703L)
#error "prefabs/multi_math.hpp requires >=C++17"
#endif // #if !(__cplusplus >= 201703L)
#endif // #if !DOXYGEN
#pragma once
#ifndef PREFABS_MULTI_MATH_HPP
#define PREFABS_MULTI_MATH_HPP

// for pr::fabs, pr::fmin, ...
#include <prefabs/math.hpp>

// for pr::multi
#include <prefabs/multi.hpp>

namespace pr {

/**
 * @defgroup multi_math Multi-dimensional array (math)
 *
 * `<prefabs/multi_math.hpp>`
 *
 * __C++ version__: >=C++17
 */
/**@{*/

/**@}*/

} // namespace pr

#if !DOXYGEN
#include "multi_math.inl"
#endif // #if !DOXYGEN

namespace pr {

/**
 * @addtogroup multi_math
 */
/**@{*/

/**
 * @brief Dot product.
 */
template <typename T, typename U, std::size_t N>
constexpr decltype(T() * U()) dot(
                        const multi<T, N>& arr0,
                        const multi<U, N>& arr1)
{
    return (arr0 * arr1).sum();
}

/**
 * @brief Dot product.
 */
template <
    typename T, 
    typename U, 
    std::size_t M, 
    std::size_t N
    >
constexpr multi<decltype(T() * U()), M> dot(
                        const multi<T, M, N>& arr0,
                        const multi<U, N>& arr1)
{
    multi<decltype(T() * U()), M> res;
    for (std::size_t i = 0; i < M; i++)
    for (std::size_t k = 0; k < N; k++) {
        res[i] += arr0[i][k] * arr1[k];
    }
    return res;
}

/**
 * @brief Dot product.
 */
template <
    typename T, 
    typename U, 
    std::size_t N, 
    std::size_t P
    >
constexpr multi<decltype(T() * U()), P> dot(
                        const multi<T, N>& arr0,
                        const multi<U, N, P>& arr1)
{
    multi<decltype(T() * U()), P> res;
    for (std::size_t j = 0; j < P; j++)
    for (std::size_t k = 0; k < N; k++) {
        res[j] += arr0[k] * arr1[k][j];
    }
    return res;
}

/**
 * @brief Dot product.
 */
template <
    typename T, 
    typename U, 
    std::size_t M, 
    std::size_t N, 
    std::size_t P
    >
constexpr multi<decltype(T() * U()), M, P> dot(
                        const multi<T, M, N>& arr0,
                        const multi<U, N, P>& arr1)
{
    multi<decltype(T() * U()), M, P> res;
    for (std::size_t i = 0; i < M; i++)
    for (std::size_t j = 0; j < P; j++)
    for (std::size_t k = 0; k < N; k++) {
        res[i][j] += arr0[i][k] * arr1[k][j];
    }
    return res;
}

// TODO kron
// TODO outer
// TODO cross
// TODO reflect
// TODO refract
// TODO length
// TODO normalize
// TODO trace
// TODO transpose
// TODO adjoint

/**@}*/

} // namespace pr

#endif // #ifndef PREFABS_MULTI_MATH_HPP
