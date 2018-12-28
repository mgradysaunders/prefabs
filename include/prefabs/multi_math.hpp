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
 * @defgroup multi_math_geometry Multi-dimensional array (math, geometry)
 * 
 * `<prefabs/multi_math.hpp>`
 */
/**@{*/

/**
 * @brief Dot product.
 *
 * @f[
 *      \sum_k x_{0[k]} x_{1[k]}
 * @f]
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
 *
 * @f[
 *      \sum_k x_{0[i,k]} x_{1[k]}
 * @f]
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
 *
 * @f[
 *      \sum_k x_{0[k]} x_{1[k,j]}
 * @f]
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
 *
 * @f[
 *      \sum_k x_{0[i,k]} x_{1[k,j]}
 * @f]
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

/**
 * @brief Outer product.
 *
 * @f[
 *      x_{0[i]} x_{1[j]}
 * @f]
 */
template <
    typename T, 
    typename U, 
    std::size_t M, 
    std::size_t P
    >
constexpr multi<decltype(T() * U()), M, P> outer(
                        const multi<T, M>& arr0,
                        const multi<U, P>& arr1)
{
    multi<decltype(T() * U()), M, P> res;
    for (std::size_t i = 0; i < M; i++)
    for (std::size_t j = 0; j < P; j++) {
        res[i][j] = arr0[i] * arr1[j];
    }
    return res;
}

/**
 * @brief 2-dimensional cross product.
 *
 * @f[
 *      \sum_j \varepsilon_{[i,j]} x_{[j]}
 * @f]
 */
template <
    typename T,
    typename U
    >
constexpr multi<T, 2> cross(const multi<T, 2>& arr)
{
    return {{arr[1], -arr[0]}};
}

/**
 * @brief 2-dimensional cross product.
 *
 * @f[
 *      \sum_{i,j} \varepsilon_{[i,j]} x_{0[i]} x_{1[j]}
 * @f]
 */
template <
    typename T,
    typename U
    >
constexpr decltype(T() * U()) cross(
                        const multi<T, 2>& arr0,
                        const multi<U, 2>& arr1)
{
    return dot(arr0, cross(arr1));
}

/**
 * @brief 3-dimensional cross product.
 *
 * @f[
 *      \sum_k \varepsilon_{[i,j,k]} x_{[k]}
 * @f]
 */
template <typename T>
constexpr multi<T, 3, 3> cross(const multi<T, 3>& arr)
{
    return {{
        {{T(), -arr[2], +arr[1]}},
        {{+arr[2], T(), -arr[0]}},
        {{-arr[1], +arr[0], T()}}
    }};
}

/**
 * @brief 3-dimensional cross product.
 *
 * @f[
 *      \sum_{j,k} \varepsilon_{[i,j,k]} x_{0[j]} x_{1[k]}
 * @f]
 */
template <typename T, typename U>
constexpr multi<decltype(T() * U()), 3> cross(
                        const multi<T, 3>& arr0,
                        const multi<U, 3>& arr1)
{
//  return dot(arr0, cross(arr1));
    return {{
        arr0[1] * arr1[2] - arr0[2] * arr1[1],
        arr0[2] * arr1[0] - arr0[0] * arr1[2],
        arr0[0] * arr1[1] - arr0[1] * arr1[0]
    }};
}

/**
 * @brief 3-dimensional cross product.
 *
 * @f[
 *      \sum_{i,j,k} \varepsilon_{[i,j,k]} x_{0[i]} x_{1[j]} x_{2[k]}
 * @f]
 */
template <typename T, typename U, typename V>
constexpr decltype(T() * U() * V()) cross(
                        const multi<T, 3>& arr0,
                        const multi<U, 3>& arr1,
                        const multi<V, 3>& arr2)
{
    return dot(arr0, cross(arr1, arr2));
}

/**
 * @brief Kronecker product.
 *
 * @f[
 *      x_{0[i/P,j/Q]} x_{1[i\%P,j\%Q]}
 * @f]
 */
template <
    typename T,
    typename U,
    std::size_t M, std::size_t N,
    std::size_t P, std::size_t Q
    >
constexpr multi<decltype(T() * U()), 
                M * P, N * Q> kron(
                        const multi<T, M, N>& arr0,
                        const multi<T, P, Q>& arr1)
{
    multi<decltype(T() * U()), M * P, N * Q> res;
    for (std::size_t i = 0; i < M * P; i++)
    for (std::size_t j = 0; j < N * Q; j++) {
        res[i][j] = arr0[i / P][j / Q] *
                    arr1[i % P][j % Q];
    }
    return res;
}

/**
 * @brief Reflect.
 *
 * @f[
 *      2 (\mathbf{x}_0 \cdot \mathbf{x}_1) \mathbf{x}_1 - \mathbf{x}_0
 * @f]
 */
template <typename T, typename U, std::size_t N>
constexpr multi<decltype(T() * U()), N> reflect(
                        const multi<T, N>& arr0,
                        const multi<U, N>& arr1)
{
    decltype(T() * U()) fac = 
    decltype(T() * U())(2) * dot(arr0, arr1);
    return fac * arr1 - arr0;
}

// TODO refract

/**
 * @brief Length.
 *
 * @f[
 *      \sqrt{\sum_k |x_{[k]}|^2}
 * @f]
 */
template <typename T, std::size_t N>
inline decltype(pr::sqrt(pr::abs(T()))) length(const multi<T, N>& arr)
{
    if constexpr (N == 1) {
        // Delegate.
        return pr::abs(arr[0]);
    }
    else if constexpr (N == 2) {
        // Delegate.
        return pr::hypot(
                    pr::abs(arr[0]), 
                    pr::abs(arr[1]));
    }
    else {

        // Deduce floating point type.
        typedef decltype(pr::sqrt(pr::abs(T()))) float_type;

        // Reduce to floating point absolutes values.
        multi<float_type, N> tmp = 
        multi<float_type, N>(pr::abs(arr));

        // Determine extremal values.
        float_type tmpmin = pr::numeric_limits<float_type>::max();
        float_type tmpmax = 0;
        for (float_type tmpval : tmp) {
            if (tmpval != 0) {
                tmpmin = pr::fmin(tmpmin, tmpval);
                tmpmax = pr::fmax(tmpmax, tmpval);
            }
        }

        // Impending overflow or underflow?
        if (tmpmax * 
            tmpmax >= pr::numeric_limits<float_type>::max() / N ||
            tmpmin <= pr::numeric_limits<float_type>::min_squarable()) {
            // Factor out maximum.
            if (tmpmax >= pr::numeric_limits<float_type>::min_invertible()) {
                tmp *= 1 / tmpmax;
            }
            else {
                tmp /= tmpmax; // Inverse overflows.
            }
            // Length.
            return pr::sqrt((tmp * tmp).sum()) * tmpmax;
        }
        else {
            // Length.
            return pr::sqrt((tmp * tmp).sum());
        }
    }
}

/**
 * @brief Length, fast (and somewhat unsafe) variant.
 *
 * @f[
 *      \sqrt{\sum_k |x_{[k]}|^2}
 * @f]
 *
 * _Fast_ means the implementation
 * - assumes no overflow/underflow,
 * - does _not_ calculate the absolute values of complex numbers
 * separately.
 */
template <typename T, std::size_t N>
__attribute__((always_inline))
inline decltype(pr::sqrt(pr::abs(T()))) fast_length(const multi<T, N>& arr)
{
    if constexpr (N == 1) {
        // Delegate.
        return pr::abs(arr[0]);
    }
    else {

        // Length.
        return pr::sqrt(pr::norm(arr).sum());
    }
}

/**
 * @brief Normalize.
 *
 * @f[
 *      \mathbf{x} / \lVert\mathbf{x}\rVert
 * @f]
 *
 * @note
 * If length is zero, returns the zero vector.
 */
template <typename T, std::size_t N>
inline multi<decltype(T()/pr::sqrt(pr::abs(T()))), N> 
                                    normalize(const multi<T, N>& arr)
{
    // Deduce floating point type.
    typedef decltype(pr::sqrt(pr::abs(T()))) float_type;

    // Length.
    float_type len = length(arr);
    if (len == 0) {
        return {}; // Zero vector.
    }

    // Normalize.
    multi<decltype(T()/pr::sqrt(pr::abs(T()))), N> res =
    multi<decltype(T()/pr::sqrt(pr::abs(T()))), N>(arr);
    if (len >= pr::numeric_limits<float_type>::min_invertible()) {
        res *= 1 / len;
    }
    else {
        res /= len; // Inverse overflows.
    }

    return res;
}

/**
 * @brief Normalize, fast (and somewhat unsafe) variant.
 *
 * @f[
 *      \mathbf{x} / \lVert\mathbf{x}\rVert
 * @f]
 *
 * _Fast_ means the implementation
 * - assumes no overflow/underflow in length calculation,
 * - assumes length is positive (and greater than or equal to 
 * minimum invertible value).
 */
template <typename T, std::size_t N>
__attribute__((always_inline))
inline multi<decltype(T()/pr::sqrt(pr::abs(T()))), N> 
                                    fast_normalize(const multi<T, N>& arr)
{
    // Normalize.
    return arr * (1 / fast_length(arr));
}

/**
 * @brief Trace.
 *
 * @f[
 *      \sum_k x_{[k, k]}
 * @f]
 */
template <typename T, std::size_t M, std::size_t N>
constexpr T trace(const multi<T, M, N>& arr)
{
    T res = arr[0][0];
    for (std::size_t i = 1; i < std::min(M, N); i++) {
        res += arr[i][i];
    }
    return res;
}

/**
 * @brief Transpose.
 *
 * @f[
 *      x_{[j,i]}
 * @f]
 */
template <typename T, std::size_t M, std::size_t N>
constexpr multi<T, N, M> transpose(const multi<T, M, N>& arr)
{
    multi<T, N, M> res;
    for (std::size_t i = 0; i < N; i++)
    for (std::size_t j = 0; j < M; j++) {
        res[i][j] = arr[j][i];
    }
    return res;
}

/**
 * @brief Adjoint.
 *
 * @f[
 *      x_{[j,i]}^*
 * @f]
 */
template <typename T, std::size_t M, std::size_t N>
constexpr multi<T, N, M> adjoint(const multi<T, M, N>& arr)
{
    multi<T, N, M> res;
    for (std::size_t i = 0; i < N; i++)
    for (std::size_t j = 0; j < M; j++) {
        res[i][j] = pr::conj(arr[j][i]);
    }
    return res;
}

/**@}*/

} // namespace pr

#endif // #ifndef PREFABS_MULTI_MATH_HPP
