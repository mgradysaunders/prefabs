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
#error "preform/multi_math.hpp requires >=C++17"
#endif // #if !(__cplusplus >= 201703L)
#endif // #if !DOXYGEN
#pragma once
#ifndef PREFORM_MULTI_MATH_HPP
#define PREFORM_MULTI_MATH_HPP

// for pr::fabs, pr::fmin, ...
#include <preform/math.hpp>

// for pr::multi
#include <preform/multi.hpp>

namespace pr {

/**
 * @defgroup multi_math Multi-dimensional array (math)
 *
 * `<preform/multi_math.hpp>`
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
 * `<preform/multi_math.hpp>`
 *
 * __C++ version__: >=C++17
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
                        const multi<T, N>& arr0, const multi<U, N>& arr1)
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
                        const multi<T, M, N>& arr0, const multi<U, N>& arr1)
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
                        const multi<T, N>& arr0, const multi<U, N, P>& arr1)
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
                        const multi<T, M, N>& arr0, const multi<U, N, P>& arr1)
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
                        const multi<T, M>& arr0, const multi<U, P>& arr1)
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
    return {arr[1], -arr[0]};
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
                        const multi<T, 2>& arr0, const multi<U, 2>& arr1)
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
    return {
        {T(), -arr[2], +arr[1]},
        {+arr[2], T(), -arr[0]},
        {-arr[1], +arr[0], T()}
    };
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
                        const multi<T, 3>& arr0, const multi<U, 3>& arr1)
{
//  return dot(arr0, cross(arr1));
    return {
        arr0[1] * arr1[2] - arr0[2] * arr1[1],
        arr0[2] * arr1[0] - arr0[0] * arr1[2],
        arr0[0] * arr1[1] - arr0[1] * arr1[0]
    };
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
                        const multi<T, 3>& arr0, const multi<U, 3>& arr1,
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
 * @brief @f$ L^2 @f$ length, fast (and somewhat unsafe) variant.
 *
 * @f[
 *      \lVert\mathbf{x}\rVert = \sqrt{\sum_k |x_{[k]}|^2}
 * @f]
 *
 * _Fast_ means the implementation
 * - calculates the sum of square moduli directly, implicitly assuming
 * the square terms neither overflow nor underflow, and the sum of square 
 * terms does not overflow.
 */
template <typename T, std::size_t N>
__attribute__((always_inline))
inline decltype(pr::sqrt(pr::abs(T()))) length_fast(const multi<T, N>& arr)
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
 * @brief @f$ L^2 @f$ length, safe variant.
 *
 * @f[
 *      \lVert\mathbf{x}\rVert = \sqrt{\sum_k |x_{[k]}|^2}
 * @f]
 *
 * _Safe_ means the implementation
 * - calculates the moduli as a preprocessing step,
 * - if impending overflow or underflow, factors the maximum modulus out 
 * from under the radical.
 */
template <typename T, std::size_t N>
inline decltype(pr::sqrt(pr::abs(T()))) length_safe(const multi<T, N>& arr)
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
            return pr::sqrt(dot(tmp, tmp)) * tmpmax;
        }
        else {
            // Length.
            return pr::sqrt(dot(tmp, tmp));
        }
    }
}

/**
 * @brief @f$ L^2 @f$ length.
 *
 * @f[
 *      \lVert\mathbf{x}\rVert = \sqrt{\sum_k |x_{[k]}|^2}
 * @f]
 *
 * @note
 * By default, calls `length_safe()`. 
 * Define `PR_DEFAULT_LENGTH_FAST` before including to call
 * `length_fast()`.
 *
 * @see `length_safe()`
 * @see `length_fast()`
 */
template <typename T, std::size_t N>
inline decltype(pr::sqrt(pr::abs(T()))) length(const multi<T, N>& arr)
{
#if PR_DEFAULT_LENGTH_FAST
    return length_fast(arr);
#else
    return length_safe(arr);
#endif // #if PR_DEFAULT_LENGTH_FAST
}

/**
 * @brief @f$ L^2 @f$ normalize, fast (and somewhat unsafe) variant.
 *
 * @f[
 *      \hat{\mathbf{x}} = \frac{\mathbf{x}}{\lVert\mathbf{x}\rVert}
 * @f]
 *
 * _Fast_ means the implementation
 * - calls `length_fast()` to calculate length,
 * - multiplies by reciprocal of length, implicitly assuming 
 * reciprocal does not overflow.
 *
 * @see `length_fast()`
 */
template <typename T, std::size_t N>
__attribute__((always_inline))
inline multi<decltype(T()/pr::sqrt(pr::abs(T()))), N>
                                    normalize_fast(const multi<T, N>& arr)
{
    // Normalize.
    return arr * (1 / length_fast(arr));
}

/**
 * @brief @f$ L^2 @f$ normalize, safe variant.
 *
 * @f[
 *      \hat{\mathbf{x}} = \frac{\mathbf{x}}{\lVert\mathbf{x}\rVert}
 * @f]
 *
 * _Safe_ means the implementation
 * - calls `length_safe()` to calculate length,
 * - if length is zero, returns array of zeros, 
 * - if length is less than the minimum invertible floating point value,
 * divides by length instead of multiplying by reciprocal.
 *
 * @see `length_safe()`
 */
template <typename T, std::size_t N>
inline multi<decltype(T()/pr::sqrt(pr::abs(T()))), N>
                                    normalize_safe(const multi<T, N>& arr)
{
    // Deduce floating point type.
    typedef decltype(pr::sqrt(pr::abs(T()))) float_type;

    // Length.
    float_type len = length_safe(arr);
    if (len == 0) {
        return {}; // Zero vector.
    }

    // Normalize.
    multi<decltype(T()/float_type()), N> res =
    multi<decltype(T()/float_type()), N>(arr);
    if (len >= pr::numeric_limits<float_type>::min_invertible()) {
        res *= 1 / len;
    }
    else {
        res /= len; // Inverse overflows.
    }

    return res;
}

/**
 * @brief @f$ L^2 @f$ normalize.
 *
 * @f[
 *      \hat{\mathbf{x}} = \frac{\mathbf{x}}{\lVert\mathbf{x}\rVert}
 * @f]
 *
 * @note
 * By default, calls `normalize_safe()`. 
 * Define `PR_DEFAULT_NORMALIZE_FAST` before including to call
 * `normalize_fast()`.
 *
 * @see `normalize_fast()`
 * @see `normalize_safe()`
 */
template <typename T, std::size_t N>
inline multi<decltype(T()/pr::sqrt(pr::abs(T()))), N>
                                    normalize(const multi<T, N>& arr)
{
#if PR_DEFAULT_NORMALIZE_FAST
    return normalize_fast(arr);
#else
    return normalize_safe(arr);
#endif // #if PR_DEFAULT_NORMALIZE_FAST
}

/**
 * @brief Trace.
 *
 * @f[
 *      \operatorname{Tr}(\mathbf{x}) = \sum_k x_{[k, k]}
 * @f]
 */
template <
    typename T, 
    std::size_t M, 
    std::size_t N
    >
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
 *      x_{[i,j]} \to x_{[j,i]}
 * @f]
 */
template <
    typename T, 
    std::size_t M, 
    std::size_t N
    >
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
 *      x_{[i,j]} \to x_{[j,i]}^*
 * @f]
 */
template <
    typename T, 
    std::size_t M, 
    std::size_t N
    >
constexpr multi<T, N, M> adjoint(const multi<T, M, N>& arr)
{
    multi<T, N, M> res;
    for (std::size_t i = 0; i < N; i++)
    for (std::size_t j = 0; j < M; j++) {
        res[i][j] = pr::conj(arr[j][i]);
    }
    return res;
}

/**
 * @brief Build real 3-dimensional orthonormal basis.
 *
 * - @f$ \alpha_0 \gets -1 / (\hat{z}_{[2]} + 1) @f$
 * - @f$ \alpha_1 \gets \alpha_0 \hat{z}_{[0]} \hat{z}_{[1]} @f$
 * - @f$ \alpha_2 \gets \alpha_0 \hat{z}_{[0]}^2 + 1 @f$
 * - @f$ \alpha_3 \gets \alpha_0 \hat{z}_{[1]}^2 + 1 @f$
 * - @f$ \hat{\mathbf{x}} \gets [\alpha_2\; \alpha_1\; -\hat{z}_{[0]}]^\top @f$
 * - @f$ \hat{\mathbf{y}} \gets [\alpha_1\; \alpha_3\; -\hat{z}_{[1]}]^\top @f$
 *
 * @note
 * As the notation suggests, the implementation assumes the input
 * vector `hatz` is unit-length.
 */
template <typename T>
inline std::enable_if_t<
       std::is_floating_point<T>::value, void> build_onb(
                    const multi<T, 3>& hatz,
                    multi<T, 3>& hatx,
                    multi<T, 3>& haty)
{
    hatx = {};
    haty = {};
    if (hatz[2] < T(-0.9999999)) {
        hatx[1] = -1;
        haty[0] = -1;
    }
    else {
        T alpha0 = -1 / (hatz[2] + 1);
        T alpha1 = alpha0 * hatz[0] * hatz[1];
        T alpha2 = alpha0 * hatz[0] * hatz[0] + 1;
        T alpha3 = alpha0 * hatz[1] * hatz[1] + 1;
        hatx = {alpha2, alpha1, -hatz[0]};
        haty = {alpha1, alpha3, -hatz[1]};
    }
}

/**
 * @brief Build real 3-dimensional orthonormal basis.
 *
 * @f[
 *      \mathbf{B} \gets
 *      \begin{bmatrix}
 *          \hat{\mathbf{x}} &
 *          \hat{\mathbf{y}} &
 *          \hat{\mathbf{z}}
 *      \end{bmatrix}
 * @f]
 *
 * @note
 * As the notation suggests, the implementation assumes the input
 * vector `hatz` is unit-length.
 */
template <typename T>
inline std::enable_if_t<
       std::is_floating_point<T>::value,
                multi<T, 3, 3>> build_onb(const multi<T, 3>& hatz)
{
    multi<T, 3> hatx;
    multi<T, 3> haty;
    build_onb(hatz, hatx, haty);
    return {
        {hatx[0], haty[0], hatz[0]},
        {hatx[1], haty[1], hatz[1]},
        {hatx[2], haty[2], hatz[2]}
    };
}

/**@}*/

/**
 * @defgroup multi_math_sampling Multi-dimensional array (math, sampling)
 *
 * `<preform/multi_math.hpp>`
 *
 * __C++ version__: >=C++17
 */
/**@{*/

/**
 * @brief Uniform disk probability density function.
 *
 * @f[
 *      f_{\text{disk}} = \frac{1}{\pi}
 * @f]
 */
template <typename T>
__attribute__((always_inline))
inline std::enable_if_t<
       std::is_floating_point<T>::value, T> uniform_disk_pdf()
{
    return pr::numeric_constants<T>::M_1_pi();
}

/**
 * @brief Uniform disk probability density function sampling routine.
 *
 * @f[
 *      \mathbf{P}_{\text{disk}}(\mathbf{u}) =
 *      \begin{bmatrix}
 *          r \cos{\theta}
 *      \\  r \sin{\theta}
 *      \end{bmatrix}
 * @f]
 * where
 * - @f$ u_{[0]}' \gets 2 u_{[0]} - 1 @f$
 * - If @f$ |u_{[0]}'| > |u_{[1]}| @f$:
 * @f[
 *      (r, \theta) =
 *          \left(u_{[0]}',
 *          \frac{\pi}{4}\frac{u_{[1]}}{u_{[0]}'}\right)
 * @f]
 * - If @f$ |u_{[0]}'| \le |u_{[1]}| @f$:
 * @f[
 *      (r, \theta) =
 *          \left(u_{[1]},
 *          \frac{\pi}{2} -
 *          \frac{\pi}{4}\frac{u_{[0]}'}{u_{[1]}}\right)
 * @f]
 *
 * @param[in] u
 * Sample in @f$ [0, 1)^2 @f$.
 */
template <typename T>
inline std::enable_if_t<
       std::is_floating_point<T>::value,
                multi<T, 2>> uniform_disk_pdf_sample(multi<T, 2> u)
{
    u = 2 * u - 1;
    if (u[0] == T(0) &&
        u[1] == T(0)) {
        return u;
    }
    else {
        T r;
        T theta;
        if (pr::fabs(u[0]) > pr::fabs(u[1])) {
            r = u[0];
            theta = pr::numeric_constants<T>::M_pi_4() * (u[1] / u[0]);
        }
        else {
            r = u[1];
            theta = pr::numeric_constants<T>::M_pi_4() * (u[0] / u[1]);
            theta = pr::numeric_constants<T>::M_pi_2() - theta;
        }
        return {
            r * pr::cos(theta),
            r * pr::sin(theta)
        };
    }
}

/**
 * @brief Uniform hemisphere probability density function.
 *
 * @f[
 *      f_{\text{hemisphere}} = \frac{1}{2\pi}
 * @f]
 */
template <typename T>
__attribute__((always_inline))
inline std::enable_if_t<
       std::is_floating_point<T>::value, T> uniform_hemisphere_pdf()
{
    return pr::numeric_constants<T>::M_1_pi() / 2;
}

/**
 * @brief Uniform hemisphere probability density function sampling routine.
 *
 * @f[
 *      \omega_{\text{hemisphere}}(\mathbf{u}) =
 *      \begin{bmatrix}
 *          \sqrt{1 - u_{[0]}^2} \cos(2\pi u_{[1]})
 *      \\  \sqrt{1 - u_{[0]}^2} \sin(2\pi u_{[1]})
 *      \\  u_{[0]}
 *      \end{bmatrix}
 * @f]
 *
 * @param[in] u
 * Sample in @f$ [0, 1)^2 @f$.
 */
template <typename T>
inline std::enable_if_t<
       std::is_floating_point<T>::value,
                multi<T, 3>> uniform_hemisphere_pdf_sample(multi<T, 2> u)
{
    T cos_theta = u[0];
    cos_theta = pr::fmax(cos_theta, T(0));
    cos_theta = pr::fmin(cos_theta, T(1));
    T sin_theta = pr::sqrt(1 - cos_theta * cos_theta);
    T phi = 2 * pr::numeric_constants<T>::M_pi() * u[1];
    return {
        sin_theta * pr::cos(phi),
        sin_theta * pr::sin(phi),
        cos_theta
    };
}

/**
 * @brief Uniform sphere probability density function.
 *
 * @f[
 *      f_{\text{sphere}} = \frac{1}{4\pi}
 * @f]
 */
template <typename T>
__attribute__((always_inline))
inline std::enable_if_t<
       std::is_floating_point<T>::value, T> uniform_sphere_pdf()
{
    return pr::numeric_constants<T>::M_1_pi() / T(4);
}

/**
 * @brief Uniform sphere probability density function sampling routine.
 *
 * @f[
 *      \omega_{\text{sphere}}(\mathbf{u}) =
 *      \begin{bmatrix}
 *          \sqrt{1 - (2u_{[0]} - 1)^2} \cos(2\pi u_{[1]})
 *      \\  \sqrt{1 - (2u_{[0]} - 1)^2} \sin(2\pi u_{[1]})
 *      \\  2u_{[0]} - 1
 *      \end{bmatrix}
 * @f]
 *
 * @param[in] u
 * Sample in @f$ [0, 1)^2 @f$.
 */
template <typename T>
inline std::enable_if_t<
       std::is_floating_point<T>::value,
                multi<T, 3>> uniform_sphere_pdf_sample(multi<T, 2> u)
{
    T cos_theta = 2 * u[0] - 1;
    cos_theta = pr::fmax(cos_theta, T(-1));
    cos_theta = pr::fmin(cos_theta, T(+1));
    T sin_theta = pr::sqrt(1 - cos_theta * cos_theta);
    T phi = 2 * pr::numeric_constants<T>::M_pi() * u[1];
    return {
        sin_theta * pr::cos(phi),
        sin_theta * pr::sin(phi),
        cos_theta
    };
}

/**
 * @brief Cosine hemisphere probability density function.
 *
 * @f[
 *      f_{\text{cosine}}(\omega_{[2]}) = \frac{\omega_{[2]}}{\pi}
 * @f]
 *
 * @param[in] w2
 * Direction component.
 */
template <typename T>
__attribute__((always_inline))
inline std::enable_if_t<
       std::is_floating_point<T>::value, T> cosine_hemisphere_pdf(T w2)
{
    return pr::numeric_constants<T>::M_1_pi() * w2;
}

/**
 * @brief Cosine hemisphere probability density function sampling routine.
 *
 * @f[
 *      \omega_{\text{cosine}}(\mathbf{u}) =
 *      \begin{bmatrix}
 *          P_{[0]}
 *      \\  P_{[1]}
 *      \\ \sqrt{1 - P_{[0]}^2 - P_{[1]}^2}
 *      \end{bmatrix}
 * @f]
 * where
 * @f[
 *      \mathbf{P} = \mathbf{P}_{\text{disk}}(\mathbf{u})
 * @f]
 *
 * @param[in] u
 * Sample in @f$ [0, 1)^2 @f$.
 */
template <typename T>
inline std::enable_if_t<
       std::is_floating_point<T>::value,
                multi<T, 3>> cosine_hemisphere_pdf_sample(multi<T, 2> u)
{
    multi<T, 2> p = uniform_disk_pdf_sample(u);
    return {
        p[0],
        p[1],
        pr::sqrt(T(1) - pr::fmin(dot(p, p), T(1)))
    };
}

/**
 * @brief Uniform cone probability density function.
 *
 * @f[
 *      f_{\text{cone}} =
 *      \frac{1}{2\pi} \frac{1}{1 - \cos(\theta_{\max})}
 * @f]
 *
 * @param[in] cos_thetamax
 * Cosine of cone angle maximum.
 */
template <typename T>
__attribute__((always_inline))
inline std::enable_if_t<
       std::is_floating_point<T>::value, T> uniform_cone_pdf(T cos_thetamax)
{
    return pr::numeric_constants<T>::M_1_pi() / 2 / (1 - cos_thetamax);
}

/**
 * @brief Uniform cone probability density function sampling routine.
 *
 * @f[
 *      \omega_{\text{cone}}(\mathbf{u}) =
 *      \begin{bmatrix}
 *          \sqrt{1-(1-u_{[0]}+u_{[0]}\cos(\theta_{\max}))^2}\cos(2\pi u_{[1]})
 *      \\  \sqrt{1-(1-u_{[0]}+u_{[0]}\cos(\theta_{\max}))^2}\sin(2\pi u_{[1]})
 *      \\  1-u_{[0]}+u_{[0]}\cos(\theta_{\max})
 *      \end{bmatrix}
 * @f]
 *
 * @param[in] cos_thetamax
 * Cosine of cone angle maximum.
 *
 * @param[in] u
 * Sample in @f$ [0, 1)^2 @f$.
 */
template <typename T>
inline std::enable_if_t<
       std::is_floating_point<T>::value,
                multi<T, 3>> uniform_cone_pdf_sample(
                                        T cos_thetamax, multi<T, 2> u)
{
    T cos_theta = (1 - u[0]) + u[0] * cos_thetamax;
    cos_theta = pr::fmax(cos_theta, T(-1));
    cos_theta = pr::fmin(cos_theta, T(+1));
    T sin_theta = pr::sqrt(1 - cos_theta * cos_theta);
    T phi = 2 * pr::numeric_constants<T>::M_pi() * u[1];
    return {
        sin_theta * pr::cos(phi),
        sin_theta * pr::sin(phi),
        cos_theta
    };
}

/**
 * @brief Henyey-Greenstein phase probability density function.
 *
 * @f[
 *      f_{\text{HG}}(g; \omega_{[2]}) =
 *      \frac{1}{4\pi}\frac{1 - g^2}{(1 + g^2 - 2g\omega_{[2]})^{3/2}}
 * @f]
 *
 * @param[in] g
 * Parameter in @f$ (-1, 1) @f$.
 *
 * @param[in] w2
 * Direction component.
 */
template <typename T>
inline std::enable_if_t<
       std::is_floating_point<T>::value, T> hg_phase_pdf(T g, T w2)
{
    if (pr::fabs(g) < T(0.00001)) {
        return uniform_sphere_pdf<T>();
    }
    else {
        T a = 1 - g * g;
        T b = 1 + g * g - 2 * g * w2;
        T b3_2 = pr::sqrt(b * b * b);
        return T(0.25) * pr::numeric_constants<T>::M_1_pi() * (a / b3_2);
    }
}

/**
 * @brief Henyey-Greenstein phase probability density function
 * sampling routine.
 *
 * @param[in] g
 * Parameter in @f$ (-1, 1) @f$.
 *
 * @param[in] u
 * Sample in @f$ [0, 1)^2 @f$.
 */
template <typename T>
inline std::enable_if_t<
       std::is_floating_point<T>::value,
                multi<T, 3>> hg_phase_pdf_sample(T g, multi<T, 2> u)
{
    if (pr::fabs(g) < T(0.00001)) {
        return uniform_sphere_pdf_sample(u);
    }
    else {
        T tmp = (1 - g * g) / (1 - g + 2 * g * u[0]);
        T cos_theta = (1 + g * g - tmp * tmp) / (2 * g);
        cos_theta = pr::fmax(cos_theta, T(-1));
        cos_theta = pr::fmin(cos_theta, T(+1));
        T sin_theta = pr::sqrt(T(1) - cos_theta * cos_theta);
        T phi = 2 * pr::numeric_constants<T>::M_pi() * u[1];
        return {
            sin_theta * pr::cos(phi),
            sin_theta * pr::sin(phi),
            cos_theta
        };
    }
}

/**@}*/

} // namespace pr

#endif // #ifndef PREFORM_MULTI_MATH_HPP
