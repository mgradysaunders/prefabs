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
template <typename T>
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
template <typename T, typename U>
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
 * @brief @f$ L^2 @f$ length, fast variant.
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
 * @brief @f$ L^2 @f$ normalize, fast variant.
 *
 * @f[
 *      \hat{\mathbf{x}} = \frac{\mathbf{x}}{\lVert\mathbf{x}\rVert}
 * @f]
 *
 * _Fast_ means the implementation
 * - calls `length_fast()` to calculate length,
 * - multiplies by reciprocal of length, implicitly assuming 
 * reciprocal does not overflow.
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
 * - if length is zero, returns all zeros, 
 * - if length is less than the minimum invertible floating point value,
 * divides by length instead of multiplying by reciprocal.
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
 * @brief Inverse by Cramer's rule.
 *
 * @f[
 *      \mathbf{x}^{-1} = 
 *      \frac{\operatorname{cof}^\top(\mathbf{x})}
 *           {\operatorname{det}(\mathbf{x})}
 * @f]
 *
 * @note
 * This method is unstable if the matrix is ill-conditioned.
 */
template <typename T>
inline multi<decltype(T()/pr::sqrt(pr::abs(T()))), 2, 2>
                inverse(const multi<T, 2, 2>& arr)
{
    // Deduce floating point type.
    typedef decltype(pr::sqrt(pr::abs(T()))) float_type;

    // Cofactor matrix.
    multi<decltype(T()/float_type()), 2, 2> cof = {
        {+arr[1][1], -arr[1][0]},
        {-arr[0][1], +arr[0][0]},
    };

    // Cofactor transpose over determinant.
    return transpose(cof) * (float_type(1) / dot(cof[0], arr[0]));
}

/**
 * @brief Inverse by Cramer's rule.
 *
 * @f[
 *      \mathbf{x}^{-1} = 
 *      \frac{\operatorname{cof}^\top(\mathbf{x})}
 *           {\operatorname{det}(\mathbf{x})}
 * @f]
 *
 * @note
 * This method is unstable if the matrix is ill-conditioned.
 */
template <typename T>
inline multi<decltype(T()/pr::sqrt(pr::abs(T()))), 3, 3>
                inverse(const multi<T, 3, 3>& arr)
{
    // Deduce floating point type.
    typedef decltype(pr::sqrt(pr::abs(T()))) float_type;

    // Cofactor matrix.
    multi<decltype(T()/float_type()), 3, 3> cof;
    cof[0] = cross(arr[1], arr[2]);
    cof[1] = cross(arr[2], arr[0]);
    cof[2] = cross(arr[0], arr[1]);

    // Cofactor transpose over determinant.
    return transpose(cof) * (float_type(1) / dot(cof[0], arr[0]));
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

} // namespace pr

// Math initializers, possible move to separate file?
namespace pr {

/**
 * @addtogroup multi_math_geometry 
 */
/**@{*/

/**
 * @brief Initializers for 2-dimensional floating point arrays.
 */
template <typename T>
struct multi_initializers<
            multi<T, 2>,
            std::enable_if_t<
            std::is_floating_point<T>::value, void>>
{
    /**
     * @brief Uniform disk probability density function.
     *
     * @f[
     *      f_{\text{disk}} = \frac{1}{\pi}
     * @f]
     */
    static T uniform_disk_pdf()
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
    static multi<T, 2> uniform_disk_pdf_sample(multi<T, 2> u)
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
};

/**
 * @brief Initializers for 3-dimensional floating point arrays.
 */
template <typename T>
struct multi_initializers<
            multi<T, 3>,
            std::enable_if_t<
            std::is_floating_point<T>::value, void>>
{
    /**
     * @brief Uniform hemisphere probability density function.
     *
     * @f[
     *      f_{\text{hemisphere}} = \frac{1}{2\pi}
     * @f]
     */
    static T uniform_hemisphere_pdf()
    {
        return pr::numeric_constants<T>::M_1_pi() / 2;
    }

    /**
     * @brief Uniform hemisphere probability density function sampling routine.
     *
     * @f[
     *      \omega_{\text{hemisphere}}(\mathbf{u}) =
     *      \begin{bmatrix}
     *          \sin(\theta) \cos(\phi)
     *      \\  \sin(\theta) \sin(\phi)
     *      \\  \cos(\theta)
     *      \end{bmatrix}
     * @f]
     * where
     * - @f$ \cos(\theta) \gets u_{[0]} @f$
     * - @f$ \sin(\theta) \gets \sqrt{1 - \cos^2(\theta)} @f$
     * - @f$ \phi \gets 2 \pi u_{[1]} @f$
     *
     *
     * @param[in] u
     * Sample in @f$ [0, 1)^2 @f$.
     */
    static multi<T, 3> uniform_hemisphere_pdf_sample(multi<T, 2> u)
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
    static T uniform_sphere_pdf()
    {
        return pr::numeric_constants<T>::M_1_pi() / T(4);
    }

    /**
     * @brief Uniform sphere probability density function sampling routine.
     *
     * @f[
     *      \omega_{\text{sphere}}(\mathbf{u}) =
     *      \begin{bmatrix}
     *          \sin(\theta) \cos(\phi)
     *      \\  \sin(\theta) \sin(\phi)
     *      \\  \cos(\theta)
     *      \end{bmatrix}
     * @f]
     * where
     * - @f$ \cos(\theta) \gets 2 u_{[0]} - 1 @f$
     * - @f$ \sin(\theta) \gets \sqrt{1 - \cos^2(\theta)} @f$
     * - @f$ \phi \gets 2 \pi u_{[1]} @f$
     *
     * @param[in] u
     * Sample in @f$ [0, 1)^2 @f$.
     */
    static multi<T, 3> uniform_sphere_pdf_sample(multi<T, 2> u)
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
     *      f_{\text{cos}}(\omega_{[2]}) = \frac{\omega_{[2]}}{\pi}
     * @f]
     *
     * @param[in] w2
     * Direction component.
     */
    static T cosine_hemisphere_pdf(T w2)
    {
        return pr::numeric_constants<T>::M_1_pi() * w2;
    }

    /**
     * @brief Cosine hemisphere probability density function sampling routine.
     *
     * @f[
     *      \omega_{\text{cos}}(\mathbf{u}) =
     *      \begin{bmatrix}
     *          P_{[0]}
     *      \\  P_{[1]}
     *      \\ \sqrt{1 - P_{[0]}^2 - P_{[1]}^2}
     *      \end{bmatrix}
     * @f]
     * where
     * - @f$ \mathbf{P} = \mathbf{P}_{\text{disk}}(\mathbf{u}) @f$
     *
     * @param[in] u
     * Sample in @f$ [0, 1)^2 @f$.
     */
    static multi<T, 3> cosine_hemisphere_pdf_sample(multi<T, 2> u)
    {
        multi<T, 2> p = multi<T, 2>::uniform_disk_pdf_sample(u);
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
     *      \frac{1}{2\pi} 
     *      \frac{1}{1 - \cos(\theta_{\text{max}})}
     * @f]
     *
     * @param[in] cos_thetamax
     * Cosine of maximum cone angle.
     */
    static T uniform_cone_pdf(T cos_thetamax)
    {
        return pr::numeric_constants<T>::M_1_pi() / 2 / (1 - cos_thetamax);
    }

    /**
     * @brief Uniform cone probability density function sampling routine.
     *
     * @f[
     *      \omega_{\text{cone}}(\mathbf{u}) =
     *      \begin{bmatrix}
     *          \sin(\theta) \cos(\phi)
     *      \\  \sin(\theta) \sin(\phi)
     *      \\  \cos(\theta)
     *      \end{bmatrix}
     * @f]
     * where
     * - @f$ \cos(\theta) \gets 1 - u_{[0]} + u_{[0]} \cos(\theta_{\max}) @f$
     * - @f$ \sin(\theta) \gets \sqrt{1 - \cos^2(\theta)} @f$
     * - @f$ \phi \gets 2 \pi u_{[1]} @f$
     *
     * @param[in] cos_thetamax
     * Cosine of maximum cone angle.
     *
     * @param[in] u
     * Sample in @f$ [0, 1)^2 @f$.
     */
    static multi<T, 3> uniform_cone_pdf_sample(T cos_thetamax, multi<T, 2> u)
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
     *      f_{\text{HG}}(\omega_{[2]}) =
     *      \frac{1}{4\pi}
     *      \frac{1 - g^2}{(1 + g^2 - 2g\omega_{[2]})^{3/2}}
     * @f]
     *
     * @param[in] g
     * Parameter in @f$ (-1, 1) @f$.
     *
     * @param[in] w2
     * Direction component.
     */
    static T hg_phase_pdf(T g, T w2)
    {
        if (pr::fabs(g) < T(0.00001)) {
            return uniform_sphere_pdf();
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
    static multi<T, 3> hg_phase_pdf_sample(T g, multi<T, 2> u)
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
};

/**
 * @brief Initializers for 3x3-dimensional floating point arrays.
 */
template <typename T>
struct multi_initializers<
            multi<T, 3, 3>, 
            std::enable_if_t<
            std::is_floating_point<T>::value, void>>
{
    /**
     * @brief Rotate counter-clockwise around arbitrary axis.
     *
     * @f[
     *      \begin{bmatrix}
     *          v_x^2   (1 - \cos{\theta}) + \cos{\theta}
     *      &   v_x v_y (1 - \cos{\theta}) - v_z \sin{\theta}
     *      &   v_x v_z (1 - \cos{\theta}) + v_y \sin{\theta}
     *      \\  v_x v_y (1 - \cos{\theta}) + v_z \cos{\theta}
     *      &   v_y^2   (1 - \cos{\theta}) + \cos{\theta}
     *      &   v_y v_z (1 - \cos{\theta}) - v_x \sin{\theta}
     *      \\  v_x v_z (1 - \cos{\theta}) - v_y \sin{\theta}
     *      &   v_y v_z (1 - \cos{\theta}) + v_x \sin{\theta}
     *      &   v_z^2   (1 - \cos{\theta}) + \cos{\theta}
     *      \end{bmatrix}
     * @f]
     *
     * @param[in] theta
     * Angle in radians.
     *
     * @param[in] hatv
     * Normalized rotation axis.
     */
    static multi<T, 3, 3> rotate(T theta, const multi<T, 3>& hatv)
    {
        T cos_theta = pr::cos(theta);
        T sin_theta = pr::sin(theta);
        T vx = hatv[0];
        T vy = hatv[1];
        T vz = hatv[2];
        T vxvx = vx * vx, vxvy = vx * vy, vxvz = vx * vz;
        T vyvy = vy * vy, vyvz = vy * vz;
        T vzvz = vz * vz;
        return {
            {vxvx * (1 - cos_theta) + cos_theta,
             vxvy * (1 - cos_theta) - vz * sin_theta,
             vxvz * (1 - cos_theta) + vy * sin_theta},
            {vxvy * (1 - cos_theta) + vz * sin_theta,
             vyvy * (1 - cos_theta) + cos_theta,
             vyvz * (1 - cos_theta) - vx * sin_theta},
            {vxvz * (1 - cos_theta) - vy * sin_theta,
             vyvz * (1 - cos_theta) + vx * sin_theta,
             vzvz * (1 - cos_theta) + cos_theta}
        };
    }

    /**
     * @brief Rotate counter-clockwise around X-axis.
     *
     * @f[
     *      \begin{bmatrix}
     *          1 & 0 & 0
     *      \\  0 & +\cos{\theta} & -\sin{\theta}
     *      \\  0 & +\sin{\theta} & +\cos{\theta}
     *      \end{bmatrix}
     * @f]
     *
     * @param[in] theta
     * Angle in radians.
     */
    static multi<T, 3, 3> rotatex(T theta)
    {
        T cos_theta = pr::cos(theta);
        T sin_theta = pr::sin(theta);
        return {
            {T(1), T(0), T(0)},
            {T(0), +cos_theta, -sin_theta},
            {T(0), +sin_theta, +cos_theta}
        };
    }

    /**
     * @brief Rotate counter-clockwise around Y-axis.
     *
     * @f[
     *      \begin{bmatrix}
     *          +\cos{\theta} & 0 & +\sin{\theta}
     *      \\  0 & 1 & 0
     *      \\  -\sin{\theta} & 0 & +\cos{\theta}
     *      \end{bmatrix}
     * @f]
     *
     * @param[in] theta
     * Angle in radians.
     */
    static multi<T, 3, 3> rotatey(T theta)
    {
        T cos_theta = pr::cos(theta);
        T sin_theta = pr::sin(theta);
        return {
            {+cos_theta, T(0), +sin_theta},
            {T(0), T(1), T(0)},
            {-sin_theta, T(0), +cos_theta}
        };
    }

    /**
     * @brief Rotate counter-clockwise around Z-axis.
     *
     * @f[
     *      \begin{bmatrix}
     *          +\cos{\theta} & -\sin{\theta} & 0
     *      \\  +\sin{\theta} & +\cos{\theta} & 0
     *      \\  0 & 0 & 1
     *      \end{bmatrix}
     * @f]
     *
     * @param[in] theta
     * Angle in radians.
     */
    static multi<T, 3, 3> rotatez(T theta)
    {
        T cos_theta = pr::cos(theta);
        T sin_theta = pr::sin(theta);
        return {
            {+cos_theta, -sin_theta, T(0)},
            {+sin_theta, +cos_theta, T(0)},
            {T(0), T(0), T(1)}
        };
    }

    /**
     * @brief Uniform scale.
     *
     * @f[
     *      \begin{bmatrix}
     *          s & 0 & 0
     *      \\  0 & s & 0
     *      \\  0 & 0 & s
     *      \end{bmatrix}
     * @f]
     *
     * @param[in] s
     * Factor.
     */
    static multi<T, 3, 3> scale(T s)
    {
        return {
            {s, T(0), T(0)},
            {T(0), s, T(0)},
            {T(0), T(0), s}
        };
    }

    /**
     * @brief Non-uniform scale.
     *
     * @f[
     *      \begin{bmatrix}
     *          s_x & 0 & 0
     *      \\  0 & s_y & 0
     *      \\  0 & 0 & s_z
     *      \end{bmatrix}
     * @f]
     *
     * @param[in] s
     * Factor.
     */
    static multi<T, 3, 3> scale(multi<T, 3> s)
    {
        return {
            {s[0], T(0), T(0)},
            {T(0), s[1], T(0)},
            {T(0), T(0), s[2]}
        };
    }
};

/**
 * @brief Initializers for 4x4-dimensional floating point arrays.
 */
template <typename T>
struct multi_initializers<
            multi<T, 4, 4>, 
            std::enable_if_t<
            std::is_floating_point<T>::value, void>>
{
    /**
     * @brief Translate.
     *
     * @f[
     *      \begin{bmatrix}
     *          1 & 0 & 0 & v_x
     *      \\  0 & 1 & 0 & v_y
     *      \\  0 & 0 & 1 & v_z
     *      \\  0 & 0 & 0 & 1
     *      \end{bmatrix}
     * @f]
     *
     * @param[in] v
     * Displacement vector.
     */
    static multi<T, 4, 4> translate(multi<T, 3> v)
    {
        return {
            {T(1), T(0), T(0), v[0]},
            {T(0), T(1), T(0), v[1]},
            {T(0), T(0), T(1), v[2]},
            {T(0), T(0), T(0), T(1)}
        };
    }

    /**
     * @brief Rotate counter-clockwise around arbitrary axis.
     *
     * @f[
     *      \begin{bmatrix}
     *          v_x^2   (1 - \cos{\theta}) + \cos{\theta}
     *      &   v_x v_y (1 - \cos{\theta}) - v_z \sin{\theta}
     *      &   v_x v_z (1 - \cos{\theta}) + v_y \sin{\theta}
     *      &   0
     *      \\  v_x v_y (1 - \cos{\theta}) + v_z \cos{\theta}
     *      &   v_y^2   (1 - \cos{\theta}) + \cos{\theta}
     *      &   v_y v_z (1 - \cos{\theta}) - v_x \sin{\theta}
     *      &   0
     *      \\  v_x v_z (1 - \cos{\theta}) - v_y \sin{\theta}
     *      &   v_y v_z (1 - \cos{\theta}) + v_x \sin{\theta}
     *      &   v_z^2   (1 - \cos{\theta}) + \cos{\theta}
     *      &   0
     *      \\  0 & 0 & 0 & 1
     *      \end{bmatrix}
     * @f]
     *
     * @param[in] theta
     * Angle in radians.
     *
     * @param[in] hatv
     * Normalized rotation axis.
     */
    static multi<T, 4, 4> rotate(T theta, multi<T, 3> hatv)
    {
        // Delegate.
        multi<T, 4, 4> res = 
        multi<T, 3, 3>::rotate(theta, hatv);
        res[3][3] = 1;
        return res;
    }

    /**
     * @brief Rotate counter-clockwise around X-axis.
     *
     * @f[
     *      \begin{bmatrix}
     *          1 & 0 & 0 & 0
     *      \\  0 & +\cos{\theta} & -\sin{\theta} & 0
     *      \\  0 & +\sin{\theta} & +\cos{\theta} & 0
     *      \\  0 & 0 & 0 & 1
     *      \end{bmatrix}
     * @f]
     *
     * @param[in] theta
     * Angle in radians.
     */
    static multi<T, 4, 4> rotatex(T theta)
    {
        // Delegate.
        multi<T, 4, 4> res = 
        multi<T, 3, 3>::rotatex(theta);
        res[3][3] = 1;
        return res;
    }
    
    /**
     * @brief Rotate counter-clockwise around Y-axis.
     *
     * @f[
     *      \begin{bmatrix}
     *          +\cos{\theta} & 0 & +\sin{\theta} & 0
     *      \\  0 & 1 & 0 & 0
     *      \\  -\sin{\theta} & 0 & +\cos{\theta} & 0
     *      \\  0 & 0 & 0 & 1
     *      \end{bmatrix}
     * @f]
     *
     * @param[in] theta
     * Angle in radians.
     */
    static multi<T, 4, 4> rotatey(T theta)
    {
        // Delegate.
        multi<T, 4, 4> res = 
        multi<T, 3, 3>::rotatey(theta);
        res[3][3] = 1;
        return res;
    }

    /**
     * @brief Rotate counter-clockwise around Z-axis.
     *
     * @f[
     *      \begin{bmatrix}
     *          +\cos{\theta} & -\sin{\theta} & 0 & 0
     *      \\  +\sin{\theta} & +\cos{\theta} & 0 & 0
     *      \\  0 & 0 & 1 & 0
     *      \\  0 & 0 & 0 & 1
     *      \end{bmatrix}
     * @f]
     *
     * @param[in] theta
     * Angle in radians.
     */
    static multi<T, 4, 4> rotatez(T theta)
    {
        // Delegate.
        multi<T, 4, 4> res = 
        multi<T, 3, 3>::rotatez(theta);
        res[3][3] = 1;
        return res;
    }

    /**
     * @brief Look-at transform.
     *
     * @f[
     *      \begin{bmatrix}
     *          \hat{\mathbf{x}}^\top
     *      &  -\hat{\mathbf{x}}^\top \mathbf{p}_{\text{from}}
     *      \\  \hat{\mathbf{y}}^\top
     *      &  -\hat{\mathbf{y}}^\top \mathbf{p}_{\text{from}}
     *      \\  \hat{\mathbf{z}}^\top
     *      &  -\hat{\mathbf{z}}^\top \mathbf{p}_{\text{from}}
     *      \\  \mathbf{0}^\top & 1
     *      \end{bmatrix}
     * @f]
     * where
     * - @f$ \mathbf{z}\gets\mathbf{p}_{\text{from}}-\mathbf{p}_{\text{to}} @f$
     * - @f$ \mathbf{x}\gets\mathbf{v}_{\text{up}}\times\mathbf{z} @f$
     * - @f$ \hat{\mathbf{z}}\gets\operatorname{normalize}(\mathbf{z}) @f$
     * - @f$ \hat{\mathbf{x}}\gets\operatorname{normalize}(\mathbf{x}) @f$
     * - @f$ \hat{\mathbf{y}}\gets\hat{\mathbf{z}}\times\hat{\mathbf{x}} @f$
     *
     * @param[in] pfrom
     * Look-from point.
     *
     * @param[in] pto
     * Look-to point.
     *
     * @param[in] vup
     * Up vector.
     *
     * @note
     * This implementation is consistent with OpenGL conventions.
     */
    static multi<T, 4, 4> lookat(
                const multi<T, 3>& pfrom, 
                const multi<T, 3>& pto,
                const multi<T, 3>& vup)
    {
        multi<T, 3> z = pfrom - pto;
        multi<T, 3> x = cross(vup, z);
        multi<T, 3> hatz = normalize(z);
        multi<T, 3> hatx = normalize(x);
        multi<T, 3> haty = cross(hatz, hatx);
        return {
            {hatx[0], hatx[1], hatx[2], -dot(hatx, pfrom)},
            {haty[0], haty[1], haty[2], -dot(haty, pfrom)},
            {hatz[0], hatz[1], hatz[2], -dot(hatz, pfrom)},
            {T(0), T(0), T(0), T(1)}
        };
    }

    /**
     * @brief Orthographic projection.
     *
     * @f[
     *      \begin{bmatrix}
     *          2 / (x1 - x0) & 0 & 0 & -(x1 + x0) / (x1 - x0)
     *      \\  0 & 2 / (y1 - y0) & 0 & -(y1 + y0) / (y1 - y0)
     *      \\  0 & 0 &-2 / (z1 - z0) & -(z1 + z0) / (z1 - z0)
     *      \\  0 & 0 & 0 & 1
     *      \end{bmatrix}
     * @f]
     *
     * @param[in] x0
     * Frustum X-coordinate 0.
     *
     * @param[in] x1
     * Frustum X-coordinate 1.
     *
     * @param[in] y0
     * Frustum Y-coordinate 0.
     *
     * @param[in] y1
     * Frustum Y-coordinate 1.
     *
     * @param[in] z0
     * Frustum Z-coordinate 0.
     *
     * @Param[in] z1
     * Frustum Z-coordinate 1.
     *
     * @note
     * This implementation is consistent with OpenGL conventions.
     */
    static multi<T, 4, 4> ortho(T x0, T x1, T y0, T y1, T z0, T z1)
    {
        multi<T, 4, 4> res;
        res[0][0] = 2 / (x1 - x0); res[0][3] = -(x1 + x0) / (x1 - x0);
        res[1][1] = 2 / (y1 - y0); res[1][3] = -(y1 + y0) / (y1 - y0);
        res[2][2] =-2 / (z1 - z0); res[2][3] = -(z1 + z0) / (z1 - z0);
        res[3][3] = 1;
        return res;
    }

    /**
     * @brief Perspective projection.
     *
     * @f[
     *      \begin{bmatrix}
     *          2 z_0 / (x_1 - x_0) & 0 & (x_1 + x_0) / (x_1 - x_0) & 0
     *      \\  0 & 2 z_0 / (y_1 - y_0) & (y_1 + y_0) / (y_1 - y_0) & 0
     *      \\  0 & 0 & -(z_1 + z_0) / (z_1 - z_0) & -2 z_1 z_0 / (z_1 - z_0)
     *      \\  0 & 0 & -1 & 0
     *      \end{bmatrix}
     * @f]
     *
     * @param[in] x0
     * Frustum X-coordinate 0 at Z-coordinate 0.
     *
     * @param[in] x1
     * Frustum X-coordinate 1 at Z-coordinate 0.
     *
     * @param[in] y0
     * Frustum Y-coordinate 0 at Z-coordinate 0.
     *
     * @param[in] y1
     * Frustum Y-coordinate 1 at Z-coordinate 0.
     *
     * @param[in] z0
     * Frustum Z-coordinate 0.
     *
     * @param[in] z1
     * Frustum Z-coordinate 1.
     *
     * @note
     * This implementation is consistent with OpenGL conventions.
     */
    static multi<T, 4, 4> persp(T x0, T x1, T y0, T y1, T z0, T z1)
    {
        multi<T, 4, 4> res;
        res[0][0] = 2 * z0 / (x1 - x0); res[0][2] = (x1 + x0) / (x1 - x0);
        res[1][1] = 2 * z0 / (y1 - y0); res[1][2] = (y1 + y0) / (y1 - y0);
        res[2][2] = -(z1 + z0) / (z1 - z0);
        res[2][3] = -2 * z1 * z0 / (z1 - z0);
        res[3][2] = -1;
        return res;
    }

    /**
     * @brief Perspective projection.
     *
     * @f[
     *      \begin{bmatrix}
     *          \cot(f / 2) / r & 0 & 0 & 0
     *      \\  0 & \cot(f / 2) & 0 & 0
     *      \\  0 & 0 & -(z_1 + z_0) / (z_1 - z_0) & -2 z_1 z_0 / (z_1 - z_0)
     *      \\  0 & 0 & -1 & 0
     *      \end{bmatrix}
     * @f]
     *
     * @param[in] f
     * Frustum Y-FOV in radians.
     *
     * @param[in] r
     * Frustum X-to-Y ratio.
     *
     * @param[in] z0
     * Frustum Z-coordinate 0.
     *
     * @param[in] z1
     * Frustum Z-coordinate 1.
     */
    static multi<T, 4, 4> persp(T f, T r, T z0, T z1)
    {
        multi<T, 4, 4> res;
        T cot_f_2 = pr::cot(f * T(0.5));
        res[0][0] = cot_f_2 / r;
        res[1][1] = cot_f_2;
        res[2][2] = -(z1 + z0) / (z1 - z0);
        res[2][3] = -2 * z1 * z0 / (z1 - z0);
        res[3][2] = -1;
        return res;
    }
};

/**@}*/

} // namespace pr

#endif // #ifndef PREFORM_MULTI_MATH_HPP
