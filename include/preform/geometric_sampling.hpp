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
#error "preform/geometric_sampling.hpp requires >=C++17"
#endif // #if !(__cplusplus >= 201703L)
#endif // #if !DOXYGEN
#pragma once
#ifndef PREFORM_GEOMETRIC_SAMPLING_HPP
#define PREFORM_GEOMETRIC_SAMPLING_HPP

// for pr::multi
#include <preform/multi.hpp>

// for pr::multi wrappers
#include <preform/multi_math.hpp>

// for pr::multi wrappers
#include <preform/multi_misc_float.hpp>

namespace pr {

/**
 * @defgroup geometric_sampling Geometric sampling
 *
 * `<preform/geometric_sampling.hpp>`
 *
 * __C++ version__: >=C++17
 */
/**@{*/

/**
 * @brief Uniform disk.
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
                multi<T, 2>> uniform_disk(multi<T, 2> u)
{
    u = T(2) * u - T(1);
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
 * @brief Uniform disk density.
 *
 * @f[
 *      f_{\text{disk}} = \frac{1}{\pi}
 * @f]
 */
template <typename T>
inline std::enable_if_t<
       std::is_floating_point<T>::value, T> uniform_disk_density()
{
    return pr::numeric_constants<T>::M_1_pi();
}

/**
 * @brief Uniform hemisphere.
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
                multi<T, 3>> uniform_hemisphere(multi<T, 2> u)
{
    T cos_theta = u[0];
    cos_theta = pr::fmax(cos_theta, T(0));
    cos_theta = pr::fmin(cos_theta, T(1));
    T sin_theta = pr::sqrt(T(1) - cos_theta * cos_theta);
    T phi = (T(2) * pr::numeric_constants<T>::M_pi()) * u[1];
    return {
        sin_theta * pr::cos(phi),
        sin_theta * pr::sin(phi),
        cos_theta
    };
}

/**
 * @brief Uniform hemisphere density.
 *
 * @f[
 *      f_{\text{hemisphere}} = \frac{1}{2\pi}
 * @f]
 */
template <typename T>
inline std::enable_if_t<
       std::is_floating_point<T>::value, T> uniform_hemisphere_density()
{
    return pr::numeric_constants<T>::M_1_pi() / T(2);
}

/**
 * @brief Uniform sphere.
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
                multi<T, 3>> uniform_sphere(multi<T, 2> u)
{
    T cos_theta = T(2) * u[0] - T(1);
    cos_theta = pr::fmax(cos_theta, T(-1));
    cos_theta = pr::fmin(cos_theta, T(+1));
    T sin_theta = pr::sqrt(T(1) - cos_theta * cos_theta);
    T phi = (T(2) * pr::numeric_constants<T>::M_pi()) * u[1];
    return {
        sin_theta * pr::cos(phi),
        sin_theta * pr::sin(phi),
        cos_theta
    };
}

/**
 * @brief Uniform sphere density.
 *
 * @f[
 *      f_{\text{sphere}} = \frac{1}{4\pi}
 * @f]
 */
template <typename T>
inline std::enable_if_t<
       std::is_floating_point<T>::value, T> uniform_sphere_density()
{
    return pr::numeric_constants<T>::M_1_pi() / T(4);
}

/**
 * @brief Cosine hemisphere.
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
                multi<T, 3>> cosine_hemisphere(multi<T, 2> u)
{
    multi<T, 2> p = uniform_disk(u);
    return {
        p[0],
        p[1],
        pr::sqrt(T(1) - pr::fmin(pr::dot(p, p), T(1)))
    };
}

/**
 * @brief Cosine hemisphere density.
 *
 * @f[
 *      f_{\text{cosine}}(\omega_{[2]}) = \frac{\omega_{[2]}}{\pi}
 * @f]
 *
 * @param[in] w2
 * Direction component.
 */
template <typename T>
inline std::enable_if_t<
       std::is_floating_point<T>::value, T> cosine_hemisphere_density(T w2)
{
    return pr::numeric_constants<T>::M_1_pi() * w2;
}

/**@}*/

} // namespace pr

#endif // #ifndef PREFORM_GEOMETRIC_SAMPLING_HPP
