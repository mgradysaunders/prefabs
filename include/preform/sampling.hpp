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
#error "preform/sampling.hpp requires >=C++17"
#endif // #if !(__cplusplus >= 201703L)
#endif // #if !DOXYGEN
#pragma once
#ifndef PREFORM_SAMPLING_HPP
#define PREFORM_SAMPLING_HPP

// for pr::multi
#include <preform/multi.hpp>

// for pr::multi wrappers
#include <preform/multi_math.hpp>

// for pr::multi wrappers
#include <preform/multi_misc_float.hpp>

// for pr::generate_canonical
#include <preform/random.hpp>

namespace pr {

/**
 * @defgroup sampling Sampling
 *
 * `<preform/sampling.hpp>`
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
    return pr::numeric_constants<T>::M_1_pi() / T(2);
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
    T sin_theta = pr::sqrt(T(1) - cos_theta * cos_theta);
    T phi = (T(2) * pr::numeric_constants<T>::M_pi()) * u[1];
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

/**
 * @brief Build 3-dimensional orthonormal basis.
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
 * @brief Build 3-dimensional orthonormal basis.
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

// Not necessary?
#if 0
/**
 * @brief Offset origin.
 *
 * @param[in] o
 * Origin.
 *
 * @param[in] oerr
 * Origin absolute error.
 *
 * @param[in] wi
 * Incident direction.
 *
 * @param[in] wg
 * Geometry normal direction.
 */
template <typename T>
inline std::enable_if_t<
       std::is_floating_point<T>::value,
                multi<T, 3>> offset_origin(
                                    const multi<T, 3>& o,
                                    const multi<T, 3>& oerr,
                                    const multi<T, 3>& wi,
                                    const multi<T, 3>& wg)
{
    // Offset.
    multi<T, 3> oprime = o + wg *
            pr::copysign(dot(pr::fabs(wg), oerr),
                         dot(wg, wi));

    // Round away.
    for (int k = 0; k < 3; k++) {
        if (oprime[k] < 0) {
            oprime[k] = fdec(oprime[k]);
        }
        else if (oprime[k] > 0) {
            oprime[k] = finc(oprime[k]);
        }
    }
    return oprime;
}
#endif

/**
 * @brief Stratify 1-dimensional canonical random samples.
 *
 * @param[in] gen
 * Generator.
 *
 * @param[in] dim
 * Number of strata.
 *
 * @param[out] arr
 * Sample array, must point to `dim` samples.
 *
 * @throw std::invalid_argument
 * If `dim <= 0` or `!arr`.
 */
template <
    typename G,
    typename P,
    typename T
    >
inline std::enable_if_t<
       std::is_integral<P>::value &&
       std::is_floating_point<T>::value, void> stratify(
                    G&& gen, 
                    const P& dim, 
                          T* arr)
{
    if (dim <= P(0) || !arr) {
        throw std::invalid_argument(__PRETTY_FUNCTION__);
    }

    // Stratify.
    for (P pos = 0; pos < dim; pos++) {
        arr[pos] = (pos + 
                pr::generate_canonical<T>(std::forward<G>(gen))) / dim;
    }

    // Shuffle into random order.
    std::shuffle(
        arr,
        arr + dim,
        std::forward<G>(gen));
}

/**
 * @brief Stratify n-dimensional canonical random samples.
 *
 * @param[in] gen
 * Generator.
 *
 * @param[in] dim
 * Number of strata in each dimension.
 *
 * @param[out] arr
 * Sample arrays, must point to `dim.prod()` samples.
 *
 * @throw std::invalid_argument
 * If `(dim <= 0).any()` or `!arr`.
 */
template <
    typename G,
    typename P,
    typename T, std::size_t N
    >
inline std::enable_if_t<
       std::is_integral<P>::value &&
       std::is_floating_point<T>::value, void> stratify(
                    G&& gen,
                    const multi<P, N>& dim,
                          multi<T, N>* arr)
{
    if ((dim <= P(0)).any() || !arr) {
        throw std::invalid_argument(__PRETTY_FUNCTION__);
    }

    // Stratify.
    multi<P, N> pos = {};
    multi<T, N>* itrarr = arr;
    do {
        for (T& val : *itrarr) {
            val = pr::generate_canonical<T>(std::forward<G>(gen));
        }
        *itrarr += pos;
        *itrarr /= dim;
        ++itrarr;

        // Increment.
        auto itrpos = pos.rbegin();
        auto itrdim = dim.rbegin();
        for (; itrpos < pos.rend(); ++itrpos, ++itrdim) {
            ++*itrpos;
            if (*itrpos >= *itrdim) {
                *itrpos = P(0);
            }
            else {
                break;
            }
        }
    }
    while ((pos != P(0)).any());

    // Shuffle into random order.
    std::shuffle(
        arr,
        arr + dim.prod(),
        std::forward<G>(gen));
}

/**
 * @brief Multiple-importance balance heuristic.
 *
 * @f[
 *      w_k = \frac{f_k}{f_k + \sum_{\ell \ne k} f_\ell}
 * @f]
 */
template <typename Tk, typename... Tl>
inline std::enable_if_t<
       (std::is_floating_point<Tk>::value && ... &&
        std::is_floating_point<Tl>::value), 
        std::common_type_t<Tk, Tl...>> balance_heuristic(Tk fk, Tl... fl)
{
    return fk / (fk + ... + fl);
}

/**
 * @brief Multiple-importance power heuristic.
 *
 * @f[
 *      w_k = \frac{f_k^2}{f_k^2 + \sum_{\ell \ne k} f_\ell^2}
 * @f]
 */
template <typename Tk, typename... Tl>
inline std::enable_if_t<
       (std::is_floating_point<Tk>::value && ... &&
        std::is_floating_point<Tl>::value), 
        std::common_type_t<Tk, Tl...>> power_heuristic(Tk fk, Tl... fl)
{
    return (fk * fk) / ((fk * fk) + ... + (fl * fl));
}

/**@}*/

} // namespace pr

#endif // #ifndef PREFORM_SAMPLING_HPP
