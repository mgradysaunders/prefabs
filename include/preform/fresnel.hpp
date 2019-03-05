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
#error "preform/fresnel.hpp requires >=C++14"
#endif // #if !(__cplusplus >= 201402L)
#endif // #if !DOXYGEN
#pragma once
#ifndef PREFORM_FRESNEL_HPP
#define PREFORM_FRESNEL_HPP

// for pr::signbit, pr::copysign, pr::sqrt, ...
#include <preform/math.hpp>

namespace pr {

/**
 * @defgroup fresnel Fresnel
 *
 * `<preform/fresnel.hpp>`
 *
 * __C++ version__: >=C++14
 */
/**@{*/

/**
 * @brief Fresnel equations for dielectric-dielectric interface.
 *
 * - @f$ \cos^2{\theta_t} = 1 - n^2 (1 - \cos^2{\theta_i}) @f$
 * - @f$ r_s = 
 *      (n \cos{\theta_i} - \cos{\theta_t}) /
 *      (n \cos{\theta_i} + \cos{\theta_t}) @f$
 * - @f$ r_p = 
 *      (\cos{\theta_i} - n \cos{\theta_t}) /
 *      (\cos{\theta_i} + n \cos{\theta_t}) @f$
 * - @f$ t_s = 1 + r_s @f$
 * - @f$ t_p = n (1 + r_p) @f$
 *
 * @param[in] cos_thetai
 * Cosine of incidence angle.
 *
 * @param[in] n0
 * Refractive index in upper hemisphere.
 *
 * @param[in] n1
 * Refractive index in lower hemisphere.
 *
 * @param[out] rs
 * Reflection s-polarization.
 *
 * @param[out] rp
 * Reflection p-polarization.
 *
 * @param[out] ts
 * Transmission s-polarization.
 *
 * @param[out] tp
 * Transmission p-polarization.
 *
 * @returns
 * Cosine of transmittance angle.
 *
 * @note
 * In the event of total internal reflection, 
 * - sets `rs = rp = 1`,
 * - sets `ts = tp = 0`, and
 * - returns `0`.
 */
template <typename T>
inline std::enable_if_t<
       std::is_floating_point<T>::value, T> fr_diel_diel(
                T cos_thetai, T n0, T n1,
                T& rs, T& rp,
                T& ts, T& tp)
{
    T ni = pr::signbit(cos_thetai) ? n1 : n0;
    T nt = pr::signbit(cos_thetai) ? n0 : n1;
    T n = ni / nt;
    T cos2_thetat = 1 - n * n * (1 - cos_thetai * cos_thetai);
    if (!(cos2_thetat > 0)) {
        rs = rp = 1;
        ts = tp = 0;
        return 0; // Total internal reflection.
    }
    else {
        T cos_thetat = pr::copysign(pr::sqrt(cos2_thetat), cos_thetai);
        rs = (n * cos_thetai - cos_thetat) / (n * cos_thetai + cos_thetat);
        rp = (cos_thetai - n * cos_thetat) / (cos_thetai + n * cos_thetat);
        ts = 1 + rs;
        tp = 1 + rp; 
        tp *= n;
        return -cos_thetat;
    }
}

/**
 * @brief Fresnel equations for dielectric-conductor interface.
 *
 * - @f$ \cos^2{\theta_t} = 1 - n^2 (1 - \cos^2{\theta_i}) @f$
 * - @f$ r_s = 
 *      (n \cos{\theta_i} - \cos{\theta_t}) /
 *      (n \cos{\theta_i} + \cos{\theta_t}) @f$
 * - @f$ r_p = 
 *      (\cos{\theta_i} - n \cos{\theta_t}) /
 *      (\cos{\theta_i} + n \cos{\theta_t}) @f$
 * - @f$ t_s = 1 + r_s @f$
 * - @f$ t_p = n (1 + r_p) @f$
 *
 * @param[in] cos_thetai
 * Cosine of incidence angle.
 *
 * @param[in] n0
 * Refractive index in upper hemisphere.
 *
 * @param[in] n1
 * Refractive index in lower hemisphere.
 *
 * @param[out] rs
 * Reflection s-polarization.
 *
 * @param[out] rp
 * Reflection p-polarization.
 *
 * @param[out] ts
 * Transmission s-polarization.
 *
 * @param[out] tp
 * Transmission p-polarization.
 *
 * @returns
 * Cosine of transmittance angle.
 */
template <typename T>
inline std::enable_if_t<
       std::is_floating_point<T>::value,
       std::complex<T>> fr_diel_cond(
                T cos_thetai, 
                T n0, 
                std::complex<T> n1, 
                std::complex<T>& rs, std::complex<T>& rp,
                std::complex<T>& ts, std::complex<T>& tp)
{
    std::complex<T> ni = pr::signbit(cos_thetai) ? n1 : n0;
    std::complex<T> nt = pr::signbit(cos_thetai) ? n0 : n1;
    std::complex<T> n = ni / nt;
    std::complex<T> cos_thetat = pr::sqrt(T(1) - n * n * 
            (T(1) - cos_thetai * cos_thetai));
    cos_thetat *= pr::copysign(T(1), cos_thetai);
    rs = (n * cos_thetai - cos_thetat) / (n * cos_thetai + cos_thetat);
    rp = (cos_thetai - n * cos_thetat) / (cos_thetai + n * cos_thetat);
    ts = T(1) + rs;
    tp = T(1) + rp; 
    tp *= n;
    return -cos_thetat;
}

/**@}*/

} // namespace pr

#endif // #ifndef PREFORM_FRESNEL_HPP
