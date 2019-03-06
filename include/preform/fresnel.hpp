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
 * @defgroup fresnel Fresnel equations
 *
 * `<preform/fresnel.hpp>`
 *
 * __C++ version__: >=C++14
 */
/**@{*/

/**
 * @brief Fresnel equations for dielectric interface.
 *
 * - @f$ \cos^2{\theta_t} = 1 - \eta^2 (1 - \cos^2{\theta_i}) @f$
 * - @f$ r_s = 
 *      (\eta \cos{\theta_i} - \cos{\theta_t}) /
 *      (\eta \cos{\theta_i} + \cos{\theta_t}) @f$
 * - @f$ r_p = 
 *      (\cos{\theta_i} - \eta \cos{\theta_t}) /
 *      (\cos{\theta_i} + \eta \cos{\theta_t}) @f$
 * - @f$ t_s = 1 + r_s @f$
 * - @f$ t_p = \eta (1 + r_p) @f$
 *
 * @param[in] eta
 * Refractive index @f$ \eta = \eta_i / \eta_t @f$.
 *
 * @param[in] cos_thetai
 * Cosine of incidence angle.
 *
 * @param[out] cos_thetat
 * Cosine of transmittance angle.
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
 * @note
 * In the case of total internal reflection, 
 * - `cos_thetat = 0`,
 * - `rs = rp = 1`, and
 * - `ts = tp = 0`.
 */
template <typename T>
inline std::enable_if_t<
       std::is_floating_point<T>::value, void> fresnel_dielectric(
                T eta, 
                T cos_thetai,
                T& cos_thetat,
                T& rs, T& rp,
                T& ts, T& tp)
{
    T cos2_thetat = 1 - eta * eta * (1 - cos_thetai * cos_thetai);
    if (!(cos2_thetat > 0)) {
        cos_thetat = 0;
        rs = rp = 1;
        ts = tp = 0; // Total internal reflection.
    }
    else {
        cos_thetat = pr::copysign(pr::sqrt(cos2_thetat), cos_thetai);
        rs = (eta * cos_thetai - cos_thetat) / (eta * cos_thetai + cos_thetat);
        rp = (cos_thetai - eta * cos_thetat) / (cos_thetai + eta * cos_thetat);
        ts = 1 + rs;
        tp = 1 + rp; 
        tp *= eta;
        cos_thetat = -cos_thetat;
    }
}

/**
 * @brief Fresnel equations for dielectric interface, unpolarized form.
 *
 * - @f$ F_r = (r_s^2 + r_p^2)/2 @f$
 * - @f$ F_t = 1 - F_r @f$
 *
 * @param[in] eta
 * Refractive index @f$ \eta = \eta_i / \eta_t @f$.
 *
 * @param[in] cos_thetai
 * Cosine of incidence angle.
 *
 * @param[out] cos_thetat
 * Cosine of transmittance angle.
 *
 * @param[out] fr
 * Reflection.
 *
 * @param[out] ft
 * Transmission.
 */
template <typename T>
inline std::enable_if_t<
       std::is_floating_point<T>::value, void> fresnel_dielectric(
                T eta,
                T cos_thetai, 
                T& cos_thetat, 
                T& fr, T& ft)
{
    T rs = 0, rp = 0;
    T ts = 0, tp = 0;
    fresnel_dielectric(
        eta, 
        cos_thetai, 
        cos_thetat, 
        rs, rp, 
        ts, tp);
    fr = T(0.5) * (rs * rs + rp * rp);
    ft = 1 - fr;
}

/**@}*/

} // namespace pr

#endif // #ifndef PREFORM_FRESNEL_HPP
