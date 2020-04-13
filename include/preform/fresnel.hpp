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
#error "preform/fresnel.hpp requires >=C++14"
#endif // #if !(__cplusplus >= 201402L)
#endif // #if !DOXYGEN
#pragma once
#ifndef PREFORM_FRESNEL_HPP
#define PREFORM_FRESNEL_HPP

// for pre::signbit, pre::copysign, pre::sqrt, ...
#include <preform/math.hpp>

namespace pre {

/**
 * @defgroup fresnel Fresnel equations
 *
 * `<preform/fresnel.hpp>`
 *
 * __C++ version__: >=C++14
 */
/**@{*/

/**
 * @name Fresnel (dielectric)
 */
/**@{*/

/**
 * @brief Fresnel equations for dielectric interface.
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
 * @par Expression
 * @parblock
 * - @f$ \cos^2{\theta_t} = 1 - \eta^2 (1 - \cos^2{\theta_i}) @f$
 * - @f$ r_s =
 *      (\eta \cos{\theta_i} - \cos{\theta_t}) /
 *      (\eta \cos{\theta_i} + \cos{\theta_t}) @f$
 * - @f$ r_p =
 *      (\cos{\theta_i} - \eta \cos{\theta_t}) /
 *      (\cos{\theta_i} + \eta \cos{\theta_t}) @f$
 * - @f$ t_s = 1 + r_s @f$
 * - @f$ t_p = \eta (1 + r_p) @f$
 * @endparblock
 *
 * @note
 * In the case of total internal reflection,
 * - `cos_thetat = 0`,
 * - `rs = rp = 1`, and
 * - `ts = tp = 0`.
 *
 * @returns
 * If total internal reflection (TIR), returns `false`
 * to indicate that there is no refracted ray. Otherwise, returns `true`.
 */
template <typename T>
inline std::enable_if_t<
       std::is_floating_point<T>::value, bool> fresnel_diel(
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
        return false;
    }
    else {
        cos_thetat = pre::copysign(pre::sqrt(cos2_thetat), cos_thetai);
        rs = (eta * cos_thetai - cos_thetat) / (eta * cos_thetai + cos_thetat);
        rp = (cos_thetai - eta * cos_thetat) / (cos_thetai + eta * cos_thetat);
        ts = 1 + rs;
        tp = 1 + rp;
        tp *= eta;
        cos_thetat = -cos_thetat;
        return true;
    }
}

/**
 * @brief Fresnel equations for dielectric interface, unpolarized form.
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
 *
 * @par Expression
 * @parblock
 * - @f$ F_r = (r_s^2 + r_p^2)/2 @f$
 * - @f$ F_t = 1 - F_r @f$
 * @endparblock
 *
 * @returns
 * If total internal reflection (TIR), returns `false`
 * to indicate that there is no refracted ray. Otherwise, returns `true`.
 */
template <typename T>
inline std::enable_if_t<
       std::is_floating_point<T>::value, bool> fresnel_diel(
                T eta,
                T cos_thetai,
                T& cos_thetat,
                T& fr, T& ft)
{
    T rs = 0, rp = 0;
    T ts = 0, tp = 0;
    bool res =
    fresnel_diel(
        eta,
        cos_thetai,
        cos_thetat,
        rs, rp,
        ts, tp);
    fr = T(0.5) * (rs * rs + rp * rp);
    ft = 1 - fr;
    return res;
}

/**
 * @brief Fresnel equations for dielectric interface, Schlick's approximation
 * of unpolarized form.
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
 *
 * @par Expression
 * @parblock
 * - @f$ R_0 = (\eta - 1)^2 / (\eta + 1)^2 @f$
 * - @f$ F_r = R_0 + (1 - R_0) (1 - \cos{\theta_i})^5 @f$
 * - @f$ F_t = 1 - F_r @f$
 * @endparblock
 *
 * @returns
 * If total internal reflection (TIR), returns `false`
 * to indicate that there is no refracted ray. Otherwise, returns `true`.
 */
template <typename T>
inline std::enable_if_t<
       std::is_floating_point<T>::value, bool> fresnel_diel_schlick(
               T eta,
               T cos_thetai,
               T& cos_thetat,
               T& fr, T& ft)
{
    T cos2_thetat = 1 - eta * eta * (1 - cos_thetai * cos_thetai);
    if (!(cos2_thetat > 0)) {
        cos_thetat = 0;
        fr = 1;
        ft = 0; // Total internal reflection.
        return false;
    }
    else {
        cos_thetat = pre::copysign(pre::sqrt(cos2_thetat), cos_thetai);
        cos_thetat = -cos_thetat;
        T sqrt_r0 = (eta - 1) / (eta + 1);
        T r0 = sqrt_r0 * sqrt_r0;
        T cos_thetai_term1 = 1 - cos_thetai;
        T cos_thetai_term2 = cos_thetai_term1 * cos_thetai_term1;
        T cos_thetai_term4 = cos_thetai_term2 * cos_thetai_term2;
        T cos_thetai_term5 = cos_thetai_term4 * cos_thetai_term1;
        fr = r0 + (1 - r0) * cos_thetai_term5;
        ft = 1 - fr;
        return true;
    }
}

// TODO doc
/**
 * @brief Fresnel's equations for dielectric thinfilm interface.
 *
 * @param[in] lambda
 * Vacuum wavelength.
 *
 * @param[in] etai
 * Absolute refractive index in incident hemisphere.
 *
 * @param[in] etaf
 * Absolute refractive index in film.
 *
 * @param[in] ellf
 * Film thickness.
 *
 * @param[in] muf
 * Film volume absorption coefficient.
 *
 * @param[in] etat
 * Absolute refractive index in transmitted hemisphere.
 *
 * @param[in] cos_thetai
 * Cosine of incidence angle.
 *
 * @param[out] cos_thetat
 * Cosine of transmittance angle.
 */
template <typename T>
inline std::enable_if_t<
       std::is_floating_point<T>::value, bool> fresnel_diel_thinfilm(
               T lambda,
               T etai,
               T etaf,
               T ellf,
               T muf,
               T etat,
               T cos_thetai,
               T& cos_thetat,
               std::complex<T>& rs, std::complex<T>& rp,
               std::complex<T>& ts, std::complex<T>& tp)
{
    // Refractive index ratios.
    T eta1 = etai / etaf;
    T eta2 = etaf / etat;

    // Angle sines.
    T sin2_thetai = 1 - cos_thetai * cos_thetai;
    T sin2_theta1 = eta1 * eta1 * sin2_thetai;
    T sin2_theta2 = eta2 * eta2 * sin2_theta1;
    T cos2_theta1 = 1 - sin2_theta1;
    T cos2_theta2 = 1 - sin2_theta2;
    if (!(cos2_theta1 > 0 &&
          cos2_theta2 > 0)) {
        cos_thetat = 0;
        rs = rp = 1;
        ts = tp = 0;
        return false;
    }

    // Angle cosines.
    T cos_theta1 = pre::sqrt(cos2_theta1);
    T cos_theta2 = pre::sqrt(cos2_theta2);
    cos_theta1 = pre::copysign(cos_theta1, cos_thetai);
    cos_theta2 = pre::copysign(cos_theta2, cos_thetai);
    cos_thetat = -cos_theta2;

    // Fresnel coefficients for interface 1.
    T rs1, rp1;
    T ts1, tp1;
    rs1 = (eta1 * cos_thetai - cos_theta1) / (eta1 * cos_thetai + cos_theta1);
    rp1 = (cos_thetai - eta1 * cos_theta1) / (cos_thetai + eta1 * cos_theta1);
    ts1 = 1 + rs1;
    tp1 = 1 + rp1;
    tp1 *= eta1;

    // Fresnel coefficients for interface 2.
    T rs2, rp2;
    T ts2, tp2;
    rs2 = (eta2 * cos_theta1 - cos_theta2) / (eta2 * cos_theta1 + cos_theta2);
    rp2 = (cos_theta1 - eta2 * cos_theta2) / (cos_theta1 + eta2 * cos_theta2);
    ts2 = 1 + rs2;
    tp2 = 1 + rp2;
    tp2 *= eta2;

    // Interference.
    T phi = T(2) * etaf * ellf / (lambda * cos_theta1);
    phi += (etai > etaf) ? T(1) : T(0);
    phi += (etaf < etat) ? T(1) : T(0);
    phi *= pre::numeric_constants<T>::M_pi();
    T alpha = muf * ellf / cos_theta1;
    std::complex<T> exp_phi = pre::exp(std::complex<T>{-alpha, phi});
    std::complex<T> exp_phi2 = exp_phi * exp_phi;
    std::complex<T> rs2_exp_phi2 = rs2 * exp_phi2;
    std::complex<T> rp2_exp_phi2 = rp2 * exp_phi2;
    T fac = (etaf * cos_theta1) / (etai * cos_thetai);
    rs = rs1 + fac * ts1 * ts1 * rs2_exp_phi2 / (T(1) + rs1 * rs2_exp_phi2);
    rp = rp1 + fac * tp1 * tp1 * rp2_exp_phi2 / (T(1) + rp1 * rp2_exp_phi2);
    ts = ts1 * ts2 * exp_phi / (T(1) + rs1 * rs2_exp_phi2);
    tp = tp1 * tp2 * exp_phi / (T(1) + rp1 * rp2_exp_phi2);
    return true;
}

/**@}*/

/**
 * @name Fresnel (dielectric-to-conducting)
 */
/**@{*/

/**
 * @brief Fresnel equations for dielectric-to-conducting interface.
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
 * @par Expression
 * @parblock
 * - @f$ \cos^2{\theta_t} = 1 - \eta^2 (1 - \cos^2{\theta_i}) @f$
 * - @f$ r_s =
 *      (\eta \cos{\theta_i} - \cos{\theta_t}) /
 *      (\eta \cos{\theta_i} + \cos{\theta_t}) @f$
 * - @f$ r_p =
 *      (\cos{\theta_i} - \eta \cos{\theta_t}) /
 *      (\cos{\theta_i} + \eta \cos{\theta_t}) @f$
 * - @f$ t_s = 1 + r_s @f$
 * - @f$ t_p = \eta (1 + r_p) @f$
 * @endparblock
 *
 * @note
 * In practice,
 * - @f$ \eta_i @f$ should be real/dielectric, while
 * - @f$ \eta_t @f$ may be complex/conducting.
 */
template <typename T>
inline std::enable_if_t<
       std::is_floating_point<T>::value, void> fresnel_diel_cond(
               std::complex<T> eta,
               T cos_thetai,
               std::complex<T>& cos_thetat,
               std::complex<T>& rs, std::complex<T>& rp,
               std::complex<T>& ts, std::complex<T>& tp)
{
    cos_thetat =
        pre::sign(cos_thetai) *
        pre::sqrt(T(1) - eta * eta *
                (T(1) - cos_thetai * cos_thetai));
    rs = (eta * cos_thetai - cos_thetat) / (eta * cos_thetai + cos_thetat);
    rp = (cos_thetai - eta * cos_thetat) / (cos_thetai + eta * cos_thetat);
    ts = T(1) + rs;
    tp = T(1) + rp;
    tp *= eta;
    cos_thetat = -cos_thetat;
}

/**
 * @brief Fresnel equations for dielectric-to-conducting interface,
 * unpolarized form.
 *
 * @param[in] eta
 * Refractive index @f$ \eta = \eta_i / \eta_t @f$.
 *
 * @param[in] cos_thetai
 * Cosine of incidence angle.
 *
 * @note
 * In practice,
 * - @f$ \eta_i @f$ should be real/dielectric, while
 * - @f$ \eta_t @f$ may be complex/conducting.
 */
template <typename T>
inline std::enable_if_t<
       std::is_floating_point<T>::value, T> fresnel_diel_cond(
               std::complex<T> eta,
               T cos_thetai)
{
    std::complex<T> cos_thetat;
    std::complex<T> rs, rp;
    std::complex<T> ts, tp;
    fresnel_diel_cond(
            eta,
            cos_thetai,
            cos_thetat,
            rs, rp,
            ts, tp);
    return T(0.5) * (pre::norm(rs) + pre::norm(rp));
}

/**@}*/

/**@}*/

} // namespace pre

#endif // #ifndef PREFORM_FRESNEL_HPP
