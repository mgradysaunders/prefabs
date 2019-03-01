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
#pragma once
#ifndef PREFORM_MICROSURFACE_HPP
#define PREFORM_MICROSURFACE_HPP

// for pr::multi
#include <preform/multi.hpp>

// for pr::multi wrappers
#include <preform/multi_math.hpp>

// for pr::pcg32, pr::normal_distribution
#include <preform/random.hpp>

namespace pr {

/**
 * @defgroup microsurface Microsurface
 *
 * `<preform/microsurface.hpp>`
 *
 * __C++ version__: >=C++17
 */
/**@{*/

/**
 * @brief Microsurface shared members (by curiously recurring 
 * template pattern).
 *
 * This implementation is based on the _Multiple-scattering microfacet BSDFs 
 * with the Smith model_ sample code by Eric Heitz.
 */
template <
    typename Tfloat, 
    typename Tspecific
    >
struct microsurface_shared
{
public:

    // Sanity check.
    static_assert(
        std::is_floating_point<Tfloat>::value,
        "Tfloat must be floating point");

    /**
     * @brief Float type.
     */
    typedef Tfloat float_type;

    /**
     * @brief Default constructor.
     */
    microsurface_shared() = default;

    /**
     * @brief Constructor.
     *
     * @param[in] alpha
     * Roughness.
     */
    microsurface_shared(multi<float_type, 2> alpha) : alpha_(alpha)
    {
    }

    /**
     * @brief Distribution of normals.
     *
     * @f[
     *      D(\omega_m) = 
     *      \frac{1}{\cos^4(\theta_m)} P_{22}([m_x\; m_y])
     * @f]
     * where
     * - @f$ m_x = -\omega_{m_x} / \cos(\theta_m) @f$,
     * - @f$ m_y = -\omega_{m_y} / \cos(\theta_m) @f$
     *
     * @param[in] wm
     * Normal direction.
     */
    float_type dwm(multi<float_type, 3> wm) const
    {
        // Invalid normal?
        if (pr::signbit(wm[2])) {
            return 0;
        }

        // Trig terms.
        float_type cos_thetam = wm[2];
        cos_thetam = pr::fmax(cos_thetam, float_type(-1));
        cos_thetam = pr::fmin(cos_thetam, float_type(+1));
        float_type cos2_thetam = cos_thetam * cos_thetam;
        float_type cos4_thetam = cos2_thetam * cos2_thetam;
        if (cos4_thetam == 0) {
            return 0;
        }

        // Slope.
        multi<float_type, 2> m = {
            -wm[0] / cos_thetam,
            -wm[1] / cos_thetam
        };

        // Slope distribution.
        float_type p22 = static_cast<const Tspecific*>(this)->p22(m);
        return pr::isfinite(p22 / cos4_thetam) ?
                            p22 / cos4_thetam : 0;
    }

    /**
     * @brief Distribution of visible normals.
     *
     * @f[
     *      D_{\omega_o}(\omega_m) = D(\omega_m)
     *         \frac{\langle{\omega_o, \omega_m}\rangle}{A_{\perp}(\omega_o)}
     * @f]
     *
     * @param[in] wo
     * Viewing direction.
     *
     * @param[in] wm
     * Normal direction.
     */
    float_type dwo(
                multi<float_type, 3> wo,
                multi<float_type, 3> wm) const
    {
        // Invalid normal?
        if (pr::signbit(wm[2])) {
            return 0;
        }

        // Dot product.
        float_type cos_theta = pr::dot(wo, wm);

        // Invalid hemisphere?
        if (pr::signbit(cos_theta)) {
            return 0;
        }

        // Projected area.
        float_type a = static_cast<const Tspecific*>(this)->aperp(wo);

        // Distribution of normals.
        float_type d = dwm(wm);

        // Distribution of visible normals.
        return pr::isfinite(d * cos_theta / a) ?
                            d * cos_theta / a : 0;
    }

    /**
     * @brief Distribution of visible normals sampling routine.
     *
     * @param[in] u
     * Sample in @f$ [0, 1)^2 @f$.
     *
     * @param[in] wo
     * Viewing direction.
     */
    multi<float_type, 3> dwo_sample(
                multi<float_type, 2> u,
                multi<float_type, 3> wo) const
    {
        // Stretch.
        multi<float_type, 3> wo11 = pr::normalize(
        multi<float_type, 3>{
            alpha_[0] * wo[0],
            alpha_[1] * wo[1],
            wo[2]
        });

        // Sample visible slope.
        multi<float_type, 2> m11 = 
            static_cast<const Tspecific*>(this)->p11_sample(u, wo11[2]);

        // Rotate.
        float_type phi = pr::atan2(wo11[1], wo11[0]);
        float_type sin_phi = pr::sin(phi);
        float_type cos_phi = pr::cos(phi);
        multi<float_type, 2> m = {
            cos_phi * m11[0] - sin_phi * m11[1],
            sin_phi * m11[0] + cos_phi * m11[1]
        };

        // Unstretch.
        m *= alpha_;

        // Invalid slope?
        if (!pr::isfinite(m[0]) ||
            !pr::isfinite(m[1])) {
            if (wo[2] != 0) {
                return {0, 0, 1};
            }
            else {
                multi<float_type, 3> wm = {wo[0], wo[1], 0};
                return pr::normalize(wm);
            }
        }

        // Normalize.
        multi<float_type, 3> wm = {-m[0], -m[1], 1};
        return pr::normalize(wm);
    }

    /**
     * @brief Height-averaged shadowing function.
     *
     * @f[
     *      G_1(\omega_o) = \frac{1}{1 + \Lambda(\omega_o)}
     * @f]
     *
     * @param[in] wo
     * Viewing direction.
     */
    float_type g1(multi<float_type, 3> wo) const
    {
        if (pr::signbit(wo[2])) {
            return 0;
        }

        return 1 / (1 + static_cast<const Tspecific*>(this)->lambda(wo));
    }

    /**
     * @brief Height-specific shadowing function.
     *
     * @f[
     *      G_1(\omega_o, h_0) = C_1(h_0)^{\Lambda(\omega_o)}
     * @f]
     *
     * @param[in] wo
     * Viewing direction.
     *
     * @param[in] h0
     * Current height.
     */
    float_type g1(multi<float_type, 3> wo, float_type h0) const
    {
        if (pr::signbit(wo[2])) {
            return 0;
        }

        return pr::pow(
               pr::normal_distribution<float_type>().cdf(h0),
               static_cast<const Tspecific*>(this)->lambda(wo));
    }

    /**
     * @brief Height sampling routine.
     *
     * @param[in] u
     * Sample in @f$ [0, 1) @f$.
     *
     * @param[in] wo
     * Viewing direction.
     *
     * @param[in] h0
     * Current height.
     */
    float_type h_sample(
                float_type u, multi<float_type, 3> wo,
                float_type h0) const
    {
        // Handle edge case.
        if (wo[2] > float_type(+0.9999)) {
            return pr::numeric_limits<float_type>::infinity(); // Exit
        }

        // Handle edge case.
        if (wo[2] < float_type(-0.9999)) {
            return 
                pr::normal_distribution<float_type>().cdfinv(u * 
                pr::normal_distribution<float_type>().cdf(h0));
        }

        // Handle edge case.
        if (pr::fabs(wo[2]) < float_type(0.0001)) {
            return h0;
        }

        // Shadowing is probability of intersection.
        float_type g = g1(wo, h0);
        if (u > 1 - g) {
            return pr::numeric_limits<float_type>::infinity(); // Exit
        }

        // Intersect.
        return 
            pr::normal_distribution<float_type>().cdfinv(
            pr::normal_distribution<float_type>().cdf(h0) /
            pr::pow(1 - u, 
                    1 / static_cast<const Tspecific*>(this)->lambda(wo)));
    }

protected:

    /**
     * @brief Roughness @f$ [\alpha_x\, \alpha_y]^\top @f$.
     */
    multi<float_type, 2> alpha_ = {
        1, 
        1
    };
};

/**
 * @brief Trowbridge-Reitz microsurface.
 */
template <typename Tfloat>
struct trowbridge_reitz_microsurface : public
                        microsurface_shared<Tfloat,
                        trowbridge_reitz_microsurface<Tfloat>>
{
public:

    // Sanity check.
    static_assert(
        std::is_floating_point<Tfloat>::value,
        "Tfloat must be floating point");

    /**
     * @brief Float type.
     */
    typedef Tfloat float_type;

    // Expose constructors.
    using microsurface_shared<Tfloat,
            trowbridge_reitz_microsurface<Tfloat>>::microsurface_shared;

    // Expose alpha_.
    using microsurface_shared<Tfloat,
            trowbridge_reitz_microsurface<Tfloat>>::alpha_;

    /**
     * @brief Projected area.
     *
     * @f[
     *      A_{\perp}(\omega_o) = 
     *      \int{\langle{\omega_o, \omega_m}\rangle D(\omega_m)}\,d\omega_m =
     *      (1 + \Lambda(\omega_o))\cos(\theta_o)
     * @f]
     *
     * @param[in] wo
     * Viewing direction.
     */
    float_type aperp(multi<float_type, 3> wo) const
    {
        // Trig terms.
        float_type cos_thetao = wo[2];
        cos_thetao = pr::fmax(cos_thetao, -float_type(1));
        cos_thetao = pr::fmin(cos_thetao, +float_type(1));
        float_type cos2_thetao = cos_thetao * cos_thetao;
        float_type sin2_thetao = 1 - cos2_thetao;
        float_type cos2_phio = wo[0] * wo[0] / sin2_thetao;
        float_type sin2_phio = wo[1] * wo[1] / sin2_thetao;

        // Projected roughness squared.
        float_type alpha2 = 
            alpha_[0] * alpha_[0] * cos2_phio +
            alpha_[1] * alpha_[1] * sin2_phio;

        // Overflow? 
        if (!pr::isfinite(alpha2)) {
            return pr::signbit(cos_thetao) ? 0 : 1;
        }
        else {
            // Evaluate.
            return 
                float_type(0.5) * cos_thetao + 
                float_type(0.5) * pr::sqrt(cos2_thetao +
                                           sin2_thetao * alpha2); 
        }
    }

    /**
     * @brief Smith shadowing term.
     *
     * @f[
     *      \Lambda(\omega_o) = 
     *      \frac{1}{2}\left(-1 + \operatorname{sign}(\mu)
     *      \sqrt{1 + \frac{1}{\mu^2}}\right)
     * @f]
     * where
     * @f[
     *      \mu = \frac{1}{\tan(\theta_o)\alpha_{\perp}(\omega_o)}
     * @f]
     * 
     * @param[in] wo
     * Viewing direction.
     */
    float_type lambda(multi<float_type, 3> wo) const
    {
        // Trig terms.
        float_type cos_thetao = wo[2];
        cos_thetao = pr::fmax(cos_thetao, -float_type(1));
        cos_thetao = pr::fmin(cos_thetao, +float_type(1));
        float_type cos2_thetao = cos_thetao * cos_thetao;
        float_type sin2_thetao = 1 - cos2_thetao;
        float_type cos2_phio = wo[0] * wo[0] / sin2_thetao;
        float_type sin2_phio = wo[1] * wo[1] / sin2_thetao;

        // Projected roughness squared.
        float_type alpha2 = 
            alpha_[0] * alpha_[0] * cos2_phio +
            alpha_[1] * alpha_[1] * sin2_phio;

        // Projected roughness.
        float_type alpha = pr::sqrt(alpha2);

        // More trig terms, evaluation terms.
        float_type sin_thetao = pr::sqrt(sin2_thetao);
        float_type cot_thetao = cos_thetao / sin_thetao;
        float_type mu = cot_thetao / alpha;
        float_type muinv2 = 1 / mu / mu;

        // Overflow?
        if (!pr::isfinite(mu) ||
            !pr::isfinite(muinv2)) {
            return pr::signbit(cos_thetao) ? -1 : 0;
        }
        else {
            // Evaluate.
            return 
                pr::copysign(float_type(0.5), mu) *
                pr::sqrt(1 + muinv2) - float_type(0.5);
        }
    }

    /**
     * @brief Distribution of slopes.
     *
     * @f[
     *      P_{22}([m_x\; m_y]^\top) = 
     *      \frac{1}{\pi\alpha_x\alpha_y}
     *      \frac{1}{(1 + m_x^2/\alpha_x^2 + m_y^2/\alpha_y^2)^2}
     * @f]
     *
     * @param[in] m
     * Slope.
     */
    float_type p22(multi<float_type, 2> m) const
    {
        float_type zeta = 1 +
            m[0] * m[0] / (alpha_[0] * alpha_[0]) +
            m[1] * m[1] / (alpha_[1] * alpha_[1]);
        return pr::numeric_constants<float_type>::M_1_pi() / 
                    (alpha_[0] * alpha_[1] * zeta * zeta);
    }

    /**
     * @brief Distribution of slopes sampling routine, without roughness.
     *
     * @param[in] u
     * Sample in @f$ [0, 1)^2 @f$.
     *
     * @param[in] cos_thetao
     * Cosine of viewing angle.
     */
    multi<float_type, 2> p11_sample(
            multi<float_type, 2> u, float_type cos_thetao) const
    {
        // Slope.
        multi<float_type, 2> m = {
            pr::numeric_limits<float_type>::quiet_NaN(),
            pr::numeric_limits<float_type>::quiet_NaN()
        };

        // Trig terms.
        cos_thetao = pr::fmax(cos_thetao, float_type(-1));
        cos_thetao = pr::fmin(cos_thetao, float_type(+1));
        float_type sec_thetao = 1 / cos_thetao;
        float_type sin_thetao = pr::sqrt(1 - cos_thetao * cos_thetao);
        float_type tan_thetao = sin_thetao / cos_thetao;
        float_type cot_thetao = cos_thetao / sin_thetao;
        if (pr::isfinite(sec_thetao) &&
            pr::isfinite(tan_thetao) &&
            pr::isfinite(cot_thetao)) {

            // Slope x.
            float_type mu = u[0] * (1 + sec_thetao) - 1;
            float_type nu = 1 / (1 - mu * mu);
            float_type q = 
                    pr::sqrt(
                    pr::fmax(float_type(0),
                        mu * mu * nu -
                        nu * (1 - nu) * tan_thetao * tan_thetao));
            float_type t0 = -nu * tan_thetao - q;
            float_type t1 = -nu * tan_thetao + q;
            m[0] = mu < 0 || t1 > cot_thetao ? t0 : t1;

            // Slope y.
            if (u[1] > float_type(0.5)) {
                u[1] = 2 * u[1] - 1;
                m[1] = 1;
            }
            else {
                u[1] = 1 - 2 * u[1];
                m[1] = -1;
            }
            m[1] *= pr::sqrt(1 + m[0] * m[0]) * 
                (u[1] * 
                (u[1] * 
                (u[1] * float_type(0.273850) - 
                        float_type(0.733690)) + 
                        float_type(0.463410))) / 
                (u[1] *
                (u[1] * 
                (u[1] * float_type(0.093073) + 
                        float_type(0.309420)) - 
                        float_type(1.000000)) + 
                        float_type(0.597999));
        }

        // Overflow? This may occur near normal incidence.
        if (!pr::isfinite(m[0]) ||
            !pr::isfinite(m[1])) {

            // Uniform sample.
            float_type r = pr::sqrt(u[0] / (1 - u[0]));
            float_type phi = 
                2 * pr::numeric_constants<float_type>::M_pi() * u[1];
            m = {
                r * pr::cos(phi),
                r * pr::sin(phi)
            };
        }
        return m;
    }
};

// TODO beckmann_microsurface

/**@}*/

} // namespace pr

#endif // #ifndef PREFORM_MICROSURFACE_HPP
