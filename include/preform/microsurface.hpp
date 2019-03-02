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

// for pr::pcg32, pr::uniform_real_distribution, ...
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
 * @brief Trowbridge-Reitz microsurface slope distribution.
 */
template <typename T>
struct trowbridge_reitz_microsurface_slope
{
public:

    // Sanity check.
    static_assert(
        std::is_floating_point<T>::value,
        "T must be floating point");

    /**
     * @brief Float type.
     */
    typedef T float_type;
    
    /**
     * @brief Non-constructible.
     */
    trowbridge_reitz_microsurface_slope() = delete;

    /**
     * @brief Smith shadowing term.
     *
     * @f[
     *      \Lambda_{11}(\omega_o) = 
     *      \frac{1}{2}\operatorname{sign}(\omega_{o_z})
     *      \sqrt{1 + 1/a^2} - \frac{1}{2}
     * @f]
     * where
     * @f[
     *      a = \frac{1}{
     *          \sqrt{\omega_{o_x}^2/\omega_{o_z}^2 +
     *                \omega_{o_y}^2/\omega_{o_z}^2}}
     * @f]
     * 
     * @param[in] wo
     * Viewing direction.
     */
    static float_type lambda11(multi<float_type, 3> wo)
    {
        float_type tmp0 = wo[0] / wo[2];
        float_type tmp1 = wo[1] / wo[2];
        float_type inva2 = tmp0 * tmp0 + tmp1 * tmp1;

        // Overflow?
        if (!pr::isfinite(inva2)) {
            return pr::signbit(wo[2]) ? -1 : 0;
        }
        else {
            // Evaluate.
            return 
                pr::copysign(float_type(0.5), wo[2]) *
                pr::sqrt(1 + inva2) - float_type(0.5);
        }
    }

    /**
     * @brief Projected area.
     *
     * @f[
     *      A_{\perp11}(\omega_o) = 
     *          (1 + \Lambda_{11}(\omega_o))\omega_{o_z} = 
     *          \frac{1}{2}\omega_{o_z} + 
     *          \frac{1}{2}\lVert{\omega_o}\rVert
     * @f]
     *
     * @param[in] wo
     * Viewing direction.
     */
    static float_type aperp11(multi<float_type, 3> wo)
    {
        return float_type(0.5) * wo[2] + 
               float_type(0.5) * pr::length(wo);
    }

    /**
     * @brief Distribution of slopes.
     *
     * @f[
     *      P_{11}([m_x\; m_y]^\top) = 
     *      \frac{1}{\pi}
     *      \frac{1}{(1 + m_x^2 + m_y^2)^2}
     * @f]
     *
     * @param[in] m
     * Slope.
     */
    static float_type p11(multi<float_type, 2> m)
    {
        float_type zeta = 1 +
            m[0] * m[0] +
            m[1] * m[1];
        return pr::numeric_constants<float_type>::M_1_pi() / (zeta * zeta);
    }

    /**
     * @brief Distribution of slopes sampling routine.
     *
     * @param[in] u
     * Sample in @f$ [0, 1)^2 @f$.
     *
     * @param[in] cos_thetao
     * Cosine of viewing angle.
     */
    static multi<float_type, 2> p11_sample(
                multi<float_type, 2> u, float_type cos_thetao)
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

/**
 * @brief Beckmann microsurface slope distribution.
 */
template <typename T>
struct beckmann_microsurface_slope
{
public:

    // Sanity check.
    static_assert(
        std::is_floating_point<T>::value,
        "T must be floating point");

    /**
     * @brief Float type.
     */
    typedef T float_type;
    
    /**
     * @brief Non-constructible.
     */
    beckmann_microsurface_slope() = delete;

    /**
     * @brief Smith shadowing term.
     *
     * @f[
     *      \Lambda_{11}(\omega_o) = 
     *      \frac{1}{2\sqrt{\pi}} 
     *      \frac{1}{a}\exp(-a^2) -
     *      \frac{1}{2}\operatorname{erfc}(a)
     * @f]
     * where
     * @f[
     *      a = \frac{1}{
     *          \sqrt{\omega_{o_x}^2/\omega_{o_z}^2 +
     *                \omega_{o_y}^2/\omega_{o_z}^2}}
     * @f]
     * 
     * @param[in] wo
     * Viewing direction.
     */
    static float_type lambda11(multi<float_type, 3> wo)
    {
        // TODO Verify
        float_type a = 
            1 / pr::hypot(
                wo[0] / wo[2],
                wo[1] / wo[2]);

        // Overflow?
        if (!pr::isfinite(a)) {
            return pr::signbit(wo[2]) ? -1 : 0;
        }
        else {
            // Evaluate.
            return float_type(0.5) * 
                (pr::numeric_constants<float_type>::M_sqrt1_pi() * 
                 pr::exp(-a * a) / a - pr::erfc(a));
        }
    }

    /**
     * @brief Projected area.
     *
     * @f[
     *      A_{\perp11}(\omega_o) = 
     *          (1 + \Lambda_{11}(\omega_o))\omega_{o_z}
     * @f]
     *
     * @param[in] wo
     * Viewing direction.
     */
    static float_type aperp11(multi<float_type, 3> wo)
    {
        return (1 + lambda11(wo)) * wo[2];
    }

    /**
     * @brief Distribution of slopes.
     *
     * @f[
     *      P_{11}([m_x\; m_y]^\top) = 
     *      \frac{1}{\pi} \exp(-m_x^2 - m_y^2)
     * @f]
     *
     * @param[in] m
     * Slope.
     */
    static float_type p11(multi<float_type, 2> m)
    {
        return pr::numeric_constants<float_type>::M_1_pi() *
               pr::exp(-(m[0] * m[0] + m[1] * m[1]));
    }

    /**
     * @brief Distribution of slopes sampling routine.
     *
     * @param[in] u
     * Sample in @f$ [0, 1)^2 @f$.
     *
     * @param[in] cos_thetao
     * Cosine of viewing angle.
     */
    static multi<float_type, 2> p11_sample(
                multi<float_type, 2> u, float_type cos_thetao)
    {
        // TODO
        return {};
    }
};

/**
 * @brief Uniform microsurface height distribution.
 */
template <typename T>
struct uniform_microsurface_height
{
public:

    // Sanity check.
    static_assert(
        std::is_floating_point<T>::value,
        "T must be floating point");

    /**
     * @brief Float type.
     */
    typedef T float_type;

    /**
     * @brief Non-constructible.
     */
    uniform_microsurface_height() = delete;

    /**
     * @brief Height probability density function.
     *
     * @param[in] h
     * Height.
     */
    __attribute__((always_inline))
    static float_type p1(float_type h)
    {
        return pr::uniform_real_distribution<float_type>(-1, +1).pdf(h);
    }

    /**
     * @brief Height cumulative distribution function.
     *
     * @param[in] h
     * Height.
     */
    __attribute__((always_inline))
    static float_type c1(float_type h)
    {
        return pr::uniform_real_distribution<float_type>(-1, +1).cdf(h);
    }

    /**
     * @brief Height cumulative distribution function inverse.
     *
     * @param[in] u
     * Sample in @f$ [0, 1) @f$.
     */
    __attribute__((always_inline))
    static float_type c1inv(float_type u)
    {
        return pr::uniform_real_distribution<float_type>(-1, +1).cdfinv(u);
    }
};

/**
 * @brief Normal microsurface height distribution.
 */
template <typename T>
struct normal_microsurface_height
{
public:

    // Sanity check.
    static_assert(
        std::is_floating_point<T>::value,
        "T must be floating point");

    /**
     * @brief Float type.
     */
    typedef T float_type;

    /**
     * @brief Non-constructible.
     */
    normal_microsurface_height() = delete;

    /**
     * @brief Height probability density function.
     *
     * @param[in] h
     * Height.
     */
    __attribute__((always_inline))
    static float_type p1(float_type h)
    {
        return pr::normal_distribution<float_type>(0, 1).pdf(h);
    }

    /**
     * @brief Height cumulative distribution function.
     *
     * @param[in] h
     * Height.
     */
    __attribute__((always_inline))
    static float_type c1(float_type h)
    {
        return pr::normal_distribution<float_type>(0, 1).cdf(h);
    }

    /**
     * @brief Height cumulative distribution function inverse.
     *
     * @param[in] u
     * Sample in @f$ [0, 1) @f$.
     */
    __attribute__((always_inline))
    static float_type c1inv(float_type u)
    {
        return pr::normal_distribution<float_type>(0, 1).cdfinv(u);
    }
};

/**
 * @brief Microsurface adapter.
 *
 * This implementation is based on the _Multiple-scattering microfacet BSDFs 
 * with the Smith model_ sample code by Eric Heitz.
 *
 * @tparam Tslope 
 * Slope distribution.
 *
 * @tparam Theight
 * Height distribution.
 */
template <typename Tslope, typename Theight>
struct microsurface_adapter
{
public:

    /**
     * @brief Float type.
     */
    typedef typename Tslope::float_type float_type;

    /**
     * @brief Default constructor.
     */
    microsurface_adapter() = default;

    /**
     * @brief Constructor.
     *
     * @param[in] alpha
     * Roughness.
     */
    microsurface_adapter(multi<float_type, 2> alpha) : alpha_(alpha)
    {
    }

    /**
     * @brief Smith shadowing term.
     *
     * @f[
     *      \Lambda(\omega_o) = 
     *      \Lambda_{11}(
     *      [\alpha_x\omega_{o_x}\;
     *       \alpha_y\omega_{o_y}\;
     *       \omega_{o_z}]^\top)
     * @f]
     *
     * @param[in] wo
     * Viewing direction.
     */
    float_type lambda(multi<float_type, 3> wo) const
    {
        // Warp.
        wo[0] *= alpha_[0];
        wo[1] *= alpha_[1];

        // Delegate.
        return Tslope::lambda11(wo);
    }

    /**
     * @brief Projected area.
     *
     * @f[
     *      A_{\perp}(\omega_o) =
     *      A_{\perp11}(
     *      [\alpha_x\omega_{o_x}\;
     *       \alpha_y\omega_{o_y}\;
     *       \omega_{o_z}]^\top)
     * @f]
     *
     * @param[in] wo
     * Viewing direction.
     */
    float_type aperp(multi<float_type, 3> wo) const
    {
        // Warp.
        wo[0] *= alpha_[0];
        wo[1] *= alpha_[1];

        // Delegate.
        return Tslope::aperp11(wo);
    }

    /**
     * @brief Distribution of slopes.
     *
     * @f[
     *      P_{22}([m_x\; m_y]^\top) = \frac{1}{\alpha_x \alpha_y} 
     *      P_{11}([m_x/\alpha_x\; m_y/\alpha_y]^\top)
     * @f]
     *
     * @param[in] m
     * Slope.
     */
    float_type p22(multi<float_type, 2> m) const
    {
        // Warp.
        m /= alpha_;
        if (!pr::isfinite(m).all()) {
            return 0;
        }

        // Delegate.
        float_type p11 = Tslope::p11(m);
        return pr::isfinite(p11 / alpha_.prod())  ?
                            p11 / alpha_.prod() : 0;
    }

    /**
     * @brief Distribution of normals.
     *
     * @f[
     *      D(\omega_m) = 
     *      \frac{1}{\cos^4{\theta_m}} 
     *      P_{22}(-[\omega_{m_x}\;
     *               \omega_{m_y}]^\top / \cos{\theta_m})
     * @f]
     *
     * @param[in] wm
     * Normal direction.
     */
    float_type d(multi<float_type, 3> wm) const
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
        float_type p22_m = p22(m);
        return pr::isfinite(p22_m / cos4_thetam) ?
                            p22_m / cos4_thetam : 0;
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

        // Area factors.
        float_type fac_numer = pr::dot(wo, wm);
        float_type fac_denom = aperp(wo);
        if (pr::signbit(fac_numer) ||
            pr::signbit(fac_denom)) {
            return 0;
        }

        // Distribution of visible normals.
        float_type d_wm = d(wm);
        return pr::isfinite(d_wm * fac_numer / fac_denom) ?
                            d_wm * fac_numer / fac_denom : 0;
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
        multi<float_type, 3> wo11 = normalize(
        multi<float_type, 3>{
            alpha_[0] * wo[0],
            alpha_[1] * wo[1],
            wo[2]
        });

        // Sample visible slope.
        multi<float_type, 2> m11 = Tslope::p11_sample(u, wo11[2]);

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
                return normalize(wm);
            }
        }

        // Normalize.
        multi<float_type, 3> wm = {-m[0], -m[1], 1};
        return normalize(wm);
    }

    /**
     * @brief Height-averaged shadowing function.
     *
     * @f[
     *      G_1(\omega_o) = \chi^+(\cos{\theta_o})
     *      \frac{1}{1 + \Lambda(\omega_o)}
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
        else {
            return 1 / (1 + lambda(wo));
        }
    }

    /**
     * @brief Height-specific shadowing function.
     *
     * @f[
     *      G_1(\omega_o, h_0) = \chi^+(\cos{\theta_o})
     *      C_1(h_0)^{\Lambda(\omega_o)}
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
        else {
            return pr::pow(Theight::c1(h0), lambda(wo));
        }
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
        // Handle cos(thetao) ~= +1.
        if (wo[2] > float_type(+0.9999)) {
            return pr::numeric_limits<float_type>::infinity(); // Exit
        }

        // Handle cos(thetao) ~= -1.
        if (wo[2] < float_type(-0.9999)) {
            return Theight::c1inv(
                   Theight::c1(h0) * u);
        }

        // Handle cos(thetao) ~= 0.
        if (pr::fabs(wo[2]) < float_type(0.0001)) {
            return h0;
        }

        // Shadowing is probability of intersection.
        float_type g = g1(wo, h0);
        if (u > 1 - g) {
            return pr::numeric_limits<float_type>::infinity(); // Exit
        }

        // Intersect.
        return Theight::c1inv(
               Theight::c1(h0) /
                    pr::pow(1 - u, 
                            1 / lambda(wo)));
    }

private:

    /**
     * @brief Roughness @f$ [\alpha_x\; \alpha_y]^\top @f$.
     */
    multi<float_type, 2> alpha_ = multi<float_type, 2>(1);
};

// TODO diffuse_microsurface_adapter

/**@}*/

} // namespace pr

#endif // #ifndef PREFORM_MICROSURFACE_HPP
