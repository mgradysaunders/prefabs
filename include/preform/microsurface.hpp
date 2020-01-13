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
#if !(__cplusplus >= 201703L)
#error "preform/microsurface.hpp requires >=C++17"
#endif // #if !(__cplusplus >= 201703L)
#endif // #if !DOXYGEN
#pragma once
#ifndef PREFORM_MICROSURFACE_HPP
#define PREFORM_MICROSURFACE_HPP

// for assert
#include <cassert>

// for pr::multi
#include <preform/multi.hpp>

// for pr::multi wrappers
#include <preform/multi_math.hpp>

// for pr::multi wrappers, pr::uniform_real_distribution, ...
#include <preform/multi_random.hpp>

// for pr::fresnel_diel, ...
#include <preform/fresnel.hpp>

namespace pr {

/**
 * @defgroup microsurface Microsurface
 *
 * `<preform/microsurface.hpp>`
 *
 * __C++ version__: >=C++17
 *
 * ### References
 *
 * 1. B. Walter, S. R. Marschner, H. Li, and K. E. Torrance,
 * &ldquo;Microfacet models for refraction through rough surfaces,&rdquo;
 * in _Proceedings of the 18th Eurographics Conference on Rendering
 * Techniques_, ser. EGSR&rsquo;07, Grenoble, France:
 * Eurographics Association, 2007, pp. 195&ndash;206.
 * ISBN: 978-3-905673-52-4.
 * Available: http://dx.doi.org/10.2312/EGWR/EGSR07/195-206
 * 2. E. Heitz, &ldquo;Understanding the masking-shadowing function in
 * microfacet-based BRDFs,&rdquo; _Journal of Computer Graphics Techniques
 * (JCGT)_, vol. 3, no. 2, pp. 48&ndash;107, Jun. 2014,
 * ISSN: 2331-7418.
 * Available: http://jcgt.org/published/0003/02/03/
 * 3. E. Heitz, J. Hanika, E. d&rsquo;Eon, and C. Dachsbacher,
 * &ldquo;Multiple-scattering microfacet BSDFs with the Smith model,&rdquo;
 * _ACM Transactions on Graphics_, vol. 35, no. 4, 1&ndash;58, Jul. 2016,
 * ISSN: 0730-0301.
 * Available: http://doi.acm.org/10.1145/2897824.2925943
 * 4. B. Burley, &ldquo;Physically based shading at Disney,&rdquo;
 * Disney Enterprises, Technical report, Aug. 2012.
 * Available: https://disney-animation.s3.amazonaws.com/library/s2012_pbs_disney_brdf_notes_v2.pdf
 * 5. M. Oren and S. K. Nayar, &ldquo;Generalization of Lambert&rsquo;s
 * reflectance model,&rdquo; in _Proceedings of the 21st Annual Conference
 * on Computer Graphics and Interactive Techniques_, ser. SIGGRAPH&rsquo;94,
 * New York, NY, USA: ACM, 1994, pp. 239&ndash;246, ISBN: 0-89791-667-0.
 * Available: http://doi.acm.org/10.1145/192161.192213
 *
 * @see
 * Eric Heitz's [page][1].
 * [1]: https://eheitzresearch.wordpress.com/240-2/
 */
/**@{*/

/**
 * @brief Microsurface Trowbridge-Reitz slope distribution.
 *
 * @tparam Tfloat
 * Float type.
 */
template <typename Tfloat>
struct microsurface_trowbridge_reitz_slope
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
     * @brief Non-constructible.
     */
    microsurface_trowbridge_reitz_slope() = delete;

    /**
     * @brief Smith shadowing term.
     *
     * @param[in] wo
     * Viewing direction.
     *
     * @par Expression
     * @f[
     *      \Lambda_{11}(\omega_o) =
     *      \frac{1}{2}\operatorname{sign}(\omega_{o_z})
     *      \sqrt{1 + 1/a^2} - \frac{1}{2} =
     *      \frac{1}{2}\frac{\lVert{\omega_o}\rVert}{\omega_{o_z}} -
     *      \frac{1}{2}
     * @f]
     * where
     * @f[
     *      a = \frac{\omega_{o_z}}{
     *          \sqrt{\omega_{o_x}^2 + \omega_{o_y}^2}}
     * @f]
     */
    static float_type lambda11(multi<float_type, 3> wo)
    {
        return float_type(0.5) * pr::length(wo) / wo[2] - float_type(0.5);
    }

    /**
     * @brief Projected area.
     *
     * @param[in] wo
     * Viewing direction.
     *
     * @par Expression
     * @f[
     *      A_{\perp11}(\omega_o) =
     *          (1 + \Lambda_{11}(\omega_o))\omega_{o_z} =
     *          \frac{1}{2}\lVert{\omega_o}\rVert +
     *          \frac{1}{2}\omega_{o_z}
     * @f]
     */
    static float_type aperp11(multi<float_type, 3> wo)
    {
        return float_type(0.5) * pr::length(wo) + float_type(0.5) * wo[2];
    }

    /**
     * @brief Distribution of slopes.
     *
     * @param[in] m
     * Slope.
     *
     * @par Expression
     * @f[
     *      P_{11}([m_x\; m_y]^\top) =
     *      \frac{1}{\pi}
     *      \frac{1}{(1 + m_x^2 + m_y^2)^2}
     * @f]
     */
    static float_type p11(multi<float_type, 2> m)
    {
        return pr::numeric_constants<float_type>::M_1_pi() /
               pr::nthpow(1 + pr::dot(m, m), 2);
    }

    /**
     * @brief Distribution of slopes sample.
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
        // Sanity check.
        assert(
            cos_thetao >= -1 &&
            cos_thetao <= +1);

        // Handle cos(thetao) ~= +1.
        if (cos_thetao > float_type(0.99999)) {
            float_type r = pr::sqrt(u[0] / (1 - u[0]));
            float_type phi =
                2 * pr::numeric_constants<float_type>::M_pi() * u[1];
            return {
                r * pr::cos(phi),
                r * pr::sin(phi)
            };
        }

        // Trig terms.
        float_type sec_thetao = 1 / cos_thetao;
        float_type sin_thetao = pr::sqrt(1 - cos_thetao * cos_thetao);
        float_type tan_thetao = sin_thetao / cos_thetao;
        float_type cot_thetao = cos_thetao / sin_thetao;

        // Slope.
        multi<float_type, 2> m;

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

        return m;
    }
};

/**
 * @brief Microsurface Beckmann slope distribution.
 *
 * @tparam Tfloat
 * Float type.
 */
template <typename Tfloat>
struct microsurface_beckmann_slope
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
     * @brief Non-constructible.
     */
    microsurface_beckmann_slope() = delete;

    /**
     * @brief Smith shadowing term.
     *
     * @param[in] wo
     * Viewing direction.
     *
     * @par Expression
     * @f[
     *      \Lambda_{11}(\omega_o) =
     *          \frac{1}{2a\sqrt{\pi}}e^{-a^2} -
     *          \frac{1}{2}\operatorname{erfc}(a)
     * @f]
     * where
     * @f[
     *      a = \frac{\omega_{o_z}}{
     *          \sqrt{\omega_{o_x}^2 + \omega_{o_y}^2}}
     * @f]
     */
    static float_type lambda11(multi<float_type, 3> wo)
    {
        float_type a = wo[2] / pr::hypot(wo[0], wo[1]);
        return (pr::numeric_constants<float_type>::M_2_sqrtpi() / 2 *
                pr::exp(-a * a) / a - pr::erfc(a)) / 2;
    }

    /**
     * @brief Projected area.
     *
     * @param[in] wo
     * Viewing direction.
     *
     * @par Expression
     * @f[
     *      A_{\perp11}(\omega_o) =
     *          (1 + \Lambda_{11}(\omega_o))\omega_{o_z} =
     *          \frac{1}{2\sqrt{\pi}}
     *          \sqrt{\omega_{o_x}^2 +
     *                \omega_{o_y}^2}e^{-a^2} +
     *          \frac{1}{2}\omega_{o_z}\operatorname{erfc}(-a)
     * @f]
     * where
     * @f[
     *      a = \frac{\omega_{o_z}}{
     *          \sqrt{\omega_{o_x}^2 + \omega_{o_y}^2}}
     * @f]
     */
    static float_type aperp11(multi<float_type, 3> wo)
    {
        float_type r = pr::hypot(wo[0], wo[1]);
        float_type a = wo[2] / r;
        return (pr::numeric_constants<float_type>::M_2_sqrtpi() / 2 *
                pr::exp(-a * a) * r + wo[2] * pr::erfc(-a)) / 2;
    }

    /**
     * @brief Distribution of slopes.
     *
     * @param[in] m
     * Slope.
     *
     * @par Expression
     * @f[
     *      P_{11}([m_x\; m_y]^\top) =
     *      \frac{1}{\pi} e^{-m_x^2 - m_y^2}
     * @f]
     */
    static float_type p11(multi<float_type, 2> m)
    {
        return pr::numeric_constants<float_type>::M_1_pi() *
               pr::exp(-pr::dot(m, m));
    }

    /**
     * @brief Distribution of slopes sample.
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
        // Sanity check.
        assert(
            cos_thetao >= -1 &&
            cos_thetao <= +1);

        // Handle cos(thetao) ~= +1.
        if (cos_thetao > float_type(0.99999)) {
            float_type r = pr::sqrt(pr::log(-u[0]));
            float_type phi =
                2 * pr::numeric_constants<float_type>::M_pi() * u[1];
            return {
                r * pr::cos(phi),
                r * pr::sin(phi)
            };
        }

        // Trig terms.
        float_type cos2_thetao = cos_thetao * cos_thetao;
        float_type sin2_thetao = 1 - cos2_thetao;
        float_type sin_thetao = pr::sqrt(sin2_thetao);
        float_type cot_thetao = cos_thetao / sin_thetao;

        auto c11 = [=](float_type a) {
            return
                (pr::numeric_constants<float_type>::M_2_sqrtpi() / 2 *
                 sin_thetao * pr::exp(-a * a) + cos_thetao * pr::erfc(-a)) / 2;
        };

        // Projected area.
        float_type aperp = c11(cot_thetao);
        float_type cnorm = 1 / aperp;
        if (aperp < float_type(0.00001) || !pr::isfinite(aperp)) {
            return {0, 0};
        }

        // Search.
        float_type erf_amin = float_type(-0.99999);
        float_type erf_amax = pr::fmax(erf_amin, pr::erf(cot_thetao));
        float_type erf_a =
            float_type(0.5) * erf_amin +
            float_type(0.5) * erf_amax;
        while (erf_amax - erf_amin > float_type(0.00001)) {

            // Out of bounds?
            if (!(erf_a >= erf_amin &&
                  erf_a <= erf_amax))  {
                // Center.
                erf_a =
                    float_type(0.5) * erf_amin +
                    float_type(0.5) * erf_amax;
            }

            // Evaluate.
            float_type a = pr::erfinv(erf_a);
            float_type c = (a >= cot_thetao ? 1 : cnorm * c11(a)) - u[0];
            if (pr::fabs(c) <= float_type(0.00001)) {
                // Convergence.
                break;
            }

            if (pr::signbit(c)) {
                if (erf_amin != erf_a) {
                    erf_amin = erf_a;
                }
                else {
                    // Convergence.
                    break;
                }
            }
            else {
                if (erf_amax != erf_a) {
                    erf_amax = erf_a;
                }
                else {
                    // Convergence.
                    break;
                }
            }

            // Newton-Raphson update.
            float_type dc_derf_a =
            float_type(0.5) * cnorm *
                    (cos_thetao - a * sin_thetao);
            erf_a -= c / dc_derf_a;
        }

        // Done.
        erf_a = pr::fmin(erf_a, erf_amax);
        erf_a = pr::fmax(erf_a, erf_amin);
        return {
            pr::erfinv(erf_a),
            pr::erfinv(2 * u[1] - 1)
        };
    }
};

/**
 * @brief Microsurface uniform height distribution.
 *
 * @tparam Tfloat
 * Float type.
 */
template <typename Tfloat>
struct microsurface_uniform_height
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
     * @brief Non-constructible.
     */
    microsurface_uniform_height() = delete;

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
 * @brief Microsurface normal/Gaussian height distribution.
 *
 * @tparam Tfloat
 * Float type.
 */
template <typename Tfloat>
struct microsurface_normal_height
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
     * @brief Non-constructible.
     */
    microsurface_normal_height() = delete;

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
 * @brief Microsurface prototype.
 *
 * @tparam Tchild
 * Child type for Curiously Recurring Template Pattern (CRTP).
 *
 * @tparam bidir_mis
 * Boolean to enable bidirectional Multiple Importance Sampling (MIS).
 */
template <typename Tchild, bool bidir_mis>
struct microsurface;

/**
 * @brief Microsurface.
 *
 * @tparam Tchild
 * Child type for Curiously Recurring Template Pattern (CRTP).
 *
 * @tparam Tfloat
 * Float type.
 *
 * @tparam Tslope
 * Slope distribution, either `microsurface_trowbridge_reitz_slope` or
 * `microsurface_beckmann_slope`.
 *
 * @tparam Theight
 * Height distribution, either `microsurface_uniform_height` or
 * `microsurface_normal_height`.
 *
 * @tparam bidir_mis
 * Boolean to enable bidirectional Multiple Importance Sampling (MIS).
 */
template <
    template <
#if !DOXYGEN
        typename,
        template <typename> typename,
        template <typename> typename
#else
        ...
#endif // #if !DOXYGEN
        > typename Tchild,
    typename Tfloat,
    template <typename> typename Tslope,
    template <typename> typename Theight,
    bool bidir_mis
    >
struct microsurface<Tchild<Tfloat, Tslope, Theight>, bidir_mis>
{
public:

    /**
     * @brier Child.
     */
    typedef Tchild<Tfloat, Tslope, Theight> child;

    /**
     * @brief Float type.
     */
    typedef Tfloat float_type;

    /**
     * @brief Default constructor.
     */
    microsurface() = default;

    /**
     * @brief Constructor.
     *
     * @param[in] alpha
     * Roughness.
     */
    microsurface(multi<float_type, 2> alpha) : alpha_(alpha)
    {
        // Clamp.
        alpha_[0] = pr::fmax(alpha_[0], float_type(0.00001));
        alpha_[1] = pr::fmax(alpha_[1], float_type(0.00001));
    }

    /**
     * @brief Smith shadowing term.
     *
     * @param[in] wo
     * Viewing direction.
     *
     * @par Expression
     * @f[
     *      \Lambda(\omega_o) =
     *      \Lambda_{11}(
     *      [\alpha_x\omega_{o_x}\;
     *       \alpha_y\omega_{o_y}\;
     *       \omega_{o_z}]^\top)
     * @f]
     */
    float_type lambda(multi<float_type, 3> wo) const
    {
        // Warp.
        wo[0] *= alpha_[0];
        wo[1] *= alpha_[1];

        // Delegate.
        return Tslope<Tfloat>::lambda11(wo);
    }

    /**
     * @brief Projected area.
     *
     * @param[in] wo
     * Viewing direction.
     *
     * @par Expression
     * @f[
     *      A_{\perp}(\omega_o) =
     *      A_{\perp11}(
     *      [\alpha_x\omega_{o_x}\;
     *       \alpha_y\omega_{o_y}\;
     *       \omega_{o_z}]^\top)
     * @f]
     */
    float_type aperp(multi<float_type, 3> wo) const
    {
        // Warp.
        wo[0] *= alpha_[0];
        wo[1] *= alpha_[1];

        // Delegate.
        return Tslope<Tfloat>::aperp11(wo);
    }

    /**
     * @brief Distribution of slopes.
     *
     * @param[in] m
     * Slope.
     *
     * @par Expression
     * @f[
     *      P_{22}([m_x\; m_y]^\top) = \frac{1}{\alpha_x \alpha_y}
     *      P_{11}([m_x/\alpha_x\; m_y/\alpha_y]^\top)
     * @f]
     */
    float_type p22(multi<float_type, 2> m) const
    {
        // Warp.
        m /= alpha_;
        if (!pr::isfinite(m).all()) {
            return 0;
        }

        // Delegate.
        float_type p11 = Tslope<Tfloat>::p11(m);
        return pr::isfinite(p11 / alpha_.prod())  ?
                            p11 / alpha_.prod() : 0;
    }

    /**
     * @brief Distribution of normals.
     *
     * @param[in] wm
     * Normal direction.
     *
     * @par Expression
     * @f[
     *      D(\omega_m) =
     *      \frac{1}{\cos^4{\theta_m}}
     *      P_{22}(-[\omega_{m_x}\;
     *               \omega_{m_y}]^\top / \cos{\theta_m})
     * @f]
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
     * @param[in] wo
     * Viewing direction.
     *
     * @param[in] wm
     * Normal direction.
     *
     * @par Expression
     * @f[
     *      D_{\omega_o}(\omega_m) = D(\omega_m)
     *         \frac{\langle{\omega_o, \omega_m}\rangle}{A_{\perp}(\omega_o)}
     * @f]
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
        float_type fac_numer = dot(wo, wm);
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
     * @brief Distribution of visible normals sample.
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
        multi<float_type, 3> wo11 = normalize_safe(
        multi<float_type, 3>{
            alpha_[0] * wo[0],
            alpha_[1] * wo[1],
            wo[2]
        });

        // Sample visible slope.
        multi<float_type, 2> m11 = Tslope<Tfloat>::p11_sample(u, wo11[2]);

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
                return normalize_safe(wm);
            }
        }

        // Normalize.
        multi<float_type, 3> wm = {-m[0], -m[1], 1};
        return normalize_safe(wm);
    }

    /**
     * @brief Height-averaged shadowing function.
     *
     * @param[in] wo
     * Viewing direction.
     *
     * @par Expression
     * @f[
     *      G_1(\omega_o) = \chi^+(\cos{\theta_o})
     *      \frac{1}{1 + \Lambda(\omega_o)}
     * @f]
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
     * @param[in] wo
     * Viewing direction.
     *
     * @param[in] h0
     * Current height.
     *
     * @par Expression
     * @f[
     *      G_1(\omega_o, h_0) = \chi^+(\cos{\theta_o})
     *      C_1(h_0)^{\Lambda(\omega_o)}
     * @f]
     */
    float_type g1(multi<float_type, 3> wo, float_type h0) const
    {
        if (pr::signbit(wo[2])) {
            return 0;
        }
        else {
            return pr::pow(Theight<Tfloat>::c1(h0), lambda(wo));
        }
    }

    /**
     * @brief Height sample.
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
        if (pr::isnan(h0)) {
            return h0;
        }

        // Handle cos(thetao) ~= +1.
        if (wo[2] > float_type(+0.99999)) {
            return pr::numeric_limits<float_type>::infinity(); // Exit
        }

        // Handle cos(thetao) ~= -1.
        if (wo[2] < float_type(-0.99999)) {
            return Theight<Tfloat>::c1inv(
                   Theight<Tfloat>::c1(h0) * u);
        }

        // Handle cos(thetao) ~= 0.
        if (pr::fabs(wo[2]) < float_type(0.00001)) {
            return h0;
        }

        // Shadowing is probability of intersection.
        float_type g = g1(wo, h0);
        if (u > 1 - g) {
            return pr::numeric_limits<float_type>::infinity(); // Exit
        }

        // Intersect.
        return Theight<Tfloat>::c1inv(
               Theight<Tfloat>::c1(h0) /
                    pr::pow(1 - u,
                            1 / lambda(wo)));
    }

public:

    /**
     * @name Multiple-scattering interface
     *
     * This computes, i.e., stochastically estimates, multiple-scattering 
     * BSDF and BSDF-PDF simultaneously using an implicit Curiously Recurring
     * Template Pattern (CRTP) interface for the microsurface phase
     * and associated sampling routine.
     *
     * Each child structure is expected to implement its microsurface 
     * phase `ps` and sampling routine `ps_sample` with the following 
     * call signatures.
     * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
     * template <typename Tgen>
     * float_type ps(
     *          Tgen& gen, 
     *          const multi<float_type, 3>& wo,
     *          const multi<float_type, 3>& wi,
     *          bool wo_outside,
     *          bool wi_outside,
     *          float_type* ek = nullptr, 
     *                 bool is_importance = false) const;
     *
     * template <typename Tgen>
     * multi<float_type, 3> ps_sample(
     *          Tgen& gen,
     *          const multi<float_type, 3>& wo,
     *          bool  wo_outside,
     *          bool& wi_outside,
     *          float_type* ek = nullptr, 
     *                 bool is_importance = false) const;
     * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
     */
    /**@{*/

    /**
     * @brief Multiple-scattering BSDF.
     *
     * @param[inout] gen
     * Generator.
     *
     * @param[in] wo
     * Outgoing direction.
     *
     * @param[in] wi
     * Incident direction.
     *
     * @param[in] kmin
     * Scattering order minimum.
     *
     * @param[in] kmax
     * Scattering order maximum, 0 for all orders.
     *
     * @param[out] pdf
     * _Optional_. Output BSDF-PDF.
     */
    template <typename Tgen>
    float_type fs(
            Tgen& gen,
            const multi<float_type, 3>& wo,
            const multi<float_type, 3>& wi,
            unsigned kmin = 0,
            unsigned kmax = 0,
            float_type* pdf = nullptr) const
    {
        float_type fs_;
        float_type fs_pdf_;
        compute_(gen, wo, wi, kmin, kmax, fs_, fs_pdf_);
        if (pdf) {
            *pdf = fs_pdf_;
        }
        return fs_;
    }

    /**
     * @brief Multiple-scattering BSDF-PDF.
     *
     * @param[inout] gen
     * Generator.
     *
     * @param[in] wo
     * Outgoing direction.
     *
     * @param[in] wi
     * Incident direction.
     *
     * @param[in] kmin
     * Scattering order minimum.
     *
     * @param[in] kmax
     * Scattering order maximum, 0 for all orders.
     */
    template <typename Tgen>
    float_type fs_pdf(
            Tgen& gen,
            const multi<float_type, 3>& wo,
            const multi<float_type, 3>& wi,
            unsigned kmin = 0,
            unsigned kmax = 0) const
    {
        float_type fs_;
        float_type fs_pdf_;
        compute_(gen, wo, wi, kmin, kmax, fs_, fs_pdf_);
        return fs_pdf_;
    }

    /**
     * @brief Multiple-scattering BSDF-PDF sample direction.
     *
     * @param[inout] gen
     * Generator.
     *
     * @param[in] wo
     * Outgoing direction.
     *
     * @param[out] k
     * Incident direction scattering order.
     */
    template <typename Tgen>
    multi<float_type, 3> fs_pdf_sample(
            Tgen& gen,
            const multi<float_type, 3>& wo, 
            unsigned& k) const
    {
        // Initial height.
        float_type hk = Theight<float_type>::c1inv(float_type(0.99999)) + 1;

        // Initial direction.
        multi<float_type, 3> wk = -wo;

        // Initial direction outside?
        bool wk_outside = wo[2] > 0;
        if (!wk_outside) {

            // Flip height.
            hk = -hk;
        }

        for (k = 0; true;) {

            {
                // Sample next height.
                float_type u = pr::generate_canonical<float_type>(gen);
                hk =
                    wk_outside 
                    ? +h_sample(u, +wk, +hk) 
                    : -h_sample(u, -wk, -hk);
                if (pr::isinf(hk)) {
                    break;
                }
            }

            // Increment.
            ++k;

            // Sample next direction.
            wk =
                static_cast<
                const child&>(*this).ps_sample( // CRTP
                        gen,
                        -wk,
                        wk_outside,
                        wk_outside);

            // NaN check.
            if (!pr::isfinite(hk) ||
                !pr::isfinite(wk).all() || wk[2] == 0) {
                return {0, 0, 1};
            }
        }

        return wk;
    }

#if !DOXYGEN
private:

    // Compute estimates of BSDF and BSDF-PDF, 
    // optionally with bidirectional multiple-importance sampling.
    template <typename Tgen>
    void compute_(
            Tgen& gen,
            const multi<float_type, 3>& wo,
            const multi<float_type, 3>& wi,
            unsigned kmin,
            unsigned kmax,
            float_type& fs_,
            float_type& fs_pdf_) const
    {
        // Compute forward contribution.
        compute_path_(
                gen, 
                wo, wi, 
                kmin, 
                kmax,
                fs_, 
                fs_pdf_, 
                false); // false = Radiance

        // If necessary, compute reverse contribution.
        if constexpr (bidir_mis) {
            float_type rev__fs_ = 0;
            float_type rev__fs_pdf_ = 0;
            compute_path_(
                    gen, 
                    wi, wo, 
                    kmin, 
                    kmax, 
                    rev__fs_, 
                    rev__fs_pdf_, 
                    true); // true = Importance

            // Exchange cosine factors.
            rev__fs_ *= pr::fabs(wi[2]);
            rev__fs_ /= pr::fabs(wo[2]);
            rev__fs_pdf_ *= pr::fabs(wi[2]);
            rev__fs_pdf_ /= pr::fabs(wo[2]);

            // Add. 
            fs_ += rev__fs_;
            fs_pdf_ += rev__fs_pdf_;
        }
    }

    // Compute estimates of BSDF and BSDF-PDF, 
    // optionally with bidirectional multiple-importance sampling.
    template <typename Tgen>
    void compute_path_(
            Tgen& gen,
            const multi<float_type, 3>& wo,
            const multi<float_type, 3>& wi,
            unsigned kmin,
            unsigned kmax,
            float_type& fs_,
            float_type& fs_pdf_,
            bool is_importance) const
    {
        // Initialize BSDF.
        fs_ = 0;
        
        // Initialize BSDF-PDF.
        fs_pdf_ = 0;

        // Outgoing direction outside material?
        const bool wo_outside = wo[2] > 0;

        // Incident direction outside material?
        const bool wi_outside = wi[2] > 0;

        // Initial energy.
        float_type ek = 1;

        // Initial height.
        float_type hk = Theight<float_type>::c1inv(float_type(0.99999)) + 1;

        // Initial direction.
        multi<float_type, 3> wk = -wo;

        // Initial direction outside material?
        bool wk_outside = wo_outside;
        if (!wk_outside) {

            // Flip height.
            hk = -hk;
        }

        // MIS weighting term.
        float_type ps1 = 0;
        (void) ps1; // Unused if bidir_mis == false.

        for (unsigned k = 0;
                      kmax == 0 || kmax > k;) {

            {
                // Sample next height.
                float_type u = pr::generate_canonical<float_type>(gen);
                hk =
                    wk_outside 
                    ? +h_sample(u, +wk, +hk) 
                    : -h_sample(u, -wk, -hk);
                if (pr::isinf(hk)) {
                    break;
                }
            }

            // Increment.
            ++k;

            if ((kmax == 0 ||
                 kmax >= k) && kmin <= k) {

                // Next event estimation.
                float_type ek_next = ek;
                float_type psk = 
                    static_cast<
                    const child&>(*this).ps( // CRTP
                            gen,
                            -wk, wi,
                            wk_outside,
                            wi_outside, 
                            &ek_next, is_importance);
                float_type fsk =
                    (wi_outside 
                    ? g1(+wi, +hk) 
                    : g1(-wi, -hk)) * psk;

                // Optionally apply MIS weighting.
                if constexpr (bidir_mis) {
                    fsk *= 
                        k == 1 
                        ? float_type(0.5) 
                        : ps1 / (ps1 + psk);
                }

                if (pr::isfinite(fsk)) {

                    // Add term to BSDF.
                    fs_ += ek_next * fsk;

                    // Add term to BSDF-PDF.
                    fs_pdf_ += fsk;
                }
            }

            // Sample next direction.
            wk =
                static_cast<
                const child&>(*this).ps_sample( // CRTP
                        gen,
                        -wk,
                        wk_outside,
                        wk_outside, 
                        &ek, is_importance);

            // Optionally remember MIS weighting term.
            if constexpr (bidir_mis) {
                if (k == 1) {
                    ps1 = 
                        static_cast<
                        const child&>(*this).ps(
                                gen,
                                wk, wo,
                                wk_outside,
                                wo_outside);
                }    
            }

            // NaN check.
            if (!pr::isfinite(hk) ||
                !pr::isfinite(wk).all() || wk[2] == 0) {

                // Nullify.
                fs_ =
                fs_pdf_ = 0;
                break;
            }
        }
    }

#endif // #if !DOXYGEN

    /**@}*/

private:

    /**
     * @brief Roughness @f$ [\alpha_x\; \alpha_y]^\top @f$.
     */
    multi<float_type, 2> alpha_ = multi<float_type, 2>(1);
};

/**
 * @brief Microsurface from Lambertian BSDF.
 *
 * @tparam Tfloat
 * Float type.
 *
 * @tparam Tslope
 * Slope distribution, either `microsurface_trowbridge_reitz_slope` or
 * `microsurface_beckmann_slope`.
 *
 * @tparam Theight
 * Height distribution, either `microsurface_uniform_height` or
 * `microsurface_normal_height`.
 */
template <
    typename Tfloat,
    template <typename> typename Tslope,
    template <typename> typename Theight
    >
struct microsurface_lambertian_bsdf :
                public microsurface<
                       microsurface_lambertian_bsdf<Tfloat, Tslope, Theight>, 
                       false>
{
public:

    /**
     * @brief Float type.
     */
    typedef Tfloat float_type;

    /**
     * @brief Default constructor.
     */
    microsurface_lambertian_bsdf() = default;

    /**
     * @brief Constructor.
     */
    template <typename... Targs>
    microsurface_lambertian_bsdf(
            float_type r0,
            float_type t0,
            Targs&&... args) :
                microsurface<
                microsurface_lambertian_bsdf<Tfloat, Tslope, Theight>, false>::
                microsurface(std::forward<Targs>(args)...),
                r0_(r0),
                t0_(t0),
                r0_prob_(r0 / (r0 + t0)),
                t0_prob_(t0 / (r0 + t0))
    {
    }

#if 0
    /**
     * @brief Single-scattering BRDF.
     *
     * @f[
     *      f_s(\omega_o, \omega_i) =
     *      \frac{\langle{\omega_m, \omega_i}\rangle}{\pi}
     *      \frac{1 + \Lambda(\omega_o)}
     *           {1 + \Lambda(\omega_o) + \Lambda(\omega_i)} L_0
     * @f]
     * where @f$ \omega_m \sim D_{\omega_o} @f$
     *
     * @param[in] u
     * Sample in @f$ [0, 1)^2 @f$.
     *
     * @param[in] wo
     * Outgoing direction.
     *
     * @param[in] wi
     * Incident direction.
     */
    float_type fs1(
            multi<float_type, 2> u,
            multi<float_type, 3> wo,
            multi<float_type, 3> wi) const
    {
        // Flip.
        if (wo[2] < 0) {
            wo[2] = -wo[2];
            wi[2] = -wi[2];
        }
        if (wo[2] == 0 || !(wi[2] > 0)) {
            return 0;
        }

        // Sample visible microsurface normal.
        multi<float_type, 3> wm = dwo_sample(u, wo);

        // Evaluate.
        float_type lambda_wo = lambda(wo);
        float_type lambda_wi = lambda(wi);
        float_type g2_given_g1 =
                (1 + lambda_wo) /
                (1 + lambda_wo + lambda_wi);
        return pr::numeric_constants<float_type>::M_1_pi() * l0_ *
               pr::fmax(pr::dot(wm, wi), float_type(0)) * g2_given_g1;
    }
#endif

public:

    /**
     * @name Multiple-scattering interface (CRTP)
     */
    /**@{*/

    /**
     * @brief Phase function.
     *
     * @param[inout] gen
     * Generator.
     *
     * @param[in] wo
     * Outgoing direction.
     *
     * @param[in] wi
     * Incident direction.
     *
     * @param[in] wo_outside
     * Outgoing direction outside material?
     *
     * @param[in] wi_outside
     * Incident direction outside material?
     *
     * @param[inout] ek
     * _Optional_. Energy.
     *
     * @param[in] is_importance
     * _Optional_. Energy is importance, not radiance?
     */
    template <typename Tgen>
    float_type ps(
            Tgen& gen,
            const multi<float_type, 3>& wo,
            const multi<float_type, 3>& wi,
            bool wo_outside,
            bool wi_outside,
            float_type* ek = nullptr, 
                   bool is_importance = false) const
    {

        // Sample visible microsurface normal.
        multi<float_type, 2> u0 = pr::generate_canonical<float_type, 2>(gen);
        multi<float_type, 3> wm =
            wo_outside ?
            +this->dwo_sample(u0, +wo) :
            -this->dwo_sample(u0, -wo);

        // Energy.
        if (ek) {
            *ek *= r0_ + t0_;
            (void) is_importance;
        }

        // Evaluate.
        if (wo_outside == wi_outside) {
            return pr::numeric_constants<float_type>::M_1_pi() *
                   pr::max(dot(+wm, wi), float_type(0)) * r0_prob_;
        }
        else {
            return pr::numeric_constants<float_type>::M_1_pi() *
                   pr::max(dot(-wm, wi), float_type(0)) * t0_prob_;
        }
    }

    /**
     * @brief Phase function sample direction.
     *
     * @param[inout] gen
     * Generator.
     *
     * @param[in] wo
     * Outgoing direction.
     *
     * @param[in] wo_outside
     * Outgoing direction outside material?
     *
     * @param[out] wi_outside
     * Incident direction outside material?
     *
     * @param[inout] ek
     * _Optional_. Energy.
     *
     * @param[in] is_importance
     * _Optional_. Energy is importance, not radiance?
     */
    template <typename Tgen>
    multi<float_type, 3> ps_sample(
            Tgen& gen,
            const multi<float_type, 3>& wo,
            bool  wo_outside,
            bool& wi_outside,
            float_type* ek = nullptr, 
                   bool is_importance = false) const
    {
        // Sample visible microsurface normal.
        multi<float_type, 2> u0 = pr::generate_canonical<float_type, 2>(gen);
        multi<float_type, 3> wm =
            wo_outside ?
            +this->dwo_sample(u0, +wo) :
            -this->dwo_sample(u0, -wo);

        // Sample direction.
        multi<float_type, 2> u1 = pr::generate_canonical<float_type, 2>(gen);
        multi<float_type, 3> wi =
        multi<float_type, 3>::cosine_hemisphere_pdf_sample(u1);
        if (pr::generate_canonical<float_type>(gen) < r0_prob_) {
            wi_outside = wo_outside;
        }
        else {
            wi_outside = !wo_outside;
            wi[2] = -wi[2];
        }

        // Energy.
        if (ek) {
            *ek *= r0_ + t0_;
            (void) is_importance;
        }

        // Expand in orthonormal basis.
        return normalize_fast(
               dot(multi<float_type, 3, 3>::build_onb(wm), wi));
    }

    /**@}*/

private:

    /**
     * @brief BRDF albedo @f$ R_0 @f$.
     */
    float_type r0_ = 1;

    /**
     * @brief BTDF albedo @f$ T_0 @f$.
     */
    float_type t0_ = 0;

    /**
     * @brief BRDF probability @f$ R_0 / (R_0 + T_0) @f$.
     */
    float_type r0_prob_ = 1;

    /**
     * @brief BTDF probability @f$ T_0 / (R_0 + T_0) @f$.
     */
    float_type t0_prob_ = 0;
};

/**
 * @brief Microsurface from dielectric BSDF.
 *
 * @tparam Tfloat
 * Float type.
 *
 * @tparam Tslope
 * Slope distribution, either `microsurface_trowbridge_reitz_slope` or
 * `microsurface_beckmann_slope`.
 *
 * @tparam Theight
 * Height distribution, either `microsurface_uniform_height` or
 * `microsurface_normal_height`.
 */
template <
    typename Tfloat,
    template <typename> typename Tslope,
    template <typename> typename Theight
    >
struct microsurface_dielectric_bsdf :
                public microsurface<
                       microsurface_dielectric_bsdf<Tfloat, Tslope, Theight>,
                       true>
{
public:

    /**
     * @brief Float type.
     */
    typedef Tfloat float_type;

    /**
     * @brief Default constructor.
     */
    microsurface_dielectric_bsdf() = default;

    /**
     * @brief Constructor.
     */
    template <typename... Targs>
    microsurface_dielectric_bsdf(
            float_type fr0,
            float_type ft0,
            float_type eta,
            Targs&&... args) :
                microsurface<
                microsurface_dielectric_bsdf<Tfloat, Tslope, Theight>, true>::
                microsurface(std::forward<Targs>(args)...),
                fr0_(fr0),
                ft0_(ft0),
                eta_(eta)
    {
    }

public:

    /**
     * @name Single-scattering interface
     */
    /**@{*/

    /**
     * @brief Single-scattering BSDF.
     *
     * @param[in] wo
     * Outgoing direction.
     *
     * @param[in] wi
     * Incident direction.
     *
     * @par Expression
     * @parblock
     * If @f$ \omega_{o_z} < 0 @f$, flip everything:
     * - @f$ \omega_{o_z} \gets -\omega_{o_z} @f$
     * - @f$ \omega_{i_z} \gets -\omega_{i_z} @f$
     * - @f$ \eta \gets 1 / \eta @f$
     *
     * If @f$ \omega_{i_z} > 0 @f$, calculate the BRDF:
     * - @f$ \mathbf{v}_m \gets \omega_o + \omega_i @f$
     * - @f$ \omega_m \gets \mathbf{v}_m / \lVert \mathbf{v}_m \rVert @f$
     * - @f$ G_2(\omega_o, \omega_i) = 1 /
     *          (1 + \Lambda(\omega_o) + \Lambda(\omega_i)) @f$
     *
     * @f[
     *      f_s(\omega_o, \omega_i) =
     *          \frac{1}{\omega_{o_z}}
     *          D(\omega_m)
     *          F_r(\omega_o \cdot \omega_m)
     *          G_2(\omega_o, \omega_i)
     * @f]
     *
     * If @f$ \omega_{i_z} < 0 @f$, calculate the BTDF:
     * - @f$ \mathbf{v}_m \gets \eta\omega_o + \omega_i @f$
     * - @f$ \mathbf{v}_m \gets -\mathbf{v}_m @f$ if @f$ v_{m_z} < 0 @f$
     * - @f$ \omega_m \gets \mathbf{v}_m / \lVert \mathbf{v}_m \rVert @f$
     * - @f$ G_2(\omega_o, \omega_i) =
     *         \beta(1 + \Lambda(\omega_o),
     *               1 + \Lambda(\omega_i)) @f$
     *
     * @f[
     *      f_s(\omega_o, \omega_i) =
     *          \frac{1}{\omega_{o_z}}
     *          D(\omega_m)
     *          F_t(\omega_o \cdot \omega_m)
     *          G_2(\omega_o, \omega_i)
     *          |\omega_o \cdot \omega_m|
     *          |\omega_i \cdot \omega_m|
     *          \frac{1}{\lVert \mathbf{v}_m \rVert^2}
     * @f]
     * @endparblock
     *
     * @note
     * @f[
     *     f_s(\omega_o, \omega_i) \frac{1}{|\omega_{i_z}|\eta_i^2} =
     *     f_s(\omega_i, \omega_o) \frac{1}{|\omega_{o_z}|\eta_o^2}
     * @f]
     */
    float_type fs1(
            multi<float_type, 3> wo,
            multi<float_type, 3> wi) const
    {
        // Flip.
        float_type eta = eta_;
        if (wo[2] < 0) {
            wo[2] = -wo[2];
            wi[2] = -wi[2];
            eta = 1 / eta;
        }
        if (!(wo[2] > float_type(0.0000001)) || // Avoid exploding.
              wi[2] == 0) {
            return 0;
        }

        if (wi[2] > 0) {

            // Microsurface normal.
            multi<float_type, 3> wm = normalize_safe(wo + wi);

            // Masking-shadowing.
            float_type lambda_wo = this->lambda(wo);
            float_type lambda_wi = this->lambda(wi);
            float_type g2 = 1 / (1 + lambda_wo + lambda_wi);

            // Reflection.
            float_type cos_thetao = dot(wo, wm);
            float_type cos_thetat;
            float_type fr, ft;
            fresnel_diel(
                eta,
                cos_thetao,
                cos_thetat,
                fr, ft);
            return this->d(wm) * fr * fr0_ * g2 / (4 * wo[2]);
        }
        else {

            // Half vector.
            multi<float_type, 3> vm = eta * wo + wi;
            if (vm[2] < 0) {
                vm = -vm;
            }

            float_type dot_vm_vm = dot(vm, vm);
            if (dot_vm_vm < float_type(1e-8)) {
                return 0;
            }

            // Microsurface normal.
            multi<float_type, 3> wm = vm / pr::sqrt(dot_vm_vm);
            float_type dot_wo_wm = dot(wo, wm);
            float_type dot_wi_wm = dot(wi, wm);
            if (!(dot_wo_wm > 0 &&
                  dot_wi_wm < 0)) {
                return 0;
            }

            // Masking-shadowing.
            float_type lambda_wo = this->lambda(+wo);
            float_type lambda_wi = this->lambda(-wi);
            float_type g2 = pr::beta(1 + lambda_wo, 1 + lambda_wi);

            // Transmission.
            float_type cos_thetao = dot_wo_wm;
            float_type cos_thetat;
            float_type fr, ft;
            fresnel_diel(
                    eta,
                    cos_thetao,
                    cos_thetat,
                    fr, ft);
            return this->d(wm) * ft * ft0_ * g2 *
                   (dot_wo_wm * -dot_wi_wm / dot_vm_vm / wo[2]);
        }
    }

    /**
     * @brief Single-scattering BSDF-PDF.
     *
     * @param[in] wo
     * Outgoing direction.
     *
     * @param[in] wi
     * Incident direction.
     *
     * @par Expression
     * @parblock
     * If @f$ \omega_{o_z} < 0 @f$, flip everything:
     * - @f$ \omega_{o_z} \gets -\omega_{o_z} @f$
     * - @f$ \omega_{i_z} \gets -\omega_{i_z} @f$
     * - @f$ \eta \gets 1 / \eta @f$
     *
     * If @f$ \omega_{i_z} > 0 @f$, calculate BRDF density:
     * - @f$ \mathbf{v}_m \gets \omega_o + \omega_i @f$
     * - @f$ \omega_m \gets \mathbf{v}_m / \lVert \mathbf{v}_m \rVert @f$
     *
     * @f[
     *      f_{s,\text{bsdf}}(\omega_o \to \omega_i) =
     *              D_{\omega_o}(\omega_m)
     *              F_r(\omega_o \cdot \omega_m)
     *              \frac{1}{4\omega_i \cdot \omega_m}
     * @f]
     *
     * If @f$ \omega_{i_z} < 0 @f$, calculate BTDF density:
     * - @f$ \mathbf{v}_m \gets -\eta\omega_o - \omega_i @f$
     * - @f$ \mathbf{v}_m \gets -\mathbf{v}_m @f$ if @f$ \eta > 1 @f$
     * - @f$ \omega_m \gets \mathbf{v}_m / \lVert \mathbf{v}_m \rVert @f$
     *
     * @f[
     *      f_{s,\text{bsdf}}(\omega_o \to \omega_i) =
     *              D_{\omega_o}(\omega_m)
     *              F_t(\omega_o \cdot \omega_m)
     *              \frac{|\omega_i \cdot \omega_m|}
     *                   {\lVert \mathbf{v}_m \rVert^2}
     * @f]
     * @endparblock
     */
    float_type fs1_pdf(
                multi<float_type, 3> wo,
                multi<float_type, 3> wi) const
    {
        // Flip.
        float_type eta = eta_;
        if (wo[2] < 0) {
            wo[2] = -wo[2];
            wi[2] = -wi[2];
            eta = 1 / eta;
        }

        // Ignore invalid samples.
        if (wo[2] == 0 ||
            wi[2] == 0) {
            return 0;
        }

        if (wi[2] > 0) {

            // Half vector.
            multi<float_type, 3> vm = wo + wi;

            // Microsurface normal.
            multi<float_type, 3> wm = normalize_safe(vm);

            // Fresnel coefficients.
            float_type cos_thetao = dot(wo, wm);
            float_type cos_thetat;
            float_type fr, ft;
            fresnel_diel(
                    eta,
                    cos_thetao,
                    cos_thetat,
                    fr, ft);

            // Fresnel coefficient weighting.
            fr *= fr0_;
            ft *= ft0_;
            float_type fr_weight = fr / (fr + ft);

            return this->dwo(wo, wm) * fr_weight / (4 * dot(wo, wm));
        }
        else {

            // Half vector.
            multi<float_type, 3> vm = eta * wo + wi;
            if (vm[2] < 0) {
                vm = -vm;
            }

            float_type dot_vm_vm = dot(vm, vm);
            if (dot_vm_vm < float_type(1e-8)) {
                return 0;
            }

            // Microsurface normal.
            multi<float_type, 3> wm = vm / pr::sqrt(dot_vm_vm);
            float_type dot_wo_wm = dot(wo, wm);
            float_type dot_wi_wm = dot(wi, wm);
            if (!(dot_wo_wm > 0 &&
                  dot_wi_wm < 0)) {
                return 0;
            }

            // Fresnel coefficients.
            float_type cos_thetao = dot_wo_wm;
            float_type cos_thetat;
            float_type fr, ft;
            fresnel_diel(
                    eta,
                    cos_thetao,
                    cos_thetat,
                    fr, ft);

            // Fresnel coefficient weighting.
            fr *= fr0_;
            ft *= ft0_;
            float_type ft_weight = ft / (fr + ft);

            return this->dwo(wo, wm) * ft_weight * (-dot_wi_wm / dot_vm_vm);
        }
    }

    /**
     * @brief Single-scattering BSDF-PDF sample direction.
     *
     * @param[in] u0
     * Sample in @f$ [0, 1) @f$.
     *
     * @param[in] u1
     * Sample in @f$ [0, 1)^2 @f$.
     *
     * @param[in] wo
     * Outgoing direction.
     */
    multi<float_type, 3> fs1_pdf_sample(
            float_type u0,
            multi<float_type, 2> u1,
            multi<float_type, 3> wo) const
    {
        // Flip.
        float_type eta = eta_;
        bool neg = false;
        if (wo[2] < 0) {
            wo[2] = -wo[2];
            eta = 1 / eta;
            neg = true;
        }

        if (wo[2] == 0) {
            return {}; // Reject sample.
        }

        // Sample visible microsurface normal.
        multi<float_type, 3> wm = this->dwo_sample(u1, wo);

        // Fresnel coefficients.
        float_type cos_thetao = dot(wo, wm);
        float_type cos_thetat;
        float_type fr, ft;
        fresnel_diel(
                eta,
                cos_thetao,
                cos_thetat,
                fr, ft);

        // Fresnel coefficent weighting.
        fr *= fr0_;
        ft *= ft0_;
        float_type fr_weight = fr / (fr + ft);

        // Result.
        multi<float_type, 3> wi;

        if (u0 < fr_weight) {

            // Reflect.
            wi = normalize_fast(
                 -wo + (2 * cos_thetao) * wm);

            // In wrong hemisphere?
            if (!(wi[2] > 0)) {
                return {}; // Reject sample.
            }
        }
        else {

            // Refract.
            wi = normalize_fast(
                 -eta * wo +
                 (eta * cos_thetao + cos_thetat) * wm);

            // In wrong hemisphere?
            if (!(wi[2] < 0)) {
                return {}; // Reject sample.
            }
        }

        // Unflip.
        if (neg) {
            wi[2] = -wi[2];
        }

        return wi;
    }

    /**@}*/

public:

    /**
     * @name Multiple-scattering interface (CRTP)
     */
    /**@{*/

    /**
     * @brief Phase function.
     *
     * @param[inout] gen
     * Generator (unused).
     *
     * @param[in] wo
     * Outgoing direction.
     *
     * @param[in] wi
     * Incident direction.
     *
     * @param[in] wo_outside
     * Outgoing direction outside material?
     *
     * @param[in] wi_outside
     * Incident direction outside material?
     *
     * @param[inout] ek
     * _Optional_. Energy.
     *
     * @param[in] is_importance
     * _Optional_. Energy is importance, not radiance?
     */
    template <typename Tgen>
    float_type ps(
            Tgen& gen,
            multi<float_type, 3> wo,
            const multi<float_type, 3>& wi,
            bool wo_outside,
            bool wi_outside,
            float_type* ek = nullptr, 
                   bool is_importance = false) const
    {
        (void) gen;
        float_type eta = (wo_outside ? eta_ : 1 / eta_);
        if (wo_outside == wi_outside) {

            // Microsurface normal.
            multi<float_type, 3> wm = normalize_safe(wo + wi);

            // Flip to outside.
            if (!wo_outside) {
                wo = -wo;
                wm = -wm;
            }

            // Fresnel coefficents.
            float_type cos_thetao = dot(wo, wm);
            float_type cos_thetat;
            float_type fr, ft;
            fresnel_diel(
                    eta,
                    cos_thetao,
                    cos_thetat,
                    fr, ft);

            // Fresnel coefficient weighting.
            fr *= fr0_;
            ft *= ft0_;
            float_type fr_weight = fr / (fr + ft);

            // Energy.
            if (ek) {
                *ek *= fr + ft;
            }

            // Reflection.
            return this->dwo(wo, wm) * fr_weight / (4 * cos_thetao);
        }
        else {

            // Half vector.
            multi<float_type, 3> vm = eta * wo + wi;
            if (vm[2] < 0) {
                vm = -vm;
            }

            // Flip to match wo.
            if (!wo_outside) {
                vm = -vm;
            }

            float_type dot_vm_vm = dot(vm, vm);
            if (dot_vm_vm < float_type(1e-8)) {
                return 0;
            }

            // Microsurface normal.
            multi<float_type, 3> wm = vm / pr::sqrt(dot_vm_vm);
            float_type dot_wo_wm = dot(wo, wm);
            float_type dot_wi_wm = dot(wi, wm);
            if (!(dot_wo_wm > 0 &&
                  dot_wi_wm < 0)) {
                return 0;
            }

            // Flip to outside.
            if (!wo_outside) {
                wo = -wo;
                wm = -wm;
            }

            // Fresnel coefficents.
            float_type cos_thetao = dot_wo_wm;
            float_type cos_thetat;
            float_type fr, ft;
            fresnel_diel(
                    eta,
                    cos_thetao,
                    cos_thetat,
                    fr, ft);

            // Fresnel coefficient weighting.
            fr *= fr0_;
            ft *= ft0_;
            float_type ft_weight = ft / (fr + ft);

            // Energy.
            if (ek) {
                *ek *= fr + ft;
                if (is_importance) {
                    *ek *= eta * eta;
                }
            }

            // Transmission.
            return this->dwo(wo, wm) * ft_weight *
                                 (-dot_wi_wm / dot_vm_vm);
        }
    }

    /**
     * @brief Phase function sample direction.
     *
     * @param[inout] gen
     * Generator.
     *
     * @param[in] wo
     * Outgoing direction.
     *
     * @param[in] wo_outside
     * Outgoing direction outside material?
     *
     * @param[out] wi_outside
     * Incident direction outside material?
     *
     * @param[inout] ek
     * _Optional_. Energy.
     *
     * @param[in] is_importance
     * _Optional_. Energy is importance, not radiance?
     */
    template <typename Tgen>
    multi<float_type, 3> ps_sample(
            Tgen& gen,
            const multi<float_type, 3>& wo,
            bool  wo_outside,
            bool& wi_outside,
            float_type* ek = nullptr, 
                   bool is_importance = false) const
    {
        // Refractive index.
        float_type eta = (wo_outside ? eta_ : 1 / eta_);

        // Sample visible microsurface normal.
        multi<float_type, 2> u0 = pr::generate_canonical<float_type, 2>(gen);
        multi<float_type, 3> wm =
            wo_outside ?
            +this->dwo_sample(u0, +wo) :
            -this->dwo_sample(u0, -wo);

        // Fresnel coefficients.
        float_type cos_thetao = dot(wo, wm);
        float_type cos_thetat;
        float_type fr, ft;
        fresnel_diel(
                eta,
                cos_thetao,
                cos_thetat,
                fr, ft);

        // Fresnel coefficient weighting.
        fr *= fr0_;
        ft *= ft0_;
        float_type fr_weight = fr / (fr + ft);

        // Energy.
        if (ek) {
            *ek *= fr + ft;
        }

        if (pr::generate_canonical<float_type>(gen) < fr_weight) {
            // Reflect.
            wi_outside = wo_outside;
            return normalize_fast(
                   -wo + 2 * cos_thetao * wm);
        }
        else {
            if (ek)
            if (is_importance) {
                *ek *= eta * eta;
            }
            // Refract.
            wi_outside = !wo_outside;
            return normalize_fast(
                   -eta * wo +
                   (eta * cos_thetao +
                          cos_thetat) * wm);
        }
    }

    /**@}*/

private:

    /**
     * @brief BRDF Fresnel coefficient @f$ F_{r,0} @f$.
     */
    float_type fr0_ = float_type(1);

    /**
     * @brief BTDF Fresnel coefficient @f$ F_{t,0} @f$.
     */
    float_type ft0_ = float_type(1);

    /**
     * @brief Refractive index @f$ \eta @f$.
     *
     * @par Expression
     * @f[
     *      \eta =
     *      \frac{\eta_{+}}
     *           {\eta_{-}}
     * @f]
     * where
     * - @f$ \eta_{+} @f$ is the refractive index in the upper hemisphere
     * - @f$ \eta_{-} @f$ is the refractive index in the lower hemisphere
     */
    float_type eta_ = float_type(1) / float_type(1.5);
};

/**
 * @brief Microsurface from conductive BRDF.
 *
 * @tparam Tfloat
 * Float type.
 *
 * @tparam Tslope
 * Slope distribution, either `microsurface_trowbridge_reitz_slope` or
 * `microsurface_beckmann_slope`.
 *
 * @tparam Theight
 * Height distribution, either `microsurface_uniform_height` or
 * `microsurface_normal_height`.
 */
template <
    typename Tfloat,
    template <typename> typename Tslope,
    template <typename> typename Theight
    >
struct microsurface_conductive_brdf :
                public microsurface<
                       microsurface_conductive_brdf<Tfloat, Tslope, Theight>,
                       true>
{
public:

    /**
     * @brief Float type.
     */
    typedef Tfloat float_type;

    /**
     * @brief Default constructor.
     */
    microsurface_conductive_brdf() = default;

    /**
     * @brief Constructor.
     */
    template <typename... Targs>
    microsurface_conductive_brdf(
            std::complex<float_type> eta,
            Targs&&... args) :
                microsurface<
                microsurface_conductive_brdf<Tfloat, Tslope, Theight>, true>::
                microsurface(std::forward<Targs>(args)...),
                eta_(eta)
    {
    }

public:

    /**
     * @name Single-scattering interface
     */
    /**@{*/

    /**
     * @brief Single-scattering BRDF.
     *
     * @param[in] wo
     * Outgoing direction.
     *
     * @param[in] wi
     * Incident direction.
     *
     * @par Expression
     * @parblock
     * If @f$ \omega_{o_z} < 0 @f$, flip everything:
     * - @f$ \omega_{o_z} \gets -\omega_{o_z} @f$
     * - @f$ \omega_{i_z} \gets -\omega_{i_z} @f$
     * - @f$ \eta \gets 1 / \eta @f$
     *
     * If @f$ \imag{\eta} > 0 @f$, quit:
     * @f[
     *      f_{s}(\omega_o \to \omega_i) = 0
     * @f]
     *
     * If @f$ \omega_{i_z} > 0 @f$, calculate the BRDF:
     * - @f$ \mathbf{v}_m \gets \omega_o + \omega_i @f$
     * - @f$ \omega_m \gets \mathbf{v}_m / \lVert \mathbf{v}_m \rVert @f$
     * - @f$ G_2(\omega_o, \omega_i) = 1 /
     *          (1 + \Lambda(\omega_o) + \Lambda(\omega_i)) @f$
     *
     * @f[
     *      f_s(\omega_o, \omega_i) =
     *          \frac{1}{\omega_{o_z}}
     *          D(\omega_m)
     *          F_r(\omega_o \cdot \omega_m)
     *          G_2(\omega_o, \omega_i)
     * @f]
     * @endparblock
     */
    float_type fs1(
            multi<float_type, 3> wo,
            multi<float_type, 3> wi) const
    {
        // Flip.
        auto eta = eta_;
        if (wo[2] < 0) {
            wo[2] = -wo[2];
            wi[2] = -wi[2];
            eta = float_type(1) / eta;
        }

        if (!(wo[2] > float_type(0.0000001)) || // Avoid exploding.
            !(wi[2] > 0) ||
            !(eta.imag() < 0)) {
            return 0;
        }

        // Microsurface normal.
        multi<float_type, 3> wm = normalize_safe(wo + wi);

        // Masking-shadowing.
        float_type lambda_wo = this->lambda(wo);
        float_type lambda_wi = this->lambda(wi);
        float_type g2 = 1 / (1 + lambda_wo + lambda_wi);

        // Reflection.
        float_type cos_thetao = dot(wo, wm);
        float_type fr = fresnel_diel_cond(eta, cos_thetao);
        return this->d(wm) * fr * g2 / (4 * wo[2]);
    }

    /**
     * @brief Single-scattering BRDF-PDF.
     *
     * @param[in] wo
     * Outgoing direction.
     *
     * @param[in] wi
     * Incident direction.
     *
     * @par Expression
     * @parblock
     * If @f$ \omega_{o_z} < 0 @f$, flip everything:
     * - @f$ \omega_{o_z} \gets -\omega_{o_z} @f$
     * - @f$ \omega_{i_z} \gets -\omega_{i_z} @f$
     * - @f$ \eta \gets 1 / \eta @f$
     *
     * If @f$ \imag{\eta} > 0 @f$, quit:
     * @f[
     *      f_{s,\text{brdf}}(\omega_o \to \omega_i) = 0
     * @f]
     *
     * If @f$ \omega_{i_z} > 0 @f$, calculate BRDF density:
     * - @f$ \mathbf{v}_m \gets \omega_o + \omega_i @f$
     * - @f$ \omega_m \gets \mathbf{v}_m / \lVert \mathbf{v}_m \rVert @f$
     * @f[
     *      f_{s,\text{brdf}}(\omega_o \to \omega_i) =
     *              D_{\omega_o}(\omega_m)
     *              \frac{1}{4\omega_i \cdot \omega_m}
     * @f]
     * @endparblock
     */
    float_type fs1_pdf(
                multi<float_type, 3> wo,
                multi<float_type, 3> wi) const
    {
        // Flip.
        auto eta = eta_;
        if (wo[2] < 0) {
            wo[2] = -wo[2];
            wi[2] = -wi[2];
            eta = float_type(1) / eta;
        }

        // Ignore invalid samples.
        if (!(wo[2] > 0) ||
            !(wi[2] > 0) ||
            !(eta.imag() < 0)) {
            return 0;
        }

        // Microsurface normal.
        multi<float_type, 3> wm = normalize_safe(wo + wi);

        // Density.
        return this->dwo(wo, wm) / (4 * dot(wo, wm));
    }

    /**
     * @brief Single-scattering BRDF-PDF sample direction.
     *
     * @param[in] u
     * Sample in @f$ [0, 1)^2 @f$.
     *
     * @param[in] wo
     * Outgoing direction.
     */
    multi<float_type, 3> fs1_pdf_sample(
            multi<float_type, 2> u,
            multi<float_type, 3> wo) const
    {
        // Flip.
        auto eta = eta_;
        bool neg = false;
        if (wo[2] < 0) {
            wo[2] = -wo[2];
            eta = float_type(1) / eta;
            neg = true;
        }

        // Ensure dielectric hemisphere.
        if (!(eta.imag() < 0)) {
            return {}; // Reject sample.
        }

        // Sample visible microsurface normal.
        multi<float_type, 3> wm = this->dwo_sample(u, wo);

        // Reflect.
        multi<float_type, 3> wi = normalize_fast(-wo + 2 * dot(wo, wm) * wm);

        // In wrong hemisphere?
        if (!(wi[2] > 0)) {
            return {}; // Reject sample.
        }

        // Unflip.
        if (neg) {
            wi[2] = -wi[2];
        }

        return wi;
    }

    /**@}*/

public:

    /**
     * @name Multiple-scattering interface (CRTP)
     */
    /**@{*/

    /**
     * @brief Phase function.
     *
     * @param[inout] gen
     * Generator (unused).
     *
     * @param[in] wo
     * Outgoing direction.
     *
     * @param[in] wi
     * Incident direction.
     *
     * @param[in] wo_outside
     * Outgoing direction outside material?
     *
     * @param[in] wi_outside
     * Incident direction outside material?
     *
     * @param[inout] ek
     * _Optional_. Energy.
     *
     * @param[in] is_importance
     * _Optional_. Energy is importance, not radiance?
     */
    template <typename Tgen>
    float_type ps(
            Tgen& gen,
            const multi<float_type, 3>& wo,
            const multi<float_type, 3>& wi,
            bool wo_outside,
            bool wi_outside,
            float_type* ek = nullptr, 
                   bool is_importance = false) const
    {
        (void) gen;

        auto eta = wo_outside ? eta_ : float_type(1) / eta_;
        if (!(eta.imag() < 0) || wo_outside != wi_outside) {
            return 0;
        }

        // Microsurface normal.
        multi<float_type, 3> wm = normalize_safe(wo + wi);
        if (!((wm[2] > 0) == wo_outside)) {
            return 0;
        }

        // Energy.
        if (ek) {
            *ek *= fresnel_diel_cond(eta, dot(wo, wm));
            (void) is_importance;
        }

        return (wo_outside 
                ? this->dwo(+wo, +wm) 
                : this->dwo(-wo, -wm)) / (4 * dot(wo, wm));
    }

    /**
     * @brief Phase function sample direction.
     *
     * @param[inout] gen
     * Generator.
     *
     * @param[in] wo
     * Outgoing direction.
     *
     * @param[in] wo_outside
     * Outgoing direction outside material?
     *
     * @param[out] wi_outside
     * Incident direction outside material?
     *
     * @param[inout] ek
     * _Optional_. Energy.
     *
     * @param[in] is_importance
     * _Optional_. Energy is importance, not radiance?
     */
    template <typename Tgen>
    multi<float_type, 3> ps_sample(
            Tgen& gen,
            const multi<float_type, 3>& wo,
            bool  wo_outside,
            bool& wi_outside,
            float_type* ek = nullptr, 
                   bool is_importance = false) const
    {
        auto eta = wo_outside ? eta_ : float_type(1) / eta_;
        if (!(eta.imag() < 0)) {
            return {}; // Reject sample.
        }

        // Sample visible microsurface normal.
        multi<float_type, 2> u0 = pr::generate_canonical<float_type, 2>(gen);
        multi<float_type, 3> wm =
                wo_outside 
                ? +this->dwo_sample(u0, +wo) 
                : -this->dwo_sample(u0, -wo);

        // Reflect.
        multi<float_type, 3> wi = normalize_fast(-wo + 2 * dot(wo, wm) * wm);
        wi_outside = wo_outside;

        // Energy.
        if (ek) {
            *ek *= fresnel_diel_cond(eta, dot(wo, wm));
            (void) is_importance;
        }

        return wi;
    }

    /**@}*/

private:

    /**
     * @brief Refractive index @f$ \eta @f$.
     *
     * @par Expression
     * @parblock
     * @f[
     *      \eta =
     *      \frac{\eta_{+}}
     *           {\eta_{-}}
     * @f]
     * where
     * - @f$ \eta_{+} @f$ is the refractive index in the upper hemisphere
     * - @f$ \eta_{-} @f$ is the refractive index in the lower hemisphere
     *
     * The implementation assumes that either
     * - @f$ \imag{\eta_{+}} > 0 @f$, @f$ \imag{\eta_{-}} = 0 @f$, or
     * - @f$ \imag{\eta_{+}} = 0 @f$, @f$ \imag{\eta_{-}} > 0 @f$.
     *
     * That is, the implementation assumes the dielectric hemisphere where
     * the BRDF is non-zero is the upper hemisphere if @f$ \imag{\eta} < 0 @f$
     * and the lower hemisphere if @f$ \imag{\eta} > 0 @f$.
     * @endparblock
     */
    std::complex<float_type> eta_ =
        float_type(1) /
        std::complex<float_type>(float_type(1.5), float_type(4.0));
};

/**
 * @brief Oren-Nayar diffuse BRDF.
 *
 * Oren-Nayar diffuse BRDF, described by M. Oren and
 * S. K. Nayar in &ldquo;Generalization of Lambert&rsquo;s reflectance
 * model&rdquo; in 1994.
 *
 * @tparam Tfloat
 * Float type.
 */
template <typename Tfloat>
struct oren_nayar_diffuse_brdf
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
     * @brief Constructor.
     *
     * @param[in] sigma
     * Standard deviation @f$ \sigma @f$ of microfacet angle in radians.
     */
    oren_nayar_diffuse_brdf(float_type sigma)
    {
        float_type sigma2 = sigma * sigma;
        a_ = 1 - float_type(0.50) * sigma2 / (sigma2 + float_type(0.33));
        b_ = float_type(0.45) * sigma2 / (sigma2 + float_type(0.09));
    }

    /**
     * @brief BRDF.
     *
     * @param[in] wo
     * Outgoing direction.
     *
     * @param[in] wi
     * Incident direction.
     *
     * @par Expression
     * @f[
     *      f_s(\omega_o, \omega_i) =
     *      \frac{|\omega_{i_z}|}{\pi}
     *      \left[A + B
     *      \frac{\max(\omega_{o_x}\omega_{i_x} +
     *                 \omega_{o_y}\omega_{i_y}, 0)}
     *           {\max(|\omega_{o_z}|, |\omega_{i_z}|)}\right]
     * @f]
     */
    float_type fs(
            multi<float_type, 3> wo,
            multi<float_type, 3> wi) const
    {
        // Flip.
        if (wo[2] < 0) {
            wo[2] = -wo[2];
            wi[2] = -wi[2];
        }
        if (wo[2] == 0 || !(wi[2] > 0)) {
            return 0;
        }

        return
            pr::numeric_constants<float_type>::M_1_pi() *
                (a_ + b_ *
                pr::fmax(
                    wo[0] * wi[0] +
                    wo[1] * wi[1], float_type(0)) /
                pr::fmax(wo[2], wi[2])) * wi[2];
    }

    /**
     * @brief BRDF-PDF.
     *
     * @param[in] wo
     * Outgoing direction.
     *
     * @param[in] wi
     * Incident direction.
     *
     * @note
     * For simplicity, this implementation samples
     * the BRDF as if it is Lambertian.
     */
    float_type fs_pdf(
            multi<float_type, 3> wo,
            multi<float_type, 3> wi) const
    {
        // Flip.
        if (wo[2] < 0) {
            wo[2] = -wo[2];
            wi[2] = -wi[2];
        }
        if (!(wi[2] > 0)) {
            return 0;
        }

        // Cosine hemisphere PDF.
        return multi<float_type, 3>::cosine_hemisphere_pdf(wi[2]);
    }

    /**
     * @brief BRDF-PDF sample direction.
     *
     * @param[in] u
     * Sample in @f$ [0, 1)^2 @f$.
     *
     * @param[in] wo
     * Outgoing direction.
     *
     * @note
     * For simplicity, this implementation samples
     * the BRDF as if it is Lambertian.
     */
    multi<float_type, 3> fs_pdf_sample(
            multi<float_type, 2> u,
            multi<float_type, 3> wo) const
    {
        multi<float_type, 3> wi =
        multi<float_type, 3>::cosine_hemisphere_pdf_sample(u);
        wi[2] = pr::copysign(wi[2], wo[2]);
        return wi;
    }

private:

    /**
     * @brief Oren-Nayar constant @f$ A @f$.
     *
     * @par Expression
     * @f[
     *      A = 1 - 0.5 \frac{\sigma^2}{\sigma^2 + 0.33}
     * @f]
     */
    float_type a_;

    /**
     * @brief Oren-Nayar constant @f$ B @f$.
     *
     * @par Expression
     * @f[
     *      B = 0.45 \frac{\sigma^2}{\sigma^2 + 0.09}
     * @f]
     */
    float_type b_;
};

/**
 * @brief Disney diffuse BRDF.
 *
 * Disney diffuse BRDF, described by B. Burley in &ldquo;Physically
 * based shading at Disney&rdquo; in 2014.
 *
 * Although this BRDF is neither strictly physically based nor developed
 * directly from microfacet theory, it is aesthetically pleasing and commonly
 * used in conjunction with microfacet reflectance lobes to simulate
 * dielectric materials.
 *
 * @tparam Tfloat
 * Float type.
 */
template <typename Tfloat>
struct disney_diffuse_brdf
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
     * @brief Constructor.
     *
     * @param[in] alpha
     * Ad hoc roughness @f$ \alpha \in [0, 1] @f$.
     */
    disney_diffuse_brdf(float_type alpha) : alpha_(alpha)
    {
    }

    /**
     * @brief BRDF.
     *
     * @param[in] wo
     * Outgoing direction.
     *
     * @param[in] wi
     * Incident direction.
     *
     * @par Expression
     * @f[
     *      f_s(\omega_o, \omega_i) =
     *      \frac{|\omega_{i_z}|}{\pi}
     *      ((1 - |\omega_{o_z}|)^5 (F_{d,90} - 1) + 1)
     *      ((1 - |\omega_{i_z}|)^5 (F_{d,90} - 1) + 1)
     * @f]
     * where @f$ F_{d,90} = 2 \alpha (\omega_o \cdot \omega_m)^2 + 0.5 @f$
     */
    float_type fs(
            multi<float_type, 3> wo,
            multi<float_type, 3> wi) const
    {
        // Flip.
        if (wo[2] < 0) {
            wo[2] = -wo[2];
            wi[2] = -wi[2];
        }
        if (wo[2] == 0 || !(wi[2] > 0)) {
            return 0;
        }

        multi<float_type, 3> wm = normalize_safe(wo + wi);
        float_type cos_thetad = dot(wo, wm);
        float_type fd90m1 = 2 * alpha_ * cos_thetad * cos_thetad -
            float_type(0.5);

        return
            pr::numeric_constants<float_type>::M_1_pi() *
            (fd90m1 * pr::nthpow(1 - wo[2], 5) + 1) *
            (fd90m1 * pr::nthpow(1 - wi[2], 5) + 1) * wi[2];
    }

    /**
     * @brief BRDF-PDF.
     *
     * @param[in] wo
     * Outgoing direction.
     *
     * @param[in] wi
     * Incident direction.
     *
     * @note
     * For simplicity, this implementation samples
     * the BRDF as if it is Lambertian.
     */
    float_type fs_pdf(
            multi<float_type, 3> wo,
            multi<float_type, 3> wi) const
    {
        // Flip.
        if (wo[2] < 0) {
            wo[2] = -wo[2];
            wi[2] = -wi[2];
        }
        if (!(wi[2] > 0)) {
            return 0;
        }

        // Cosine hemisphere PDF.
        return multi<float_type, 3>::cosine_hemisphere_pdf(wi[2]);
    }

    /**
     * @brief BRDF-PDF sample direction.
     *
     * @param[in] u
     * Sample in @f$ [0, 1)^2 @f$.
     *
     * @param[in] wo
     * Outgoing direction.
     *
     * @note
     * For simplicity, this implementation samples
     * the BRDF as if it is Lambertian.
     */
    multi<float_type, 3> fs_pdf_sample(
            multi<float_type, 2> u,
            multi<float_type, 3> wo) const
    {
        multi<float_type, 3> wi =
        multi<float_type, 3>::cosine_hemisphere_pdf_sample(u);
        wi[2] = pr::copysign(wi[2], wo[2]);
        return wi;
    }

private:

    /**
     * @brief Ad hoc roughness @f$ \alpha \in [0, 1] @f$.
     */
    float_type alpha_;
};

/**@}*/

} // namespace pr

#endif // #ifndef PREFORM_MICROSURFACE_HPP
