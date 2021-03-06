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
#error "preform/medium.hpp requires >=C++17"
#endif // #if !(__cplusplus >= 201703L)
#endif // #if !DOXYGEN
#pragma once
#ifndef PREFORM_MEDIUM_HPP
#define PREFORM_MEDIUM_HPP

// for pre::multi
#include <preform/multi.hpp>

// for pre::multi wrappers
#include <preform/multi_math.hpp>

// for pre::multi wrappers, pre::uniform_real_distribution, ...
#include <preform/multi_random.hpp>

namespace pre {

/**
 * @defgroup medium Medium
 *
 * `<preform/medium.hpp>`
 *
 * __C++ version__: >=C++17
 */
/**@{*/

/**
 * @brief Henyey-Greenstein phase.
 *
 * @tparam Tfloat
 * Float type.
 */
template <typename Tfloat>
struct hg_phase
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
    hg_phase() = default;

    /**
     * @brief Constructor.
     */
    hg_phase(float_type g) : g_(g)
    {
    }

    /**
     * @brief Phase function.
     *
     * @param[in] wo
     * Outgoing direction.
     *
     * @param[in] wi
     * Incident direction.
     *
     * @par Expression
     * @f[
     *      p_s(\omega_o \to \omega_i) =
     *      \frac{1}{4\pi}
     *      \frac{1 - g^2}
     *          {(1 + g^2 - 2g\omega_o\cdot\omega_i)^{3/2}}
     * @f]
     */
    float_type ps(
            const multi<float_type, 3>& wo,
            const multi<float_type, 3>& wi) const
    {
        return multi<float_type, 3>::hg_phase_pdf(g_, dot(-wo, wi));
    }

    /**
     * @brief Phase function sample direction.
     *
     * @param[in] u
     * Sample in @f$ [0, 1)^2 @f$.
     *
     * @param[in] wo
     * Outgoing direction.
     *
     * @note
     * This samples directions matching the
     * distribution of @f$ p_s @f$, such that path throughput
     * is updated trivially as @f$ \beta' \gets \beta @f$.
     */
    multi<float_type, 3> ps_sample(
            const multi<float_type, 2>& u,
            const multi<float_type, 3>& wo) const
    {
        return normalize_fast(
               dot(
               multi<float_type, 3, 3>::build_onb(-wo),
               multi<float_type, 3>::hg_phase_pdf_sample(g_, u)));
    }

private:

    /**
     * @brief Shape parameter @f$ g \in (-1, 1) @f$.
     */
    float_type g_ = 0;
};

/**
 * @brief Henyey-Greenstein phase stack.
 *
 * @tparam Tfloat
 * Float type.
 *
 * @tparam Nlobes
 * Number of lobes.
 */
template <typename Tfloat, std::size_t Nlobes>
struct hg_phase_stack
{
public:

    // Sanity check.
    static_assert(
        std::is_floating_point<Tfloat>::value,
        "Tfloat must be floating point");

    // Sanity check.
    static_assert(
        Nlobes > 0,
        "Nlobes must be greater than 0");

    /**
     * @brief Float type.
     */
    typedef Tfloat float_type;

    /**
     * @brief Default constructor.
     */
    hg_phase_stack()
    {
        w_[0] = 1;
    }

    /**
     * @brief Constructor.
     *
     * @note
     * For efficiency, this does not validate the @f$ w_k @f$ in
     * any respect. It is the user's responsibility to ensure that
     * the @f$ w_k @f$ are nonnegative and that @f$ \sum_k w_k = 1 @f$.
     */
    hg_phase_stack(
            const multi<float_type, Nlobes>& g,
            const multi<float_type, Nlobes>& w) :
                g_(g),
                w_(w)
    {
    }

    /**
     * @brief Phase function.
     *
     * @param[in] wo
     * Outgoing direction.
     *
     * @param[in] wi
     * Incident direction.
     *
     * @par Expression
     * @f[
     *      p_s(\omega_o \to \omega_i) =
     *      \sum_k
     *      \frac{1}{4\pi}
     *      \frac{1 - g_k^2}
     *          {(1 + g_k^2 - 2g_k\omega_o\cdot\omega_i)^{3/2}} w_k
     * @f]
     */
    float_type ps(
            const multi<float_type, 3>& wo,
            const multi<float_type, 3>& wi) const
    {
        float_type res = 0;
        float_type dot_wo_wi = dot(-wo, wi);
        for (std::size_t k = 0; k < Nlobes; k++) {
            res += w_[k] *
            multi<float_type, 3>::hg_phase_pdf(g_[k], dot_wo_wi);
        }
        return res;
    }

    /**
     * @brief Phase function sample direction.
     *
     * @param[in] u
     * Sample in @f$ [0, 1)^2 @f$.
     *
     * @param[in] wo
     * Outgoing direction.
     *
     * @note
     * This samples directions matching the
     * distribution of @f$ p_s @f$, such that path throughput
     * is updated trivially as @f$ \beta' \gets \beta @f$.
     */
    multi<float_type, 3> ps_sample(
            multi<float_type, 2> u,
            multi<float_type, 3> wo) const
    {
        // Sample index.
        std::size_t k = 0;
        for (float_type wsum = 0; k < Nlobes; k++) {
            if (u[0] < (wsum + w_[k]) || k == Nlobes - 1) {
                // Stretch to [0, 1).
                u[0] -= wsum;
                u[0] /= w_[k];

                // Just to be safe.
                u[0] = pre::fmax(u[0], float_type(0));
                u[0] = pre::fmin(u[0], float_type(1) -
                       pre::numeric_limits<float_type>::machine_epsilon());
                break;
            }

            // Increment.
            wsum =
            wsum + w_[k];
        }

        // Sample.
        return normalize_fast(
               dot(
               multi<float_type, 3, 3>::build_onb(-wo),
               multi<float_type, 3>::hg_phase_pdf_sample(g_[k], u)));
    }

private:

    /**
     * @brief Shape parameters @f$ g_k \in (-1, 1) @f$.
     */
    multi<float_type, Nlobes> g_ = {};

    /**
     * @brief Weights @f$ w_k \in (0, 1) @f$, normalized
     * such that @f$ \sum_k w_k = 1 @f$.
     */
    multi<float_type, Nlobes> w_ = {};
};

/**
 * @brief Rayleigh phase.
 *
 * @tparam Tfloat
 * Float type.
 */
template <typename Tfloat>
struct rayleigh_phase
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
    rayleigh_phase() = default;

    /**
     * @brief Constructor.
     */
    rayleigh_phase(float_type rho) : rho_(rho)
    {
        rho_ = pre::fmin(rho_, float_type(+0.9999));
        rho_ = pre::fmax(rho_, float_type(-0.9999));
    }

    /**
     * @brief Phase function.
     *
     * @param[in] wo
     * Outgoing direction.
     *
     * @param[in] wi
     * Incident direction.
     *
     * @par Expression
     * @f[
     *      p_s(\omega_o \to \omega_i) =
     *      \frac{3}{16\pi}
     *      \left\lbrack{
     *          \frac{1 -  \gamma}{1 + 2\gamma} (\omega_o\cdot\omega_i)^2 +
     *          \frac{1 + 3\gamma}{1 + 2\gamma}
     *      }\right\rbrack
     * @f]
     * where
     * @f[
     *      \gamma = \frac{\rho}{2 - \rho}
     * @f]
     */
    float_type ps(
                const multi<float_type, 3>& wo,
                const multi<float_type, 3>& wi) const
    {
        float_type gam = rho_ / (2 - rho_);
        float_type cos_theta = dot(wo, wi);
        return pre::numeric_constants<float_type>::M_1_pi() *
               float_type(0.1875) * ((1 - gam) * cos_theta * cos_theta +
                                      1 + 3 * gam) / (1 + 2 * gam);
    }

    /**
     * @brief Phase function sample direction.
     *
     * @param[in] u
     * Sample in @f$ [0, 1)^2 @f$.
     *
     * @param[in] wo
     * Outgoing direction.
     *
     * @note
     * This samples directions matching the
     * distribution of @f$ p_s @f$, such that path throughput
     * is updated trivially as @f$ \beta' \gets \beta @f$.
     */
    multi<float_type, 3> ps_sample(
                const multi<float_type, 2>& u,
                const multi<float_type, 3>& wo) const
    {
        // Sample zenith cosine and uniform azimuth angle.
        float_type xi0 = (1 + rho_) / (1 - rho_);
        float_type xi1 = (1 + 3 * xi0) * (2 * u[0] - 1) / 2;
        float_type sqrt_term = pre::sqrt(xi1 * xi1 + xi0 * xi0 * xi0);
        float_type cbrt_term = pre::cbrt(xi1 + sqrt_term);
        float_type cos_thetai = cbrt_term - xi0 / cbrt_term;
        cos_thetai = pre::fmax(cos_thetai, float_type(-1));
        cos_thetai = pre::fmin(cos_thetai, float_type(+1));
        float_type sin_thetai = pre::sqrt(1 - cos_thetai * cos_thetai);
        float_type phi = 2 * pre::numeric_constants<float_type>::M_pi() * u[1];

        // Compute direction.
        multi<float_type, 3> wi = {
            sin_thetai * pre::cos(phi),
            sin_thetai * pre::sin(phi),
            cos_thetai
        };
        return dot(multi<float_type, 3, 3>::build_onb(-wo), wi);
    }

private:

    /**
     * @brief Depolarization factor @f$ \rho \in (-1, 1) @f$.
     */
    float_type rho_ = 0;
};

/**
 * @brief Microvolume with SGGX distribution.
 *
 * @tparam Tfloat
 * Float type.
 */
template <typename Tfloat>
struct microvolume_sggx
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
    microvolume_sggx() = default;

    /**
     * @brief Constructor.
     *
     * @param[in] q
     * Orthogonal basis.
     *
     * @param[in] s
     * Distribution diagonal elements, representing squared projected areas
     * in each axis of orthogonal basis.
     *
     * @note
     * For efficiency, the implementation does not verify the orthogonal basis
     * matrix is valid. It is up to the caller to ensure this matrix is
     * orthogonal and determinant 1.
     */
    microvolume_sggx(
            const multi<float_type, 3, 3>& q,
            const multi<float_type, 3>& s)
    {
        // Construct matrix.
        s_[0][0] = s[0];
        s_[1][1] = s[1];
        s_[2][2] = s[2];
        s_ = dot(q, dot(s_, transpose(q)));

        // Construct matrix inverse.
        sinv_[0][0] = 1 / s[0];
        sinv_[1][1] = 1 / s[1];
        sinv_[2][2] = 1 / s[2];
        sinv_ = dot(q, dot(sinv_, transpose(q)));

        // Determinant root.
        sdet1_2_ = pre::sqrt(s.prod());
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
     *          \sqrt{\omega_o^\top\mathbf{S}\omega_o}
     * @f]
     */
    float_type aperp(const multi<float_type, 3>& wo) const
    {
        return pre::sqrt(dot(wo, dot(s_, wo)));
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
     *          \frac{1}{\pi}
     *          \frac{1}{\sqrt{|\mathbf{S}|}
     *                  (\omega_m^\top\mathbf{S}^{-1}\omega_m)^2}
     * @f]
     */
    float_type d(const multi<float_type, 3>& wm) const
    {
        return pre::numeric_constants<float_type>::M_1_pi() / (sdet1_2_ *
               pre::nthpow(dot(wm, dot(sinv_, wm)), 2));
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
                const multi<float_type, 3>& wo,
                const multi<float_type, 3>& wm) const
    {
        // Area factors.
        float_type fac_numer = dot(wo, wm);
        float_type fac_denom = aperp(wo);
        if (pre::signbit(fac_numer) ||
            pre::signbit(fac_denom)) {
            return 0;
        }

        // Distribution of visible normals.
        float_type d_wm = d(wm);
        return pre::isfinite(d_wm * fac_numer / fac_denom) ?
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
                const multi<float_type, 2>& u,
                const multi<float_type, 3>& wo) const
    {
        // Build orthonormal basis.
        multi<float_type, 3, 3> q =
        multi<float_type, 3, 3>::build_onb(wo);

        // Project distribution matrix into basis.
        multi<float_type, 3, 3> s = dot(transpose(q), dot(s_, q));

        // Compute component vectors.
        float_type tmp0 = pre::sqrt(s[2][2]);
        float_type tmp1 = pre::sqrt(s[1][1] * s[2][2] - s[1][2] * s[2][1]);
        multi<float_type, 3> mx = {sdet1_2_ / tmp1, 0, 0};
        multi<float_type, 3> my = {
            -(s[2][0] * s[2][1] -
              s[1][0] * s[2][2]) / tmp1,
            tmp1,
            0
        };
        multi<float_type, 3> mz = {
            s[2][0],
            s[2][1],
            s[2][2]
        };
        my *= 1 / tmp0;
        mz *= 1 / tmp0;

        // Construct visible normal.
        float_type phi = 2 * pre::numeric_constants<float_type>::M_pi() * u[1];
        float_type ux = pre::sqrt(u[0]) * pre::cos(phi);
        float_type uy = pre::sqrt(u[0]) * pre::sin(phi);
        float_type uz = pre::sqrt(
                        pre::fmax(float_type(0), 1 - ux * ux - uy * uy));
        return dot(q, normalize_safe(ux * mx + uy * my + uz * mz));
    }

public:

    /**
     * @brief Distribution matrix @f$ \mathbf{S} @f$.
     */
    multi<float_type, 3, 3> s_ = {
        {1, 0, 0},
        {0, 1, 0},
        {0, 0, 1}
    };

    /**
     * @brief Distribution matrix inverse @f$ \mathbf{S}^{-1} @f$.
     */
    multi<float_type, 3, 3> sinv_ = {
        {1, 0, 0},
        {0, 1, 0},
        {0, 0, 1}
    };

    /**
     * @brief Distribution matrix determinant root @f$ \sqrt{|\mathbf{S}|} @f$.
     */
    float_type sdet1_2_ = 1;
};

/**
 * @brief Microvolume SGGX specular (reflection) phase.
 *
 * @tparam Tfloat
 * Float type.
 */
template <typename Tfloat>
struct microvolume_sggx_specular_phase : public microvolume_sggx<Tfloat>
{
public:

    /**
     * @brief Float type.
     */
    typedef Tfloat float_type;

    /**
     * @brief Default constructor.
     */
    microvolume_sggx_specular_phase() = default;

    /**
     * @brief Constructor.
     */
    template <typename... Targs>
    microvolume_sggx_specular_phase(Targs&&... args) :
            microvolume_sggx<Tfloat>::
            microvolume_sggx(std::forward<Targs>(args)...)
    {
    }

    /**
     * @brief Phase function.
     *
     * @param[in] wo
     * Outgoing direction.
     *
     * @param[in] wi
     * Incident direction.
     *
     * @par Expression
     * @f[
     *      p_s(\omega_o \to \omega_i) =
     *      \frac{1}{4}
     *      \frac{D(\omega_h)}{A_{\perp}(\omega_o)}
     * @f]
     * where
     * @f[
     *      \omega_h =
     *      \frac{\omega_o + \omega_i}
     *           {\lVert\omega_o + \omega_i\rVert}
     * @f]
     */
    float_type ps(
            const multi<float_type, 3>& wo,
            const multi<float_type, 3>& wi) const
    {
        multi<float_type, 3> wh = normalize_safe(wo + wi);
        if ((wh == 0).all()) { // Normalization error?
            return 0;
        }
        else {
            return this->d(wh) / (4 * this->aperp(wo));
        }
    }

    /**
     * @brief Phase function sample direction.
     *
     * @param[in] u
     * Sample in @f$ [0, 1)^2 @f$.
     *
     * @param[in] wo
     * Outgoing direction.
     *
     * @note
     * This samples directions matching the
     * distribution of @f$ p_s @f$, such that path throughput
     * is updated trivially as @f$ \beta' \gets \beta @f$.
     */
    multi<float_type, 3> ps_sample(
            const multi<float_type, 2>& u,
            const multi<float_type, 3>& wo) const
    {
        multi<float_type, 3> wm = this->dwo_sample(u, wo);
        multi<float_type, 3> wi = normalize_fast(-wo + 2 * dot(wo, wm) * wm);
        return wi;
    }
};

/**
 * @brief Microvolume SGGX diffuse (reflection) phase.
 *
 * @tparam Tfloat
 * Float type.
 */
template <typename Tfloat>
struct microvolume_sggx_diffuse_phase : public microvolume_sggx<Tfloat>
{
public:

    /**
     * @brief Float type.
     */
    typedef Tfloat float_type;

    /**
     * @brief Default constructor.
     */
    microvolume_sggx_diffuse_phase() = default;

    /**
     * @brief Constructor.
     */
    template <typename... Targs>
    microvolume_sggx_diffuse_phase(Targs&&... args) :
            microvolume_sggx<Tfloat>::
            microvolume_sggx(std::forward<Targs>(args)...)
    {
    }

    /**
     * @brief Phase function.
     *
     * @param[in] u
     * Sample in @f$ [0, 1)^2 @f$.
     *
     * @param[in] wo
     * Outgoing direction.
     *
     * @param[in] wi
     * Incident direction.
     *
     * @par Expression
     * @f[
     *      p_s(\omega_o \to \omega_i) =
     *         \frac{\langle{\omega_m, \omega_i}\rangle}{\pi}
     * @f]
     * where @f$ \omega_m \sim D_{\omega_o} @f$
     */
    float_type ps(
            const multi<float_type, 2>& u,
            const multi<float_type, 3>& wo,
            const multi<float_type, 3>& wi) const
    {
        // Sample visible microvolume normal.
        multi<float_type, 3> wm = this->dwo_sample(u, wo);

        // Evaluate.
        return pre::numeric_constants<float_type>::M_1_pi() *
               pre::fmax(dot(wi, wm), float_type(0));
    }

    /**
     * @brief Phase function sample direction.
     *
     * @param[in] u0
     * Sample in @f$ [0, 1)^2 @f$.
     *
     * @param[in] u1
     * Sample in @f$ [0, 1)^2 @f$.
     *
     * @param[in] wo
     * Outgoing direction.
     *
     * @param[out] pdf
     * _Optional_. Output PDF (associated with specific way the
     * incident direction is sampled).
     *
     * @note
     * This samples directions matching the
     * distribution of @f$ p_s @f$, such that path throughput
     * is updated trivially as @f$ \beta' \gets \beta @f$.
     */
    multi<float_type, 3> ps_sample(
            const multi<float_type, 2>& u0,
            const multi<float_type, 2>& u1,
            const multi<float_type, 3>& wo,
            float_type* pdf = nullptr) const
    {
        // Sample visible microvolume normal.
        multi<float_type, 3> wm = this->dwo_sample(u0, wo);

        // Sample direction.
        multi<float_type, 3> wi =
        multi<float_type, 3>::cosine_hemisphere_pdf_sample(u1);

        // TODO To calculate the correct result in the test program, the
        // additional factor of 1/2 below is necessary? Not sure why, or
        // if this calculation is otherwise flawed?
        if (pdf) {
            *pdf =
                pre::numeric_constants<float_type>::M_1_pi() *
                float_type(0.5) * wi[2];
        }

        // Expand in orthonormal basis.
        return normalize_fast(
               dot(multi<float_type, 3, 3>::build_onb(wm), wi));
    }
};

#if 0

/**
 * @brief Half-space linear anisotropic phase BRDF.
 *
 * @tparam Tfloat
 * Float type.
 */
template <typename Tfloat>
struct halfspace_linear_anisotropic_phase_brdf
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
    halfspace_linear_anisotropic_phase_brdf() = default;

    /**
     * @brief Constructor.
     */
    halfspace_linear_anisotropic_phase_brdf(
            float_type b,
            float_type mua) :
                b_(b),
                mua_(mua)
    {
        // Restrict.
        b_ =
            pre::fmax(float_type(-1),
            pre::fmin(float_type(+1), b_));
        mua_ =
            pre::fmax(float_type(0), mua_);
    }

    /**
     * @brief Multiple-scattering BRDF.
     *
     * @param[in] wo
     * Outgoing direction.
     *
     * @param[in] wi
     * Incident direction.
     */
    float_type fm(
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

        // Cosines.
        float_type cos_thetao = wo[2];
        float_type cos_thetai = wi[2];

        // Compute single scattering term.
        float_type f1 =
            float_type(0.25) *
            pre::numeric_constants<float_type>::M_1_pi() *
            (1 - b_ * dot(wo, wi)) /
            (1 + cos_thetao / cos_thetai);

        // Compute multiple scattering Lambertian term.
        float_type r =
            float_type(0.5) *
            (1 - cos_thetao * pre::log(1 + 1 / cos_thetao)) *
            (1 + b_ * cos_thetao * cos_thetao) - float_type(0.25) *
                 b_ * cos_thetao;
        float_type fn = (1 - r) *
            pre::numeric_constants<float_type>::M_1_pi() * cos_thetai;

        // Absorption non-zero?
        if (mua_ > 0) {

            // Single-scattering absorption.
            f1 /= 1 + mua_;

            // Multiple-scattering absorption.
            float_type ratio =
                float_type(0.5) *
                (pre::sqrt(r * r - 6 * r + 5) + r - 1);
            fn /= 1 + mua_;
            fn *=
                (1 - ratio) /
                (1 - ratio + mua_);
        }

        if (!pre::isfinite(f1) ||
            !pre::isfinite(fn)) {
            return 0;
        }
        else {
            // Result.
            return f1 + fn;
        }
    }

    // TODO fm_pdf

private:

    /**
     * @brief Shape parameter @f$ b \in [-1, +1] @f$.
     */
    float_type b_ = 0;

    /**
     * @brief Absorption coefficient @f$ \mu_a \ge 0 @f$.
     */
    float_type mua_ = 0;
};
#endif

/**
 * @brief Homogeneous medium.
 *
 * @tparam Tfloat
 * Float type.
 */
template <typename Tfloat>
struct homogeneous_medium
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
    homogeneous_medium() = default;

    /**
     * @brief Constructor.
     */
    homogeneous_medium(float_type mua, float_type mus) :
            mua_(mua),
            mus_(mus),
            mu_(mua + mus)
    {
    }

public:

    /**
     * @name Accessors
     */
    /**@{*/

    /**
     * @brief Absorption coefficient @f$ \mu_s @f$.
     */
    float_type mua() const
    {
        return mus_;
    }

    /**
     * @brief Scattering coefficient @f$ \mu_s @f$.
     */
    float_type mus() const
    {
        return mus_;
    }

    /**
     * @brief Extinction coefficient @f$ \mu = \mu_s + \mu_a @f$.
     */
    float_type mu() const
    {
        return mu_;
    }

    /**@}*/

public:

    /**
     * @brief Transmittance.
     *
     * @param[in] d
     * Distance.
     *
     * @par Expression
     * @f[
     *      \tau(d) =
     *      \begin{cases}
     *          e^{-\mu d}  & 0 \le d
     *      \\  0           & \text{otherwise}
     *      \end{cases}
     * @f]
     */
    float_type tau(float_type d) const
    {
        if (!(d >= 0)) {
            return 0;
        }
        else {
            d = pre::fmin(d, pre::numeric_limits<float_type>::max());
            return pre::exp(-mu_ * d);
        }
    }

    /**
     * @brief Transmittance probability density function.
     *
     * @param[in] d
     * Distance.
     *
     * @par Expression
     * @f[
     *      \tau_{\text{pdf}}(d) =
     *      \begin{cases}
     *          \mu e^{-\mu d}  & 0 \le d
     *      \\  0               & \text{otherwise}
     *      \end{cases}
     * @f]
     */
    float_type tau_pdf(float_type d) const
    {
        if (!(d >= 0)) {
            return 0;
        }
        else {
            d = pre::fmin(d, pre::numeric_limits<float_type>::max());
            return mu_ * pre::exp(-mu_ * d);
        }
    }

    /**
     * @brief Transmittance probability density function sampling
     * routine.
     *
     * @param[in] u
     * Sample in @f$ [0, 1) @f$.
     */
    float_type tau_pdf_sample(float_type u) const
    {
        return -(u < float_type(0.5) ? 
                 pre::log1p(-u) : pre::log(1 - u)) / mu_;
    }

private:

    /**
     * @brief Absorption coefficient @f$ \mu_a \ge 0 @f$.
     */
    float_type mua_ = 0;

    /**
     * @brief Scattering coefficient @f$ \mu_s \ge 0 @f$.
     */
    float_type mus_ = 0;

    /**
     * @brief Extinction coefficient @f$ \mu = \mu_a + \mu_s @f$.
     */
    float_type mu_ = 0;
};

/**@}*/

} // namespace pre

#endif // #ifndef PREFORM_MEDIUM_HPP
