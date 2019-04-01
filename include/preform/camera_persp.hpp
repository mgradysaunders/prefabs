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
#error "preform/camera_persp.hpp requires >=C++17"
#endif // #if !(__cplusplus >= 201703L)
#endif // #if !DOXYGEN
#pragma once
#ifndef PREFORM_CAMERA_PERSP_HPP
#define PREFORM_CAMERA_PERSP_HPP

// for pr::multi
#include <preform/multi.hpp>

// for pr::multi wrappers
#include <preform/multi_math.hpp>

namespace pr {

/**
 * @defgroup camera_persp Camera (perspective)
 *
 * `<preform/camera_persp.hpp>`
 *
 * __C++ version__: >=C++17
 */
/**@{*/

/**
 * @brief Camera (perspective).
 *
 * @image html camera_persp.svg
 *
 * @tparam T
 * Float type.
 */
template <typename T>
struct camera_persp
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

public:

    /**
     * @brief Default constructor.
     */
    camera_persp() = default;

    /**
     * @brief Constructor.
     *
     * @param[in] pc
     * Position.
     *
     * @param[in] hatx
     * Basis direction.
     *
     * @param[in] haty
     * Basis direction.
     *
     * @param[in] hatz
     * Basis direction.
     *
     * @param[in] sx
     * Screen size.
     *
     * @param[in] sy
     * Screen size.
     *
     * @param[in] f
     * Y-FOV in radians.
     *
     * @param[in] qmin
     * Sub-region bounding point.
     *
     * @param[in] qmax
     * Sub-region bounding point.
     *
     * @throw std::invalid_argument
     * Unless
     * - `sx > 0`,
     * - `sy > 0`, 
     * - `f > 0 && f < pi`,
     * - `qmin[0] < qmax[0]`, and
     * - `qmin[1] < qmax[1]`.
     */
    camera_persp(
        const multi<float_type, 3>& pc,
        const multi<float_type, 3>& hatx,
        const multi<float_type, 3>& haty,
        const multi<float_type, 3>& hatz,
        float_type sx,
        float_type sy,
        float_type f,
        const multi<float_type, 2>& qmin = {0, 0},
        const multi<float_type, 2>& qmax = {1, 1}) :
            pc_(pc),
            hatx_(hatx),
            haty_(haty),
            hatz_(hatz),
            sx_(sx),
            sy_(sy),
            sz_(sy / (2 * pr::tan(f / 2))),
            qmin_(qmin),
            qmax_(qmax),
            a_(sx_ * sy_ / (sz_ * sz_) * (qmax_ - qmin_).prod())
    {
        if (!(sx > 0 &&
              sy > 0 &&
              f > 0 &&
              f < pr::numeric_constants<float_type>::M_pi() &&
              qmin[0] < qmax[0] &&
              qmin[1] < qmax[1])) {
            throw std::invalid_argument(__PRETTY_FUNCTION__);
        }
    }

    /**
     * @brief Look-at.
     *
     * - @f$ \mathbf{p}_c \gets \mathbf{p}_0 @f$
     * - @f$ \hat{\mathbf{z}} \gets \normalize(\mathbf{p}_0 - \mathbf{p}_1) @f$
     * - @f$ \hat{\mathbf{x}} \gets \normalize(
     *           -\mathbf{y}_0\times \hat{\mathbf{z}}) @f$
     * - @f$ \hat{\mathbf{y}} \gets 
     *       \hat{\mathbf{z}} \times \hat{\mathbf{x}} @f$
     *
     * @param[in] p0
     * Point from.
     *
     * @param[in] p1
     * Point to.
     *
     * @param[in] y0
     * Up vector.
     *
     * @param[in] sx
     * Screen size.
     *
     * @param[in] sy
     * Screen size.
     *
     * @param[in] f
     * Y-FOV in radians.
     *
     * @param[in] qmin
     * Sub-region bounding point.
     *
     * @param[in] qmax
     * Sub-region boundint point.
     *
     * @note
     * For convenience, the implementation crosses with @f$ -\mathbf{y}_0 @f$, 
     * such that the camera coordinate system is upside-down (since pixel 
     * coordinates increase downward).
     */
    static camera_persp look_at(
                const multi<float_type, 3>& p0,
                const multi<float_type, 3>& p1,
                const multi<float_type, 3>& y0,
                float_type sx,
                float_type sy,
                float_type f,
                const multi<float_type, 2>& qmin = {0, 0},
                const multi<float_type, 2>& qmax = {1, 1})
    {
        multi<float_type, 3> pc = p0;
        multi<float_type, 3> hatz = normalize(p1 - p0);
        multi<float_type, 3> hatx = normalize(cross(-y0, hatz));
        multi<float_type, 3> haty = cross(hatz, hatx);
        return {
            pc, 
            hatx, 
            haty, 
            hatz, 
            sx,
            sy, 
            f,
            qmin,
            qmax
        };
    }

    /**
     * @brief Emissive importance.
     *
     * Emissive importance is given by 
     * @f[
     *      Z_e(\mathbf{p}, \omega) = 
     *          \delta(\mathbf{p} - \mathbf{p}_c) 
     *          \frac{\chi^+(\omega \in \Omega_c)}{A\cos^3\theta}
     * @f]
     * where @f$ \chi^+(\omega \in \Omega_c) @f$ is an abuse of 
     * notation (of the Heaviside step function) indicating @f$ \omega @f$ 
     * must be within @f$ \Omega_c @f$, the region of solid angle 
     * corresponding to the projection of the rectangular screen 
     * onto the unit sphere.
     *
     * By this definition, @f$ Z_e @f$ satisfies the normalization
     * condition
     * @f[
     *      \int_{\mathcal{A}} 
     *      \int_{\mathcal{S}^2}
     *      Z_e(\mathbf{p}, \omega)
     *      \,d{\omega}
     *      \,d\mathcal{A}(\mathbf{p}) = 1.
     * @f]
     *
     * @param[in] w
     * Direction.
     *
     * @param[out] r
     * Raster coordinate. _Optional_.
     *
     * @note
     * As the position distribution is a delta, this 
     * calculates only the part of @f$ Z_e @f$ which depends on 
     * @f$ \omega @f$.
     *
     * @note
     * In _Physically based rendering: from theory to 
     * implementation (3rd edition)_, Pharr, Humphreys, and Jakob 
     * define emissive importance for perspective cameras as
     * @f[
     *      W_e(\mathbf{p}, \omega) = 
     *          \delta(\mathbf{p} - \mathbf{p}_c) 
     *          \frac{\chi^+(\omega \in \Omega_c)}{A\cos^4\theta}
     * @f]
     * which satisfies the alternative normalization condition
     * @f[
     *      \int_{\mathcal{A}} 
     *      \int_{\mathcal{S}^2}
     *      W_e(\mathbf{p}, \omega)|\cos\theta|
     *      \,d{\omega}
     *      \,d\mathcal{A}(\mathbf{p}) = 1.
     * @f]
     * This is sensible in their implementation, as they allow 
     * perspective cameras to have &ldquo;lenses&rdquo; with non-zero
     * area. The implementation here, however, requires that perspective
     * cameras are pinholes, i.e., point sources of importance.
     */
    float_type ze(const multi<float_type, 3>& w,
                        multi<float_type, 2>* r = nullptr) const
    {
        float_type cos_theta = dot(hatz_, w);
        if (!(cos_theta > 0)) {
            return 0;
        }

        // Normalized raster coordinate.
        multi<float_type, 2> q = {
            sz_ * dot(hatx_, w) / (sx_ * cos_theta) + float_type(0.5),
            sz_ * dot(haty_, w) / (sy_ * cos_theta) + float_type(0.5)
        };

        // Clip.
        if (!((q >= qmin_).all() &&
              (q <= qmax_).all())) {
            return 0;
        }

        // Raster coordinate.
        if (r) {
            *r = {
                sx_ * q[0],
                sy_ * q[1]
            };
        }

        // Importance.
        float_type cos2_theta = cos_theta * cos_theta;
        float_type cos3_theta = cos_theta * cos2_theta;
        return 1 / (a_ * cos3_theta);
    }

    /**
     * @brief Emissive importance sampling routine.
     *
     * @param[in] u
     * Sample in @f$ [0, 1)^2 @f$.
     *
     * @param[out] p
     * Position.
     *
     * @param[out] w
     * Direction.
     *
     * @param[out] r
     * Raster coordinate. _Optional_.
     *
     * @returns
     * Returns emissive importance @f$ Z_e @f$.
     */
    float_type ze_sample(
            const multi<float_type, 2>& u,
            multi<float_type, 3>& p,
            multi<float_type, 3>& w,
            multi<float_type, 2>* r = nullptr) const
    {
        // Position.
        p = pc_;

        multi<float_type, 2> q = (1 - u) * qmin_ + u * qmax_;
        multi<float_type, 3> v = {
            sx_ * (q[0] - float_type(0.5)),
            sy_ * (q[1] - float_type(0.5)),
            sz_
        };
        // Direction.
        w = fastnormalize(
                hatx_ * v[0] + 
                haty_ * v[1] +
                hatz_ * v[2]);

        // Raster coordinate.
        if (r) {
            *r = {
                sx_ * q[0],
                sy_ * q[1]
            };
        }

        // Importance.
        float_type cos_theta = v[2] / fastlength(v);
        float_type cos2_theta = cos_theta * cos_theta;
        float_type cos3_theta = cos_theta * cos2_theta;
        return 1 / (a_ * cos3_theta);
    }

    /**
     * @brief Incident importance sampling routine.
     *
     * Incident importance is given by
     * @f[
     *      Z_i(\omega_i; \mathbf{p}_{\text{ref}}) = 
     *      \frac{1}{d^2} Z_e(\mathbf{p}_{\text{hit}}, -\omega_i)
     * @f]
     * where @f$ \mathbf{p}_{\text{hit}} @f$ is the hit point on the
     * camera (from the ray @f$ (\mathbf{p}_{\text{ref}}, \omega_i) @f$)
     * and @f$ d @f$ is the distance between @f$ \mathbf{p}_{\text{ref}} @f$
     * and @f$ \mathbf{p}_{\text{hit}} @f$.
     *
     * This definition uses the convention that a pinhole camera is a
     * point source of importance, giving rise to the @f$ 1 / d^2 @f$ falloff. 
     * As a direct result, the associated solid angle sampling density is 
     * trivially @f$ f_\omega = 
     * \delta(\omega - \omega_i) =
     * \delta(\mathbf{p}_{\text{hit}} - \mathbf{p}_c) @f$.
     *
     * @param[in] pref
     * Reference point.
     *
     * @param[out] wi
     * Incident direction.
     *
     * @param[out] r
     * Raster coordinate. _Optional_.
     *
     * @returns
     * Returns incident importance.
     *
     * @note
     * In _Physically based rendering: from theory to 
     * implementation (3rd edition)_, Pharr, Humphreys, and Jakob 
     * define incident importance for perspective cameras as
     * @f[
     *      W_i(\omega_i; \mathbf{p}_{\text{ref}}) = 
     *                W_e(\mathbf{p}_{\text{hit}}, -\omega_i).
     * @f]
     * This is sensible in their implementation, as they allow 
     * perspective cameras to have &ldquo;lenses&rdquo; with non-zero
     * area. Thus, they obtain solid angle sampling density from area 
     * sampling density as 
     * @f[
     *      f_\omega = \frac{d^2}{|\hat{\mathbf{z}}\cdot-\omega_i|}
     *      f_{\mathcal{A}}
     * @f]
     * where, for pinhole cameras, 
     * @f$ f_{\mathcal{A}} = \delta(\mathbf{p}_{\text{hit}} - 
     * \mathbf{p}_c) @f$. Moreover,
     * the ratio of @f$ W_i @f$ to its sampling density is equivalent to 
     * the ratio of @f$ Z_i @f$ to its sampling density.
     */
    float_type zi_sample(
            const multi<float_type, 3>& pref,
                  multi<float_type, 3>& wi,
                  multi<float_type, 2>* r = nullptr) const
    {
        wi = pc_ - pref;
        float_type d2 = dot(wi, wi);
        if (!(d2 > 0)) {
            return 0;
        }

        // Normalize.
        wi /= pr::sqrt(d2);

        // Inverse square falloff.
        return ze(-wi, r) / d2;
    }

private:

    /**
     * @brief Position @f$ \mathbf{p}_c @f$.
     */
    multi<float_type, 3> pc_;

    /**
     * @brief Basis direction @f$ \hat{\mathbf{x}} @f$.
     */
    multi<float_type, 3> hatx_;

    /**
     * @brief Basis direction @f$ \hat{\mathbf{y}} @f$.
     */
    multi<float_type, 3> haty_;

    /**
     * @brief Basis direction @f$ \hat{\mathbf{z}} @f$.
     */
    multi<float_type, 3> hatz_;

    /**
     * @brief Screen size @f$ s_x @f$.
     */
    float_type sx_ = 0;

    /**
     * @brief Screen size @f$ s_y @f$.
     */
    float_type sy_ = 0;

    /**
     * @brief Screen distance @f$ s_z @f$.
     *
     * @f[
     *      s_z = 
     *      \frac{s_y}{2} 
     *      \cot\left[\frac{f}{2}\right]
     * @f]
     */
    float_type sz_ = 0;

    /**
     * @brief Sub-region bounding point @f$ \mathbf{q}_{\min} @f$.
     */
    multi<float_type, 2> qmin_ = {0, 0};

    /**
     * @brief Sub-region bounding point @f$ \mathbf{q}_{\max} @f$.
     */
    multi<float_type, 2> qmax_ = {1, 1};

    /**
     * @brief Normalization @f$ A @f$.
     *
     * @f[
     * \begin{aligned}
     *      A &= \alpha s_x s_y \frac{1}{s_z^2}
     * \\     &= \alpha \frac{s_x}{s_y} \cdot 4\tan^2\left[\frac{f}{2}\right]
     * \end{aligned}
     * @f]
     * where @f$ \alpha = 
     * ({q_{\max}}_x - {q_{\min}}_x) 
     * ({q_{\max}}_y - {q_{\min}}_y) @f$
     */
    float_type a_ = 0;
};

/**@}*/

} // namespace pr

#endif // #ifndef PREFORM_CAMERA_PERSP_HPP
