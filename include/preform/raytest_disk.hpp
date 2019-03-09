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
#error "preform/raytest_disk.hpp requires >=C++17"
#endif // #if !(__cplusplus >= 201703L)
#endif // #if !DOXYGEN
#pragma once
#ifndef PREFORM_RAYTEST_DISK_HPP
#define PREFORM_RAYTEST_DISK_HPP

// for pr::multi
#include <preform/multi.hpp>

// for pr::multi wrappers
#include <preform/multi_math.hpp>

// for pr::float_bounds
#include <preform/float_bounds.hpp>

// for pr::aabb
#include <preform/aabb.hpp>

namespace pr {

/**
 * @defgroup raytest_disk Ray-test (disk)
 *
 * `<preform/raytest_disk.hpp>`
 *
 * __C++ version__: >=C++17
 */
/**@{*/

/**
 * @brief Ray-test (disk).
 */
template <typename T>
struct raytest_disk
{

    // Sanity check.
    static_assert(
        std::is_floating_point<T>::value, 
        "T must be floating point");

    /**
     * @brief Float type.
     */
    typedef T float_type;

    /**
     * @brief Ray information.
     */
    struct ray_info
    {
    public:

        /**
         * @brief Default constructor.
         */
        ray_info() = default;

        /**
         * @brief Constructor.
         *
         * @param[in] o
         * Origin.
         *
         * @param[in] d
         * Direction.
         *
         * @param[in] tmin
         * Parameter minimum.
         *
         * @param[in] tmax
         * Parameter maximum.
         */
        ray_info(
            const multi<float_type, 3>& o,
            const multi<float_type, 3>& d,
            float_type tmin = 0,
            float_type tmax = 
                pr::numeric_limits<float_type>::infinity()) :
                o(o), d(d), 
                tmin(tmin), 
                tmax(tmax)
        {
        }

        /**
         * @brief Constructor.
         *
         * @param[in] o
         * Origin.
         *
         * @param[in] oerr
         * Origin absolute error.
         *
         * @param[in] d
         * Direction.
         *
         * @param[in] derr
         * Direction absolute error.
         *
         * @param[in] tmin
         * Parameter minimum.
         *
         * @param[in] tmax
         * Parameter maximum.
         */
        ray_info(
            const multi<float_type, 3>& o,
            const multi<float_type, 3>& oerr,
            const multi<float_type, 3>& d,
            const multi<float_type, 3>& derr,
            float_type tmin = 0,
            float_type tmax = 
                pr::numeric_limits<float_type>::infinity()) :
                o(o), oerr(oerr),
                d(d), derr(derr),
                tmin(tmin), 
                tmax(tmax)
        {
        }

    public:

        /**
         * @brief Origin @f$ \mathbf{o} @f$.
         */
        multi<float_type, 3> o = {};

        /**
         * @brief Origin absolute error @f$ \mathbf{o}_{\text{err}} @f$.
         */
        multi<float_type, 3> oerr = {};

        /**
         * @brief Direction @f$ \mathbf{d} @f$.
         */
        multi<float_type, 3> d = {};

        /**
         * @brief Direction absolute error @f$ \mathbf{d}_{\text{err}} @f$.
         */
        multi<float_type, 3> derr = {};

        /**
         * @brief Parameter minimum @f$ t_{\min} @f$.
         */
        float_type tmin = 0;

        /**
         * @brief Parameter maximum @f$ t_{\max} @f$.
         */
        float_type tmax = pr::numeric_limits<float_type>::infinity();
    };

    /**
     * @brief Hit information.
     */
    struct hit_info
    {
    public:

        /**
         * @brief Position @f$ \mathbf{p} @f$.
         */
        multi<float_type, 3> p = {};

        /**
         * @brief Position absolute error @f$ \mathbf{p}_{\text{err}} @f$.
         */
        multi<float_type, 3> perr = {};

        /**
         * @brief Surface parameter @f$ u @f$.
         */
        float_type u = 0;

        /**
         * @brief Surface parameter @f$ v @f$.
         */
        float_type v = 0;

        /**
         * @brief Partial derivative @f$ \partial{\mathbf{p}}/\partial{u} @f$.
         */
        multi<float_type, 3> dp_du = {};

        /**
         * @brief Partial derivative @f$ \partial{\mathbf{p}}/\partial{v} @f$.
         */
        multi<float_type, 3> dp_dv = {};
    };

public:

    /**
     * @brief Default constructor.
     */
    raytest_disk() = default;

    /**
     * @brief Constructor.
     *
     * @param[in] rmin
     * Radius minimum.
     *
     * @param[in] rmax
     * Radius maximum.
     *
     * @param[in] h
     * Height.
     *
     * @param[in] phimax
     * Sweep maximum.
     *
     * @throw std::invalid_argument
     * Unless
     * - `rmin > 0`,
     * - `rmax > rmin`,
     * - `phimax > 0`, and
     * - `phimax <= 2 * pi`.
     */
    raytest_disk(
            float_type rmin,
            float_type rmax,
            float_type h = 0,
            float_type phimax = 
                2 * pr::numeric_constants<float_type>::M_pi()) :
                rmin_(rmin),
                rmax_(rmax),
                h_(h),
                phimax_(phimax)
    {
        if (!(rmin_ > 0 &&
              rmax_ > rmin_ &&
              phimax_ > 0 &&
              phimax_ <= 2 * pr::numeric_constants<float_type>::M_pi())) {
            throw std::invalid_argument(__PRETTY_FUNCTION__);
        }
    }

    /**
     * @brief Bounding box.
     */
    aabb<float_type, 3> bounding_box() const
    {
        return {
            {-rmax_, -rmax_, h_},
            {+rmax_, +rmax_, h_}
        };
    }

    /**
     * @brief Surface area.
     *
     * @f[
     *      A = \frac{1}{2} \phi_{\max} (r_{\max}^2 - r_{\min}^2)
     * @f]
     */
    float_type surface_area() const
    {
        return float_type(0.5) * phimax_ * (rmax_ * rmax_ - rmin_ * rmin_);
    }

    /**
     * @brief Evaluate.
     *
     * @param[in] u
     * Parameter in @f$ [0, 1) @f$.
     *
     * @param[in] v
     * Parameter in @f$ [0, 1) @f$.
     */
    hit_info evaluate(
                float_type u, 
                float_type v) const
    {
        hit_info hit;
        float_type r = (1 - u) * rmin_ + u * rmax_;
        float_type phi = v * phimax_;
        float_type sin_phi = pr::sin(phi);
        float_type cos_phi = pr::cos(phi);

        // Position.
        hit.p = {
            r * cos_phi,
            r * sin_phi,
            h_
        };

        // Position absolute error.
        hit.perr = {};

        // Surface parameters.
        hit.u = u;
        hit.v = v;

        // Surface partial derivatives.
        hit.dp_du = {
            (rmax_ - rmin_) * cos_phi,
            (rmax_ - rmin_) * sin_phi,
            0
        };
        hit.dp_dv = {
            -phimax_ * hit.p[1],
            +phimax_ * hit.p[0],
            0
        };

        return hit;
    }

    /**
     * @brief Intersect.
     *
     * @param[in] ray
     * Ray information.
     *
     * @param[out] hit
     * Hit information. _Optional_.
     *
     * @returns
     * If intersection, returns parameteric value. Else, 
     * returns NaN.
     */
    float_type intersect(const ray_info& ray, hit_info* hit = nullptr) const
    {
        // Parameter.
        float_bounds<float_type> o2 = {ray.o[2], ray.oerr[2]};
        float_bounds<float_type> d2 = {ray.d[2], ray.derr[2]};
        float_bounds<float_type> t = (h_ - o2) / d2;

        // Reject uncertain intersections.
        if (!(t.upper_bound() <= ray.tmax &&
              t.lower_bound() >= ray.tmin)) {
            return pr::numeric_limits<float_type>::quiet_NaN();
        }

        // Position.
        multi<float_type, 3> p = {
            ray.o[0] + ray.d[0] * t.value(),
            ray.o[1] + ray.d[1] * t.value(),
            h_
        };
        if (p[0] == 0 &&
            p[1] == 0) {
            p[0] = float_type(1e-6);
        }

        // Radius.
        float_type r = pr::hypot(p[0], p[1]);

        // Clip.
        if (!(r <= rmax_ &&
              r >= rmin_)) {
            return pr::numeric_limits<float_type>::quiet_NaN();
        }

        // Sweep angle.
        float_type phi = pr::atan2(p[1], p[0]);
        if (phi < 0) {
            phi += 2 * pr::numeric_constants<float_type>::M_pi();
        }

        // Clip.
        if (phi > phimax_) {
            return pr::numeric_limits<float_type>::quiet_NaN();
        }

        if (hit) {

            // Position.
            hit->p = p;

            // Position absolute error.
            hit->perr = {};

            // Surface parameters.
            hit->u = (r - rmin_) / (rmax_ - rmin_);
            hit->v = phi / phimax_;

            // Surface partial derivatives.
            float_type cos_phi = p[0] / r;
            float_type sin_phi = p[1] / r;
            hit->dp_du = {
                (rmax_ - rmin_) * cos_phi,
                (rmax_ - rmin_) * sin_phi,
                0
            };
            hit->dp_dv = {
                -phimax_ * p[1],
                +phimax_ * p[0],
                0
            };
        }

        // Success.
        return t.value();
    }

private:

    /**
     * @brief Radius minimum @f$ r_{\min} @f$.
     */
    float_type rmin_ = 0;

    /**
     * @brief Radius maximum @f$ r_{\max} @f$.
     */
    float_type rmax_ = 1;

    /**
     * @brief Height @f$ h @f$.
     */
    float_type h_ = 0;

    /**
     * @brief Sweep maximum @f$ \phi_{\max} @f$.
     */
    float_type phimax_ = 2 * pr::numeric_constants<float_type>::M_pi();
};

/**@}*/

} // namespace pr

#endif // #ifndef PREFORM_RAYTEST_DISK_HPP
