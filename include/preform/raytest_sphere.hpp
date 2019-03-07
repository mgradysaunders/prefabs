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
#error "preform/raytest_sphere.hpp requires >=C++17"
#endif // #if !(__cplusplus >= 201703L)
#endif // #if !DOXYGEN
#pragma once
#ifndef PREFORM_RAYTEST_SPHERE_HPP
#define PREFORM_RAYTEST_SPHERE_HPP

// for pr::multi
#include <preform/multi.hpp>

// for pr::multi wrappers
#include <preform/multi_math.hpp>

// for pr::float_bounds
#include <preform/float_bounds.hpp>

namespace pr {

/**
 * @defgroup raytest_sphere Ray-test (sphere)
 *
 * `<preform/raytest_sphere.hpp>`
 *
 * __C++ version__: >=C++17
 */
/**@{*/

/**
 * @brief Ray-test (sphere).
 *
 * This implementation is based on the implementation of Pharr, Humphreys, 
 * and Jakob in _Physically based rendering: from theory to implementation 
 * (3rd edition)_.
 */
template <typename T>
struct raytest_sphere
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
    raytest_sphere() = default;

    /**
     * @brief Constructor.
     *
     * @param[in] r
     * Radius.
     *
     * @param[in] thetamin
     * Polar angle minimum.
     *
     * @param[in] thetamax
     * Polar angle maximum.
     *
     * @param[in] phimax
     * Azimuthal angle maximum.
     *
     * @throw std::invalid_argument
     * Unless
     * - `r > 0`,
     * - `thetamin >= 0 && thetamin <= pi`,
     * - `thetamax >= 0 && thetamax <= pi`,
     * - `thetamax > thetamin`, and
     * - `phimax >= 0 && phimax <= 2 * pi`.
     */
    raytest_sphere(
        float_type r, 
        float_type thetamin, 
        float_type thetamax,
        float_type phimax) :
            r_(r),
            thetamin_(thetamin),
            thetamax_(thetamax),
            phimax_(phimax)
    {
        if (!(r_ > 0 &&
            thetamin_ >= 0 && 
            thetamax_ >= 0 && 
            thetamin_ <= pr::numeric_constants<float_type>::M_pi() &&
            thetamax_ <= pr::numeric_constants<float_type>::M_pi() &&
            thetamax_ > thetamin_ &&
            phimax_ >= 0 &&
            phimax_ <= 2 * pr::numeric_constants<float_type>::M_pi())) {
            throw std::invalid_argument(__PRETTY_FUNCTION__);
        }

        zmin_ = r_ * pr::cos(thetamax_);
        zmax_ = r_ * pr::cos(thetamin_);
        if (!(zmin_ < zmax_)) {
            std::swap(zmin_, zmax_); // Shouldn't happen
        }
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
        // Assemble bounds.
        multi<float_bounds<float_type>, 3> o;
        multi<float_bounds<float_type>, 3> d;
        for (int k = 0; k < 3; k++) {
            o[k] = {
                ray.o[k],
                fdec(ray.o[k] - ray.oerr[k]),
                finc(ray.o[k] + ray.oerr[k])
            };
            d[k] = {
                ray.d[k],
                fdec(ray.d[k] - ray.derr[k]),
                finc(ray.d[k] + ray.derr[k])
            };
        }

        // Quadratic roots.
        float_bounds<float_type> r = r_;
        float_bounds<float_type> t0, t1;
        float_bounds<float_type>::solve_poly2(
            dot(o, o) - r * r,
            dot(d, o) * float_type(2),
            dot(d, d),
            t0, t1);
        if (!(t0.upper_bound() <= ray.tmax) ||
            !(t1.lower_bound() >= ray.tmin)) {
            return pr::numeric_limits<float_type>::quiet_NaN();
        }

        // Select root.
        float_bounds<float_type> t = t0;
        if (t.lower_bound() < ray.tmin) {
            t = t1;
            if (t.upper_bound() > ray.tmax) {
                return pr::numeric_limits<float_type>::quiet_NaN();
            }
        }

        // Position.
        multi<float_type, 3> p = 
            normalize(ray.o + ray.d * t.value()) * r_;
        if (p[0] == 0 && 
            p[1] == 0) {
            p[0] = float_type(1e-6) * r_;
        }

        // Azimuthal angle.
        float_type phi = pr::atan2(p[1], p[0]);
        if (phi < 0) {
            phi += 2 * pr::numeric_constants<float_type>::M_pi();
        }

        // Clip.
        if ((zmin_ > -r_ && p[2] < zmin_) ||
            (zmax_ < +r_ && p[2] > zmax_) ||
             phi > phimax_) {
            
            // Already farther root?
            if (t.value() == t1.value()) {
                return pr::numeric_limits<float_type>::quiet_NaN();
            }

            // Farther root.
            t = t1;

            // Position.
            p = normalize(ray.o + ray.d * t.value()) * r_;
            if (p[0] == 0 && 
                p[1] == 0) {
                p[0] = float_type(1e-6) * r_;
            }

            // Azimuthal angle.
            phi = pr::atan2(p[1], p[0]);
            if (phi < 0) {
                phi += 2 * pr::numeric_constants<float_type>::M_pi();
            }
    
            // Clip.
            if ((zmin_ > -r_ && p[2] < zmin_) ||
                (zmax_ < +r_ && p[2] > zmax_) ||
                 phi > phimax_) {
                return pr::numeric_limits<float_type>::quiet_NaN();
            }
        }

        if (hit) {

            // Position.
            hit->p = p;

            // Position absolute error.
            hit->perr = 
                pr::fabs(p) * 
                pr::numeric_limits<float_type>::echelon(5);

            // Polar angle.
            float_type cos_theta = p[2] / r_;
            cos_theta = pr::fmax(cos_theta, float_type(-1));
            cos_theta = pr::fmin(cos_theta, float_type(+1));
            float_type sin_theta = pr::sqrt(1 - cos_theta * cos_theta);
            float_type theta = pr::acos(cos_theta);

            // Surface parameters.
            hit->u = phi / phimax_;
            hit->v = (theta - thetamin_) / 
                     (thetamax_ - thetamin_);

            // Surface partial derivatives.
            float_type zr = pr::hypot(p[0], p[1]);
            float_type cos_phi = p[0] / zr;
            float_type sin_phi = p[1] / zr;
            hit->dp_du = {
                -phimax_ * p[1],
                +phimax_ * p[0],
                0
            };
            hit->dp_dv = {
                +(r_ * (thetamax_ - thetamin_)) * cos_theta * cos_phi,
                +(r_ * (thetamax_ - thetamin_)) * cos_theta * sin_phi,
                -(r_ * (thetamax_ - thetamin_)) * sin_theta
            };
        }

        // Success.
        return t.value();
    }

private:

    /**
     * @brief Radius @f$ r @f$.
     */
    float_type r_ = 1;

    /**
     * @brief Polar angle minimum @f$ \theta_{\min} @f$.
     */
    float_type thetamin_ = 0;

    /**
     * @brief Polar angle maximum @f$ \theta_{\max} @f$.
     */
    float_type thetamax_ = pr::numeric_constants<float_type>::M_pi();

    /**
     * @brief Height minimum @f$ z_{\min} = r\cos{\theta_{\max}} @f$.
     */
    float_type zmin_ = -1;

    /**
     * @brief Height maximum @f$ z_{\max} = r\cos{\theta_{\min}} @f$.
     */
    float_type zmax_ = +1;

    /**
     * @brief Azimuthal angle maximum @f$ \phi_{\max} @f$.
     */
    float_type phimax_ = 2 * pr::numeric_constants<float_type>::M_pi();
};

/**@}*/

} // namespace pr

#endif // #ifndef PREFORM_RAYTEST_SPHERE_HPP
