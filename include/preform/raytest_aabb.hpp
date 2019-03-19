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
#error "preform/raytest_aabb.hpp requires >=C++17"
#endif // #if !(__cplusplus >= 201703L)
#endif // #if !DOXYGEN
#pragma once
#ifndef PREFORM_RAYTEST_AABB_HPP
#define PREFORM_RAYTEST_AABB_HPP

// for pr::multi
#include <preform/multi.hpp>

// for pr::multi wrappers
#include <preform/multi_math.hpp>

// for pr::float_interval
#include <preform/float_interval.hpp>

// for pr::aabb
#include <preform/aabb.hpp>

namespace pr {

/**
 * @defgroup raytest_aabb Ray-test (AABB)
 *
 * `<preform/raytest_aabb.hpp>`
 *
 * __C++ version__: >=C++17
 */
/**@{*/

/**
 * @brief Ray-test (AABB).
 *
 * @tparam T
 * Float type.
 */
template <typename T>
struct raytest_aabb
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
            // Delegate.
            cache();
        }
#if 0
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
            // Delegate.
            cache();
        }
#endif

    public:

        /**
         * @brief Origin @f$ \mathbf{o} @f$.
         */
        multi<float_type, 3> o = {};

#if 0
        /**
         * @brief Origin absolute error @f$ \mathbf{o}_{\text{err}} @f$.
         */
        multi<float_type, 3> oerr = {};
#endif

        /**
         * @brief Direction @f$ \mathbf{d} @f$.
         */
        multi<float_type, 3> d = {};

#if 0
        /**
         * @brief Direction absolute error @f$ \mathbf{d}_{\text{err}} @f$.
         */
        multi<float_type, 3> derr = {};
#endif

        /**
         * @brief Parameter minimum @f$ t_{\min} @f$.
         */
        float_type tmin = 0;

        /**
         * @brief Parameter maximum @f$ t_{\max} @f$.
         */
        float_type tmax = pr::numeric_limits<float_type>::infinity();

    private:

        /**
         * @name Cache variables
         */
        /**@{*/

        /**
         * @brief Direction minimum index @f$ \mathbf{d}_{\min} @f$.
         *
         * @note
         * @f[
         *      d_{\min[k]} =
         *      \begin{cases}
         *          1 & d_{[k]} < 0
         *      \\  0 & d_{[k]} \ge 0
         *      \end{cases}
         * @f]
         */
        multi<int, 3> dmin;

        /**
         * @brief Direction maximum index @f$ \mathbf{d}_{\max} @f$.
         *
         * @note
         * @f[
         *      d_{\max[k]} = 1 - d_{\min[k]}
         * @f]
         */
        multi<int, 3> dmax;

        /**
         * @brief Direction inverse @f$ \mathbf{d}^{-1} @f$.
         */
#if 0
        multi<float_interval<float_type>, 3> dinv;
#else
        multi<float_type, 3> dinv;
#endif

        /**
         * @brief Cache.
         */
        void cache()
        {
            dmin = pr::signbit(d);
            dmax = 1 - dmin;
#if 0
            for (int j = 0; j < 3; j++) {
                dinv[j] = float_type(1) /
                          float_interval<float_type>{d[j], derr[j]};
            }
#else
            dinv = 1 / d;
#endif
        }

        friend struct raytest_aabb<T>;

        /**@}*/
    };

public:

    /**
     * @brief Default constructor.
     */
    raytest_aabb() = default;

    /**
     * @brief Constructor.
     *
     * @param[in] box
     * Axis-aligned bounding box.
     */
    raytest_aabb(const aabb<float_type, 3>& box) : box_(box)
    {
    }

    /**
     * @brief Intersect.
     *
     * @param[in] ray
     * Ray information.
     *
     * @returns
     * If intersection, returns parameteric value. Else,
     * returns NaN.
     */
    float_type intersect(const ray_info& ray) const
    {
#if 0
        // Assemble intervals.
        multi<float_interval<float_type>, 3> o;
     /* multi<float_interval<float_type>, 3> d; */
        for (int j = 0; j < 3; j++) {
            o[j] = {ray.o[j], ray.oerr[j]};
         /* d[j] = {ray.d[j], ray.derr[j]}; */
        }

        // Slab intersection.
        float_interval<float_type> tmin, tkmin;
        float_interval<float_type> tmax, tkmax;

        // Slab 0.
        tmin = (box_[ray.dmin[0]][0] - o[0]) * ray.dinv[0];
        tmax = (box_[ray.dmax[0]][0] - o[0]) * ray.dinv[0];
        for (int k = 1; k < 3; k++) {

            // Slab k.
            tkmin = (box_[ray.dmin[k]][k] - o[k]) * ray.dinv[k];
            tkmax = (box_[ray.dmax[k]][k] - o[k]) * ray.dinv[k];

            // Reject certain misses.
            if (!(tmin.lower_bound() < tkmax.upper_bound() &&
                  tmax.upper_bound() > tkmin.lower_bound())) {
                // No intersection.
                return pr::numeric_limits<float_type>::quiet_NaN();
            }

            // Update.
            if (tmin.value() < tkmin.value()) tmin = tkmin;
            if (tmax.value() > tkmax.value()) tmax = tkmax;
        }

        // Reject uncertain hits.
        if (!(tmin.upper_bound() < ray.tmax &&
              tmax.lower_bound() > ray.tmin)) {
            // No intersection.
            return pr::numeric_limits<float_type>::quiet_NaN();
        }

        return tmin.value() > ray.tmin ?
               tmin.value() : tmax.value();
#else
        // Slab intersection.
        float_type tmin, tkmin;
        float_type tmax, tkmax;

        // Slab 0.
        tmin = (box_[ray.dmin[0]][0] - ray.o[0]) * ray.dinv[0];
        tmax = (box_[ray.dmax[0]][0] - ray.o[0]) * ray.dinv[0];
        tmax *= 1 + 2 * pr::numeric_limits<float_type>::echelon(3);
        for (int k = 1; k < 3; k++) {

            // Slab k.
            tkmin = (box_[ray.dmin[k]][k] - ray.o[k]) * ray.dinv[k];
            tkmax = (box_[ray.dmax[k]][k] - ray.o[k]) * ray.dinv[k];
            tkmax *= 1 + 2 * pr::numeric_limits<float_type>::echelon(3);

            // Reject certain misses.
            if (!(tmin < tkmax &&
                  tmax > tkmin)) {
                // No intersection.
                return pr::numeric_limits<float_type>::quiet_NaN();
            }

            // Update.
            tmin = pr::fmax(tmin, tkmin);
            tmax = pr::fmin(tmax, tkmax);
        }

        // Reject uncertain hits.
        if (!(tmin < ray.tmax &&
              tmax > ray.tmin)) {
            // No intersection.
            return pr::numeric_limits<float_type>::quiet_NaN();
        }

        return tmin > ray.tmin ? tmin : tmax;
#endif
    }

private:

    /**
     * @brief Axis-aligned bounding box.
     */
    aabb<float_type, 3> box_ = {
        {-1, -1, -1},
        {+1, +1, +1}
    };
};

/**@}*/

} // namespace pr

#endif // #ifndef PREFORM_RAYTEST_AABB_HPP