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
#error "preform/raytest_triangle.hpp requires >=C++17"
#endif // #if !(__cplusplus >= 201703L)
#endif // #if !DOXYGEN
#pragma once
#ifndef PREFORM_RAYTEST_TRIANGLE_HPP
#define PREFORM_RAYTEST_TRIANGLE_HPP

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
 * @defgroup raytest_triangle Ray-test (triangle)
 *
 * `<preform/raytest_triangle.hpp>`
 *
 * __C++ version__: >=C++17
 */
/**@{*/

/**
 * @brief Ray-test (triangle).
 */
template <typename T>
struct raytest_triangle
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

#if __GNUC__ && !__clang__

    // Sanity check.
    static_assert(
        std::is_same<T, float>::value ||
        std::is_same<T, double>::value,
        "T must be float or double");

    /**
     * @brief Double type.
     */
    typedef std::conditional_t<
            std::is_same<T, float>::value, double,
            __float128> double_type;

#else

    // Sanity check.
    static_assert(
        std::is_same<T, float>::value,
        "T must be float");

    /**
     * @brief Double type.
     */
    typedef double double_type;

#endif // #if __GNUC__ && !__clang__

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
            // Initialize permutation.
            k[2] = pr::fabs(o).argmax();
            k[0] = (k[2] + 1) % 3;
            k[1] = (k[0] + 1) % 3;

            // Initialize shear.
            h[2] = 1 / d[k[2]];
            h[0] = d[k[0]] * h[2];
            h[1] = d[k[1]] * h[2];
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
            // Initialize permutation.
            k[2] = pr::fabs(o).argmax();
            k[0] = (k[2] + 1) % 3;
            k[1] = (k[0] + 1) % 3; 

            // Initialize shear.
            h[2] = 1 / d[k[2]];
            h[0] = d[k[0]] * h[2];
            h[1] = d[k[1]] * h[2];
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
         * @brief Permutation @f$ \mathbf{k} @f$.
         *
         * @note
         * @f[
         *      \mathbf{k} = 
         *      \begin{bmatrix}
         *          (\ell + 1) \% 3
         *      \\  (\ell + 2) \% 3
         *      \\   \ell
         *      \end{bmatrix}
         * @f]
         * where 
         * @f[
         *      \ell = \operatorname{argmax}
         *      \begin{bmatrix}
         *          |d_{[0]}|
         *      \\  |d_{[1]}|
         *      \\  |d_{[2]}|
         *      \end{bmatrix}
         * @f]
         */
        multi<int, 3> k;

        /**
         * @brief Shear @f$ \mathbf{h} @f$.
         *
         * @note
         * @f[
         *      \mathbf{h} = 
         *      \frac{1}{d_{[k_{[2]}]}}
         *      \begin{bmatrix}
         *          d_{[k_{[0]}]}
         *      \\  d_{[k_{[1]}]}
         *      \\  1
         *      \end{bmatrix}
         * @f]
         */
        multi<float_type, 3> h;

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
         * @brief Surface parameters @f$ \mathbf{s} @f$.
         */
        multi<float_type, 2> s = {};

        /**
         * @brief Partial @f$ \partial{\mathbf{p}}/\partial{s_{[0]}} @f$.
         */
        multi<float_type, 3> dp_ds0 = {};

        /**
         * @brief Partial @f$ \partial{\mathbf{p}}/\partial{s_{[1]}} @f$.
         */
        multi<float_type, 3> dp_ds1 = {};
    };

public:

    /**
     * @brief Default constructor.
     */
    raytest_triangle() = default;

    /**
     * @brief Constructor.
     */
    raytest_triangle(
            const multi<float_type, 3>& p0,
            const multi<float_type, 3>& p1,
            const multi<float_type, 3>& p2) :
                p_{p0, p1, p2}
    {
    }

    /**
     * @brief Bounding box.
     */
    aabb<float_type, 3> bounding_box() const
    {
        aabb<float_type, 3> box;
        box |= p_[0];
        box |= p_[1];
        box |= p_[2];
        return box;
    }

    /**
     * @brief Surface area.
     *
     * @f[
     *      A = \lVert \mathbf{q}_1 \times \mathbf{q}_2 \rVert
     * @f]
     * where
     * - @f$ \mathbf{q}_1 = \mathbf{p}_1 - \mathbf{p}_0 @f$
     * - @f$ \mathbf{q}_2 = \mathbf{p}_2 - \mathbf{p}_0 @f$
     */
    float_type surface_area() const
    {
        multi<float_type, 3> q1 = p_[1] - p_[0];
        multi<float_type, 3> q2 = p_[2] - p_[0];
        return length(cross(q1, q2));
    }

    /**
     * @brief Surface area probability density function.
     *
     * @f[
     *      f = \frac{1}{A}
     * @f]
     */
    float_type surface_area_pdf() const
    {
        return 1 / surface_area();
    }

    /**
     * @brief Surface area probability density function sampling routine.
     *
     * @param[in] u
     * Sample in @f$ [0, 1)^2 @f$.
     */
    hit_info surface_area_pdf_sample(multi<float_type, 2> u) const
    {
        // Delegate.
        return operator()({1 - pr::sqrt(u[0]), u[1] * pr::sqrt(u[0])});
    }

    // TODO verify, incorporate prior error?
#if 0
    float_type intersect(const ray_info& ray,
                               hit_info* hit = nullptr) const
    {
        // Shear in XY.
        multi<float_type, 3> g[3];
        multi<float_type, 3> h[3];
        for (int j = 0; j < 3; j++) {
            g[j] = (p_[j] - ray.o).swizzle(ray.k);
            h[j][0] = pr::fma(-ray.h[0], g[j][2], g[j][0]);
            h[j][1] = pr::fma(-ray.h[1], g[j][2], g[j][1]); 
        }

        // Compute preliminary barycentric coordinates.
        float_type b[3];
        bool any_zero = false;
        for (int j = 0; j < 3; j++) {
            int j1 = (j + 1) % 3;
            int j2 = (j + 2) % 3;
            b[j] = h[j1][1] * h[j2][0] - 
                   h[j1][0] * h[j2][1];
            if (b[j] == float_type(0)) {
                any_zero = true;
                break;
            }
        }

        // Any zero?
        if (any_zero) {
            // Recalculate in double precision.
            for (int j = 0; j < 3; j++) {
                int j1 = (j + 1) % 3;
                int j2 = (j + 2) % 3;
                b[j] = double_type(h[j1][1]) * double_type(h[j2][0]) - 
                       double_type(h[j1][0]) * double_type(h[j2][1]);
            }
        }

        // Is outside?
        if ((b[0] < float_type(0) || 
             b[1] < float_type(0) || 
             b[2] < float_type(0)) &&
            (b[0] > float_type(0) || 
             b[1] > float_type(0) || 
             b[2] > float_type(0))) {
            // No intersection.
            return pr::numeric_limits<float_type>::quiet_NaN();
        }

        // Is parallel?
        float_type q = b[0] + b[1] + b[2];
        if (!(pr::fabs(q) > float_type(0))) {
            // No intersection.
            return pr::numeric_limits<float_type>::quiet_NaN();
        }

        // Normalize barycentric coordinates.
        if (pr::fabs(q) < 
            pr::numeric_limits<float_type>::min_invertible()) {
            b[0] /= q;
            b[1] /= q;
            b[2] /= q;
        }
        else {
            float_type invq = 1 / q;
            b[0] *= invq;
            b[1] *= invq;
            b[2] *= invq;
        }

        // Shear in Z.
        h[0][2] = ray.h[2] * g[0][2];
        h[1][2] = ray.h[2] * g[1][2];
        h[2][2] = ray.h[2] * g[2][2];

        // Compute parametric value.
        float_type t = 
            pr::fma(b[0], h[0][2],
            pr::fma(b[1], h[1][2], b[2] * h[2][2]));

        // Is out of range?
        if (!(t > ray.tmin && 
              t < ray.tmax)) {
            // No intersection.
            return pr::numeric_limits<float_type>::quiet_NaN();
        }

        // Bound error on h.
        float_type hmax[3];
        float_type herr[3];
        for (int j = 0; j < 3; j++) {
            hmax[j] = 
            pr::fmax(pr::fabs(h[0][j]),
            pr::fmax(pr::fabs(h[1][j]), pr::fabs(h[2][j])));
        }
        herr[0] = pr::numeric_limits<float_type>::echelon(5) * hmax[0];
        herr[1] = pr::numeric_limits<float_type>::echelon(5) * hmax[1];
        herr[2] = pr::numeric_limits<float_type>::echelon(3) * hmax[2];

        // Bound error on b.
        float_type bmax = 
            pr::fmax(pr::fabs(b[0]), 
            pr::fmax(pr::fabs(b[1]), pr::fabs(b[2])));
        float_type berr = 
            float_type(2) * 
            (pr::numeric_limits<float_type>::echelon(2) * 
                hmax[0] * hmax[1] + 
                herr[1] * hmax[0] + 
                herr[0] * hmax[1]);

        // Bound error on t.
        float_type terr = 
            float_type(3) * 
            (pr::numeric_limits<float_type>::echelon(3) * 
                bmax * hmax[2] + 
                berr * hmax[2] + 
                herr[2] * bmax);

        // Is within error bound?
        if (t < pr::finc(ray.tmin + terr)) {
            // No intersection.
            return pr::numeric_limits<float_type>::quiet_NaN();
        }

        if (hit) {
            multi<float_type, 3> bp[3] = {
                b[0] * p_[0],
                b[1] * p_[1],
                b[2] * p_[2]
            };

            // Position.
            hit->p = bp[0] + bp[1] + bp[2];

            // Position absolute error.
            hit->perr = 
                pr::fabs(bp[0]) +
                pr::fabs(bp[1]) +
                pr::fabs(bp[2]);
            hit->perr *= pr::numeric_limits<float_type>::echelon(6);

            // Surface parameters.
            hit->s[0] = b[1];
            hit->s[1] = b[2];

            // Surface partial derivatives.
            hit->dp_ds0 = p_[1] - p_[0];
            hit->dp_ds1 = p_[2] - p_[0];
        }

        return t;
    }
#endif

    /**
     * @brief Evaluate.
     *
     * @param[in] s
     * Parameters in @f$ [0, 1)^2 @f$.
     */
    hit_info operator()(multi<float_type, 2> s) const
    {
        hit_info hit;
        multi<float_type, 3> b = {1 - s[0] - s[1], s[0], s[1]};
        multi<float_type, 3> bp[3] = {
            b[0] * p_[0],
            b[1] * p_[1],
            b[2] * p_[2]
        };

        // Position.
        hit.p = bp[0] + bp[1] + bp[2];

        // Position absolute error.
        hit.perr = 
            pr::fabs(bp[0]) + 
            pr::fabs(bp[1]) + 
            pr::fabs(bp[2]);
        hit.perr *= pr::numeric_limits<float_type>::echelon(6);

        // Surface parameters.
        hit.s = s;

        // Surface partial derivatives.
        hit.dp_ds0 = p_[1] - p_[0];
        hit.dp_ds1 = p_[2] - p_[0];
        return hit;
    }

private:

    /**
     * @brief Points @f$ \mathbf{p}_k @f$.
     */
    multi<float_type, 3> p_[3] = {
        {0, 0, 0},
        {1, 0, 0},
        {0, 1, 0}
    };
};

/**@}*/

} // namespace pr

#endif // #ifndef PREFORM_RAYTEST_TRIANGLE_HPP
