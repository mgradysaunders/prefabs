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

namespace pr {

/**
 * @addtogroup raytest
 */
/**@{*/

/**
 * @brief Ray-test (triangle).
 *
 * @tparam T
 * Float type.
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

    /**
     * @brief Ray information.
     */
    struct ray_type
    {
    public:

        /**
         * @brief Default constructor.
         */
        ray_type() = default;

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
        ray_type(
            const multi<float_type, 3>& o,
            const multi<float_type, 3>& d,
            float_type tmin = 0,
            float_type tmax =
                pr::numeric_limits<float_type>::infinity()) :
                o(o), d(d),
                tmin(tmin),
                tmax(tmax)
        {
            cache();
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
        ray_type(
            const multi<float_type, 3>& o, const multi<float_type, 3>& oerr,
            const multi<float_type, 3>& d, const multi<float_type, 3>& derr,
            float_type tmin = 0,
            float_type tmax =
                pr::numeric_limits<float_type>::infinity()) :
                o(o), oerr(oerr),
                d(d), derr(derr),
                tmin(tmin),
                tmax(tmax)
        {
            cache();
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

    private:

        /**
         * @name Cache variables
         */
        /**@{*/

        /**
         * @brief Permutation @f$ \mathbf{k} @f$.
         *
         * @note
         * @f[
         *      \mathbf{k} =
         *          \begin{bmatrix}
         *              (\ell + 1) \% 3
         *          \\  (\ell + 2) \% 3
         *          \\   \ell
         *          \end{bmatrix}
         * @f]
         * where
         * @f[
         *      \ell =
         *          \operatorname{argmax}
         *          \begin{bmatrix}
         *              |d_{[0]}|
         *          \\  |d_{[1]}|
         *          \\  |d_{[2]}|
         *          \end{bmatrix}
         * @f]
         */
        multi<int, 3> k;

        /**
         * @brief Shear @f$ \mathbf{h}_{\mathrm{r}} @f$.
         *
         * @note
         * @f[
         *      \mathbf{h}_{\mathrm{r}} =
         *          \frac{1}{d_{[k_{[2]}]}}
         *          \begin{bmatrix}
         *              d_{[k_{[0]}]}
         *          \\  d_{[k_{[1]}]}
         *          \\  1
         *          \end{bmatrix}
         * @f]
         */
        multi<float_interval<float_type>, 3> hr;

        /**@}*/

        /**
         * @brief Cache.
         */
        void cache()
        {
            // Permutation.
            k[2] = pr::fabs(d).argmax();
            k[0] = (k[2] + 1) % 3;
            k[1] = (k[2] + 2) % 3;

            // Shear.
            hr[2] =
                float_type(1) /
                float_interval<float_type>{d[k[2]], derr[k[2]]};
            hr[0] = hr[2] * float_interval<float_type>{d[k[0]], derr[k[0]]};
            hr[1] = hr[2] * float_interval<float_type>{d[k[1]], derr[k[1]]};
        }

        friend struct raytest_triangle<float_type>;
    };

    /**
     * @brief Hit information.
     */
    struct hit_type
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
         * @brief Surface parameters @f$ \mathbf{b} @f$.
         */
        multi<float_type, 3> b = {};
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
     * - @f$ \mathbf{q}_1 \gets \mathbf{p}_1 - \mathbf{p}_0 @f$
     * - @f$ \mathbf{q}_2 \gets \mathbf{p}_2 - \mathbf{p}_0 @f$
     * @f[
     *      A = \lVert \mathbf{q}_1 \times \mathbf{q}_2 \rVert
     * @f]
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
     *      f_{A} = \frac{1}{A}
     * @f]
     */
    float_type surface_area_pdf() const
    {
        return float_type(1) / surface_area();
    }

    /**
     * @brief Surface area probability density function sampling routine.
     *
     * - @f$ \mu_0 \gets \sqrt{u_{[0]}} @f$
     * - @f$ \mu_1 \gets u_{[1]} \mu_0 @f$
     * - @f$ b_0 \gets 1 - \mu_0 @f$
     * - @f$ b_1 \gets \mu_1 @f$
     * - @f$ b_2 \gets \mu_0 - \mu_1 @f$
     * @f[
     *      \mathbf{p}_{\text{hit}}(\mathbf{u}) = 
     *          b_0 \mathbf{p}_0 + 
     *          b_1 \mathbf{p}_1 +
     *          b_2 \mathbf{p}_2
     * @f]
     *
     * @param[in] u
     * Sample in @f$ [0, 1)^2 @f$.
     */
    hit_type surface_area_pdf_sample(multi<float_type, 2> u) const
    {
        // Delegate.
        float_type mu0 = pr::sqrt(u[0]);
        float_type mu1 = u[1] * mu0;
        return operator()({
            1 - mu0,
            mu1,
            mu0 - mu1
        });
    }

    /**
     * @brief Solid angle probability density function.
     *
     * - @f$ \mathbf{q}_1 \gets \mathbf{p}_1 - \mathbf{p}_0 @f$
     * - @f$ \mathbf{q}_2 \gets \mathbf{p}_2 - \mathbf{p}_0 @f$
     * - @f$ \mathbf{v}_g \gets \mathbf{q}_1 \times \mathbf{q}_2 @f$
     * - @f$ \mathbf{v}_i \gets 
     *       \mathbf{p}_{\text{hit}} - \mathbf{p}_{\text{ref}} @f$
     * - @f$ \omega_i \gets \normalize(\mathbf{v}_i) @f$
     * - @f$ \omega_g \gets \normalize(\mathbf{v}_g) @f$
     * - @f$ A \gets \lVert \mathbf{v}_g \rVert @f$
     * @f[
     *      f_{\omega}(
     *          \mathbf{p}_{\text{ref}} \to
     *          \mathbf{p}_{\text{hit}}) = 
     *          \frac{1}{A} 
     *          \frac{|\mathbf{v}_i \cdot \mathbf{v}_i|}
     *               {|\omega_i \cdot \omega_g|} =
     *          \frac{|\mathbf{v}_i \cdot \mathbf{v}_i|^{3/2}}
     *               {|\mathbf{v}_i \cdot \mathbf{v}_g|}
     * @f]
     *
     * @param[in] pref
     * Reference point.
     *
     * @param[in] phit
     * Hit point.
     *
     * @note
     * For efficiency, the implementation assumes 
     * @f$ \mathbf{p}_{\text{hit}} @f$ is actually on the surface.
     */
    float_type solid_angle_pdf(
            const multi<float_type, 3>& pref,
            const multi<float_type, 3>& phit) const
    {
        multi<float_type, 3> q1 = p_[1] - p_[0];
        multi<float_type, 3> q2 = p_[2] - p_[0];
        multi<float_type, 3> vg = cross(q1, q2);
        multi<float_type, 3> vi = phit - pref;
        float_type dot_vi_vg = dot(vi, vg);
        float_type dot_vi_vi = dot(vi, vi);
        if (dot_vi_vg == 0 ||
            dot_vi_vi == 0) {
            return 0;
        }
        else {
            return dot_vi_vi * 
                pr::sqrt(dot_vi_vi) / 
                pr::fabs(dot_vi_vg);
        }
    }

    /**
     * @brief Solid angle probability density function sampling routine.
     *
     * @param[in] u
     * Sample in @f$ [0, 1)^2 @f$.
     *
     * @param[in] pref
     * Reference point.
     */
    hit_type solid_angle_pdf_sample(
            const multi<float_type, 2>& u,
            const multi<float_type, 3>& pref) const
    {
        (void) pref;
        return surface_area_pdf_sample(u);
    }

    /**
     * @brief Evaluate.
     *
     * @param[in] b
     * Parameters in @f$ [0, 1)^3 @f$.
     */
    hit_type operator()(multi<float_type, 3> b) const
    {
        hit_type hit;
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
        hit.b = b;

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
    float_type intersect(const ray_type& ray,
                               hit_type* hit = nullptr) const
    {
        // Assemble intervals.
        multi<float_interval<float_type>, 3> o;
        multi<float_interval<float_type>, 3> d;
        for (int j = 0; j < 3; j++) {
            o[j] = {ray.o[j], ray.oerr[j]};
            d[j] = {ray.d[j], ray.derr[j]};
        }

        // Shear in XY.
        multi<float_interval<float_type>, 3> g[3];
        multi<float_interval<float_type>, 3> h[3];
        for (int j = 0; j < 3; j++) {
            g[j] = (p_[j] - o).swizzle(ray.k);
            h[j][0] = g[j][0] - ray.hr[0] * g[j][2];
            h[j][1] = g[j][1] - ray.hr[1] * g[j][2];
        }

        // Preliminary barycentric coordinates.
        float_interval<float_type> b[3];
        bool any_lt_zero = false;
        bool any_gt_zero = false;
        for (int j = 0; j < 3; j++) {
            int jp1 = (j + 1) % 3;
            int jp2 = (j + 2) % 3;
            b[j] =
                h[jp1][1] * h[jp2][0] -
                h[jp1][0] * h[jp2][1];

            // Track.
            any_lt_zero |= !(b[j].upper_bound() >= 0);
            any_gt_zero |= !(b[j].lower_bound() <= 0);
        }

        // Reject certain misses.
        if (any_lt_zero &&
            any_gt_zero) {
            // No intersection.
            return pr::numeric_limits<float_type>::quiet_NaN();
        }

        // Is parallel?
        float_interval<float_type> q = b[0] + b[1] + b[2];
        if (q.contains(0)) {
            // No intersection.
            return pr::numeric_limits<float_type>::quiet_NaN();
        }

        // Barycentric coordinates.
        if (q.abs_lower_bound() <
            pr::numeric_limits<float_type>::min_invertible()) {
            b[0] /= q;
            b[1] /= q;
            b[2] /= q;
        }
        else {
            float_interval<float_type> qinv =
            float_type(1) / q;
            b[0] *= qinv;
            b[1] *= qinv;
            b[2] *= qinv;
        }

        // Shear in Z.
        h[0][2] = ray.hr[2] * g[0][2];
        h[1][2] = ray.hr[2] * g[1][2];
        h[2][2] = ray.hr[2] * g[2][2];

        // Parametric value.
        float_interval<float_type> t =
            b[0] * h[0][2] +
            b[1] * h[1][2] +
            b[2] * h[2][2];

        // Reject uncertain hits.
        if (!(t.upper_bound() < ray.tmax &&
              t.lower_bound() > ray.tmin)) {
            // No intersection.
            return pr::numeric_limits<float_type>::quiet_NaN();
        }

        if (hit) {
            // Delegate.
            *hit =
            operator()({
                b[0].value(),
                b[1].value(),
                b[2].value()
            });
        }

        // Success.
        return t.value();
    }

private:

    /**
     * @brief Points @f$ \mathbf{p}_j @f$.
     */
    multi<float_type, 3> p_[3] = {
        {0, 0, 0},
        {1, 0, 0},
        {0, 1, 0}
    };
};

/**@}*/

} // namespace pr
