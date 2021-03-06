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
#error "preform/bspline.hpp requires >=C++17"
#endif // #if !(__cplusplus >= 201703L)
#endif // #if !DOXYGEN
#pragma once
#ifndef PREFORM_BSPLINE_HPP
#define PREFORM_BSPLINE_HPP

// for std::vector
#include <vector>

// for pre::fmin, pre::fmax, ...
#include <preform/math.hpp>

// for pre::multi
#include <preform/multi.hpp>

// for pre::multi wrappers
#include <preform/multi_math.hpp>

// for pre::dense_matrix_view
#include <preform/dense_matrix_view.hpp>

namespace pre {

/**
 * @defgroup bspline B-spline
 *
 * `<preform/bspline.hpp>`
 *
 * __C++ version__: >=C++17
 */
/**@{*/

/**
 * @brief B-spline basis functions.
 *
 * This helper-structure represents the B-spline basis functions for
 * a specific degree @f$ d @f$ and knot sequence @f$ \{ u_k \} @f$ where
 * @f$ k = 0, 1, \ldots, m - 1 @f$.
 *
 * Moreover, this structure is an interface for accessing the
 * knot vector, and is used internally by the `bspline_curve` and
 * `bspline_patch` classes to compute the B-spline basis functions
 * @f$ B_{i,j} @f$ efficiently. It is important to note that this
 * structure does _not_ manage any memory, and should not be directly
 * constructed by client code.
 *
 * @tparam Tfloat
 * Float type.
 */
template <typename Tfloat>
struct bspline_basis_functions
{
public:

    // Sanity check.
    static_assert(
        std::is_floating_point<Tfloat>::value,
        "Tfloat must be floating point");

    /**
     * @brief Size type.
     */
    typedef std::size_t size_type;

    /**
     * @brief Difference type.
     */
    typedef std::ptrdiff_t difference_type;

    /**
     * @brief Float type.
     */
    typedef Tfloat float_type;

public:

    /**
     * @brief Degree @f$ d @f$.
     */
    difference_type degree() const
    {
        return b_.size0() - 1;
    }

    /**
     * @brief Number of knots @f$ m @f$.
     */
    difference_type num_knots() const
    {
        return b_.size1();
    }

    /**
     * @brief Number of control points @f$ n @f$.
     */
    difference_type num_control_points() const
    {
        return b_.size1() - b_.size0();
    }

    /**
     * @brief Domain minimum @f$ u_d @f$.
     */
    float_type domain_min() const
    {
        return u_[degree()];
    }

    /**
     * @brief Domain maximum @f$ u_n @f$.
     */
    float_type domain_max() const
    {
        return u_[num_control_points()];
    }

    /**
     * @brief Set knots.
     *
     * @throw std::invalid_argument
     * Unless knot sequence is non-decreasing.
     */
    template <typename Tinput_itr>
    void set_knots(Tinput_itr srcitr)
    {
        // Number of knots.
        difference_type m = num_knots();

        if (m > 0) {
            float_type* dstitr = &u_[0];
            float_type* dstend = &u_[0] + m;

            // Set first knot.
            *dstitr = *srcitr;
            ++dstitr;
            ++srcitr;
            // Set remaining knots.
            while (dstitr < dstend) {
                *dstitr = *srcitr;
                ++dstitr;
                ++srcitr;

                // Ensure non-decreasing.
                if (!(*(dstitr - 1) >=
                      *(dstitr - 2))) {
                    throw std::invalid_argument(__PRETTY_FUNCTION__);
                }
            }
        }
    }

    /**
     * @brief Set knots open.
     *
     * Assuming 0-based indexing,
     * - set knots @f$ u_k = 0 @f$ for @f$ k \in [0, d) @f$,
     * - set knots @f$ u_k = 1 @f$ for @f$ k \in [n, m) @f$,
     * - set knots @f$ u_k = (k - d) / (n - d) @f$ for @f$ k \in [d, n) @f$.
     *
     * This forces the spline to touch the first and
     * last control points at @f$ t = 0 @f$ and @f$ t = 1 @f$
     * respectively. However, the spline is not differentiable at these
     * points.
     */
    void set_knots_open()
    {
        // Degree.
        difference_type d = degree();

        // Number of knots.
        difference_type m = num_knots();

        // Number of control points.
        difference_type n = num_control_points();

        for (difference_type k = 0; k < d; k++) {
            u_[k] = float_type(0);
        }
        for (difference_type k = d; k < n; k++) {
            u_[k] = float_type(k - d) / float_type(n - d);
        }
        for (difference_type k = n; k < m; k++) {
            u_[k] = float_type(1);
        }
    }

    /**
     * @brief Set knots closed.
     *
     * Assuming 0-based indexing, set knots
     * @f$ u_k = (k - d) / (n - d) @f$ for @f$ k \in [0, m) @f$. Then,
     * the first @f$ d @f$ knots and the last @f$ d @f$ knots are outside
     * the standard @f$ [0, 1] @f$ domain. That is,
     * - knots @f$ u_k < 0 @f$ for @f$ k \in [0, d) @f$,
     * - knots @f$ u_k > 1 @f$ for @f$ k \in (n, m) @f$.
     *
     * This forces the spline to be properly differentiable.
     * However, the spline never touches the first and last control
     * points. This is still useful for closed curves, because control
     * points may be repeated to ensure continuity at @f$ t = 0 @f$
     * and @f$ t = 1 @f$.
     *
     * @note
     * To form a closed curve, it is necessary to (cyclically) repeat
     * control points @f$ d @f$ times.
     */
    void set_knots_closed()
    {
        // Degree.
        difference_type d = degree();

        // Number of knots.
        difference_type m = num_knots();

        // Number of control points.
        difference_type n = num_control_points();

        for (difference_type k = 0; k < m; k++) {
            u_[k] = float_type(k - d) / float_type(n - d);
        }
    }

    /**
     * @brief Knots data.
     */
    float_type* knots_data()
    {
        return u_;
    }

    /**
     * @brief Knots data, const variant.
     */
    const float_type* knots_data() const
    {
        return u_;
    }

private:

    /**
     * @brief Update basis functions.
     *
     * For a given spline argument @f$ t @f$,
     * efficiently calculate @f$ B_{d,k} @f$ for @f$ k \in [0, m) @f$.
     *
     * @param[in] t
     * Spline argument.
     */
    difference_type update(float_type t) const
    {
        // Degree.
        difference_type d = degree();

        // Number of control points.
        difference_type n = num_control_points();

        // Clamp.
        t = pre::fmax(domain_min(),
            pre::fmin(domain_max(), t));

        // Compute i such that u[i] <= t < u[i + 1]
        difference_type i = d;
        while (i + 1 < n && !(t < u_[i + 1])) {
            i++;
        }

        // Update initial entry.
        b_[0][i] = 1;

        for (difference_type j = 1; j <= d; j++) {

            // Update vertical entries.
            b_[j][i] = b_[j - 1][i] *
                        ((t - u_[i]) / (u_[i + j] - u_[i]));

            // Update diagonal entries.
            b_[j][i - j] = b_[j - 1][i - j + 1] *
                        ((t - u_[i + 1]) / (u_[i - j + 1] - u_[i + 1]));
        }

        // Update interior entries.
        for (difference_type j = 2; j <= d; j++)
        for (difference_type k = i - j + 1; k < i; k++) {
            b_[j][k] =
                b_[j - 1][k + 0] * ((t - u_[k]) / (u_[k + j] - u_[k])) +
                b_[j - 1][k + 1] * ((t - u_[k + j + 1]) /
                            (u_[k + 1] - u_[k + j + 1]));
        }

        // Return relevant index.
        return i;
    }

private:

    /**
     * @brief Knots.
     *
     * Non-decreasing knot sequence @f$ \{ u_k \} @f$
     * where @f$ k \in [0, m) @f$.
     */
    float_type* u_ = {};

    /**
     * @brief Basis functions.
     *
     * Basis functions @f$ B_{j,k} @f$ where
     * @f$ j \in [0, d + 1) @f$ is the degree and
     * @f$ k \in [0, m) @f$ is the knot index.
     */
    mutable dense_matrix_view<float_type*> b_ = {};

private:

    // Declare friend.
    template <typename, std::size_t, typename>
    friend class bspline_curve;

    // Declare friend.
    template <typename, std::size_t, typename>
    friend class bspline_patch;

    // Declare friend.
    template <typename, std::size_t, typename>
    friend class nurbs_curve;

    // Declare friend.
    template <typename, std::size_t, typename>
    friend class nurbs_patch;
};

/**
 * @brief B-spline curve.
 *
 * @tparam Tfloat
 * Float type.
 *
 * @tparam N
 * Control point dimension.
 *
 * @tparam Talloc
 * Allocator type.
 */
template <
    typename Tfloat, std::size_t N,
    typename Talloc = std::allocator<char>
    >
class bspline_curve
{
public:

    // Sanity check.
    static_assert(
        std::is_floating_point<Tfloat>::value,
        "Tfloat must be floating point");

    /**
     * @brief Size type.
     */
    typedef std::size_t size_type;

    /**
     * @brief Difference type.
     */
    typedef std::ptrdiff_t difference_type;

    /**
     * @brief Float type.
     */
    typedef Tfloat float_type;

    /**
     * @brief Point type.
     */
    typedef std::conditional_t<N == 1, Tfloat, multi<Tfloat, N>> point_type;

    /**
     * @brief Float allocator type.
     */
    typedef typename std::allocator_traits<Talloc>::
            template rebind_alloc<float_type> float_allocator_type;

    /**
     * @brief Point allocator type.
     */
    typedef typename std::allocator_traits<Talloc>::
            template rebind_alloc<point_type> point_allocator_type;

public:

    /**
     * @brief Default constructor.
     */
    bspline_curve() = default;

    /**
     * @brief Constructor.
     *
     * @param[in] n
     * Number of control points.
     *
     * @param[in] d
     * Degree.
     *
     * @param[in] alloc
     * Allocator.
     *
     * @throw std::invalid_argument
     * Unless `n > 1` and `n > d`.
     */
    explicit
    bspline_curve(
            size_type n,
            size_type d,
            const Talloc& alloc = {})
    {
        // Validate.
        if (!(n > 1) ||
            !(n > d)) {
            throw std::invalid_argument(__PRETTY_FUNCTION__);
        }

        // Number of knots.
        size_type m = n + d + 1;

        // Initialize basis functions.
        new (&basis_vals_)
        std::vector<
            float_type,
            float_allocator_type>(
                m + m * (d + 1),
                float_allocator_type(alloc));
        basis_.u_ = &basis_vals_[0];
        basis_.b_ = {
            // offset
            &basis_vals_[m],
            // stride0
            difference_type(1),
            // stride1
            difference_type(d + 1),
            // size0
            difference_type(d + 1),
            // size1
            difference_type(m)
        };

        // Initialize control points.
        new (&p_)
        std::vector<
            point_type,
            point_allocator_type>(
                n,
                point_allocator_type(alloc));
    }

    /**
     * @brief Copy constructor.
     */
    bspline_curve(const bspline_curve& other) :
            basis_(other.basis_),
            basis_vals_(other.basis_vals_),
            p_(other.p_)
    {
        // Fix pointers.
        if (basis_vals_.size() > 0) {
            basis_.u_ = &basis_vals_[0];
            basis_.b_.offset_ = &basis_vals_[basis_.num_knots()];
        }
    }

    /**
     * @brief Move constructor.
     */
    bspline_curve(bspline_curve&& other) :
            basis_(std::move(other.basis_)),
            basis_vals_(std::move(other.basis_vals_)),
            p_(std::move(other.p_))
    {
        // Nullify.
        other.basis_ = {};
    }

    /**
     * @brief Copy assignment operator.
     */
    bspline_curve& operator=(const bspline_curve& other)
    {
        if (this != &other) {

            // Default copy.
            basis_ = other.basis_;
            basis_vals_ = other.basis_vals_;
            p_ = other.p_;

            // Fix pointers.
            if (basis_vals_.size() > 0) {
                basis_.u_ = &basis_vals_[0];
                basis_.b_.offset_ = &basis_vals_[basis_.num_knots()];
            }
        }

        // Done.
        return *this;
    }

    /**
     * @brief Move assignment operator.
     */
    bspline_curve& operator=(bspline_curve&& other)
    {
        // Default move.
        basis_ = std::move(other.basis_);
        basis_vals_ = std::move(other.basis_vals_);
        p_ = std::move(other.p_);

        // Nullify.
        other.basis_ = {};

        // Done.
        return *this;
    }

    /**
     * @brief Resize.
     *
     * @param[in] n
     * Number of control points.
     *
     * @param[in] d
     * Degree.
     *
     * @throw std::invalid_argument
     * Unless `n > 1` and `n > d`.
     */
    void resize(
            size_type n,
            size_type d)
    {
        // Validate.
        if (!(n > 1) ||
            !(n > d)) {
            throw std::invalid_argument(__PRETTY_FUNCTION__);
        }

        // Number of knots.
        size_type m = n + d + 1;

        // Resize basis functions.
        basis_vals_.resize(m + m * (d + 1));
        basis_.u_ = &basis_vals_[0];
        basis_.b_ = {
            // offset
            &basis_vals_[m],
            // stride0
            difference_type(1),
            // stride1
            difference_type(d + 1),
            // size0
            difference_type(d + 1),
            // size1
            difference_type(m)
        };

        // Resize control points.
        p_.resize(n);
    }

    /**
     * @brief Clear.
     */
    void clear()
    {
        // Clear basis functions.
        basis_ = {};

        // Clear basis functions values.
        basis_vals_.clear();

        // Clear control points.
        p_.clear();
    }

public:

    /**
     * @name Accessors
     */
    /**@{*/

    /**
     * @brief Basis functions.
     */
    bspline_basis_functions<float_type>& basis()
    {
        return basis_;
    }

    /**
     * @brief Basis functions, const variant.
     */
    const bspline_basis_functions<float_type>& basis() const
    {
        return basis_;
    }

    /**
     * @brief Set control points.
     */
    template <typename Tinput_itr>
    void set_control_points(Tinput_itr srcitr)
    {
        difference_type n = basis_.num_control_points();
        difference_type j = 0;
        while (j < n) {
            p_[j] = *srcitr++;
            j++;
        }
    }

    /**
     * @brief Control points data.
     */
    point_type* control_points_data()
    {
        return p_.data();
    }

    /**
     * @brief Control points data, const variant.
     */
    const point_type* control_points_data() const
    {
        return p_.data();
    }

    /**@}*/

public:

    /**
     * @brief Evaluate.
     *
     * @param[in] t
     * Spline argument.
     */
    point_type evaluate(float_type t) const
    {
        // Result.
        point_type res = point_type();

        // Evaluate.
        difference_type d = basis_.degree();
        difference_type i = basis_.update(t);
        difference_type j = i - d;
        dense_vector_view<float_type*> b = basis_.b_[d];
        while (j <= i) {
            res += b[j] * p_[j];
            j++;
        }
        return res;
    }

private:

    /**
     * @brief Basis functions.
     */
    bspline_basis_functions<float_type> basis_;

    /**
     * @brief Basis functions values.
     */
    std::vector<float_type, float_allocator_type> basis_vals_;

    /**
     * @brief Control points.
     */
    std::vector<point_type, point_allocator_type> p_;
};

/**
 * @brief B-spline patch.
 *
 * @tparam Tfloat
 * Float type.
 *
 * @tparam N
 * Control point dimension.
 *
 * @tparam Talloc
 * Allocator type.
 */
template <
    typename Tfloat, std::size_t N,
    typename Talloc = std::allocator<char>
    >
class bspline_patch
{
public:

    // Sanity check.
    static_assert(
        std::is_floating_point<Tfloat>::value,
        "Tfloat must be floating point");

    /**
     * @brief Size type.
     */
    typedef std::size_t size_type;

    /**
     * @brief Difference type.
     */
    typedef std::ptrdiff_t difference_type;

    /**
     * @brief Float type.
     */
    typedef Tfloat float_type;

    /**
     * @brief Point type.
     */
    typedef std::conditional_t<N == 1, Tfloat, multi<Tfloat, N>> point_type;

    /**
     * @brief Float allocator type.
     */
    typedef typename std::allocator_traits<Talloc>::
            template rebind_alloc<float_type> float_allocator_type;

    /**
     * @brief Point allocator type.
     */
    typedef typename std::allocator_traits<Talloc>::
            template rebind_alloc<point_type> point_allocator_type;

public:

    /**
     * @brief Default constructor.
     */
    bspline_patch() = default;

    /**
     * @brief Constructor.
     *
     * @param[in] n
     * Number of control points in each dimension.
     *
     * @param[in] d
     * Degree in each dimension.
     *
     * @param[in] alloc
     * Allocator.
     *
     * @throw std::invalid_argument
     * Unless `(n > 1).all()` and `(n > d).all()`.
     */
    explicit
    bspline_patch(
            multi<size_type, 2> n,
            multi<size_type, 2> d,
            const Talloc& alloc = {})
    {
        // Validate.
        if (!(n > size_type(1)).all() ||
            !(n > d).all()) {
            throw std::invalid_argument(__PRETTY_FUNCTION__);
        }

        // Number of knots.
        multi<size_type, 2> m = n + d + 1;

        // Initialize basis functions.
        new (&basis_vals_)
        std::vector<
            float_type,
            float_allocator_type>(
                m[0] + m[0] * (d[0] + 1) +
                m[1] + m[1] * (d[1] + 1),
                float_allocator_type(alloc));
        basis0_.u_ = &basis_vals_[0];
        basis0_.b_ = {
            // offset
            &basis_vals_[m[0]],
            // stride0
            difference_type(1),
            // stride1
            difference_type(d[0] + 1),
            // size0
            difference_type(d[0] + 1),
            // size1
            difference_type(m[0])
        };
        basis1_.u_ = &basis_vals_[m[0] + m[0] * (d[0] + 1)];
        basis1_.b_ = {
            // offset
            &basis_vals_[m[0] + m[0] * (d[0] + 1) + m[1]],
            // stride0
            difference_type(1),
            // stride1
            difference_type(d[1] + 1),
            // size0
            difference_type(d[1] + 1),
            // size1
            difference_type(m[1])
        };

        // Initialize control points.
        new (&p_)
        std::vector<
            point_type,
            point_allocator_type>(
                n.prod(),
                point_allocator_type(alloc));
    }

    /**
     * @brief Copy constructor.
     */
    bspline_patch(const bspline_patch& other) :
            basis0_(other.basis0_),
            basis1_(other.basis1_),
            basis_vals_(other.basis_vals_),
            p_(other.p_)
    {
        // Fix pointers.
        if (basis_vals_.size() > 0) {
            difference_type d0 = basis0_.degree();
            difference_type m0 = basis0_.num_knots();
            difference_type m1 = basis1_.num_knots();
            basis0_.u_ =
                &basis_vals_[0];
            basis0_.b_.offset_ =
                &basis_vals_[m0];
            basis1_.u_ =
                &basis_vals_[m0 + m0 * (d0 + 1)];
            basis1_.b_.offset_ =
                &basis_vals_[m0 + m0 * (d0 + 1) + m1];
        }
    }

    /**
     * @brief Move constructor.
     */
    bspline_patch(bspline_patch&& other) :
            basis0_(std::move(other.basis0_)),
            basis1_(std::move(other.basis1_)),
            basis_vals_(std::move(other.basis_vals_)),
            p_(std::move(other.p_))
    {
        // Nullify.
        other.basis0_ = {};
        other.basis1_ = {};
    }

    /**
     * @brief Copy assignment operator.
     */
    bspline_patch& operator=(const bspline_patch& other)
    {
        if (this != &other) {

            // Default copy.
            basis0_ = other.basis0_;
            basis1_ = other.basis1_;
            basis_vals_ = other.basis_vals_;
            p_ = other.p_;

            // Fix pointers.
            if (basis_vals_.size() > 0) {
                difference_type d0 = basis0_.degree();
                difference_type m0 = basis0_.num_knots();
                difference_type m1 = basis1_.num_knots();
                basis0_.u_ =
                    &basis_vals_[0];
                basis0_.b_.offset_ =
                    &basis_vals_[m0];
                basis1_.u_ =
                    &basis_vals_[m0 + m0 * (d0 + 1)];
                basis1_.b_.offset_ =
                    &basis_vals_[m0 + m0 * (d0 + 1) + m1];
            }
        }

        // Done.
        return *this;
    }

    /**
     * @brief Move assignment operator.
     */
    bspline_patch& operator=(bspline_patch&& other)
    {
        // Default move.
        basis0_ = std::move(other.basis0_);
        basis1_ = std::move(other.basis1_);
        basis_vals_ = std::move(other.basis_vals_);
        p_ = std::move(other.p_);

        // Nullify.
        other.basis0_ = {};
        other.basis1_ = {};

        // Done.
        return *this;
    }

    /**
     * @brief Resize.
     *
     * @param[in] n
     * Number of control points in each dimension.
     *
     * @param[in] d
     * Degree in each dimension.
     *
     * @throw std::invalid_argument
     * Unless `(n > 1).all()` and `(n > d).all()`.
     */
    void resize(
            multi<size_type, 2> n,
            multi<size_type, 2> d)
    {
        // Validate.
        if (!(n > size_type(1)).all() ||
            !(n > d).all()) {
            throw std::invalid_argument(__PRETTY_FUNCTION__);
        }

        // Number of knots.
        multi<size_type, 2> m = n + d + 1;

        // Resize basis functions.
        basis_vals_.resize(
                m[0] + m[0] * (d[0] + 1) +
                m[1] + m[1] * (d[1] + 1));
        basis0_.u_ = &basis_vals_[0];
        basis0_.b_ = {
            // offset
            &basis_vals_[m[0]],
            // stride0
            difference_type(1),
            // stride1
            difference_type(d[0] + 1),
            // size0
            difference_type(d[0] + 1),
            // size1
            difference_type(m[0])
        };
        basis1_.u_ = &basis_vals_[m[0] + m[0] * (d[0] + 1)];
        basis1_.b_ = {
            // offset
            &basis_vals_[m[0] + m[0] * (d[0] + 1) + m[1]],
            // stride0
            difference_type(1),
            // stride1
            difference_type(d[1] + 1),
            // size0
            difference_type(d[1] + 1),
            // size1
            difference_type(m[1])
        };

        // Resize control points.
        p_.resize(n.prod());
    }

    /**
     * @brief Clear.
     */
    void clear()
    {
        // Clear basis functions in dimension 0.
        basis0_ = {};

        // Clear basis functions in dimension 1.
        basis1_ = {};

        // Clear basis functions values.
        basis_vals_.clear();

        // Clear control points.
        p_.clear();
    }

public:

    /**
     * @name Accessors
     */
    /**@{*/

    /**
     * @brief Basis functions in dimension 0.
     */
    bspline_basis_functions<float_type>& basis0()
    {
        return basis0_;
    }

    /**
     * @brief Basis functions in dimension 1.
     */
    bspline_basis_functions<float_type>& basis1()
    {
        return basis1_;
    }

    /**
     * @brief Basis functions in dimension 0, const variant.
     */
    const bspline_basis_functions<float_type>& basis0() const
    {
        return basis0_;
    }

    /**
     * @brief Basis functions in dimension 1, const variant.
     */
    const bspline_basis_functions<float_type>& basis1() const
    {
        return basis1_;
    }

    /**
     * @brief Set control points.
     */
    template <typename Tinput_itr>
    void set_control_points(Tinput_itr srcitr)
    {
        multi<difference_type, 2> n = {
            basis0_.num_control_points(),
            basis1_.num_control_points()
        };
        for (difference_type j0 = 0; j0 < n[0]; j0++)
        for (difference_type j1 = 0; j1 < n[1]; j1++) {
            p_[j0 * n[1] + j1] = *srcitr++;
        }
    }

    /**
     * @brief Control points data.
     */
    point_type* control_points_data()
    {
        return p_.data();
    }

    /**
     * @brief Control points data, const variant.
     */
    const point_type* control_points_data() const
    {
        return p_.data();
    }

    /**@}*/

public:

    /**
     * @brief Evaluate.
     *
     * @param[in] t
     * Spline arguments.
     */
    point_type evaluate(multi<float_type, 2> t) const
    {
        // Result.
        point_type res = point_type();

        // Evaluate.
        difference_type d0 = basis0_.degree();
        difference_type d1 = basis1_.degree();
        difference_type i0 = basis0_.update(t[0]);
        difference_type i1 = basis1_.update(t[1]);
        dense_vector_view<const float_type*> b0 = basis0_.b_[d0];
        dense_vector_view<const float_type*> b1 = basis1_.b_[d1];
        difference_type n1 = basis1_.num_control_points();
        for (difference_type j0 = i0 - d0; j0 <= i0; j0++)
        for (difference_type j1 = i1 - d1; j1 <= i1; j1++) {
            res += (b0[j0] * b1[j1]) * p_[j0 * n1 + j1];
        }
        return res;
    }

private:

    /**
     * @brief Basis functions for dimension 0.
     */
    bspline_basis_functions<float_type> basis0_;

    /**
     * @brief Basis functions for dimension 1.
     */
    bspline_basis_functions<float_type> basis1_;

    /**
     * @brief Basis functions values.
     */
    std::vector<float_type, float_allocator_type> basis_vals_;

    /**
     * @brief Control points.
     */
    std::vector<point_type, point_allocator_type> p_;
};

/**
 * @brief NURBS curve.
 *
 * @tparam Tfloat
 * Float type.
 *
 * @tparam N
 * Control point dimension.
 *
 * @tparam Talloc
 * Allocator type.
 */
template <
    typename Tfloat, std::size_t N,
    typename Talloc = std::allocator<char>
    >
class nurbs_curve
{
public:

    // Sanity check.
    static_assert(
        std::is_floating_point<Tfloat>::value,
        "Tfloat must be floating point");

    /**
     * @brief Size type.
     */
    typedef std::size_t size_type;

    /**
     * @brief Difference type.
     */
    typedef std::ptrdiff_t difference_type;

    /**
     * @brief Float type.
     */
    typedef Tfloat float_type;

    /**
     * @brief Point type.
     */
    typedef std::conditional_t<N == 1, Tfloat, multi<Tfloat, N>> point_type;

    /**
     * @brief Float allocator type.
     */
    typedef typename std::allocator_traits<Talloc>::
            template rebind_alloc<float_type> float_allocator_type;

    /**
     * @brief Point allocator type.
     */
    typedef typename std::allocator_traits<Talloc>::
            template rebind_alloc<point_type> point_allocator_type;

public:

    /**
     * @brief Default constructor.
     */
    nurbs_curve() = default;

    /**
     * @brief Constructor.
     *
     * @param[in] n
     * Number of control points.
     *
     * @param[in] d
     * Degree.
     *
     * @param[in] alloc
     * Allocator.
     *
     * @throw std::invalid_argument
     * Unless `n > 1` and `n > d`.
     */
    explicit
    nurbs_curve(
            size_type n,
            size_type d,
            const Talloc& alloc = {})
    {
        // Validate.
        if (!(n > 1) ||
            !(n > d)) {
            throw std::invalid_argument(__PRETTY_FUNCTION__);
        }

        // Number of knots.
        size_type m = n + d + 1;

        // Initialize basis functions.
        new (&basis_vals_)
        std::vector<
            float_type,
            float_allocator_type>(
                m + m * (d + 1),
                float_allocator_type(alloc));
        basis_.u_ = &basis_vals_[0];
        basis_.b_ = {
            // offset
            &basis_vals_[m],
            // stride0
            difference_type(1),
            // stride1
            difference_type(d + 1),
            // size0
            difference_type(d + 1),
            // size1
            difference_type(m)
        };

        // Initialize weights.
        new (&w_)
        std::vector<
            float_type,
            float_allocator_type>(
                n,
                float_type(1),
                float_allocator_type(alloc));

        // Initialize control points.
        new (&p_)
        std::vector<
            point_type,
            point_allocator_type>(
                n,
                point_allocator_type(alloc));
    }

    /**
     * @brief Copy constructor.
     */
    nurbs_curve(const nurbs_curve& other) :
            basis_(other.basis_),
            basis_vals_(other.basis_vals_),
            w_(other.w_),
            p_(other.p_)
    {
        // Fix pointers.
        if (basis_vals_.size() > 0) {
            basis_.u_ = &basis_vals_[0];
            basis_.b_.offset_ = &basis_vals_[basis_.num_knots()];
        }
    }

    /**
     * @brief Move constructor.
     */
    nurbs_curve(nurbs_curve&& other) :
            basis_(std::move(other.basis_)),
            basis_vals_(std::move(other.basis_vals_)),
            w_(std::move(other.w_)),
            p_(std::move(other.p_))
    {
        // Nullify.
        other.basis_ = {};
    }

    /**
     * @brief Copy assignment operator.
     */
    nurbs_curve& operator=(const nurbs_curve& other)
    {
        if (this != &other) {

            // Default copy.
            basis_ = other.basis_;
            basis_vals_ = other.basis_vals_;
            w_ = other.w_;
            p_ = other.p_;

            // Fix pointers.
            if (basis_vals_.size() > 0) {
                basis_.u_ = &basis_vals_[0];
                basis_.b_.offset_ = &basis_vals_[basis_.num_knots()];
            }
        }

        // Done.
        return *this;
    }

    /**
     * @brief Move assignment operator.
     */
    nurbs_curve& operator=(nurbs_curve&& other)
    {
        // Default move.
        basis_ = std::move(other.basis_);
        basis_vals_ = std::move(other.basis_vals_);
        w_ = std::move(other.w_);
        p_ = std::move(other.p_);

        // Nullify.
        other.basis_ = {};

        // Done.
        return *this;
    }

    /**
     * @brief Resize.
     *
     * @param[in] n
     * Number of control points.
     *
     * @param[in] d
     * Degree.
     *
     * @throw std::invalid_argument
     * Unless `n > 1` and `n > d`.
     */
    void resize(
            size_type n,
            size_type d)
    {
        // Validate.
        if (!(n > 1) ||
            !(n > d)) {
            throw std::invalid_argument(__PRETTY_FUNCTION__);
        }

        // Number of knots.
        size_type m = n + d + 1;

        // Resize basis functions.
        basis_vals_.resize(m + m * (d + 1));
        basis_.u_ = &basis_vals_[0];
        basis_.b_ = {
            // offset
            &basis_vals_[m],
            // stride0
            difference_type(1),
            // stride1
            difference_type(d + 1),
            // size0
            difference_type(d + 1),
            // size1
            difference_type(m)
        };

        // Resize weights.
        w_.resize(n, float_type(1));

        // Resize control points.
        p_.resize(n);
    }

    /**
     * @brief Clear.
     */
    void clear()
    {
        // Clear basis functions.
        basis_ = {};

        // Clear basis functions values.
        basis_vals_.clear();

        // Clear weights.
        w_.clear();

        // Clear control points.
        p_.clear();
    }

public:

    /**
     * @name Accessors
     */
    /**@{*/

    /**
     * @brief Basis functions.
     */
    bspline_basis_functions<float_type>& basis()
    {
        return basis_;
    }

    /**
     * @brief Basis functions, const variant.
     */
    const bspline_basis_functions<float_type>& basis() const
    {
        return basis_;
    }

    /**
     * @brief Set weights.
     */
    template <typename Tinput_itr>
    void set_weights(Tinput_itr srcitr)
    {
        difference_type n = basis_.num_control_points();
        difference_type j = 0;
        while (j < n) {
            w_[j] = *srcitr++;
            j++;
        }
    }

    /**
     * @brief Weights data.
     */
    float_type* weights_data()
    {
        return w_.data();
    }

    /**
     * @brief Weights data, const variant.
     */
    const float_type* weights_data() const
    {
        return w_.data();
    }

    /**
     * @brief Set control points.
     */
    template <typename Tinput_itr>
    void set_control_points(Tinput_itr srcitr)
    {
        difference_type n = basis_.num_control_points();
        difference_type j = 0;
        while (j < n) {
            p_[j] = *srcitr++;
            j++;
        }
    }

    /**
     * @brief Control points data.
     */
    point_type* control_points_data()
    {
        return p_.data();
    }

    /**
     * @brief Control points data, const variant.
     */
    const point_type* control_points_data() const
    {
        return p_.data();
    }

    /**@}*/

public:

    /**
     * @brief Evaluate.
     *
     * @param[in] t
     * Spline argument.
     */
    point_type evaluate(float_type t) const
    {
        // Result.
        point_type res = point_type();

        // Normalization factor.
        float_type fac = float_type();

        // Evaluate.
        difference_type d = basis_.degree();
        difference_type i = basis_.update(t);
        difference_type j = i - d;
        dense_vector_view<float_type*> b = basis_.b_[d];
        while (j <= i) {
            res += (b[j] * w_[j]) * p_[j];
            fac += (b[j] * w_[j]);
            j++;
        }

        // Normalize.
        res /= fac;
        return res;
    }

private:

    /**
     * @brief Basis functions.
     */
    bspline_basis_functions<float_type> basis_;

    /**
     * @brief Basis functions values.
     */
    std::vector<float_type, float_allocator_type> basis_vals_;

    /**
     * @brief Weights.
     */
    std::vector<float_type, float_allocator_type> w_;

    /**
     * @brief Control points.
     */
    std::vector<point_type, point_allocator_type> p_;

};

/**
 * @brief NURBS patch.
 *
 * @tparam Tfloat
 * Float type.
 *
 * @tparam N
 * Control point dimension.
 *
 * @tparam Talloc
 * Allocator type.
 */
template <
    typename Tfloat, std::size_t N,
    typename Talloc = std::allocator<char>
    >
class nurbs_patch
{
public:

    // Sanity check.
    static_assert(
        std::is_floating_point<Tfloat>::value,
        "Tfloat must be floating point");

    /**
     * @brief Size type.
     */
    typedef std::size_t size_type;

    /**
     * @brief Difference type.
     */
    typedef std::ptrdiff_t difference_type;

    /**
     * @brief Float type.
     */
    typedef Tfloat float_type;

    /**
     * @brief Point type.
     */
    typedef std::conditional_t<N == 1, Tfloat, multi<Tfloat, N>> point_type;

    /**
     * @brief Float allocator type.
     */
    typedef typename std::allocator_traits<Talloc>::
            template rebind_alloc<float_type> float_allocator_type;

    /**
     * @brief Point allocator type.
     */
    typedef typename std::allocator_traits<Talloc>::
            template rebind_alloc<point_type> point_allocator_type;

public:

    /**
     * @brief Default constructor.
     */
    nurbs_patch() = default;

    /**
     * @brief Constructor.
     *
     * @param[in] n
     * Number of control points in each dimension.
     *
     * @param[in] d
     * Degree in each dimension.
     *
     * @param[in] alloc
     * Allocator.
     *
     * @throw std::invalid_argument
     * Unless `(n > 1).all()` and `(n > d).all()`.
     */
    explicit
    nurbs_patch(
            multi<size_type, 2> n,
            multi<size_type, 2> d,
            const Talloc& alloc = {})
    {
        // Validate.
        if (!(n > size_type(1)).all() ||
            !(n > d).all()) {
            throw std::invalid_argument(__PRETTY_FUNCTION__);
        }

        // Number of knots.
        multi<size_type, 2> m = n + d + 1;

        // Initialize basis functions.
        new (&basis_vals_)
        std::vector<
            float_type,
            float_allocator_type>(
                m[0] + m[0] * (d[0] + 1) +
                m[1] + m[1] * (d[1] + 1),
                float_allocator_type(alloc));
        basis0_.u_ = &basis_vals_[0];
        basis0_.b_ = {
            // offset
            &basis_vals_[m[0]],
            // stride0
            difference_type(1),
            // stride1
            difference_type(d[0] + 1),
            // size0
            difference_type(d[0] + 1),
            // size1
            difference_type(m[0])
        };
        basis1_.u_ = &basis_vals_[m[0] + m[0] * (d[0] + 1)];
        basis1_.b_ = {
            // offset
            &basis_vals_[m[0] + m[0] * (d[0] + 1) + m[1]],
            // stride0
            difference_type(1),
            // stride1
            difference_type(d[1] + 1),
            // size0
            difference_type(d[1] + 1),
            // size1
            difference_type(m[1])
        };

        // Initialize weights.
        new (&w_)
        std::vector<
            float_type,
            float_allocator_type>(
                n.prod(),
                float_type(1),
                float_allocator_type(alloc));

        // Initialize control points.
        new (&p_)
        std::vector<
            point_type,
            point_allocator_type>(
                n.prod(),
                point_allocator_type(alloc));
    }

    /**
     * @brief Copy constructor.
     */
    nurbs_patch(const nurbs_patch& other) :
            basis0_(other.basis0_),
            basis1_(other.basis1_),
            basis_vals_(other.basis_vals_),
            w_(other.w_),
            p_(other.p_)
    {
        // Fix pointers.
        if (basis_vals_.size() > 0) {
            difference_type d0 = basis0_.degree();
            difference_type m0 = basis0_.num_knots();
            difference_type m1 = basis1_.num_knots();
            basis0_.u_ =
                &basis_vals_[0];
            basis0_.b_.offset_ =
                &basis_vals_[m0];
            basis1_.u_ =
                &basis_vals_[m0 + m0 * (d0 + 1)];
            basis1_.b_.offset_ =
                &basis_vals_[m0 + m0 * (d0 + 1) + m1];
        }
    }

    /**
     * @brief Move constructor.
     */
    nurbs_patch(nurbs_patch&& other) :
            basis0_(std::move(other.basis0_)),
            basis1_(std::move(other.basis1_)),
            basis_vals_(std::move(other.basis_vals_)),
            w_(std::move(other.w_)),
            p_(std::move(other.p_))
    {
        // Nullify.
        other.basis0_ = {};
        other.basis1_ = {};
    }

    /**
     * @brief Copy assignment operator.
     */
    nurbs_patch& operator=(const nurbs_patch& other)
    {
        if (this != &other) {

            // Default copy.
            basis0_ = other.basis0_;
            basis1_ = other.basis1_;
            basis_vals_ = other.basis_vals_;
            w_ = other.w_;
            p_ = other.p_;

            // Fix pointers.
            if (basis_vals_.size() > 0) {
                difference_type d0 = basis0_.degree();
                difference_type m0 = basis0_.num_knots();
                difference_type m1 = basis1_.num_knots();
                basis0_.u_ =
                    &basis_vals_[0];
                basis0_.b_.offset_ =
                    &basis_vals_[m0];
                basis1_.u_ =
                    &basis_vals_[m0 + m0 * (d0 + 1)];
                basis1_.b_.offset_ =
                    &basis_vals_[m0 + m0 * (d0 + 1) + m1];
            }
        }

        // Done.
        return *this;
    }

    /**
     * @brief Move assignment operator.
     */
    nurbs_patch& operator=(nurbs_patch&& other)
    {
        // Default move.
        basis0_ = std::move(other.basis0_);
        basis1_ = std::move(other.basis1_);
        basis_vals_ = std::move(other.basis_vals_);
        w_ = std::move(other.w_);
        p_ = std::move(other.p_);

        // Nullify.
        other.basis0_ = {};
        other.basis1_ = {};

        // Done.
        return *this;
    }

    /**
     * @brief Resize.
     *
     * @param[in] n
     * Number of control points in each dimension.
     *
     * @param[in] d
     * Degree in each dimension.
     *
     * @throw std::invalid_argument
     * Unless `(n > 1).all()` and `(n > d).all()`.
     */
    void resize(
            multi<size_type, 2> n,
            multi<size_type, 2> d)
    {
        // Validate.
        if (!(n > size_type(1)).all() ||
            !(n > d).all()) {
            throw std::invalid_argument(__PRETTY_FUNCTION__);
        }

        // Number of knots.
        multi<size_type, 2> m = n + d + 1;

        // Resize basis functions.
        basis_vals_.resize(
                m[0] + m[0] * (d[0] + 1) +
                m[1] + m[1] * (d[1] + 1));
        basis0_.u_ = &basis_vals_[0];
        basis0_.b_ = {
            // offset
            &basis_vals_[m[0]],
            // stride0
            difference_type(1),
            // stride1
            difference_type(d[0] + 1),
            // size0
            difference_type(d[0] + 1),
            // size1
            difference_type(m[0])
        };
        basis1_.u_ = &basis_vals_[m[0] + m[0] * (d[0] + 1)];
        basis1_.b_ = {
            // offset
            &basis_vals_[m[0] + m[0] * (d[0] + 1) + m[1]],
            // stride0
            difference_type(1),
            // stride1
            difference_type(d[1] + 1),
            // size0
            difference_type(d[1] + 1),
            // size1
            difference_type(m[1])
        };

        // Resize weights.
        w_.resize(n.prod(), float_type(1));

        // Resize control points.
        p_.resize(n.prod());
    }

    /**
     * @brief Clear.
     */
    void clear()
    {
        // Clear basis functions in dimension 0.
        basis0_ = {};

        // Clear basis functions in dimension 1.
        basis1_ = {};

        // Clear basis functions values.
        basis_vals_.clear();

        // Clear weights.
        w_.clear();

        // Clear control points.
        p_.clear();
    }

public:

    /**
     * @name Accessors
     */
    /**@{*/

    /**
     * @brief Basis functions in dimension 0.
     */
    bspline_basis_functions<float_type>& basis0()
    {
        return basis0_;
    }

    /**
     * @brief Basis functions in dimension 1.
     */
    bspline_basis_functions<float_type>& basis1()
    {
        return basis1_;
    }

    /**
     * @brief Basis functions in dimension 0, const variant.
     */
    const bspline_basis_functions<float_type>& basis0() const
    {
        return basis0_;
    }

    /**
     * @brief Basis functions in dimension 1, const variant.
     */
    const bspline_basis_functions<float_type>& basis1() const
    {
        return basis1_;
    }

    /**
     * @brief Set weights.
     */
    template <typename Tinput_itr>
    void set_weights(Tinput_itr srcitr)
    {
        multi<difference_type, 2> n = {
            basis0_.num_control_points(),
            basis1_.num_control_points()
        };
        for (difference_type j0 = 0; j0 < n[0]; j0++)
        for (difference_type j1 = 0; j1 < n[1]; j1++) {
            w_[j0 * n[1] + j1] = *srcitr++;
        }
    }

    /**
     * @brief Weights data.
     */
    float_type* weights_data()
    {
        return w_.data();
    }

    /**
     * @brief Weights data, const variant.
     */
    const float_type* weights_data() const
    {
        return w_.data();
    }

    /**
     * @brief Set control points.
     */
    template <typename Tinput_itr>
    void set_control_points(Tinput_itr srcitr)
    {
        multi<difference_type, 2> n = {
            basis0_.num_control_points(),
            basis1_.num_control_points()
        };
        for (difference_type j0 = 0; j0 < n[0]; j0++)
        for (difference_type j1 = 0; j1 < n[1]; j1++) {
            p_[j0 * n[1] + j1] = *srcitr++;
        }
    }

    /**
     * @brief Control points data.
     */
    point_type* control_points_data()
    {
        return p_.data();
    }

    /**
     * @brief Control points data, const variant.
     */
    const point_type* control_points_data() const
    {
        return p_.data();
    }

    /**@}*/

public:

    /**
     * @brief Evaluate.
     *
     * @param[in] t
     * Spline arguments.
     */
    point_type evaluate(multi<float_type, 2> t) const
    {
        // Result.
        point_type res = point_type();

        // Normalization factor.
        float_type fac = float_type();

        // Evaluate.
        difference_type d0 = basis0_.degree();
        difference_type d1 = basis1_.degree();
        difference_type i0 = basis0_.update(t[0]);
        difference_type i1 = basis1_.update(t[1]);
        dense_vector_view<const float_type*> b0 = basis0_.b_[d0];
        dense_vector_view<const float_type*> b1 = basis1_.b_[d1];
        difference_type n1 = basis1_.num_control_points();
        for (difference_type j0 = i0 - d0; j0 <= i0; j0++)
        for (difference_type j1 = i1 - d1; j1 <= i1; j1++) {
            res += (b0[j0] * b1[j1] * w_[j0 * n1 + j1]) * p_[j0 * n1 + j1];
            fac += (b0[j0] * b1[j1] * w_[j0 * n1 + j1]);
        }

        // Normalize.
        res /= fac;
        return res;
    }

private:

    /**
     * @brief Basis functions for dimension 0.
     */
    bspline_basis_functions<float_type> basis0_;

    /**
     * @brief Basis functions for dimension 1.
     */
    bspline_basis_functions<float_type> basis1_;

    /**
     * @brief Basis functions values.
     */
    std::vector<float_type, float_allocator_type> basis_vals_;

    /**
     * @brief Weights.
     */
    std::vector<float_type, float_allocator_type> w_;

    /**
     * @brief Control points.
     */
    std::vector<point_type, point_allocator_type> p_;
};

/**@}*/

} // namespace pre

#endif // #ifndef PREFORM_BSPLINE_HPP
