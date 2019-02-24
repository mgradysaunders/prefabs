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
#if !(__cplusplus >= 201402L)
#error "preform/blas.hpp requires >=C++14"
#endif // #if !(__cplusplus >= 201402L)
#endif // #if !DOXYGEN
#pragma once
#ifndef PREFORM_BLAS_HPP
#define PREFORM_BLAS_HPP

// for std::vector
#include <vector>

// for pr::sqrt, pr::fmin, pr::fmax, ...
#include <preform/math.hpp>

// for pr::dense_vector_view
#include <preform/dense_vector_view.hpp>

// for pr::dense_matrix_view
#include <preform/dense_matrix_view.hpp>

namespace pr {

/**
 * @defgroup blas Basic linear algebra subroutines
 *
 * `<preform/blas.hpp>`
 *
 * __C++ version__: >=C++14
 */
/**@{*/

template <typename T>
struct default_blas_traits
{
    typedef T value_type;

    typedef T float_type;

    __attribute__((always_inline))
    static float_type abs(const value_type& x)
    {
        return pr::fabs(x);
    }

    __attribute__((always_inline))
    static float_type norm(const value_type& x)
    {
        return x * x;
    }

    __attribute__((always_inline))
    static value_type conj(const value_type& x)
    {
        return x;
    }

    __attribute__((always_inline))
    static value_type sign(const value_type& x)
    {
        return pr::copysign(value_type(1), x);
    }
};

template <typename T>
struct default_blas_traits<std::complex<T>>
{
    typedef std::complex<T> value_type;

    typedef T float_type;

    __attribute__((always_inline))
    static float_type abs(const value_type& x)
    {
        return pr::abs(x);
    }

    __attribute__((always_inline))
    static float_type norm(const value_type& x)
    {
        return pr::norm(x);
    }

    __attribute__((always_inline))
    static value_type conj(const value_type& x)
    {
        return pr::conj(x);
    }

    static value_type sign(const value_type& x)
    {
        return pr::sign(x);
    }
};

template <
    typename T, 
    typename Ttraits = default_blas_traits<T>
    >
struct blas
{
public:

    /**
     * @brief Value type.
     */
    typedef typename Ttraits::value_type value_type;

    /**
     * @brief Float type.
     */
    typedef typename Ttraits::float_type float_type;

    // Sanity check.
    static_assert(
        std::is_floating_point<float_type>::value,
        "float_type must be floating point");

    /**
     * @brief Wrap `Ttraits::abs()`.
     */
    __attribute__((always_inline))
    static float_type abs(const value_type& x)
    {
        return Ttraits::abs(x);
    }

    /**
     * @brief Wrap `Ttraits::norm()`.
     */
    __attribute__((always_inline))
    static float_type norm(const value_type& x)
    {
        return Ttraits::norm(x);
    }

    /**
     * @brief Wrap `Ttraits::conj()`.
     */
    __attribute__((always_inline))
    static value_type conj(const value_type& x)
    {
        return Ttraits::conj(x);
    }

    /**
     * @brief Wrap `Ttraits::sign()`.
     */
    __attribute__((always_inline))
    static value_type sign(const value_type& x)
    {
        return Ttraits::sign(x);
    }

public:

    /**
     * @name Basic operations
     */
    /**@{*/

    /**
     * @brief Dot product.
     *
     * @f[
     *      \mathbf{x} \cdot
     *      \mathbf{y} = \sum_{k=0}^{n-1} x_{[k]} y_{[k]}
     * @f]
     *
     * @param[in] x
     * Vector @f$ \mathbf{x} @f$.
     *
     * @param[in] y
     * Vector @f$ \mathbf{y} @f$.
     *
     * @throw std::invalid_argument
     * Unless `x.size() == y.size()`.
     */
    static value_type dot(
                dense_vector_view<const value_type*> x,
                dense_vector_view<const value_type*> y)
    {
        if (x.size() != y.size()) {
            throw std::invalid_argument(__PRETTY_FUNCTION__);
        }

        // Default construct.
        value_type res = {};

        // Dot.
        auto itrx = x.begin();
        auto itry = y.begin();
        for (; itry < y.end(); ++itrx, ++itry) {
            res = (*itrx) * (*itry) + res;
        }

        return res;
    }

    /**
     * @brief Dot product, conjugating left argument.
     *
     * @f[
     *      \bar{\mathbf{x}} \cdot
     *           \mathbf{y} = \sum_{k=0}^{n-1} \bar{x}_{[k]} y_{[k]}
     * @f]
     *
     * @param[in] x
     * Vector @f$ \mathbf{x} @f$.
     *
     * @param[in] y
     * Vector @f$ \mathbf{y} @f$.
     * 
     * @throw std::invalid_argument
     * Unless `x.size() == y.size()`.
     */
    static value_type dot_conj(
                dense_vector_view<const value_type*> x,
                dense_vector_view<const value_type*> y)
    {
        if (x.size() != y.size()) {
            throw std::invalid_argument(__PRETTY_FUNCTION__);
        }

        // Default construct.
        value_type res = {};

        // Dot.
        auto itrx = x.begin();
        auto itry = y.begin();
        for (; itry < y.end(); ++itrx, ++itry) {
            res = conj(*itrx) * (*itry) + res;
        }

        return res;
    }

    /**
     * @brief Reflect.
     *
     * @f[
     *      \mathbf{y} \gets
     *      \mathbf{y} - 2 (\mathbf{x} \cdot \mathbf{y}) \mathbf{x}
     * @f]
     *
     * @param[in] x
     * Vector @f$ \mathbf{x} @f$.
     *
     * @param[inout] y
     * Vector @f$ \mathbf{y} @f$.
     *
     * @throw std::invalid_argument
     * Unless `x.size() == y.size()`.
     */
    static void reflect(
                dense_vector_view<const value_type*> x,
                dense_vector_view<value_type*> y)
    {
        if (x.size() != y.size()) {
            throw std::invalid_argument(__PRETTY_FUNCTION__);
        }

        // Default construct.
        value_type fac = {};

        // Dot.
        {
            auto itrx = x.begin();
            auto itry = y.begin();
            for (; itry < y.end(); ++itrx, ++itry) {
                fac = (*itrx) * (*itry) + fac;
            }
        }

        // Absorb multiplier.
        fac *= float_type(-2);

        // Update.
        {
            auto itrx = x.begin();
            auto itry = y.begin();
            for (; itry < y.end(); ++itrx, ++itry) {
                *itry = (*itrx) * fac + (*itry);
            }
        }
    }

    /**
     * @brief Reflect, conjugating left argument.
     *
     * @f[
     *      \mathbf{y} \gets
     *      \mathbf{y} - 2 (\bar{\mathbf{x}} \cdot \mathbf{y}) \mathbf{x}
     * @f]
     *
     * @param[in] x
     * Vector @f$ \mathbf{x} @f$.
     *
     * @param[inout] y
     * Vector @f$ \mathbf{y} @f$.
     *
     * @throw std::invalid_argument
     * Unless `x.size() == y.size()`.
     */
    static void reflect_conj(
                dense_vector_view<const value_type*> x,
                dense_vector_view<value_type*> y)
    {
        if (x.size() != y.size()) {
            throw std::invalid_argument(__PRETTY_FUNCTION__);
        }

        // Default construct.
        value_type fac = {};

        // Dot.
        {
            auto itrx = x.begin();
            auto itry = y.begin();
            for (; itry < y.end(); ++itrx, ++itry) {
                fac = conj(*itrx) * (*itry) + fac;
            }
        }

        // Absorb multiplier.
        fac *= float_type(-2);

        // Update.
        {
            auto itrx = x.begin();
            auto itry = y.begin();
            for (; itry < y.end(); ++itrx, ++itry) {
                *itry = (*itrx) * fac + (*itry);
            }
        }
    }

    /**
     * @brief @f$ L^2 @f$ length.
     *
     * @param[in] x
     * Vector @f$ \mathbf{x} @f$.
     */
    static float_type length(dense_vector_view<const value_type*> x)
    {
        switch (x.size()) {
            case 1: 
                // Delegate.
                return float_type(abs(x[0]));
            case 2: 
                // Delegate.
                return float_type(pr::hypot(abs(x[0]), abs(x[1])));
            default:
                break;
        }

        // Temporary vector.
        static thread_local std::vector<float_type> tmpvec;
        if (tmpvec.size() < std::size_t(x.size())) {
            tmpvec.resize(x.size());
        }

        // Empty?
        if (x.empty()) {
            // Use as signal to free memory.
            tmpvec.clear();
            tmpvec.shrink_to_fit();
            return 0;
        }

        // Temporary values vector view.
        dense_vector_view<float_type*> tmp = {tmpvec.data(), 1, x.size()};

        // Temporary extrema.
        float_type tmpmin = pr::numeric_limits<float_type>::max();
        float_type tmpmax = 0;
        {
            // Initialize.
            auto itrtmp = tmp.begin();
            auto itrx = x.begin();
            for (; itrx < x.end(); ++itrtmp, ++itrx) {
                *itrtmp = abs(*itrx);
                if (*itrtmp != 0) {
                    tmpmin = pr::fmin(tmpmin, *itrtmp);
                    tmpmax = pr::fmax(tmpmax, *itrtmp);
                }
            }
        }

        // Impending overflow or underflow?
        if (tmpmax * 
            tmpmax >= pr::numeric_limits<float_type>::max() / x.size() ||
            tmpmin <= pr::numeric_limits<float_type>::min_squarable()) {
            // Factor out maximum.
            if (tmpmax >= pr::numeric_limits<float_type>::min_invertible()) {
                tmp *= 1 / tmpmax;
            }
            else {
                tmp /= tmpmax; // Inverse overflows.
            }

            // Length.
            float_type tmpsum = 0;
            for (auto itrtmp = tmp.begin();
                      itrtmp < tmp.end(); ++itrtmp) {
                tmpsum = (*itrtmp) * (*itrtmp) + tmpsum;
            }
            return pr::sqrt(tmpsum) * tmpmax;
        }
        else {

            // Length.
            float_type tmpsum = 0;
            for (auto itrtmp = tmp.begin();
                      itrtmp < tmp.end(); ++itrtmp) {
                tmpsum = (*itrtmp) * (*itrtmp) + tmpsum;
            }
            return pr::sqrt(tmpsum);
        }
    }

    /**
     * @brief @f$ L^2 @f$ normalize.
     *
     * @param[inout] x
     * Vector @f$ \mathbf{x} @f$.
     */
    static void normalize(dense_vector_view<value_type*> x)
    {
        float_type tmplen = length(x);
        if (tmplen >= pr::numeric_limits<float_type>::min_invertible()) {
            x *= 1 / tmplen;
        }
        else if (tmplen != 0) {
            x /= tmplen;   
        }
    }

    /**@}*/

public:

#if 0
    /**
     * @brief
     */
    static void householderl(
                int p,
                int q,
                dense_matrix_view<value_type*> x,
                dense_matrix_view<value_type*> y = {})
    {
        // Temporary vector.
        static thread_local std::vector<value_type> tmpvec;
        if (tmpvec.size() < std::size_t(x.size0())) {
            tmpvec.resize(x.size0());
        }

        // Empty?
        if (x.empty()) {
            // Use as signal to free memory.
            tmpvec.clear();
            tmpvec.shrink_to_fit();
            return;
        }

        // Quit if out of range.
        if (p < 0 || p >= x.size0() ||
            q < 0 || q >= x.size1()) {
            return;
        }

        // Temporary vector view.
        dense_vector_view<value_type*> w = {tmpvec.data(), 1, x.size0() - p};

        // Copy qth column.
        std::copy(
            x.transpose()[q].range(p, x.size0()).begin(),
            x.transpose()[q].range(p, x.size0()).end(),
            w.begin());

        // Compute reflector.
        value_type alpha = sign(w[0]) * length(w);
        w[0] += alpha;

        // Normalize.
        normalize(w);

        // Reflect qth column.
        x[p][q] = -alpha;
        std::fill(
            x.transpose()[q].range(p + 1, x.size0()).begin(),
            x.transpose()[q].range(p + 1, x.size0()).end(),
            value_type());

        // Reflect remaining columns. For j < q, assume no effect.
        for (int j = q + 1; j < x.size1(); j++) {
            reflect_conj(w, x.transpose()[j].range(p, x.size0()));
        }

        // Accumulate.
        if (!y.empty()) {

            // Sanity check.
            if (x.size0() != y.size0()) {
                throw std::invalid_argument(__PRETTY_FUNCTION__);
            }

            // Reflect columns.
            for (int j = 0; j < y.size1(); j++) {
                reflect_conj(w, y.transpose()[j].range(p, x.size0()));
            }
        }
    }

    /**
     * @brief
     */
    static void householderr(
                int p,
                int q,
                dense_matrix_view<value_type*> x,
                dense_matrix_view<value_type*> y = {})
    {
        // Delegate.
        householderl(
                q, p, 
                x.transpose(),
                y.transpose());
    }
#endif

};

/**@}*/

} // namespace pr

#endif // #ifndef PREFORM_BLAS_HPP
