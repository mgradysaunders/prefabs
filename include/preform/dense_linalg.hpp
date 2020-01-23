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
#if !(__cplusplus >= 201402L)
#error "preform/dense_linalg.hpp requires >=C++14"
#endif // #if !(__cplusplus >= 201402L)
#endif // #if !DOXYGEN
#pragma once
#ifndef PREFORM_DENSE_LINALG_HPP
#define PREFORM_DENSE_LINALG_HPP

#include <algorithm>
#include <numeric>
#include <vector>
#include <preform/math.hpp>
#include <preform/dense_vector_view.hpp>
#include <preform/dense_matrix_view.hpp>

namespace pr {

/**
 * @defgroup dense_linalg Dense linear algebra
 *
 * `<preform/dense_linalg.hpp>`
 *
 * __C++ version__: >=C++14
 */
/**@{*/

/**
 * @brief Dense linear algebra.
 *
 * @tparam Tvalue
 * Value type, must be either floating point or complex.
 */
template <typename Tvalue>
struct dense_linalg
{
public:

    // Sanity check.
    static_assert(
        std::is_floating_point<Tvalue>::value || is_complex<Tvalue>::value,
        "Tvalue must be floating point or complex");

    /**
     * @brief Value type.
     */
    typedef Tvalue value_type;

    /**
     * @brief Float type.
     */
    typedef decltype(pr::real(Tvalue())) float_type;

    /**
     * @brief Dot product.
     *
     * @param[in] x
     * Vector @f$ \mathbf{x} @f$.
     *
     * @param[in] y
     * Vector @f$ \mathbf{y} @f$.
     *
     * @par Expression
     * @f[
     *      \mathbf{x}^\top
     *      \mathbf{y} = \sum_{k=0}^{n-1} x_{[k]} y_{[k]}
     * @f]
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
     * @param[in] x
     * Vector @f$ \mathbf{x} @f$.
     *
     * @param[in] y
     * Vector @f$ \mathbf{y} @f$.
     *
     * @par Expression
     * @f[
     *      \mathbf{x}^\dagger
     *      \mathbf{y} = \sum_{k=0}^{n-1} \bar{x}_{[k]} y_{[k]}
     * @f]
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
            res = pr::conj(*itrx) * (*itry) + res;
        }

        return res;
    }

    /**
     * @brief @f$ L^2 @f$ length.
     *
     * @param[in] x
     * Vector @f$ \mathbf{x} @f$.
     *
     * @par Expression
     * @f[
     *      \sqrt{\sum_{j=0}^{n-1} \bar{x}_{[j]} x_{[j]}}
     * @f]
     *
     * @note
     * This method maintains a static thread-local buffer of
     * temporary values. If `x` is empty, the implementation
     * resets the buffer and returns immediately.
     */
    static float_type length(dense_vector_view<const value_type*> x)
    {
        if (x.size() == 1) {
            // Delegate.
            return pr::abs(x[0]);
        }

        // Temporary buffer.
        static thread_local std::vector<float_type> tmpbuf;
        if (tmpbuf.size() < std::size_t(x.size())) {
            tmpbuf.resize(x.size());
        }

        // Empty?
        if (x.empty()) {
            // Use as signal to free buffer.
            tmpbuf.clear();
            tmpbuf.shrink_to_fit();
            return 0;
        }

        // Temporary values vector view.
        dense_vector_view<float_type*> tmp = {
            &tmpbuf[0],
            &tmpbuf[0] + x.size()
        };

        // Temporary extrema.
        float_type tmpmin = pr::numeric_limits<float_type>::infinity();
        float_type tmpmax = 0;
        {
            // Initialize.
            auto itrtmp = tmp.begin();
            auto itrx = x.begin();
            for (; itrx < x.end(); ++itrtmp, ++itrx) {
                *itrtmp = pr::abs(*itrx);
                if (*itrtmp != 0) {
                    if (tmpmin > *itrtmp) {
                        tmpmin = *itrtmp;
                    }
                    if (tmpmax < *itrtmp) {
                        tmpmax = *itrtmp;
                    }
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
                tmpsum = pr::fma(*itrtmp, *itrtmp, tmpsum);
            }
            return pr::sqrt(tmpsum) * tmpmax;
        }
        else {

            // Length.
            float_type tmpsum = 0;
            for (auto itrtmp = tmp.begin();
                      itrtmp < tmp.end(); ++itrtmp) {
                tmpsum = pr::fma(*itrtmp, *itrtmp, tmpsum);
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
        float_type len = length(x);
        if (len >= pr::numeric_limits<float_type>::min_invertible()) {
            x *= float_type(1) / len;
        }
        else if (len != 0) {
            x /= len;
        }
    }

    /**
     * @brief Reflect.
     *
     * @param[in] x
     * Vector @f$ \mathbf{x} @f$.
     *
     * @param[inout] y
     * Vector @f$ \mathbf{y} @f$.
     *
     * @par Expression
     * @f[
     *      \mathbf{y} \gets
     *      \mathbf{y} - 2 \mathbf{x} (\mathbf{x}^\dagger \mathbf{y})
     * @f]
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
                fac = pr::conj(*itrx) * (*itry) + fac;
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
     * @brief Adjoint.
     *
     * @param[in] x
     * Matrix @f$ \mathbf{X} @f$.
     *
     * @param[inout] y
     * Matrix @f$ \mathbf{Y} @f$.
     * 
     * @par Expression
     * @f[
     *      Y_{[j,i]} = \bar{X}_{[i,j]}
     * @f]
     *
     * @throw std::invalid_argument
     * If `x` is not empty and the dimensions of `y`
     * do not match. If `x` is empty and `y` is empty
     * or not square.
     *
     * @note
     * If `x` is empty and `y` is square, the implementation operates
     * on `y` in-place.
     */
    static void adjoint(
                dense_matrix_view<const value_type*> x,
                dense_matrix_view<value_type*> y)
    {
        if (x.empty()) {

            // Ensure square.
            if (y.size0() != y.size1() || y.empty()) {
                throw std::invalid_argument(__PRETTY_FUNCTION__);
            }

            // Swap.
            for (int i = 0; i < y.size0(); i++) {
                y[i][i] = pr::conj(y[i][i]);
                for (int j = i + 1; j < y.size0(); j++) {
                    value_type tmp0 = pr::conj(y[i][j]);
                    value_type tmp1 = pr::conj(y[j][i]);
                    y[i][j] = tmp1;
                    y[j][i] = tmp0;
                }
            }
        }
        else {

            // Ensure matching dimensions.
            if (x.size0() != y.size1() ||
                x.size1() != y.size0()) {
                throw std::invalid_argument(__PRETTY_FUNCTION__);
            }

            // Copy.
            for (int i = 0; i < x.size0(); i++)
            for (int j = 0; j < x.size1(); j++) {
                y[j][i] = pr::conj(x[i][j]);
            }
        }
    }

public:

    /**
     * @name Householder transformations
     */
    /**@{*/

    /**
     * @brief Householder step.
     *
     * @param[in] p
     * Target index @f$ p @f$.
     *
     * @param[in] q
     * Target index @f$ q @f$.
     *
     * @param[inout] x
     * Matrix @f$ \mathbf{X} @f$.
     *
     * @param[inout] y
     * Matrix @f$ \mathbf{Y} @f$. _Optional_.
     * 
     * @par Expression
     * @parblock
     * For @f$ \mathbf{X} \in T^{m \times n} @f$, construct
     * the @f$ (p, q) @f$ reflector @f$ \mathbf{w} \in T^{m-p} @f$ as
     * follows:
     * - let @f$ \mathbf{w} = X_{[p:m,q]} @f$;
     * - let @f$ \alpha = \operatorname{sign}(w_{[0]})
     *                    \lVert\mathbf{w}\rVert @f$;
     * - update @f$ \mathbf{w} \gets
     *              \mathbf{w} + \alpha\mathbf{\delta}_0 @f$;
     * - update @f$ \mathbf{w} \gets
     *              \mathbf{w} / \lVert\mathbf{w}\rVert @f$ to normalize.
     *
     * For @f$ j \ge q @f$, reflect
     * the @f$ X_{[p:m,j]} @f$ over @f$ \mathbf{w} @f$. This
     * assumes sequential application of Householder steps, such that
     * @f$ X_{[p:m,j]} = 0 @f$ for @f$ j < q @f$.
     * @endparblock
     *
     * @note
     * This method maintains a static thread-local buffer of
     * temporary values. If `x` is empty, the implementation
     * resets the buffer and returns immediately.
     */
    static void hhstepl(
                int p,
                int q,
                dense_matrix_view<value_type*> x,
                dense_matrix_view<value_type*> y = {})
    {
        // Temporary buffer.
        static thread_local std::vector<value_type> tmpbuf;
        if (tmpbuf.size() < std::size_t(x.size0())) {
            tmpbuf.resize(x.size0());
        }

        // Empty?
        if (x.empty()) {
            // Use as signal to free buffer.
            tmpbuf.clear();
            tmpbuf.shrink_to_fit();
            return;
        }

        // Quit if out of range.
        if (p < 0 || p >= x.size0() ||
            q < 0 || q >= x.size1()) {
            return;
        }

        // Temporary buffer view.
        dense_vector_view<value_type*> w = {
            &tmpbuf[0],
            &tmpbuf[0] + x.size0() - p
        };

        // Copy qth column.
        std::copy(
            x.transpose()[q].range(p, x.size0()).begin(),
            x.transpose()[q].range(p, x.size0()).end(),
            w.begin());

        // Compute reflector.
        value_type alpha = pr::sign(w[0]) * length(w);
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
            reflect(w, x.transpose()[j].range(p, x.size0()));
        }

        // Accumulate.
        if (!y.empty()) {

            // Sanity check.
            if (x.size0() != y.size0()) {
                throw std::invalid_argument(__PRETTY_FUNCTION__);
            }

            // Reflect columns.
            for (int j = 0; j < y.size1(); j++) {
                reflect(w, y.transpose()[j].range(p, x.size0()));
            }
        }
    }

    /**
     * @brief Householder step.
     *
     * @param[in] p
     * Target index @f$ p @f$.
     *
     * @param[in] q
     * Target index @f$ q @f$.
     *
     * @param[inout] x
     * Matrix @f$ \mathbf{X} @f$.
     *
     * @param[inout] y
     * Matrix @f$ \mathbf{Y} @f$. _Optional_.
     *
     * @note
     * This method calls `hhstepl()` with the appropriate
     * view transform.
     */
    static void hhstepr(
                int p,
                int q,
                dense_matrix_view<value_type*> x,
                dense_matrix_view<value_type*> y = {})
    {
        // Conjugate.
        for (int i = 0; i < x.size0(); i++)
        for (int j = 0; j < x.size1(); j++) {
            x[i][j] = pr::conj(x[i][j]);
        }
        for (int i = 0; i < y.size0(); i++)
        for (int j = 0; j < y.size1(); j++) {
            y[i][j] = pr::conj(y[i][j]);
        }

        // Delegate.
        hhstepl(q, p, x.transpose(), y.transpose());

        // Conjugate.
        for (int i = 0; i < x.size0(); i++)
        for (int j = 0; j < x.size1(); j++) {
            x[i][j] = pr::conj(x[i][j]);
        }
        for (int i = 0; i < y.size0(); i++)
        for (int j = 0; j < y.size1(); j++) {
            y[i][j] = pr::conj(y[i][j]);
        }
    }

    /**@}*/

public:

    /**
     * @name Unitary-triangular decomposition
     */
    /**@{*/

    /**
     * @brief QR-decomposition.
     *
     * @param[inout] x
     * Matrix @f$ \mathbf{X} \to \mathbf{R} @f$.
     *
     * @param[out] q
     * Matrix @f$ \mathbf{Q} @f$. _Optional_.
     *
     * @throw std::invalid_argument
     * If 
     * - `x` is empty,
     * - `q` is non-empty and `q.size0() != x.size0()`, or
     * - `q` is non-empty and `q.size1() != x.size0()`.
     */
    static void qr(
                dense_matrix_view<value_type*> x,
                dense_matrix_view<value_type*> q = {})
    {
        // Ensure valid.
        if (x.empty()) {
            throw std::invalid_argument(__PRETTY_FUNCTION__);
        }

        // Load identity.
        if (!q.empty()) {
            // Ensure square.
            if (q.size0() != x.size0() ||
                q.size1() != x.size0()) {
                throw std::invalid_argument(__PRETTY_FUNCTION__);
            }
            for (int i = 0; i < x.size0(); i++)
            for (int j = 0; j < x.size1(); j++) {
                q[i][j] = value_type(float_type(i == j ? 1 : 0));
            }
        }

        // Householder reduction.
        for (int k = 0; k < x.diag().size() - 1; k++) {
            hhstepl(k, k, x, q);
        }

        // Adjoint.
        if (!q.empty()) {
            adjoint({}, q);
        }
    }

    /**
     * @brief QL-decomposition.
     *
     * @param[inout] x
     * Matrix @f$ \mathbf{X} \to \mathbf{L} @f$.
     *
     * @param[out] q
     * Matrix @f$ \mathbf{Q} @f$. _Optional_.
     *
     * @note
     * This method calls `qr()` with the appropriate view transform.
     */
    static void ql(
                dense_matrix_view<value_type*> x,
                dense_matrix_view<value_type*> q = {})
    {
        // Delegate.
        qr(x.block(x.size0(), x.size1(), 0, 0),
           q.block(q.size0(), q.size1(), 0, 0));
    }

    /**
     * @brief RQ-decomposition.
     *
     * @param[inout] x
     * Matrix @f$ \mathbf{X} \to \mathbf{R} @f$.
     *
     * @param[out] q
     * Matrix @f$ \mathbf{Q} @f$. _Optional_.
     *
     * @note
     * This method calls `qr()` with the appropriate view transform.
     */
    static void rq(
                dense_matrix_view<value_type*> x,
                dense_matrix_view<value_type*> q = {})
    {
        // Conjugate.
        for (int i = 0; i < x.size0(); i++)
        for (int j = 0; j < x.size1(); j++) {
            x[i][j] = pr::conj(x[i][j]);
        }
        for (int i = 0; i < q.size0(); i++)
        for (int j = 0; j < q.size1(); j++) {
            q[i][j] = pr::conj(q[i][j]);
        }

        // Delegate.
        qr(x.block(x.size0(), x.size1(), 0, 0).transpose(),
           q.block(q.size0(), q.size1(), 0, 0).transpose());

        // Conjugate.
        for (int i = 0; i < x.size0(); i++)
        for (int j = 0; j < x.size1(); j++) {
            x[i][j] = pr::conj(x[i][j]);
        }
        for (int i = 0; i < q.size0(); i++)
        for (int j = 0; j < q.size1(); j++) {
            q[i][j] = pr::conj(q[i][j]);
        }
    }

    /**
     * @brief LQ-decomposition.
     *
     * @param[inout] x
     * Matrix @f$ \mathbf{X} \to \mathbf{L} @f$.
     *
     * @param[out] q
     * Matrix @f$ \mathbf{Q} @f$. _Optional_.
     *
     * @note
     * This method calls `qr()` with the appropriate view transform.
     */
    static void lq(
                dense_matrix_view<value_type*> x,
                dense_matrix_view<value_type*> q = {})
    {
        // Conjugate.
        for (int i = 0; i < x.size0(); i++)
        for (int j = 0; j < x.size1(); j++) {
            x[i][j] = pr::conj(x[i][j]);
        }
        for (int i = 0; i < q.size0(); i++)
        for (int j = 0; j < q.size1(); j++) {
            q[i][j] = pr::conj(q[i][j]);
        }

        // Delegate.
        qr(x.transpose(),
           q.transpose());

        // Conjugate.
        for (int i = 0; i < x.size0(); i++)
        for (int j = 0; j < x.size1(); j++) {
            x[i][j] = pr::conj(x[i][j]);
        }
        for (int i = 0; i < q.size0(); i++)
        for (int j = 0; j < q.size1(); j++) {
            q[i][j] = pr::conj(q[i][j]);
        }
    }

    /**@}*/

public:

    /**
     * @name Triangular-triangular decomposition
     */
    /**@{*/

    // TODO Doc
    /**
     * @brief Cholesky decomposition.
     *
     * @param[inout] x
     * Matrix @f$ \mathbf{X} \to \mathbf{L} @f$.
     *
     * @param[out] p
     * Vector @f$ \mathbf{P} @f$. _Optional_.
     */
    static void chol(
                dense_matrix_view<value_type*> x,
                dense_vector_view<int*> p = {})
    {
        // Ensure valid.
        if (x.empty() ||
            x.size0() != x.size1()) {
            throw std::invalid_argument(__PRETTY_FUNCTION__);
        }

        // Initialize pivot.
        if (!p.empty()) {
            // Ensure valid.
            if (p.size() != x.size0()) {
                throw std::invalid_argument(__PRETTY_FUNCTION__);
            }
            // Initialize.
            std::iota(
                    p.begin(),
                    p.end(),
                    0);
        }

        // Iterate.
        for (int k = 0; k < x.size0(); k++) {

            // Pivot.
            if (!p.empty()) {
                int l = k;
                float_type fac = 0;
                float_type tmp = 0;
                for (int i = k; i < x.size0(); i++) {
                    tmp = pr::abs(x[i][i]);
                    if (fac < tmp) {
                        fac = tmp;
                        l = i;
                    }
                }
                if (k != l) {
                    // Swap.
                    std::swap(p[k], p[l]);
                    std::swap_ranges(
                        x[k].begin(), x[k].end(),
                        x[l].begin());
                    std::swap_ranges(
                        x.transpose()[k].begin(), x.transpose()[k].end(),
                        x.transpose()[l].begin());
                }

                // Positive semi-definite?
                if (!(pr::abs(x[k][k]) >
                      pr::abs(x[0][0]) *
                      pr::numeric_limits<float_type>::epsilon())) {
                    for (int i = k; i < x.size0(); i++)
                    for (int j = k; j < i + 1; j++) {
                        x[i][j] = value_type();
                    }
                    break;
                }
            }

            if (x[k][k] != pr::real(x[k][k])) {
                throw std::runtime_error(__PRETTY_FUNCTION__);
            }

            // Update diagonal entry.
            x[k][k] = pr::sqrt(pr::real(x[k][k]));

            // Update off-diagonal entries.
            float_type invxkk = 1 / pr::real(x[k][k]);
            for (int i = k + 1; i < x.size0(); i++) {
                x[i][k] *= invxkk;
                if (!pr::isfinite(x[i][k])) {
                    throw std::runtime_error(__PRETTY_FUNCTION__);
                }
            }
            for (int i = k + 1; i < x.size0(); i++)
            for (int j = k + 1; j < i + 1; j++) {
                x[i][j] -= x[i][k] * pr::conj(x[j][k]);
            }
        }

        // To lower triangular.
        for (int i = 1; i < x.size0(); i++)
        for (int j = 0; j < i; j++) {
            x[j][i] = value_type();
        }
    }

    /**@}*/
};

/**@}*/

} // namespace pr

#endif // #ifndef PREFORM_DENSE_LINALG_HPP
