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
#if !(__cplusplus >= 201402L)
#error "preform/blas.hpp requires >=C++14"
#endif // #if !(__cplusplus >= 201402L)
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

namespace blas {

/**
 * @brief Dot product.
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
template <
    typename Trandom_itr,
    typename Urandom_itr
    >
inline auto dot(
            dense_vector_view<Trandom_itr> x,
            dense_vector_view<Urandom_itr> y)
{
    if (x.size() != y.size()) {
        throw std::invalid_argument(__PRETTY_FUNCTION__);
    }

    // Default construct.
    decltype(
    typename dense_vector_view<Trandom_itr>::value_type() * 
    typename dense_vector_view<Urandom_itr>::value_type()) res = {};

    // Dot.
    auto itrx = x.begin();
    auto itry = y.begin();
    for (; itrx < x.end(); ++itrx, ++itry) {
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
 * @param[in] func
 * Predicate.
 *
 * @throw std::invalid_argument
 * Unless `x.size() == y.size()`.
 */
template <
    typename Trandom_itr,
    typename Urandom_itr,
    typename F
    >
inline auto dot_conj(
            dense_vector_view<Trandom_itr> x,
            dense_vector_view<Urandom_itr> y,
            F&& func)
{
    if (x.size() != y.size()) {
        throw std::invalid_argument(__PRETTY_FUNCTION__);
    }

    // Default construct.
    decltype(
    typename dense_vector_view<Trandom_itr>::value_type() * 
    typename dense_vector_view<Urandom_itr>::value_type()) res = {};

    // Dot.
    auto itrx = x.begin();
    auto itry = y.begin();
    for (; itrx < x.end(); ++itrx, ++itry) {
        res = std::forward<F>(func)(*itrx) * (*itry) + res;
    }

    return res;
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
 * @throw std::invalid_argument
 * Unless `x.size() == y.size()`.
 */
template <
    typename Trandom_itr,
    typename Urandom_itr
    >
inline void reflect(
            dense_vector_view<Trandom_itr> x,
            dense_vector_view<Urandom_itr> y)
{
    if (x.size() != y.size()) {
        throw std::invalid_argument(__PRETTY_FUNCTION__);
    }

    // Default construct.
    decltype(
    typename dense_vector_view<Trandom_itr>::value_type() * 
    typename dense_vector_view<Urandom_itr>::value_type()) fac = {};

    // Dot.
    {
        auto itrx = x.begin();
        auto itry = y.begin();
        for (; itry < y.end(); ++itrx, ++itry) {
            fac = (*itrx) * (*itry) + fac;
        }
    }

    // Absorb multiplier.
    fac *= typename dense_vector_view<Trandom_itr>::value_type(-2);

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
 * @param[in] x
 * Vector @f$ \mathbf{x} @f$.
 *
 * @param[inout] y
 * Vector @f$ \mathbf{y} @f$.
 *
 * @param[in] func
 * Predicate.
 *
 * @throw std::invalid_argument
 * Unless `x.size() == y.size()`.
 */
template <
    typename Trandom_itr,
    typename Urandom_itr,
    typename F
    >
inline void reflect_conj(
            dense_vector_view<Trandom_itr> x,
            dense_vector_view<Urandom_itr> y,
            F&& func)
{
    if (x.size() != y.size()) {
        throw std::invalid_argument(__PRETTY_FUNCTION__);
    }

    // Default construct.
    decltype(
    typename dense_vector_view<Trandom_itr>::value_type() * 
    typename dense_vector_view<Urandom_itr>::value_type()) fac = {};

    // Dot.
    {
        auto itrx = x.begin();
        auto itry = y.begin();
        for (; itry < y.end(); ++itrx, ++itry) {
            fac = std::forward<F>(func)(*itrx) * (*itry) + fac;
        }
    }

    // Absorb multiplier.
    fac *= typename dense_vector_view<Trandom_itr>::value_type(-2);

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
 */
template <typename Trandom_itr>
inline auto length(dense_vector_view<Trandom_itr> x)
{
    // Value type.
    typedef typename 
            dense_vector_view<Trandom_itr>::
            value_type value_type;

    // Float type.
    typedef decltype(pr::sqrt(pr::abs(value_type()))) float_type;

    // Sanity check.
    static_assert(
        std::is_floating_point<float_type>::value, "???");

    switch (x.size()) {
        case 0: 
            // Error?
            return float_type(0);
        case 1: 
            // Delegate.
            return float_type(pr::abs(x[0]));
        case 2: 
            // Delegate.
            return float_type(pr::hypot(pr::abs(x[0]), pr::abs(x[1])));
        default:
            break;
    }

    // Temporary vector.
    static thread_local std::vector<float_type> tmpvec;
    if (tmpvec.size() < std::size_t(x.size())) {
        tmpvec.resize(x.size());
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
            *itrtmp = pr::abs(*itrx);
            if (*itrtmp != float_type(0)) {
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
        return pr::sqrt(dot(tmp, tmp)) * tmpmax;
    }
    else {
        // Length.
        return pr::sqrt(dot(tmp, tmp));
    }
}

} // namespace blas

/**@}*/

} // namespace pr

#endif // #ifndef PREFORM_BLAS_HPP
