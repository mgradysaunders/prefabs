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
#error "preform/image_filters.hpp requires >=C++17"
#endif // #if !(__cplusplus >= 201703L)
#endif // #if !DOXYGEN
#pragma once
#ifndef PREFORM_IMAGE_FILTERS_HPP
#define PREFORM_IMAGE_FILTERS_HPP

// for pr::multi
#include <preform/multi.hpp>

// for pr::multi wrappers
#include <preform/multi_math.hpp>

// for pr::multi wrappers
#include <preform/multi_misc_float.hpp>

namespace pr {

/**
 * @defgroup image_filters Image filters
 *
 * `<preform/image_filters.hpp>`
 *
 * __C++ version__: >=C++17
 */
/**@{*/

/**
 * @brief Box filter.
 */
template <typename T, std::size_t N>
struct box_filter
{
    // Sanity check.
    static_assert(
        std::is_floating_point<T>::value,
        "T must be floating point");

    /**
     * @brief Radii.
     */
    multi<T, N> r = multi<T, N>(T(0.5));

    /**
     * @brief Evaluate.
     */
    T operator()(multi<T, N> x) const
    {
        // Absolute value.
        x = pr::fabs(x);

        // Calculate.
        multi<T, N> y = {};
        for (int l = 0; l < int(N); l++) {
            if (x[l] < r[l]) {
                y[l] = T(0.5);
                continue;
            }
            if (x[l] == r[l]) {
                // On boundary, reduce by half.
                y[l] = T(0.25);
                continue;
            }

            // Early out.
            return T(0);
        }

        // Normalize.
        y /= r;
        return y.prod();
    }
};

/**
 * @brief Box filter (2-dimensional).
 */
template <typename T>
using box_filter2 = box_filter<T, 2>;

/**
 * @brief Box filter (3-dimensional).
 */
template <typename T>
using box_filter3 = box_filter<T, 3>;

/**
 * @brief Triangle filter.
 */
template <typename T, std::size_t N>
struct triangle_filter
{
    // Sanity check.
    static_assert(
        std::is_floating_point<T>::value,
        "T must be floating point");

    /**
     * @brief Radii.
     */
    multi<T, N> r = multi<T, N>(T(1));

    /**
     * @brief Evaluate.
     */
    T operator()(multi<T, N> x) const
    {
        // Absolute value.
        x = pr::fabs(x);

        if (!(x < r).all()) {
            // Early out.
            return T(0);
        }

        // Calculate.
        multi<T, N> y = T(1) - x / r;

        // Normalize.
        y /= r;
        return y.prod();
    }
};

/**
 * @brief Triangle filter (2-dimensional).
 */
template <typename T>
using triangle_filter2 = triangle_filter<T, 2>;

/**
 * @brief Triangle filter (3-dimensional).
 */
template <typename T>
using triangle_filter3 = triangle_filter<T, 3>;

/**
 * @brief Mitchell filter.
 */
template <typename T, std::size_t N>
struct mitchell_filter
{
    // Sanity check.
    static_assert(
        std::is_floating_point<T>::value,
        "T must be floating point");

    /**
     * @brief Radii.
     */
    multi<T, N> r = multi<T, N>(T(1));

    /**
     * @brief Mitchell coefficient @f$ B @f$.
     */
    T b = T(1) / T(3);

    /**
     * @brief Mitchell coefficient @f$ C @f$.
     */
    T c = T(1) / T(3);

    /**
     * @brief Evaluate.
     */
    T operator()(multi<T, N> x) const
    {
        // Absolute value.
        x = pr::fabs(x);

        if (!(x < r).all()) {
            // Early out.
            return T(0);
        }

        // Remap.
        multi<T, N> t = T(2) * (x / r);

        // Coefficients for t > 1.
        T a0[4] = {
            T(4) / T(3) * b + T(4) * c,
            T(2) * b + T(8) * c,
            b + T(5) * c,
            c + T(1) / T(6) * b
        };
        a0[1] = -a0[1];
        a0[3] = -a0[3];

        // Coefficients for t < 1.
        T a1[4] = {
            T(1) - T(1) / T(3) * b,
            T(0),
            T(2) * b - T(3) + c,
            T(2) - T(3) / T(2) * b - c
        };

        // Coefficients.
        multi<T, N> a[4];
        for (int l = 0; l < int(N); l++) {
            if (t[l] > T(1)) {
                a[0][l] = a0[0];
                a[1][l] = a0[1];
                a[2][l] = a0[2];
                a[3][l] = a0[3];
            }
            else {
                a[0][l] = a1[0];
                a[1][l] = a1[1];
                a[2][l] = a1[2];
                a[3][l] = a1[3];
            }
        }

        // Calculate.
        multi<T, N> y = a[3];
        y = pr::fma(y, t, a[2]);
        y = pr::fma(y, t, a[1]);
        y = pr::fma(y, t, a[0]);

        // Normalize.
        y *= T(2) / r;
        return y.prod();
    }
};

/**
 * @brief Mitchell filter (2-dimensional).
 */
template <typename T>
using mitchell_filter2 = mitchell_filter<T, 2>;

/**
 * @brief Mitchell filter (3-dimensional).
 */
template <typename T>
using mitchell_filter3 = mitchell_filter<T, 3>;

/**@}*/

} // namespace pr

#endif // #ifndef PREFORM_IMAGE_FILTERS_HPP
