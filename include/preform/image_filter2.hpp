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
#error "preform/image_filter2.hpp requires >=C++17"
#endif // #if !(__cplusplus >= 201703L)
#endif // #if !DOXYGEN
#pragma once
#ifndef PREFORM_IMAGE_FILTER2_HPP
#define PREFORM_IMAGE_FILTER2_HPP

// for pr::multi
#include <preform/multi.hpp>
#include <preform/multi_math.hpp>
#include <preform/multi_float_helpers.hpp>

namespace pr {

/**
 * @defgroup image_filter2 Image filters (2-dimensional)
 *
 * `<preform/image_filter2.hpp>`
 *
 * __C++ version__: >=C++17
 */
/**@{*/

/**
 * @brief Box filter.
 */
template <typename T>
struct box_filter2
{
    // Sanity check.
    static_assert(
        std::is_floating_point<T>::value,
        "T must be floating point");

    /**
     * @brief Radii.
     */
    multi<T, 2> rad = multi<T, 2>(T(0.5));

    /**
     * @brief Evaluate.
     */
    T operator()(multi<T, 2> vec) const
    {
        multi<T, 2> res = {};
        for (int k = 0; k < 2; k++) {
            // Absolute value.
            vec[k] = pr::fabs(vec[k]);

            if (vec[k] < rad[k]) {
                res[k] = T(0.50) / rad[k];
            }
            else if (vec[k] == rad[k]) {
                // On boundary, reduce by half.
                res[k] = T(0.25) / rad[k];
            }
            else {
                // Early out.
                return T(0);
            }
        }
        return res[0] * res[1];
    }
};

/**
 * @brief Triangle filter.
 */
template <typename T>
struct triangle_filter2
{
    // Sanity check.
    static_assert(
        std::is_floating_point<T>::value,
        "T must be floating point");

    /**
     * @brief Radii.
     */
    multi<T, 2> rad = multi<T, 2>(T(1));

    /**
     * @brief Evaluate.
     */
    T operator()(multi<T, 2> vec) const
    {
        multi<T, 2> res = {};
        for (int k = 0; k < 2; k++) {
            // Absolute value.
            vec[k] = pr::fabs(vec[k]);

            if (vec[k] < rad[k]) {
                res[k] = T(1) - pr::fabs(vec[k]) / rad[k];
                res[k] = res[k] / rad[k];
            }
            else {
                // Early out.
                return T(0);
            }
        }
        return res[0] * res[1];
    }
};

/**
 * @brief Mitchell filter.
 */
template <typename T>
struct mitchell_filter2
{
    // Sanity check.
    static_assert(
        std::is_floating_point<T>::value,
        "T must be floating point");

    /**
     * @brief Radii.
     */
    multi<T, 2> rad = multi<T, 2>(T(1));

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
    T operator()(multi<T, 2> vec) const
    {
        multi<T, 2> res = {};
        for (int k = 0; k < 2; k++) {
            // Absolute value.
            vec[k] = pr::fabs(vec[k]);

            if (vec[k] < rad[k]) {
                // Rescale.
                T t = T(2) * (vec[k] / rad[k]);
                if (t > T(1)) {
                    // Cubic coefficients.
                    T a3 = c + T(1) / T(6) * b;
                    T a2 = b + T(5) * c;
                    T a1 = T(2) * b + T(8) * c;
                    T a0 = T(4) / T(3) * b + T(4) * c;
                    a3 = -a3;
                    a1 = -a1;

                    // Cubic.
                    res[k] = a3;
                    res[k] = pr::fma(res[k], t, a2);
                    res[k] = pr::fma(res[k], t, a1);
                    res[k] = pr::fma(res[k], t, a0);
                }
                else {
                    // Cubic coefficients.
                    T a3 = T(2) - T(3) / T(2) * b - c;
                    T a2 = T(2) * b - T(3) + c;
                    T a1 = T(0);
                    T a0 = T(1) - T(1) / T(3) * b;

                    // Cubic.
                    res[k] = a3;
                    res[k] = pr::fma(res[k], t, a2);
                    res[k] = pr::fma(res[k], t, a1);
                    res[k] = pr::fma(res[k], t, a0);
                }
                // Normalize.
                res[k] *= T(2) / rad[k];
            }
            else {
                // Early out.
                return T(0);
            }
        }
        return res[0] * res[1];
    }
};

/**@}*/

} // namespace pr

#endif // #ifndef PREFORM_IMAGE_FILTER2_HPP
