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
#error "preform/image_helpers.hpp requires >=C++17"
#endif // #if !(__cplusplus >= 201703L)
#endif // #if !DOXYGEN
#pragma once
#ifndef PREFORM_IMAGE_HELPERS_HPP
#define PREFORM_IMAGE_HELPERS_HPP

// for pr::wrap, pr::wrap_mirror
#include <preform/int_helpers.hpp>

// for pr::multi
#include <preform/multi.hpp>
#include <preform/multi_math.hpp>
#include <preform/multi_float_helpers.hpp>

// for pr::lerp, pr::catmull
#include <preform/interp.hpp>

namespace pr {

/**
 * @defgroup image_helpers Image helpers
 *
 * `<preform/image_helpers.hpp>`
 *
 * __C++ version__: >=C++17
 */
/**@{*/

/**
 * @brief Cycle mode.
 */
enum class cycle_mode
{
    /**
     * @brief Clamp.
     */
    clamp,

    /**
     * @brief Repeat.
     */
    repeat,

    /**
     * @brief Repeat with mirroring.
     */
    mirror
};

/**
 * @brief Cycle mode to string.
 */
constexpr const char* to_string(cycle_mode mode)
{
    switch (mode) {
        case cycle_mode::clamp: return "clamp";
        case cycle_mode::repeat: return "repeat";
        case cycle_mode::mirror: return "mirror";
        default: break;
    }
    return "unknown";
}

#if !DOXYGEN

// Apply cycle op.
template <typename T>
constexpr std::enable_if_t<
          std::is_integral<T>::value, T> cycle(
                                         cycle_mode mode, 
                                         T index, 
                                         T count)
{
    switch (mode) {
        case cycle_mode::clamp:
            return std::max<T>(0,
                   std::min<T>(index, count - 1));
        case cycle_mode::repeat: return wrap(index, count);
        case cycle_mode::mirror: return wrap_mirror(index, count);
        default: break;
    }
    return 0;
}

// Apply cycle op, multi wrapper.
template <typename T, std::size_t N>
constexpr std::enable_if_t<
          std::is_integral<T>::value,
          multi<T, N>> cycle(
                        const multi<cycle_mode, N>& mode,
                        const multi<T, N>& index,
                        const multi<T, N>& count)
{
    multi<T, N> res;
    auto itrmode = mode.begin();
    auto itrindex = index.begin();
    auto itrcount = count.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrmode, ++itrindex, ++itrcount, ++itrres) {
        *itrres = cycle(
                    *itrmode, 
                    *itrindex, 
                    *itrcount);
    }
    return res;
}

#endif // #if !DOXYGEN

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
    multi<T, N> rad = multi<T, N>(T(0.5));

    /**
     * @brief Evaluate.
     */
    T operator()(multi<T, N> vec) const
    {
        multi<T, N> res = {};
        for (int k = 0; k < int(N); k++) {
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
        return res.prod();
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
    multi<T, N> rad = multi<T, N>(T(1));

    /**
     * @brief Evaluate.
     */
    T operator()(multi<T, N> vec) const
    {
        multi<T, N> res = {};
        for (int k = 0; k < int(N); k++) {
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
        return res.prod();
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
    multi<T, N> rad = multi<T, N>(T(1));

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
    T operator()(multi<T, N> vec) const
    {
        multi<T, N> res = {};
        for (int k = 0; k < int(N); k++) {
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
        return res.prod();
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

#endif // #ifndef PREFORM_IMAGE_HELPERS_HPP
