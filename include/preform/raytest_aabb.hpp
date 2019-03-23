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
 * @defgroup raytest_aabb Ray-test (axis-aligned bounding box)
 *
 * `<preform/raytest_aabb.hpp>`
 *
 * __C++ version__: >=C++17
 */
/**@{*/

/**
 * @brief Ray-test (axis-aligned bounding box).
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
    struct ray_type
    {
    public:

        /**
         * @brief Default constructor.
         */
        ray_type() = default;

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

#if 0
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
    float_type intersect(const ray_type& ray) const
    {
        return 0;
    }
#endif

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
