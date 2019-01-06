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
#error "prefabs/neumaier_sum.hpp requires >=C++14"
#endif // #if !(__cplusplus >= 201402L)
#endif // #if !DOXYGEN
#pragma once
#ifndef PREFABS_NEUMAIER_SUM_HPP
#define PREFABS_NEUMAIER_SUM_HPP

#include <prefabs/math.hpp>

namespace pr {

/**
 * @defgroup neumaier_sum Neumaier sum
 *
 * `<prefabs/neumaier_sum.hpp>`
 *
 * __C++ version__: >=C++14
 */
/**@{*/

/**
 * @brief Neumaier summation algorithm.
 */
template <typename T>
class neumaier_sum
{
public:

    // Sanity check.
    static_assert(
        std::is_floating_point<T>::value, 
        "T must be floating point");

    /**
     * @brief Default constructor
     */
    constexpr neumaier_sum() = default;

    /**
     * @brief Constructor.
     *
     * @param[in] s
     * Sum @f$ s @f$.
     *
     * @param[in] t
     * Low-order term @f$ t @f$.
     */
    constexpr neumaier_sum(T s, T t = T()) : s_(s), t_(t)
    {
    }

public:

    /**
     * @brief Add term.
     *
     * @note
     * - @f$ s' \gets s \oplus x @f$
     * - @f$ t' \gets t \oplus ((\max(s, x) \ominus s') \oplus \min(s, x)) @f$
     */
    neumaier_sum& operator+=(T x)
    {
        volatile T s = s_ + x;
        volatile T t = t_ + ((pr::fmax(s_, x) - s) + pr::fmin(s_, x));
        s_ = s;
        t_ = t;
        return *this;
    }

    /**
     * @brief Add negative term.
     */
    neumaier_sum& operator-=(T x)
    {
        return *this += -x;
    }

    /**
     * @brief Cast as different precision.
     */
    template <typename U>
    operator neumaier_sum<U>() const
    {
        return {
            static_cast<U>(s_),
            static_cast<U>(t_)
        };
    }

    /**
     * @brief Cast as float.
     */
    explicit operator T() const
    {
        return s_ + t_;
    }

private:

    /**
     * @brief Sum @f$ s @f$.
     */
    T s_ = 0;

    /**
     * @brief Low-order term @f$ t @f$.
     */
    T t_ = 0;
};

/**@}*/

/**@}*/

} // namespace pr

#endif // #ifndef PREFABS_NEUMAIER_SUM_HPP
