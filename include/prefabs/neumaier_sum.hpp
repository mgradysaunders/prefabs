/* Copyright (c) 2018 M. Grady Saunders
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
#pragma once
#ifndef PREFABS_NEUMAIER_SUM_HPP
#define PREFABS_NEUMAIER_SUM_HPP

#include <prefabs/math.hpp>

namespace pr {

/**
 * @defgroup neumaier_sum Neumaier sum
 *
 * `<prefabs/neumaier_sum.hpp>`
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
    static_assert(std::is_floating_point<T>::value, "invalid type");

    /**
     * @name Constructors
     */
    /**@{*/

    /**
     * @brief Default constructor
     */
    constexpr neumaier_sum() = default;

    /**
     * @brief Default copy constructor
     */
    constexpr neumaier_sum(const neumaier_sum&) = default;

    /**
     * @brief Default move constructor
     */
    constexpr neumaier_sum(neumaier_sum&&) = default;

    /**
     * @brief Constructor.
     */
    constexpr neumaier_sum(T s) : s_(s)
    {
    }

    /**@}*/

public:

    /**
     * @name Assign operators
     */
    /**@{*/

    /**
     * @brief Default copy assign.
     */
    constexpr neumaier_sum& operator=(const neumaier_sum&) = default;

    /**
     * @brief Default move assign.
     */
    constexpr neumaier_sum& operator=(neumaier_sum&&) = default;

    /**@}*/

public:

    /**
     * @name Arithmetic operators
     */
    /**@{*/

    /**
     * @brief Add term.
     *
     * @f[
     *      \begin{aligned}
     *          s' &\gets s \oplus x
     *      \\  t' &\gets t \oplus ((\max(s, x) \ominus s') \oplus \min(s, x))
     *      \end{aligned}
     * @f]
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
     * @brief Subtract term.
     */
    neumaier_sum& operator-=(T x)
    {
        return *this -= x;
    }

    /**@}*/

    /**
     * @brief Cast as float.
     */
    operator T() const
    {
        return s_ + t_;
    }

private:

    /**
     * @brief @f$ s @f$, sum.
     */
    T s_ = 0;

    /**
     * @brief @f$ t @f$, low-order term.
     */
    T t_ = 0;
};

/**@}*/

/**@}*/

} // namespace pr

#endif // #ifndef PREFABS_NEUMAIER_SUM_HPP