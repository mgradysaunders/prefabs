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
#error "preform/multi_float_interval.hpp requires >=C++17"
#endif // #if !(__cplusplus >= 201703L)
#endif // #if !DOXYGEN
#pragma once
#ifndef PREFORM_MULTI_FLOAT_INTERVAL_HPP
#define PREFORM_MULTI_FLOAT_INTERVAL_HPP

// for pr::float_interval
#include <preform/float_interval.hpp>

// for pr::multi
#include <preform/multi.hpp>

namespace pr {

/**
 * @defgroup multi_float_interval Multi-dimensional array (float interval)
 *
 * `<preform/multi_float_interval.hpp>`
 *
 * __C++ version__: >=C++17
 */
/**@{*/

/**
 * @brief Initializers and member functions for multi-dimensional
 * arrays of float intervals.
 */
template <typename T, std::size_t M, std::size_t... N>
struct multi_initializers<multi<float_interval<T>, M, N...>>
{
public:

    /**
     * @name Initializers
     */
    /**@{*/

    /**
     * @brief Broadcast initializer.
     *
     * @param[in] x
     * Value.
     *
     * @param[in] x0
     * Value lower bound.
     *
     * @param[in] x1
     * Value upper bound.
     */
    static 
    multi<float_interval<T>, M, N...> make_interval(
                    const multi<T, M, N...>& x,
                    const multi<T, M, N...>& x0,
                    const multi<T, M, N...>& x1)
    {
        multi<float_interval<T>, M, N...> res;
        auto itrx = x.begin();
        auto itrx0 = x0.begin();
        auto itrx1 = x1.begin();
        auto itrres = res.begin();
        for (; itrres < res.end(); ++itrx, ++itrx0, ++itrx1, ++itrres) {
            if constexpr (sizeof...(N) == 0) {
                *itrres = float_interval<T>{*itrx, *itrx0, *itrx1};
            }
            else {
                *itrres = multi<float_interval<T>, N...>
                        ::make_interval(*itrx, *itrx0, *itrx1);
            }
        }
        return res;
    }

    /**
     * @brief Broadcast initializer.
     *
     * @param[in] x
     * Value.
     *
     * @param[in] xerr
     * Value absolute error.
     */
    static 
    multi<float_interval<T>, M, N...> make_interval(
                    const multi<T, M, N...>& x,
                    const multi<T, M, N...>& xerr)
    {
        multi<float_interval<T>, M, N...> res;
        auto itrx = x.begin();
        auto itrxerr = xerr.begin();
        auto itrres = res.begin();
        for (; itrres < res.end(); ++itrx, ++itrxerr, ++itrres) {
            if constexpr (sizeof...(N) == 0) {
                *itrres = float_interval<T>{*itrx, *itrxerr};
            }
            else {
                *itrres = multi<float_interval<T>, N...>
                        ::make_interval(*itrx, *itrxerr);
            }
        }
        return res;
    }

    /**@}*/

public:

    /**
     * @name Member functions
     */
    /**@{*/

    /**
     * @brief Broadcast `float_interval::value()`.
     */
    multi<T, M, N...> value() const
    {
        multi<T, M, N...> res;
        const multi<float_interval<T>, M, N...>& this_ref = 
            *static_cast<const multi<float_interval<T>, M, N...>*>(this);
        auto itrval = this_ref.begin();
        auto itrres = res.begin();
        for (; itrres < res.end(); ++itrval, ++itrres) {
            *itrres = itrval->value();
        }
        return res;
    }

    /**
     * @brief Broadcast `float_interval::lower_bound()`.
     */
    multi<T, M, N...> lower_bound() const
    {
        multi<T, M, N...> res;
        const multi<float_interval<T>, M, N...>& this_ref = 
            *static_cast<const multi<float_interval<T>, M, N...>*>(this);
        auto itrval = this_ref.begin();
        auto itrres = res.begin();
        for (; itrres < res.end(); ++itrval, ++itrres) {
            *itrres = itrval->lower_bound();
        }
        return res;
    }

    /**
     * @brief Broadcast `float_interval::upper_bound()`.
     */
    multi<T, M, N...> upper_bound() const
    {
        multi<T, M, N...> res;
        const multi<float_interval<T>, M, N...>& this_ref = 
            *static_cast<const multi<float_interval<T>, M, N...>*>(this);
        auto itrval = this_ref.begin();
        auto itrres = res.begin();
        for (; itrres < res.end(); ++itrval, ++itrres) {
            *itrres = itrval->upper_bound();
        }
        return res;
    }

    /**
     * @brief Broadcast `float_interval::abs_lower_bound()`.
     */
    multi<T, M, N...> abs_lower_bound() const
    {
        multi<T, M, N...> res;
        const multi<float_interval<T>, M, N...>& this_ref = 
            *static_cast<const multi<float_interval<T>, M, N...>*>(this);
        auto itrval = this_ref.begin();
        auto itrres = res.begin();
        for (; itrres < res.end(); ++itrval, ++itrres) {
            *itrres = itrval->abs_lower_bound();
        }
        return res;
    }

    /**
     * @brief Broadcast `float_interval::upper_bound()`.
     */
    multi<T, M, N...> abs_upper_bound() const
    {
        multi<T, M, N...> res;
        const multi<float_interval<T>, M, N...>& this_ref = 
            *static_cast<const multi<float_interval<T>, M, N...>*>(this);
        auto itrval = this_ref.begin();
        auto itrres = res.begin();
        for (; itrres < res.end(); ++itrval, ++itrres) {
            *itrres = itrval->abs_upper_bound();
        }
        return res;
    }

    /**
     * @brief Broadcast `float_interval::abs_error()`.
     */
    multi<T, M, N...> abs_error() const
    {
        multi<T, M, N...> res;
        const multi<float_interval<T>, M, N...>& this_ref = 
            *static_cast<const multi<float_interval<T>, M, N...>*>(this);
        auto itrval = this_ref.begin();
        auto itrres = res.begin();
        for (; itrres < res.end(); ++itrval, ++itrres) {
            *itrres = itrval->abs_error();
        }
        return res;
    }

    /**
     * @brief Broadcast `float_interval::rel_error()`.
     */
    multi<T, M, N...> rel_error() const
    {
        multi<T, M, N...> res;
        const multi<float_interval<T>, M, N...>& this_ref = 
            *static_cast<const multi<float_interval<T>, M, N...>*>(this);
        auto itrval = this_ref.begin();
        auto itrres = res.begin();
        for (; itrres < res.end(); ++itrval, ++itrres) {
            *itrres = itrval->rel_error();
        }
        return res;
    }

    /**@}*/
};

/**@}*/

} // namespace pr

#endif // #ifndef PREFORM_MULTI_FLOAT_INTERVAL_HPP
