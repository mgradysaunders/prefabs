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
#error "preform/tabulation.hpp requires >=C++14"
#endif // #if !(__cplusplus >= 201402L)
#pragma once
#ifndef PREFORM_TABULATION_HPP
#define PREFORM_TABULATION_HPP

// for std::lower_bound
#include <algorithm>

// for std::iterator_traits, std::next, ...
#include <iterator>

// for std::invalid_argument
#include <stdexcept>

// for pr::lerp
#include <preform/interp.hpp>

// for pr::neumaier_sum
#include <preform/neumaier_sum.hpp>

namespace pr {

/**
 * @defgroup tabulation Tabulation
 *
 * `<preform/tabulation.hpp>`
 *
 * __C++ version__: >= C++14
 */
/**@{*/

/**
 * @brief Find index. 
 *
 * @f$ j \in [1, n) \implies
 *     t_{[j - 1]} < t \le t_{[j]} @f$
 *
 * @pre
 * - @f$ n \ge 2 @f$
 * - @f$ t_{[j - 1]} \le t_{[j]} @f$ for all @f$ j \in [1, n) @f$
 */
template <
    typename T,
    typename Tforward_itr
    >
inline typename 
       std::iterator_traits<Tforward_itr>::difference_type
       table_hunt(
            const T& t,
            Tforward_itr tfrom, 
            Tforward_itr tto)
{
    typedef 
        typename std::iterator_traits<Tforward_itr>::difference_type
        difference_type;
    difference_type n = std::distance(tfrom, tto);
    difference_type j = std::distance(tfrom, std::lower_bound(tfrom, tto, t));
    if (j < difference_type(1))
        j = difference_type(1);
    if (j > difference_type(n - 1))
        j = difference_type(n - 1);
    return j;
}

/**
 * @brief Table linear interpolation in 1 dimension.
 *
 * - @f$ \mu \gets (t - t_{[j - 1]}) / (t_{[j]} - t_{[j - 1]}) @f$
 * - @f$ (1 - \mu) f_{[j - 1]} + \mu f_{[j]} @f$
 *
 * @pre
 * - @f$ n \ge 2 @f$
 * - @f$ t_{[j - 1]} \le t_{[j]} @f$ for all @f$ j \in [1, n) @f$
 */
template <
    typename T,
    typename Tforward_itr,
    typename Uforward_itr,
    typename U = typename
        std::iterator_traits<Uforward_itr>::value_type
    >
inline std::enable_if_t<
       std::is_floating_point<T>::value,
       decltype(T() * U())> table_lerp1(
                const T& t,
                Tforward_itr tfrom, Tforward_itr tto,
                Uforward_itr ffrom)
{
    auto j = pr::table_hunt(t, tfrom, tto);
    T t0 = *std::next(tfrom, j - 1), t1 = *std::next(tfrom, j);
    U f0 = *std::next(ffrom, j - 1), f1 = *std::next(ffrom, j);
    return pr::lerp((t - t0) / (t1 - t0), f0, f1);
}

/**
 * @brief Table linear interpolation in 2 dimensions.
 *
 * - @f$ \mu \gets (s - s_{[i - 1]}) / (s_{[i]} - s_{[i - 1]}) @f$
 * - @f$ \nu \gets (t - t_{[j - 1]}) / (t_{[j]} - t_{[j - 1]}) @f$
 * - @f$ f_0 \gets (1 - \nu) f_{[i - 1, j - 1]} + \nu f_{[i - 1, j]} @f$
 * - @f$ f_1 \gets (1 - \nu) f_{[i, j - 1]} + \nu f_{[i, j]} @f$
 * - @f$ (1 - \mu) f_0 + \mu f_1 @f$
 *
 * @pre
 * - @f$ m \ge 2 @f$
 * - @f$ n \ge 2 @f$
 * - @f$ s_{[i - 1]} \le s_{[i]} @f$ for all @f$ i \in [1, m) @f$
 * - @f$ t_{[j - 1]} \le t_{[j]} @f$ for all @f$ j \in [1, n) @f$
 */
template <
    typename T,
    typename Tforward_itr,
    typename Uforward_itr,
    typename U = typename
        std::iterator_traits<Uforward_itr>::value_type
    >
inline std::enable_if_t<
       std::is_floating_point<T>::value, 
       decltype(T() * U())> table_lerp2(
                const T& s,
                const T& t,
                Tforward_itr sfrom, Tforward_itr sto,
                Tforward_itr tfrom, Tforward_itr tto,
                Uforward_itr ffrom)
{
    auto n = std::distance(tfrom, tto);
    auto i = pr::table_hunt(s, sfrom, sto);
    auto j = pr::table_hunt(t, tfrom, tto);
    T s0 = *std::next(sfrom, i - 1), s1 = *std::next(sfrom, i);
    T t0 = *std::next(tfrom, j - 1), t1 = *std::next(tfrom, j);
    T mu = (s - s0) / (s1 - s0);
    T nu = (t - t0) / (t1 - t0);
    U f00 = *std::next(ffrom, (i - 1) * n + j - 1);
    U f01 = *std::next(ffrom, (i - 1) * n + j);
    U f10 = *std::next(ffrom, i * n + j - 1);
    U f11 = *std::next(ffrom, i * n + j);
    return pr::lerp(mu,
            pr::lerp(nu, f00, f01),
            pr::lerp(nu, f10, f11));
}

/**
 * @brief Table trapezoidal integration in 1 dimension.
 *
 * - @f$ g_{[0]} \gets 0 @f$
 * - @f$ g_{[j]} \gets
 *       g_{[j - 1]} +
 *          (t_{[j]} - t_{[j - 1]})
 *          (f_{[j]} + f_{[j - 1]}) / 2 @f$
 */
template <
    typename Tforward_itr,
    typename Toutput_itr,
    typename T = typename 
        std::iterator_traits<Toutput_itr>::value_type
    >
inline std::enable_if_t<
       std::is_floating_point<T>::value, 
       void> table_int1(
                Tforward_itr tfrom, Tforward_itr tto,
                Tforward_itr ffrom,
                Toutput_itr gfrom)
{
    if (tfrom != tto) {
        pr::neumaier_sum<T> fint(0);
        auto itrt0 = tfrom, itrt1 = std::next(tfrom);
        auto itrf0 = ffrom, itrf1 = std::next(ffrom);
        auto itrg = gfrom;
        *itrg++ = T(fint);
        for (; itrt1 != tto; 
                ++itrt0, ++itrt1, 
                ++itrf0, ++itrf1,
                ++itrg) {
            *itrg = T(fint +=
                    (*itrt1 - *itrt0) *
                    (*itrf1 + *itrf0) * T(0.5));
        }
    }
}

/**
 * @brief Table summation in 1 dimension.
 *
 * - @f$ g_{[0]} \gets f_{[0]} @f$
 * - @f$ g_{[j]} \gets g_{[j - 1]} + f_{[j]} @f$
 */
template <
    typename Tforward_itr,
    typename Toutput_itr,
    typename T = typename 
        std::iterator_traits<Toutput_itr>::value_type
    >
inline std::enable_if_t<
       std::is_floating_point<T>::value, 
       void> table_sum1(
                Tforward_itr ffrom, Tforward_itr fto,
                Toutput_itr gfrom)
{
    pr::neumaier_sum<T> fsum(0);
    auto itrf = ffrom;
    auto itrg = gfrom;
    for (; itrf != fto; ++itrf, ++itrg) {
        *itrg = T(fsum += *itrf);
    }
}

/**@}*/

} // namespace pr

#endif // #ifndef PREFORM_TABULATION_HPP
