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
#error "preform/multi_random.hpp requires >=C++17"
#endif // #if !(__cplusplus >= 201703L)
#endif // #if !DOXYGEN
#pragma once
#ifndef PREFORM_MULTI_RANDOM_HPP
#define PREFORM_MULTI_RANDOM_HPP

// for pr::generate_canonical, ...
#include <preform/random.hpp>

// for pr::multi
#include <preform/multi.hpp>

namespace pr {

/**
 * @defgroup multi_random Multi-dimensional array (random)
 *
 * `<preform/multi_random.hpp>`
 *
 * __C++ version__: >=C++17
 */
/**@{*/

/**
 * @brief Generate multi-array of canonical random samples.
 */
template <
    typename T, std::size_t M, std::size_t... N,
    typename G
    >
inline std::enable_if_t<
       std::is_floating_point<T>::value,
            multi<T, M, N...>> generate_canonical(G&& gen)
{
    multi<T, M, N...> u;
    for (auto& uk : u) {
        uk = pr::generate_canonical<T, N...>(std::forward<G>(gen));
    }
    return u;
}

/**
 * @brief Generate and stratify canonical random samples in @f$ N @f$
 * dimensions.
 *
 * @param[out] arr
 * Sample array, must point to `dim.prod()` contiguous elements.
 *
 * @param[in] dim
 * Sample array dimensions.
 *
 * @param[inout] gen
 * Generator.
 *
 * @throw std::invalid_argument
 * If `arr == nullptr` or `(dim <= 0).any()`.
 */
template <
    typename T, std::size_t N,
    typename P,
    typename G
    >
inline std::enable_if_t<
       std::is_floating_point<T>::value &&
       std::is_integral<P>::value,
            void> stratify(
                        multi<T, N>* arr,
                        multi<P, N> dim,
                        G&& gen)
{
    if (arr == nullptr || (dim <= P(0)).any()) {
        throw std::invalid_argument(__PRETTY_FUNCTION__);
    }

    // Stratify.
    multi<P, N> pos = {};
    multi<T, N>* itrarr = arr;
    do {
        for (T& val : *itrarr) {
            val = pr::generate_canonical<T>(std::forward<G>(gen));
        }
        *itrarr += pos;
        *itrarr /= dim;
        ++itrarr;

        // Increment.
        auto itrpos = pos.rbegin();
        auto itrdim = dim.rbegin();
        for (; itrpos < pos.rend(); ++itrpos, ++itrdim) {
            ++*itrpos;
            if (*itrpos >= *itrdim) {
                *itrpos = 0;
            }
            else {
                break;
            }
        }
    }
    while ((pos != P(0)).any());

    // Shuffle into random order.
    std::shuffle(
        arr,
        arr + dim.prod(),
        std::forward<G>(gen));
}

/**
 * @name Multiple-importance
 */
/**@{*/

/**
 * @brief Multiple-importance balance heuristic.
 *
 * @f[
 *      w_k = \frac{f_k}{f_k + \sum_{\ell \ne k} f_\ell}
 * @f]
 *
 * @note
 * This is included here only because it requires C++17
 * fold expressions, else it would be included in `preform/random.hpp`.
 */
template <typename Tk, typename... Tl>
inline std::enable_if_t<
       (std::is_floating_point<Tk>::value && ... &&
        std::is_floating_point<Tl>::value),
        std::common_type_t<Tk, Tl...>> balance_heuristic(Tk fk, Tl... fl)
{
    return fk / (fk + ... + fl);
}

/**
 * @brief Multiple-importance power heuristic.
 *
 * @f[
 *      w_k = \frac{f_k^2}{f_k^2 + \sum_{\ell \ne k} f_\ell^2}
 * @f]
 *
 * @note
 * This is included here only because it requires C++17
 * fold expressions, else it would be included in `preform/random.hpp`.
 */
template <typename Tk, typename... Tl>
inline std::enable_if_t<
       (std::is_floating_point<Tk>::value && ... &&
        std::is_floating_point<Tl>::value),
        std::common_type_t<Tk, Tl...>> power_heuristic(Tk fk, Tl... fl)
{
    return (fk * fk) / ((fk * fk) + ... + (fl * fl));
}

/**@}*/

/**@}*/

} // namespace pr

#endif // #ifndef PREFORM_MULTI_RANDOM_HPP
