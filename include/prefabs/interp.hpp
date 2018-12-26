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
#if !DOXYGEN
#if !(__cplusplus >= 201402L)
#error "prefabs/interp.hpp requires >=C++14"
#endif // #if !(__cplusplus >= 201402L)
#endif // #if !DOXYGEN
#pragma once
#ifndef PREFABS_INTERP_HPP
#define PREFABS_INTERP_HPP

namespace pr {

/**
 * @defgroup interp Interpolation
 *
 * `<prefabs/interp.hpp>`
 *
 * __C++ version__: >=C++14
 */
/**@{*/

/**
 * @brief Generic linear interpolation.
 *
 * @param [in] t 
 * Factor.
 *
 * @param [in] x 
 * Value at @f$ t = 0 @f$.
 *
 * @param [in] y 
 * Value at @f$ t = 1 @f$.
 */
template <
    typename T, 
    typename U
    >
constexpr auto lerp(T t, const U& x, const U& y)
{
    return (1 - t) * x + t * y;
}

/**
 * @brief Generic cubic Hermite interpolation.
 *
 * @param [in] t 
 * Factor.
 *
 * @param [in] x 
 * Value at @f$ t = 0 @f$.
 *
 * @param [in] u 
 * Slope at @f$ t = 0 @f$.
 *
 * @param [in] v 
 * Slope at @f$ t = 1 @f$.
 *
 * @param [in] y 
 * Value at @f$ t = 1 @f$.
 *
 * @see Wikipedia's article for [Cubic Hermite spline][1].
 * [1]: https://en.wikipedia.org/wiki/Cubic_Hermite_spline
 */
template <
    typename T,
    typename U
    >
constexpr auto hermite(T t, const U& x, const U& u, const U& v, const U& y)
{
    T s = 1 - t;
    return (s * s * (1 + 2 * t)) * x + (s * s * t) * u +
           (t * t * (3 - 2 * t)) * y - (t * t * s) * v;
}

/**
 * @brief Generic cubic Catmull-Rom interpolation.
 *
 * @param [in] t 
 * Factor.
 *
 * @param [in] w 
 * Antecedent value at @f$ t = 0 @f$.
 *
 * @param [in] x 
 * Value at @f$ t = 0 @f$.
 *
 * @param [in] y 
 * Value at @f$ t = 1 @f$.
 *
 * @param [in] z
 * Subsequent value at @f$ t = 1 @f$.
 *
 * @see Wikipedia's article for [Cubic Hermite spline][1].
 * [1]: https://en.wikipedia.org/wiki/Cubic_Hermite_spline#Catmull–Rom_spline
 */
template <
    typename T,
    typename U
    >
constexpr auto catmull(T t, const U& w, const U& x, const U& y, const U& z)
{
    return hermite(t, x, y - w, z - x, y);
}

/**@}*/

} // namespace pr

#endif // #ifndef PREFABS_INTERP_HPP
