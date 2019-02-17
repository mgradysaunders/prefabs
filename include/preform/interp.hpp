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
#error "preform/interp.hpp requires >=C++14"
#endif // #if !(__cplusplus >= 201402L)
#endif // #if !DOXYGEN
#pragma once
#ifndef PREFORM_INTERP_HPP
#define PREFORM_INTERP_HPP

namespace pr {

/**
 * @defgroup interp Interpolation
 *
 * `<preform/interp.hpp>`
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
 * @param [in] p0
 * Control point at @f$ t = 0 @f$.
 *
 * @param [in] p1
 * Control point at @f$ t = 1 @f$.
 */
template <
    typename Tvalue, 
    typename Tcontrol
    >
__attribute__((always_inline))
constexpr auto lerp(
            Tvalue t, 
            const Tcontrol& p0, 
            const Tcontrol& p1)
{
    return (1 - t) * p0 + t * p1;
}

/**
 * @brief Generic cubic Hermite interpolation.
 *
 * @param [in] t 
 * Factor.
 *
 * @param [in] p0
 * Control point at @f$ t = 0 @f$.
 *
 * @param [in] m0
 * Slope at @f$ t = 0 @f$.
 *
 * @param [in] m1
 * Slope at @f$ t = 1 @f$.
 *
 * @param [in] p1 
 * Control point at @f$ t = 1 @f$.
 *
 * @see Wikipedia's article for [Cubic Hermite spline][1].
 * [1]: https://en.wikipedia.org/wiki/Cubic_Hermite_spline
 */
template <
    typename Tvalue,
    typename Tcontrol
    >
constexpr auto hermite(
            Tvalue t, 
            const Tcontrol& p0, 
            const Tcontrol& m0, 
            const Tcontrol& m1, 
            const Tcontrol& p1)
{
    Tvalue s = t - 1;
    Tvalue h00 = s * s * (1 + 2 * t), h10 = s * s * t;
    Tvalue h01 = t * t * (3 - 2 * t), h11 = t * t * s;
    return (h00 * p0 + h10 * m0) +
           (h01 * p1 + h11 * m1);
}

/**
 * @brief Generate cubic Hermite interpolation deriviatve.
 *
 * @param [in] t 
 * Factor.
 *
 * @param [in] p0
 * Control point at @f$ t = 0 @f$.
 *
 * @param [in] m0
 * Slope at @f$ t = 0 @f$.
 *
 * @param [in] m1
 * Slope at @f$ t = 1 @f$.
 *
 * @param [in] p1 
 * Control point at @f$ t = 1 @f$.
 */
template <
    typename Tvalue,
    typename Tcontrol
    >
constexpr auto hermite_deriv(
            Tvalue t, 
            const Tcontrol& p0, 
            const Tcontrol& m0, 
            const Tcontrol& m1, 
            const Tcontrol& p1)
{
    Tvalue g00 = 6 * t * (t - 1);
    Tvalue g10 = 3 * t * t - 4 * t + 1;
    Tvalue g11 = 3 * t * t - 2 * t;
    return g00 * (p0 - p1) + g10 * m0 + g11 * m1;
}

/**
 * @brief Generic cubic Catmull-Rom interpolation.
 *
 * @param [in] t 
 * Factor.
 *
 * @param [in] pprev
 * Previous control point.
 *
 * @param [in] p0
 * Control point at @f$ t = 0 @f$.
 *
 * @param [in] p1
 * Control point at @f$ t = 1 @f$.
 *
 * @param [in] pnext
 * Next control point.
 *
 * @see Wikipedia's article for [Cubic Hermite spline][1].
 * [1]: https://en.wikipedia.org/wiki/Cubic_Hermite_spline#Catmull–Rom_spline
 */
template <
    typename Tvalue,
    typename Tcontrol
    >
__attribute__((always_inline))
constexpr auto catmull(
            Tvalue t, 
            const Tcontrol& pprev, 
            const Tcontrol& p0, 
            const Tcontrol& p1, 
            const Tcontrol& pnext)
{
    return hermite(t, p0, p1 - pprev, pnext - p0, p1);
}

/**
 * @brief Generic cubic Catmull-Rom interpolation derivative.
 *
 * @param [in] t 
 * Factor.
 *
 * @param [in] pprev
 * Previous control point.
 *
 * @param [in] p0
 * Control point at @f$ t = 0 @f$.
 *
 * @param [in] p1
 * Control point at @f$ t = 1 @f$.
 *
 * @param [in] pnext
 * Next control point.
 *
 * @see Wikipedia's article for [Cubic Hermite spline][1].
 * [1]: https://en.wikipedia.org/wiki/Cubic_Hermite_spline#Catmull–Rom_spline
 */
template <
    typename Tvalue,
    typename Tcontrol
    >
__attribute__((always_inline))
constexpr auto catmull_deriv(
            Tvalue t, 
            const Tcontrol& pprev, 
            const Tcontrol& p0, 
            const Tcontrol& p1, 
            const Tcontrol& pnext)
{
    return hermite_deriv(t, p0, p1 - pprev, pnext - p0, p1);
}

/**@}*/

} // namespace pr

#endif // #ifndef PREFORM_INTERP_HPP
