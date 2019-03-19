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
#error "preform/periodic_noise_adapter1.hpp requires >=C++17"
#endif // #if !(__cplusplus >= 201703L)
#endif // #if !DOXYGEN
#pragma once
#ifndef PREFORM_PERIODIC_NOISE_ADAPTER1_HPP
#define PREFORM_PERIODIC_NOISE_ADAPTER1_HPP

// for std::forward
#include <utility>

// for pr::multi
#include <preform/multi.hpp>

// for pr::multi wrappers
#include <preform/multi_math.hpp>

namespace pr {

/**
 * @defgroup periodic_noise1 Periodic noise adapter (1-dimensional)
 *
 * `<preform/periodic_noise_adapter1.hpp>`
 *
 * __C++ version__: >=C++17
 */
/**@{*/

/**
 * @brief Periodic noise adapter (1-dimensional).
 *
 * Periodic noise adapter which evaluates a 2-dimensional noise
 * function on the contour of a circle.
 */
template <typename Tnoise2>
class periodic_noise_adapter1 : public Tnoise2
{
public:

    /**
     * @brief Float type.
     */
    typedef typename Tnoise2::float_type float_type;

    /**
     * @brief Default constructor.
     */
    periodic_noise_adapter1() = default;

    /**
     * @brief Constructor.
     *
     * @param[in] c0
     * Center.
     *
     * @param[in] r0
     * Radius.
     *
     * @param[in] t0
     * Period.
     */
    template <typename... Targs>
    periodic_noise_adapter1(
            multi<float_type, 2> c0,
            float_type r0,
            float_type t0,
            Targs&&... args) :
                Tnoise2(std::forward<Targs>(args)...),
                c0_(c0),
                r0_(r0),
                t0_(t0)
    {
    }

    /**
     * @brief Evaluate.
     *
     * @param[in] t
     * Coordinate.
     *
     * @param[out] ds_dt
     * Noise derivative. _Optional_.
     */
    float_type evaluate(
                float_type t,
                float_type* ds_dt = nullptr) const
    {
        // Circle.
        float_type pi = pr::numeric_constants<float_type>::M_pi();
        float_type omega = 2 * pi / t0_;
        float_type cos_omegat = pr::cos(omega * t);
        float_type sin_omegat = pr::sin(omega * t);
        multi<float_type, 2> u = {
            c0_[0] + r0_ * cos_omegat,
            c0_[1] + r0_ * sin_omegat
        };

        // Delegate.
        multi<float_type, 2> ds_du;
        float_type s = Tnoise2::evaluate(u, ds_dt ? &ds_du : nullptr);
        if (ds_dt) {

            // Chain rule.
            *ds_dt =
                ds_du[0] * (r0_ * omega) * -sin_omegat +
                ds_du[1] * (r0_ * omega) * +cos_omegat;
        }
        return s;
    }

private:

    /**
     * @brief Center @f$ \mathbf{c}_0 @f$.
     */
    multi<float_type, 2> c0_ = {};

    /**
     * @brief Radius @f$ r_0 @f$.
     */
    float_type r0_ = 1;

    /**
     * @brief Period @f$ t_0 @f$.
     */
    float_type t0_ = 1;

public:

    /**
     * @name Accessors
     */
    /**@{*/

    /**
     * @brief Get center.
     */
    __attribute__((always_inline))
    multi<float_type, 2> center() const
    {
        return c0_;
    }

    /**
     * @brief Set center, return previous center.
     */
    __attribute__((always_inline))
    multi<float_type, 2> center(multi<float_type, 2> val)
    {
        multi<float_type, 2> c0 = c0_; c0_ = val; return c0;
    }

    /**
     * @brief Get radius.
     */
    __attribute__((always_inline))
    float_type radius() const
    {
        return r0_;
    }

    /**
     * @brief Set radius, return previous radius.
     */
    __attribute__((always_inline))
    float_type radius(float_type val)
    {
        float_type r0 = r0_; r0_ = val; return r0;
    }

    /**
     * @brief Get period.
     */
    __attribute__((always_inline))
    float_type period() const
    {
        return t0_;
    }

    /**
     * @brief Set period, return previous period.
     */
    __attribute__((always_inline))
    float_type period(float_type val)
    {
        float_type t0 = t0_; t0_ = val; return t0;
    }

    /**@}*/
};

// Prototype
#if !DOXYGEN
    template <typename> class simplex_noise2;
#endif // #if !DOXYGEN

/**
 * @brief Template alias for convenience.
 *
 * @note
 * Requires that client code includes `<preform/simplex_noise2.hpp>`.
 */
template <typename T>
using periodic_simplex_noise1 =
      periodic_noise_adapter1<simplex_noise2<T>>;

// Prototype
#if !DOXYGEN
    template <typename> class worley_noise2;
#endif // #if !DOXYGEN

/**
 * @brief Template alias for convenience.
 *
 * @note
 * Requires that client code includes `<preform/worley_noise2.hpp>`.
 */
template <typename T>
using periodic_worley_noise1 =
      periodic_noise_adapter1<worley_noise2<T>>;

/**@}*/

} // namespace pr

#endif // #ifndef PREFORM_PERIODIC_NOISE_ADAPTER1_HPP
