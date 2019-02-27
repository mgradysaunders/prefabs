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
#pragma once
#ifndef PREFORM_PERIODIC_NOISE_ADAPTER2_HPP
#define PREFORM_PERIODIC_NOISE_ADAPTER2_HPP

// for std::forward
#include <utility>

// for pr::multi
#include <preform/multi.hpp>

// for pr::multi wrappers
#include <preform/multi_math.hpp>

namespace pr {

/**
 * @defgroup periodic_noise2 Periodic noise adapter (2-dimensional)
 *
 * `<preform/periodic_noise_adapter2.hpp>`
 *
 * __C++ version__: >=C++17
 */
/**@{*/

/**
 * @brief Periodic noise adapter (2-dimensional).
 *
 * Periodic noise adapter which evaluates a 3-dimensional noise 
 * function on the surface of a torus.
 */
template <typename Tnoise3>
class periodic_noise_adapter2 : public Tnoise3
{
public:

    /**
     * @brief Float type.
     */
    typedef typename Tnoise3::float_type float_type;

    /**
     * @brief Default constructor.
     */
    periodic_noise_adapter2() = default;

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
    periodic_noise_adapter2(
            multi<float_type, 3> c0,
            multi<float_type, 2> r0,
            multi<float_type, 2> t0,
            Targs&&... args) :
                Tnoise3(std::forward<Targs>(args)...),
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
     * Noise partial derviatives. _Optional_.
     */
    float_type evaluate(
                multi<float_type, 2> t,
                multi<float_type, 2>* ds_dt = nullptr) const
    {
        // Torus.
        float_type pi = pr::numeric_constants<float_type>::M_pi();
        multi<float_type, 2> omega = 2 * pi / t0_;
        multi<float_type, 2> cos_omegat = pr::cos(omega * t);
        multi<float_type, 2> sin_omegat = pr::sin(omega * t);
        multi<float_type, 3> u = {
            c0_[0] + (r0_[0] + r0_[1] * cos_omegat[1]) * cos_omegat[0],
            c0_[1] + (r0_[0] + r0_[1] * cos_omegat[1]) * sin_omegat[0],
            c0_[2] + r0_[1] * sin_omegat[1]
        };

        // Delegate.
        multi<float_type, 3> ds_du;
        float_type s = Tnoise3::evaluate(u, ds_dt ? &ds_du : nullptr);
        if (ds_dt) {

            // Torus partial derivatives.
            float_type tmp0 = omega[0] * (r0_[0] + r0_[1] * cos_omegat[1]);
            float_type tmp1 = omega[1] * r0_[1];
            multi<float_type, 3, 2> du_dt = {
                {-tmp0 * sin_omegat[0], -tmp1 * sin_omegat[1] * cos_omegat[0]},
                {+tmp0 * cos_omegat[0], -tmp1 * sin_omegat[1] * sin_omegat[0]},
                {0, +tmp1 * cos_omegat[1]}
            };

            // Chain rule.
            *ds_dt = pr::dot(ds_du, du_dt);
        }
        return s;
    }

private:

    /**
     * @brief Center @f$ \mathbf{c}_0 @f$.
     */
    multi<float_type, 3> c0_ = {};

    /**
     * @brief Radius @f$ \mathbf{r}_0 @f$.
     */
    multi<float_type, 2> r0_ = {4, 1};

    /**
     * @brief Period @f$ \mathbf{t}_0 @f$.
     */
    multi<float_type, 2> t0_ = {4, 1};

public:

    /**
     * @name Accessors
     */
    /**@{*/

    /**
     * @brief Get center.
     */
    __attribute__((always_inline))
    multi<float_type, 3> center() const
    {
        return c0_;
    }

    /**
     * @brief Set center, return previous center.
     */
    __attribute__((always_inline))
    multi<float_type, 3> center(multi<float_type, 3> val)
    {
        multi<float_type, 3> c0 = c0_; c0_ = val; return c0;
    }

    /**
     * @brief Get radius.
     */
    __attribute__((always_inline))
    multi<float_type, 2> radius() const
    {
        return r0_;
    }

    /**
     * @brief Set radius, return previous radius.
     */
    __attribute__((always_inline))
    multi<float_type, 2> radius(multi<float_type, 2> val)
    {
        multi<float_type, 2> r0 = r0_; r0_ = val; return r0;
    }

    /**
     * @brief Get period.
     */
    __attribute__((always_inline))
    multi<float_type, 2> period() const
    {
        return t0_;
    }

    /**
     * @brief Set period, return previous period.
     */
    __attribute__((always_inline))
    multi<float_type, 2> period(multi<float_type, 2> val)
    {
        multi<float_type, 2> t0 = t0_; t0_ = val; return t0;
    }

    /**@}*/
};

// Prototype
#if !DOXYGEN
    template <typename> class simplex_noise3;
#endif // #if !DOXYGEN

/**
 * @brief Template alias for convenience.
 *
 * @note
 * Requires that client code includes `<preform/simplex_noise3.hpp>`.
 */
template <typename T>
using periodic_simplex_noise2 = 
      periodic_noise_adapter2<simplex_noise3<T>>;

// Prototype
#if !DOXYGEN
    template <typename> class worley_noise3;
#endif // #if !DOXYGEN

/**
 * @brief Template alias for convenience.
 *
 * @note
 * Requires that client code includes `<preform/worley_noise3.hpp>`.
 */
template <typename T>
using periodic_worley_noise2 = 
      periodic_noise_adapter2<worley_noise3<T>>;

/**@}*/

} // namespace pr

#endif // #ifndef PREFORM_PERIODIC_NOISE_ADAPTER2_HPP
