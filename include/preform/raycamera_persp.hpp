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
#error "preform/raycamera_persp.hpp requires >=C++17"
#endif // #if !(__cplusplus >= 201703L)
#endif // #if !DOXYGEN
#pragma once
#ifndef PREFORM_RAYCAMERA_PERSP_HPP
#define PREFORM_RAYCAMERA_PERSP_HPP

// for pr::multi
#include <preform/multi.hpp>

// for pr::multi wrappers
#include <preform/multi_math.hpp>

namespace pr {

/**
 * @defgroup raycamera_persp Ray-camera (perspective)
 *
 * `<preform/raycamera_persp.hpp>`
 *
 * __C++ version__: >=C++17
 */
/**@{*/

/**
 * @brief Ray-camera (perspective).
 *
 * @image html raycamera_persp.svg
 *
 * @tparam T
 * Float type.
 */
template <typename T>
struct raycamera_persp
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
     * @brief Default constructor.
     */
    raycamera_persp() = default;

    /**
     * @brief Constructor.
     *
     * @param[in] o
     * Origin.
     *
     * @param[in] hatx
     * Basis direction.
     *
     * @param[in] haty
     * Basis direction.
     *
     * @param[in] hatz
     * Basis direction.
     *
     * @param[in] w
     * Screen width.
     *
     * @param[in] h
     * Screen height.
     *
     * @param[in] f
     * Y-FOV in radians.
     *
     * @param[in] qmin
     * Sub-region bounding point.
     *
     * @param[in] qmax
     * Sub-region bounding point.
     *
     * @throw std::invalid_argument
     * Unless
     * - `w > 0`,
     * - `h > 0`, 
     * - `f > 0 && f < pi`,
     * - `qmin[0] < qmax[0]`, and
     * - `qmin[1] < qmax[1]`.
     */
    raycamera_persp(
        const multi<float_type, 3>& o,
        const multi<float_type, 3>& hatx,
        const multi<float_type, 3>& haty,
        const multi<float_type, 3>& hatz,
        float_type w,
        float_type h,
        float_type f,
        const multi<float_type, 2>& qmin = {0, 0},
        const multi<float_type, 2>& qmax = {1, 1}) :
            o_(o),
            hatx_(hatx),
            haty_(haty),
            hatz_(hatz),
            w_(w),
            h_(h),
            l_(h / (2 * pr::tan(f / 2))),
            qmin_(qmin),
            qmax_(qmax)
    {
        if (!(w > 0 &&
              h > 0 &&
              f > 0 &&
              f < pr::numeric_constants<float_type>::M_pi() &&
              qmin[0] < qmax[0] &&
              qmin[1] < qmax[1])) {
            throw std::invalid_argument(__PRETTY_FUNCTION__);
        }
    }

    /**
     * @brief Look-at.
     *
     * - @f$ \mathbf{o} \gets \mathbf{p}_0 @f$
     * - @f$ \hat{\mathbf{z}} \gets \normalize(\mathbf{p}_0 - \mathbf{p}_1) @f$
     * - @f$ \hat{\mathbf{x}} \gets \normalize(
     *           -\mathbf{y}_0\times \hat{\mathbf{z}}) @f$
     * - @f$ \hat{\mathbf{y}} \gets 
     *       \hat{\mathbf{z}} \times \hat{\mathbf{x}} @f$
     *
     * @param[in] p0
     * Point from.
     *
     * @param[in] p1
     * Point to.
     *
     * @param[in] y0
     * Up vector.
     *
     * @param[in] w
     * Screen width.
     *
     * @param[in] h
     * Screen height.
     *
     * @param[in] f
     * Y-FOV in radians.
     *
     * @param[in] qmin
     * Sub-region bounding point.
     *
     * @param[in] qmax
     * Sub-region boundint point.
     *
     * @note
     * For convenience, the implementation crosses with @f$ -\mathbf{y}_0 @f$, 
     * such that the camera coordinate system is upside-down (since pixel 
     * coordinates increase downward).
     */
    static raycamera_persp look_at(
                const multi<float_type, 3>& p0,
                const multi<float_type, 3>& p1,
                const multi<float_type, 3>& y0,
                float_type w,
                float_type h,
                float_type f,
                const multi<float_type, 2>& qmin = {0, 0},
                const multi<float_type, 2>& qmax = {1, 1})
    {
        multi<float_type, 3> o = p0;
        multi<float_type, 3> hatz = normalize(p1 - p0);
        multi<float_type, 3> hatx = normalize(cross(-y0, hatz));
        multi<float_type, 3> haty = cross(hatz, hatx);
        return {
            o, 
            hatx, 
            haty, 
            hatz, 
            w, h, 
            f,
            qmin,
            qmax
        };
    }

    // TODO importance functions
    // TODO sampling 

private:

    /**
     * @brief Origin @f$ \mathbf{o} @f$.
     */
    multi<float_type, 3> o_;

    /**
     * @brief Basis direction @f$ \hat{\mathbf{x}} @f$.
     */
    multi<float_type, 3> hatx_;

    /**
     * @brief Basis direction @f$ \hat{\mathbf{y}} @f$.
     */
    multi<float_type, 3> haty_;

    /**
     * @brief Basis direction @f$ \hat{\mathbf{z}} @f$.
     */
    multi<float_type, 3> hatz_;

    /**
     * @brief Screen width @f$ w @f$.
     */
    float_type w_ = 0;

    /**
     * @brief Screen height @f$ h @f$.
     */
    float_type h_ = 0;

    /**
     * @brief Pinhole distance @f$ l @f$.
     */
    float_type l_ = 0;

    /**
     * @brief Sub-region bounding point @f$ \mathbf{q}_{\min} @f$.
     */
    multi<float_type, 2> qmin_ = {0, 0};

    /**
     * @brief Sub-region bounding point @f$ \mathbf{q}_{\max} @f$.
     */
    multi<float_type, 2> qmax_ = {1, 1};
};

/**@}*/

} // namespace pr

#endif // #ifndef PREFORM_RAYCAMERA_PERSP_HPP
