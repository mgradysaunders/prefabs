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
#error "preform/image2.hpp requires >=C++17"
#endif // #if !(__cplusplus >= 201703L)
#endif // #if !DOXYGEN
#pragma once
#ifndef PREFORM_IMAGE2_HPP
#define PREFORM_IMAGE2_HPP

// for pr::block_array2
#include <preform/block_array2.hpp>

// for pr::fclamp, pr::frepeat, ...
#include <preform/float_helpers.hpp>

// for pr::fclamp, pr::frepeat, ...
#include <preform/multi_float_helpers.hpp>

// for pr::lerp, pr::catmull
#include <preform/interp.hpp>

namespace pr {

/**
 * @defgroup image2 Image (2-dimensional)
 *
 * `<preform/image2.hpp>`
 *
 * __C++ version__: >=C++17
 */
/**@{*/

/**
 * @brief Image (2-dimensional).
 */
template <
    typename Tfloat, 
    typename T, std::size_t N, 
    typename Talloc = std::allocator<T>
    >
class image2 : public block_array2<multi<T, N>, 
                            typename std::allocator_traits<Talloc>::
                            template rebind_alloc<multi<T, N>>>
{
public:

    // Sanity check.
    static_assert(
        std::is_floating_point<Tfloat>::value,
        "Tfloat must be floating point");

    // Sanity check.
    static_assert(
        std::is_arithmetic<T>::value,
        "T must be arithmetic");

    /**
     * @brief Float type.
     */
    typedef Tfloat float_type;

    /**
     * @brief Entry type.
     */
    typedef T entry_type;

#if !DOXYGEN
    /**
     * @brief Base type.
     */
    typedef block_array2<multi<T, N>,
            typename std::allocator_traits<Talloc>::
            template rebind_alloc<multi<T, N>>> base;
#endif // #if !DOXYGEN

    /**
     * @brief Size type.
     */
    typedef typename base::size_type size_type;

public:

    // Inherit constructors.
    using base::base;

public:

    /**
     * @name Sampling
     */
    /**@{*/

    /**
     * @brief Average.
     *
     * @param[in] locmin
     * Location minimum in index coordinates.
     *
     * @param[in] locmax
     * Location maximum in index coordinates.
     *
     * @param[in] cyc_mode
     * Cycle modes.
     */
    multi<float_type, N> average(
            multi<float_type, 2> locmin,
            multi<float_type, 2> locmax,
            multi<cycle_mode, 2> cyc_mode = 
            multi<cycle_mode, 2>::value(cycle_mode::clamp)) const
    {
        if (this->empty()) {
            return {};
        }
        else if ((locmin == locmax).all()) {
            return sample0(locmin, cyc_mode);
        }
        else {

            // Shift.
            locmin -= float_type(0.5);
            locmax -= float_type(0.5);
            if (locmin[0] > locmax[0]) std::swap(locmin[0], locmax[0]);
            if (locmin[1] > locmax[1]) std::swap(locmin[1], locmax[1]);

            // Floor.
            multi<float_type, 2> loc0 = pr::floor(locmin);
            multi<float_type, 2> loc1 = pr::floor(locmax) + 1;

            // Integrate.
            multi<float_type, N> numer = {};
            float_type denom = 0;
            for (int i = int(loc0[0]); i < int(loc1[0]); i++)
            for (int j = int(loc0[1]); j < int(loc1[1]); j++) {
                multi<float_type, 2> cur = {{
                    float_type(i),
                    float_type(j)
                }};
                multi<float_type, 2> cur0 = pr::fmax(locmin, cur);
                multi<float_type, 2> cur1 = pr::fmin(locmax, cur + 1);
                numer += (cur1 - cur0).prod() * lookup(cur, cyc_mode);
                denom += (cur1 - cur0).prod();
            }

            // Average.
            return numer / denom;
        }
    }

    /**
     * @brief Sample, no interpolation.
     *
     * @param[in] loc
     * Location in index coordinates.
     *
     * @param[in] cyc_mode
     * Cycle modes.
     */
    multi<float_type, N> sample0(
            multi<float_type, 2> loc, 
            multi<cycle_mode, 2> cyc_mode =
            multi<cycle_mode, 2>::value(cycle_mode::clamp)) const
    {
        if (this->empty()) {
            return {};
        }
        else {
            // Shift.
            loc -= float_type(0.5);
            return lookup(loc, cyc_mode);
        }
    }

    /**
     * @brief Sample, linear interpolation.
     *
     * @param[in] loc
     * Location in index coordinates.
     *
     * @param[in] cyc_mode
     * Cycle modes.
     */
    multi<float_type, N> sample1(
            multi<float_type, 2> loc,
            multi<cycle_mode, 2> cyc_mode =
            multi<cycle_mode, 2>::value(cycle_mode::clamp)) const
    {
        if (this->empty()) {
            return {};
        }
        else {
            // Shift.
            loc -= float_type(0.5);

            // Floor.
            multi<float_type, 2> loc0 = pr::floor(loc);
            loc -= loc0;

            // Interpolate.
            multi<float_type, N> val0[2];
            for (size_type i = 0; i < 2; i++) {

                multi<float_type, N> val1[2];
                for (size_type j = 0; j < 2; j++) {
                    val1[j] = 
                        lookup(loc0 + 
                        multi<float_type, 2>{{
                            float_type(i),
                            float_type(j)
                        }}, cyc_mode);
                }

                // Linear interpolation.
                val0[i] = lerp(loc[1], val1[0], val1[1]);
            }

            // Linear interpolation.
            return lerp(loc[0], val0[0], val0[1]);
        }
    }

    /**
     * @brief Sample, cubic interpolation.
     *
     * @param[in] loc
     * Location in index coordinates.
     *
     * @param[in] cyc_mode
     * Cycle modes.
     */
    multi<float_type, N> sample2(
            multi<float_type, 2> loc,
            multi<cycle_mode, 2> cyc_mode =
            multi<cycle_mode, 2>::value(cycle_mode::clamp)) const
    {
        if (this->empty()) {
            return {};
        }
        else {
            // Shift.
            loc -= float_type(0.5);

            // Floor.
            multi<float_type, 2> loc0 = pr::floor(loc);
            loc -= loc0;

            // Interpolate.
            multi<float_type, N> val0[4];
            for (size_type i = 0; i < 4; i++) {

                multi<float_type, N> val1[4];
                for (size_type j = 0; j < 4; j++) {
                    val1[j] = 
                        lookup(loc0 + 
                        multi<float_type, 2>{{
                            float_type(i) - 1,
                            float_type(j) - 1
                        }}, cyc_mode);
                }

                // Catmull-Rom interpolation.
                val0[i] = 
                    catmull(
                        loc[1],
                        val1[0], val1[1],
                        val1[2], val1[3]);
            }

            // Catmull-Rom interpolation.
            return 
                catmull(
                    loc[0], 
                    val0[0], val0[1], 
                    val0[2], val0[3]);
        }
    }

    /**
     * @brief Sample.
     *
     * @param[in] samp
     * Sample method, either 0, 1, or 2.
     *
     * @param[in] loc
     * Location in index coordinates.
     *
     * @param[in] cyc_mode
     * Cycle modes.
     */
    multi<float_type, N> sample(
            int samp,
            multi<float_type, 2> loc,
            multi<cycle_mode, 2> cyc_mode =
            multi<cycle_mode, 2>::value(cycle_mode::clamp)) const
    {
        switch (samp) {
            default:
            case 0: return sample0(loc, cyc_mode);
            case 1: return sample1(loc, cyc_mode);
            case 2: return sample2(loc, cyc_mode);
        }

        // Unreachable.
        return {};
    }

    /**
     * @brief Resample.
     *
     * @param[in] samp
     * Sample method, either 0, 1, or 2.
     *
     * @param[in] count
     * Count.
     *
     * @param[in] cyc_mode
     * Cycle modes.
     */
    void resample(
            int samp,
            multi<size_type, 2> count,
            multi<cycle_mode, 2> cyc_mode =
            multi<cycle_mode, 2>::value(cycle_mode::clamp))
    {
        // Target size is equivalent?
        if ((count == this->user_size_).all()) {
            // Do nothing.
        }
        // Target size is zero?
        else if ((count == size_type(0)).any()) {
            this->clear();
            // Do nothing.
        }
        // Target size requires upsampling and downsampling?
        else if (!((count <= this->user_size_).all() ||
                   (count >= this->user_size_).all())) {
            // Resample 0th dimension.
            resample(samp, {{count[0], this->user_size_[1]}}, cyc_mode);
            // Resample 1st dimension.
            resample(samp, {{this->user_size_[0], count[1]}}, cyc_mode);
        }
        else {

            // Temporary image.
            image2 image(std::move(*this));

            // Resize.
            this->resize(count);

            // Scale factor.
            multi<float_type, 2> scale_fac =
                multi<float_type, 2>(image.user_size_) /
                multi<float_type, 2>(this->user_size_);

            // Requires downsampling?
            if ((this->user_size_ <= image.user_size_).all()) {
                for (size_type i = 0; i < this->user_size_[0]; i++)
                for (size_type j = 0; j < this->user_size_[1]; j++) {
                    // Downsample.
                    multi<float_type, 2> locmin = {{
                        scale_fac[0] * float_type(i),
                        scale_fac[1] * float_type(j)
                    }};
                    multi<float_type, 2> locmax = {{
                        scale_fac[0] * float_type(i + 1),
                        scale_fac[1] * float_type(j + 1)
                    }};
                    this->operator()(i, j) = 
                        fstretch<entry_type>(
                                 image.average(locmin, locmax, cyc_mode));
                }
            }
            else {
                for (size_type i = 0; i < this->user_size_[0]; i++)
                for (size_type j = 0; j < this->user_size_[1]; j++) {
                    // Upsample.
                    multi<float_type, 2> loc = {{
                        scale_fac[0] * (float_type(i) + float_type(0.5)),
                        scale_fac[1] * (float_type(j) + float_type(0.5))
                    }};
                    this->operator()(i, j) = 
                        fstretch<entry_type>(
                                 image.sample(samp, loc, cyc_mode));
                }
            }
        }
    }

    /**@}*/

private:

    /**
     * @brief Lookup pixel.
     *
     * @param[in] loc
     * Location in pixel coordinates.
     *
     * @param[in] cyc_mode
     * Cycle modes.
     */
    multi<float_type, N> lookup(
                multi<float_type, 2> loc,
                multi<cycle_mode, 2> cyc_mode) const
    {
        // Cycle.
        multi<float_type, 2> locmin =
        multi<float_type, 2>::value(float_type(-0.5));
        multi<float_type, 2> locmax = 
            float_type(0.5) * multi<float_type, 2>(this->user_size_) +
            float_type(0.5) * multi<float_type, 2>(this->user_size_ - 1);
        loc = fcycle(loc, locmin, locmax, cyc_mode);

        // Round edge cases appropriately.
        for (size_type k = 0; k < 2; k++) {
            if (loc[k] == locmin[k]) loc[k] = finc(loc[k]);
            if (loc[k] == locmax[k]) loc[k] = fdec(loc[k]);
        }

        // Lookup.
        return fstretch<float_type>(
                    this->data_[
                    this->convert(
                        multi<size_type, 2>(pr::round(loc)) % 
                        this->user_size_)]);
    }
};

/**@}*/

} // namespace pr

#endif // #ifndef PREFORM_IMAGE2_HPP