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

// for pr::cycle_mode, ...
#include <preform/image_helpers.hpp>

// for pr::block_array2
#include <preform/block_array2.hpp>

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
class image2 : public block_array2<
                    multi<T, N>,
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
    typedef block_array2<
            multi<T, N>,
            typename std::allocator_traits<Talloc>::
            template rebind_alloc<multi<T, N>>> base;

    using typename base::size_type;
#endif // #if !DOXYGEN

public:

    // Inherit constructors.
    using base::base;

public:

    /**
     * @name Cycle mode
     */
    /**@{*/

    /**
     * @brief Get cycle mode.
     */
    multi<cycle_mode, 2> cyc_mode() const
    {
        return cyc_mode_;
    }

    /**
     * @brief Set cycle mode.
     */
    multi<cycle_mode, 2> cyc_mode(multi<cycle_mode, 2> mode)
    {
        multi<cycle_mode, 2> res = cyc_mode_;
        cyc_mode_ = mode;
        return res;
    }

    /**
     * @brief Set cycle mode.
     */
    multi<cycle_mode, 2> cyc_mode(cycle_mode mode)
    {
        return cyc_mode(multi<cycle_mode, 2>(mode));
    }

    /**@}*/

public:

    /**
     * @name Sampling
     */
    /**@{*/

    /**
     * @brief Average.
     *
     * @param[in] locmin
     * Location minimum.
     *
     * @param[in] locmax
     * Location maximum.
     */
    multi<float_type, N> average(
            multi<float_type, 2> locmin,
            multi<float_type, 2> locmax) const
    {
        if (this->empty()) {
            return {};
        }
        else if ((locmin == locmax).all()) {
            return sample0(locmin);
        }
        else {

            // Shift.
            locmin -= float_type(0.5);
            locmax -= float_type(0.5);
            if (locmin[0] > locmax[0]) std::swap(locmin[0], locmax[0]);
            if (locmin[1] > locmax[1]) std::swap(locmin[1], locmax[1]);

            // Floor.
            multi<int, 2> indmin = fastfloor(locmin);
            multi<int, 2> indmax = fastfloor(locmax);

            // Integrate.
            multi<float_type, N> numer = {};
            float_type denom = 0;
            for (int i = indmin[0]; i <= indmax[0]; i++)
            for (int j = indmin[1]; j <= indmax[1]; j++) {
                multi<int, 2> ind = {i, j};
                multi<float_type, 2> loc0 = ind;
                multi<float_type, 2> loc1 = ind + 1;

                // Clamp.
                loc0 = pr::fmax(loc0, locmin);
                loc1 = pr::fmin(loc1, locmax);

                // Compute contribution.
                numer += (loc1 - loc0).prod() * fetch(ind);
                denom += (loc1 - loc0).prod();
            }

            // Average.
            return numer / denom;
        }
    }

    /**
     * @brief Sample, no interpolation.
     *
     * @param[in] loc
     * Location.
     */
    multi<float_type, N> sample0(multi<float_type, 2> loc) const
    {
        if (this->empty()) {
            return {};
        }
        else {
            return fetch(fastfloor(loc));
        }
    }

    /**
     * @brief Sample, linear interpolation.
     *
     * @param[in] loc
     * Location.
     */
    multi<float_type, N> sample1(multi<float_type, 2> loc) const
    {
        if (this->empty()) {
            return {};
        }
        else {
            // Shift.
            loc -= float_type(0.5);

            // Floor.
            multi<int, 2> ind = fastfloor(loc);
            loc -= ind;

            // Interpolate.
            multi<float_type, N> val0[2];
            for (int i = 0; i < 2; i++) {

                multi<float_type, N> val1[2];
                for (int j = 0; j < 2; j++) {
                    val1[j] =
                        fetch(ind +
                        multi<int, 2>{i, j});
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
     * Location.
     */
    multi<float_type, N> sample2(multi<float_type, 2> loc) const
    {
        if (this->empty()) {
            return {};
        }
        else {
            // Shift.
            loc -= float_type(0.5);

            // Floor.
            multi<int, 2> ind = fastfloor(loc);
            loc -= ind;

            // Interpolate.
            multi<float_type, N> val0[4];
            for (int i = 0; i < 4; i++) {

                multi<float_type, N> val1[4];
                for (int j = 0; j < 4; j++) {
                    val1[j] =
                        fetch(ind +
                        multi<int, 2>{
                            i - 1,
                            j - 1
                        });
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
     * Sampling method, either 0, 1, or 2.
     *
     * @param[in] loc
     * Location.
     */
    multi<float_type, N> sample(int samp, multi<float_type, 2> loc) const
    {
        switch (samp) {
            default:
            case 0: return sample0(loc);
            case 1: return sample1(loc);
            case 2: return sample2(loc);
        }

        // Unreachable.
        return {};
    }

    /**
     * @brief Resample.
     *
     * @param[in] samp
     * Sampling method, either 0, 1, or 2.
     *
     * @param[in] count
     * Count.
     */
    void resample(int samp, multi<size_type, 2> count)
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
            resample(samp, {count[0], this->user_size_[1]});
            // Resample 1st dimension.
            resample(samp, {count[0], count[1]});
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
                    multi<float_type, 2> locmin = {
                        scale_fac[0] * float_type(i),
                        scale_fac[1] * float_type(j)
                    };
                    multi<float_type, 2> locmax = {
                        scale_fac[0] * float_type(i + 1),
                        scale_fac[1] * float_type(j + 1)
                    };
                    this->operator()(i, j) =
                        fstretch<entry_type>(
                                 image.average(locmin, locmax));
                }
            }
            else {
                for (size_type i = 0; i < this->user_size_[0]; i++)
                for (size_type j = 0; j < this->user_size_[1]; j++) {
                    // Upsample.
                    multi<float_type, 2> loc = {
                        scale_fac[0] * (float_type(i) + float_type(0.5)),
                        scale_fac[1] * (float_type(j) + float_type(0.5))
                    };
                    this->operator()(i, j) =
                        fstretch<entry_type>(
                                 image.sample(samp, loc));
                }
            }
        }
    }

    /**@}*/

    /**
     * @brief Reconstruct.
     *
     * @param[in] val
     * Value.
     *
     * @param[in] loc
     * Location.
     *
     * @param[in] filtrad
     * Filter radii, beyond which filter is zero.
     *
     * @param[in] filt
     * Filter function.
     */
    template <typename Tfilt>
    void reconstruct(
            multi<float_type, N> val,
            multi<float_type, 2> loc,
            multi<float_type, 2> filtrad,
            Tfilt&& filt)
    {
        // Shift.
        loc -= float_type(0.5);

        // Determine indices.
        multi<int, 2> indmin = fastceil(loc - filtrad);
        multi<int, 2> indmax = fastfloor(loc + filtrad);
        for (int k = 0; k < 2; k++) {
            if (cyc_mode_[k] == cycle_mode::clamp) {
                indmin[k] = std::max(indmin[k], 0);
                indmax[k] = std::min(indmax[k], int(this->user_size_[k]) - 1);
            }
        }

        // Reconstruct.
        for (int i = indmin[0]; i <= indmax[0]; i++)
        for (int j = indmin[1]; j <= indmax[1]; j++) {
            multi<int, 2> ind = {i, j};
            float_type tmp = std::forward<Tfilt>(filt)(ind - loc);
            if (tmp != float_type(0)) {

                // Lookup entry.
                multi<entry_type, N>& ent =
                    this->operator[](
                    this->convert(cycle<int, 2>(
                                        cyc_mode_, ind,
                                        this->user_size_)));

                // Add.
                ent = fstretch<entry_type>(
                      fstretch<float_type>(ent) + val * tmp);
            }
        }
    }

private:

    /**
     * @brief Cycle mode.
     */
    multi<cycle_mode, 2> cyc_mode_ =
    multi<cycle_mode, 2>(cycle_mode::clamp);

#if !DOXYGEN

    /**
     * @brief Fetch.
     */
    __attribute__((always_inline))
    multi<float_type, N> fetch(multi<int, 2> ind) const
    {
        return
            fstretch<float_type>(
                this->operator[](
                this->convert(cycle<int, 2>(
                                    cyc_mode_, ind, 
                                    this->user_size_))));
    }

#endif // #if !DOXYGEN
};

/**@}*/

} // namespace pr

#endif // #ifndef PREFORM_IMAGE2_HPP
