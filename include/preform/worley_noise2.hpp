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
#error "preform/worley_noise2.hpp requires >=C++17"
#endif // #if !(__cplusplus >= 201703L)
#endif // #if !DOXYGEN
#pragma once
#ifndef PREFORM_WORLEY_NOISE2_HPP
#define PREFORM_WORLEY_NOISE2_HPP

// for pr::multi
#include <preform/multi.hpp>
#include <preform/multi_math.hpp>

// for pr::wrap
#include <preform/int_helpers.hpp>

// for pr::pcg_xsh_rr, pr::generate_canonical
#include <preform/random.hpp>

namespace pr {

/**
 * @defgroup worley_noise2 Worley noise (2-dimensional)
 *
 * `<preform/worley_noise2.hpp>`
 *
 * __C++ version__: >=C++17
 */
/**@{*/

/**
 * @brief Worley noise (2-dimensional).
 *
 * @image html worley_noise2.png
 */
template <typename T>
class worley_noise2
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
    worley_noise2() = default;

    /**
     * @brief Constructor.
     */
    worley_noise2(int seed) : seed_(seed)
    {
    }

    /**
     * @brief Evaluate.
     *
     * @param[in] t
     * Coordinate.
     *
     * @param[out] ds_dt
     * Noise partial derivatives. _Optional_.
     *
     * @note
     * Noise is in range @f$ [0, 1) @f$.
     */
    float_type evaluate(multi<float_type, 2> t,
                        multi<float_type, 2>* ds_dt = nullptr) const
    {
        // Zero initialize.
        if (ds_dt) {
            *ds_dt = {};
        }

        // Floor.
        multi<float_type, 2> t0 = pr::floor(t);

        // Half square distance to nearest vertex.
        float_type s = pr::numeric_limits<float_type>::infinity();

        multi<int, 2> w0 = t0;
        multi<int, 2> wk;
        for (wk[0] = w0[0] - 1; wk[0] <= w0[0] + 1; wk[0]++)
        for (wk[1] = w0[1] - 1; wk[1] <= w0[1] + 1; wk[1]++) {

            // Generator.
            pcg32_xsh_rr gen(
                seed_,
                cantor(
                    wk[0], 
                    wk[1]));

            // Vertex.
            multi<float_type, 2> vk = {{
                float_type(wk[0]) + pr::generate_canonical<float_type>(gen),
                float_type(wk[1]) + pr::generate_canonical<float_type>(gen)
            }};

            // Offset.
            multi<float_type, 2> tk = t - vk;

            // Half square distance.
            float_type sk = 
            float_type(0.5) * pr::dot(tk, tk);
            if (s > sk) {
                s = sk;
                if (ds_dt) { 
                    *ds_dt = tk;
                }
            }
        }
        
        // Square root.
        s = pr::sqrt(s);

        // Chain rule.
        if (ds_dt) { 
            if (s == float_type(0)) {
                *ds_dt = {};
            }
            else {
                *ds_dt /= float_type(2) * s;
            }
        }

        return s;
    }

private:

    /**
     * @brief Seed.
     */
    int seed_ = 0;
};

/**@}*/

} // namespace pr

#endif // #ifndef PREFORM_WORLEY_NOISE2_HPP
