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
#error "preform/simplex_noise4.hpp requires >=C++17"
#endif // #if !(__cplusplus >= 201703L)
#endif // #if !DOXYGEN
#pragma once
#ifndef PREFORM_SIMPLEX_NOISE4_HPP
#define PREFORM_SIMPLEX_NOISE4_HPP

// for pr::multi
#include <preform/multi.hpp>

// for pr::multi wrappers
#include <preform/multi_math.hpp>

// for pr::wrap
#include <preform/misc_int.hpp>

// for pr::pcg32
#include <preform/random.hpp>

namespace pr {

/**
 * @defgroup simplex_noise4 Simplex noise (4-dimensional)
 *
 * `<preform/simplex_noise4.hpp>`
 *
 * __C++ version__: >=C++17
 */
/**@{*/

/**
 * @brief Simplex noise (4-dimensional).
 *
 * @image html simplex_noise4.png
 */
template <typename T>
class simplex_noise4
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
    simplex_noise4() = default;

    /**
     * @brief Constructor.
     */
    simplex_noise4(int seed) : seed_(seed)
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
     * Noise is in range @f$ (-1, 1) @f$.
     */
    float_type evaluate(multi<float_type, 4> t,
                        multi<float_type, 4>* ds_dt = nullptr) const
    {
        // f = (sqrt(n + 1) - 1) / n
        static const float_type f = 
            float_type(0.25) * pr::sqrt(float_type(5)) -
            float_type(0.25);

        // g = (1 - 1 / sqrt(n + 1)) / n
        static const float_type g = 
            float_type(0.25) - 
            float_type(0.25) / pr::sqrt(float_type(5));

        // Zero initialize.
        if (ds_dt) {
            *ds_dt = {};
        }

        // Skew.
        multi<float_type, 4> u = t + f * t.sum();

        // Floor.
        multi<float_type, 4> u0 = pr::floor(u);

        // Shift to [0, 1)^4.
        u -= u0;

        // Descending order.
        multi<int, 4> uorder;
        {
            std::pair<T, int> usort[4] = {
                {u[0], 0},
                {u[1], 1},
                {u[2], 2},
                {u[3], 3}
            };
            std::sort(
                &usort[0],
                &usort[0] + 4,
                [](const std::pair<T, int>& usort0,
                   const std::pair<T, int>& usort1) {
                    return usort0.first > usort1.first;
                });
            uorder = {
                usort[0].second,
                usort[1].second,
                usort[2].second,
                usort[3].second
            };
        }

        // Gradient.
        auto grad = [&](multi<int, 4> w) -> multi<float_type, 4>
        {
            // Generator.
            pcg32 gen(
                seed_,
                cantor(
                    w[0],
                    w[1],
                    w[2],
                    w[3]));

            // Initialize gradient.
            multi<float_type, 4> x;
            int k = gen(4);
            int k0 = (k + 0) & 3;
            int k1 = (k + 1) & 3;
            int k2 = (k + 2) & 3;
            int l = gen();
            x[k0] = l & 1 ? 1 : -1;
            x[k1] = l & 2 ? 1 : -1;
            x[k2] = l & 4 ? 1 : -1;
            return x;
        };

        // Sum.
        float_type s = float_type(0);

        // Skewed simplex vertex.
        multi<int, 4> w0 = u0;

        // Skewed simplex vertex offset.
        multi<int, 4> wk = {};

        // Unskewed simplex vertex.
        multi<float_type, 4> v0 = u0 - g * u0.sum();

        for (int k = 0; k < 5; k++) {

            // Unskewed simplex vertex.
            multi<float_type, 4> vk = v0 + wk - g * k;

            // Offset.
            multi<float_type, 4> tk = t - vk;

            // Cutoff radius.
            float_type r = float_type(0.7);

            // Factor.
            float_type q = r * r - pr::dot(tk, tk);

            if (q > float_type(0)) {

                // Gradient.
                multi<float_type, 4> xk = grad(w0 + wk);

                // Projection onto gradient.
                float_type pk = pr::dot(tk, xk);

                float_type q2 = q * q;

                // Add term.
                s += (q2 * q2) * pk;

                // Add partial derivatives.
                if (ds_dt) {
                    *ds_dt += 
                        (q2 * q2) * xk - 
                        float_type(8) * (q2 * q) * pk * tk;
                }
            }

            // Bump skewed simplex vertex offset.
            if (k < 4) {
                wk[uorder[k]]++;
            }
        }

        // Scale to approximately [-0.997, 0.997].
        s *= float_type(68.6);

        // Chain rule.
        if (ds_dt) {
            *ds_dt *= float_type(68.6);
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

#endif // #ifndef PREFORM_SIMPLEX_NOISE4_HPP
