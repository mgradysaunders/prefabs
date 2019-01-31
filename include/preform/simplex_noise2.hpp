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
#error "preform/simplex_noise2.hpp requires >=C++17"
#endif // #if !(__cplusplus >= 201703L)
#endif // #if !DOXYGEN
#pragma once
#ifndef PREFORM_SIMPLEX_NOISE2_HPP
#define PREFORM_SIMPLEX_NOISE2_HPP

// for pr::multi
#include <preform/multi.hpp>
#include <preform/multi_math.hpp>

// for pr::wrap
#include <preform/int_helpers.hpp>

// for pr::pcg32_xsh_rr
#include <preform/random.hpp>

namespace pr {

/**
 * @defgroup simplex_noise2 Simplex noise (2-dimensional)
 *
 * `<preform/simplex_noise2.hpp>`
 *
 * __C++ version__: >=C++17
 */
/**@{*/

/**
 * @brief Simplex noise (2-dimensional).
 *
 * @image html simplex_noise2.png
 */
template <typename T>
class simplex_noise2
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
    simplex_noise2() = default;

    /**
     * @brief Constructor.
     */
    simplex_noise2(int seed) : seed_(seed)
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
    float_type evaluate(multi<float_type, 2> t,
                        multi<float_type, 2>* ds_dt = nullptr) const
    {
        // f = (sqrt(n + 1) - 1) / n
        static const float_type f = 
                float_type(0.5) * pr::sqrt(float_type(3)) - 
                float_type(0.5);

        // g = (1 - 1 / sqrt(n + 1)) / n
        static const float_type g = 
                float_type(0.5) -
                float_type(0.5) / pr::sqrt(float_type(3));

        // Zero initialize.
        if (ds_dt) {
            *ds_dt = {};
        }

        // Skew.
        multi<float_type, 2> u = t + f * t.sum();

        // Floor.
        multi<float_type, 2> u0 = pr::floor(u);

        // Shift to [0, 1)^2.
        u -= u0;

        // Descending order.
        multi<int, 2> uorder = {{0, 1}};
        if (!(u[0] >= u[1])) {
            std::swap(
                    uorder[0], 
                    uorder[1]);
        }

        // Gradient.
        auto grad = [&](multi<int, 2> w) -> multi<float_type, 2>
        {
            // Generator.
            pcg32_xsh_rr gen(
                seed_,
                cantor(
                    w[0],
                    w[1]));

            // Initialize gradient.
            multi<float_type, 2> x;
            int k = gen(2);
            int k0 = (k + 0) & 1;
            int k1 = (k + 1) & 1;
            int l = gen();
            x[k0] = l & 1 ? 1 : -1;
            x[k1] = l & 2 ? 2 : -2;
            return x;
        };

        // Sum.
        float_type s = float_type(0);

        // Skewed simplex vertex.
        multi<int, 2> w0 = u0;

        // Skewed simplex vertex offset.
        multi<int, 2> wk = {};

        // Unskewed simplex vertex.
        multi<float_type, 2> v0 = u0 - g * u0.sum();

        for (int k = 0; k < 3; k++) {

            // Unskewed simplex vertex.
            multi<float_type, 2> vk = v0 + wk - g * k;

            // Offset.
            multi<float_type, 2> tk = t - vk;

            // Cutoff radius.
            float_type r = float_type(0.7);

            // Factor.
            float_type q = r * r - pr::dot(tk, tk);

            if (q > float_type(0)) {

                // Gradient.
                multi<float_type, 2> xk = grad(w0 + wk);

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
            if (k < 2) {
                wk[uorder[k]]++;
            }
        }

        // Scale to approximately [-0.997, 0.997].
        s *= float_type(50.5);

        // Chain rule.
        if (ds_dt) {
            *ds_dt *= float_type(50.5);
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

#endif // #ifndef PREFORM_SIMPLEX_NOISE2_HPP
