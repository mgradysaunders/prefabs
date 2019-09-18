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
#error "preform/medium.hpp requires >=C++17"
#endif // #if !(__cplusplus >= 201703L)
#endif // #if !DOXYGEN
#pragma once
#ifndef PREFORM_MEDIUM_HPP
#define PREFORM_MEDIUM_HPP

// for pr::multi
#include <preform/multi.hpp>

// for pr::multi wrappers
#include <preform/multi_math.hpp>

namespace pr {

/**
 * @defgroup medium Medium
 *
 * `<preform/medium.hpp>`
 *
 * __C++ version__: >=C++17
 */
/**@{*/

/**
 * @brief Henyey-Greenstein phase.
 *
 * @tparam T
 * Float type.
 */
template <typename T>
struct hg_phase
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
    hg_phase() = default;

    /**
     * @brief Constructor.
     */
    hg_phase(float_type g) : g_(g)
    {
    }

    /**
     * @brief Phase function.
     *
     * @f[
     *      p_s(\omega_o, \omega_i) =
     *      \frac{1}{4\pi}
     *      \frac{1 - g^2}
     *          {(1 + g^2 - 2g\omega_o\cdot\omega_i)^{3/2}}
     * @f]
     *
     * @param[in] wo
     * Outgoing direction.
     *
     * @param[in] wi
     * Incident direction.
     */
    float_type ps(
            multi<float_type, 3> wo,
            multi<float_type, 3> wi) const
    {
        return multi<float_type, 3>::hg_phase_pdf(g_, dot(wo, wi));
    }

    /**
     * @brief Phase function sampling routine.
     *
     * @param[in] u
     * Sample in @f$ [0, 1)^2 @f$.
     *
     * @param[in] wo
     * Outgoing direction.
     *
     * @note
     * This samples directions matching the 
     * distribution of @f$ p_s @f$, such that path throughput 
     * is updated trivially as @f$ \beta' \gets \beta @f$.
     */
    multi<float_type, 3> ps_sample(
            multi<float_type, 2> u,
            multi<float_type, 3> wo) const
    {
        return dot(
                multi<float_type, 3, 3>::build_onb(wo), 
                multi<float_type, 3>::hg_phase_pdf_sample(g_, u));
    }

private:

    /**
     * @brief Shape parameter @f$ g \in (-1, 1) @f$.
     */
    float_type g_ = 0;
};

/**
 * @brief Henyey-Greenstein phase stack.
 *
 * @tparam T
 * Float type.
 *
 * @tparam Nlobes
 * Number of lobes.
 */
template <typename T, std::size_t Nlobes>
struct hg_phase_stack
{
public:

    // Sanity check.
    static_assert(
        std::is_floating_point<T>::value,
        "T must be floating point");

    // Sanity check.
    static_assert(
        Nlobes > 0,
        "Nlobes must be greater than 0");

    /**
     * @brief Float type.
     */
    typedef T float_type;

    /**
     * @brief Default constructor.
     */
    hg_phase_stack()
    {
        w_[0] = 1;
    }

    /**
     * @brief Constructor.
     *
     * @note
     * For efficiency, this does not validate the @f$ w_k @f$ in 
     * any respect. It is the user's responsibility to ensure that
     * the @f$ w_k @f$ are nonnegative and that @f$ \sum_k w_k = 1 @f$.
     */
    hg_phase_stack(
            const multi<float_type, Nlobes>& g,
            const multi<float_type, Nlobes>& w) :
                g_(g),
                w_(w)
    {
    }

    /**
     * @brief Phase function.
     *
     * @f[
     *      p_s(\omega_o, \omega_i) =
     *      \sum_k
     *      \frac{1}{4\pi}
     *      \frac{1 - g_k^2}
     *          {(1 + g_k^2 - 2g_k\omega_o\cdot\omega_i)^{3/2}} w_k
     * @f]
     *
     * @param[in] wo
     * Outgoing direction.
     *
     * @param[in] wi
     * Incident direction.
     */
    float_type ps(
            multi<float_type, 3> wo,
            multi<float_type, 3> wi) const
    {
        float_type res = 0;
        float_type dot_wo_wi = dot(wo, wi);
        for (std::size_t k = 0; k < Nlobes; k++) {
            res += w_[k] * 
            multi<float_type, 3>::hg_phase_pdf(g_[k], dot_wo_wi);
        }
        return res;
    }

    /**
     * @brief Phase function sampling routine.
     *
     * @param[in] u
     * Sample in @f$ [0, 1)^2 @f$.
     *
     * @param[in] wo
     * Outgoing direction.
     *
     * @note
     * This samples directions matching the 
     * distribution of @f$ p_s @f$, such that path throughput 
     * is updated trivially as @f$ \beta' \gets \beta @f$.
     */
    multi<float_type, 3> ps_sample(
            multi<float_type, 2> u,
            multi<float_type, 3> wo) const
    {
        // Sample index.
        std::size_t k = 0;
        for (float_type wsum = 0; k < Nlobes; k++) {
            if (u[0] < (wsum + w_[k])) {
                // Stretch to [0, 1).
                u[0] -= wsum;
                u[0] /= w_[k];

                // Just to be safe.
                u[0] = pr::fmax(u[0], float_type(0));
                u[0] = pr::fmin(u[0], float_type(1) - 
                       pr::numeric_limits<float_type>::machine_epsilon());
                break;
            }

            // Increment.
            wsum = 
            wsum + w_[k];
        }

        // Sample.
        return dot(
                multi<float_type, 3, 3>::build_onb(wo), 
                multi<float_type, 3>::hg_phase_pdf_sample(g_[k], u));
    }

private:

    /**
     * @brief Shape parameters @f$ g_k \in (-1, 1) @f$.
     */
    multi<float_type, Nlobes> g_ = {};

    /**
     * @brief Weights @f$ w_k \in (0, 1) @f$, normalized 
     * such that @f$ \sum_k w_k = 1 @f$.
     */
    multi<float_type, Nlobes> w_ = {};
};

/**@}*/

} // namespace pr

#endif // #ifndef PREFORM_MEDIUM_HPP
