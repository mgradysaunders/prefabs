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
#error "preform/running_stat.hpp requires >=C++14"
#endif // #if !(__cplusplus >= 201402L)
#endif // #if !DOXYGEN
#pragma once
#ifndef PREFORM_RUNNING_STAT_HPP
#define PREFORM_RUNNING_STAT_HPP

#include <preform/math.hpp>

namespace pr {

/**
 * @defgroup running_stat Running statistic
 *
 * `<preform/running_stat.hpp>`
 *
 * __C++ version__: >=C++14
 */
/**@{*/

/**
 * @brief Running statistic.
 *
 * @see 
 * This Wikipedia article on [Algorithms for calculating variance][1].
 * [1]: https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance
 */
template <typename T = double>
class running_stat
{
public:

    // Sanity check.
    static_assert(
        std::is_floating_point<T>::value, 
        "T must be floating point");

public:

    /**
     * @brief Default constructor.
     */
    constexpr running_stat() = default;

    /**
     * @brief Constructor.
     *
     * @param[in] n
     * Sample count @f$ n @f$.
     *
     * @param[in] m0
     * Sample mean @f$ m @f$.
     *
     * @param[in] m1
     * Sample central moment @f$ M_2 @f$.
     *
     * @param[in] m2
     * Sample central moment @f$ M_3 @f$.
     *
     * @param[in] m3
     * Sample central moment @f$ M_4 @f$.
     */
    constexpr running_stat(
            long long n,
            T m0, 
            T m1,
            T m2,
            T m3) :
                n_(n),
                m_{m0, m1, m2, m3}
    {
    }

public:

    /**
     * @name Metrics
     */
    /**@{*/

    /**
     * @brief Sample count.
     */
    long long count() const
    {
        return n_;
    }

    /**
     * @brief Sample mean.
     */
    T mean() const
    {
        return m_[0];
    }

    /**
     * @brief Sample variance.
     *
     * @note
     * @f[
     *      \sigma^2 = \frac{M_2}{n - 1}
     * @f]
     */
    T variance() const
    {
        return m_[1] / T(n_ - 1);
    }

    /**
     * @brief Sample standard deviation.
     *
     * @note
     * @f[
     *      \sigma = \sqrt{\frac{M_2}{n - 1}}
     * @f]
     */
    T standard_deviation() const
    {
        return pr::sqrt(variance());
    }

    /**
     * @brief Sample skewness.
     *
     * @note
     * @f[
     *      g_1 = M_3 \sqrt{\frac{n}{M_2^3}}
     * @f]
     */
    T skewness() const
    {
        return m_[2] * pr::sqrt(T(n_) / pr::nthpow(m_[1], 3));
    }

    /**
     * @brief Sample kurtosis.
     *
     * @note
     * @f[
     *      g_2 = n \frac{M_4}{M_2^2} - 3
     * @f]
     */
    T kurtosis() const
    {
        return m_[3] * T(n_) / pr::nthpow(m_[1], 2) - 3;
    }

    /**@}*/

public:

    /**
     * @brief No-op.
     */
    running_stat operator+() const
    {
        return *this;
    }

    /**
     * @brief Negate.
     *
     * @note
     * - @f$ n' \gets n @f$
     * - @f$ m' \gets -m @f$
     * - @f$ M_2' \gets +M_2 @f$
     * - @f$ M_3' \gets -M_3 @f$
     * - @f$ M_4' \gets +M_4 @f$
     */
    running_stat operator-() const
    {
        return {n_, -m_[0], m_[1], -m_[2], m_[3]};
    }

    /**
     * @brief Add term.
     *
     * @note
     * - @f$ n' \gets n + 1 @f$
     * - @f$ m' \gets (n m + x) / n' @f$
     * - @f$ s \gets (x - m) / n' @f$
     * - @f$ t \gets (x - m)^2 n / n' @f$
     * - @f$ M_2' \gets M_2 + t @f$
     * - @f$ M_3' \gets M_3 + s t (n - 1) + 3 s M_2 @f$
     * - @f$ M_4' \gets M_4 + s^2 t (n^2 - n + 1) + 6 s^2 M_2 - 4 s M_3 @f$
     */
    running_stat operator+(T x) const
    {
        long long n1 = n_ + 1;
        T d = x - m_[0];
        T s = d / n1;
        T t = d * n_ * s;
        return {
            n1,
            (n_ * m_[0] + x) / n1,
            m_[1] + t,
            m_[2] + s * (t * (n_ - 1) - 3 * m_[1]),
            m_[3] + s * (s * (t * (n_ * (n_ - 1) + 1) + 
            6 * m_[1]) - 4 * m_[2])
        };
    }

    /**
     * @brief Add negative term.
     */
    running_stat operator-(T x) const
    {
        return *this + -x;
    }

    /**
     * @brief Add terms.
     *
     * @note
     * - @f$ n' \gets n_A + n_B @f$
     * - @f$ r_A \gets n_A / n' @f$
     * - @f$ r_B \gets n_B / n' @f$
     * - @f$ d \gets m_B - m_A @f$
     * - @f$ p \gets n_A n_B / n' @f$
     * - @f$ q \gets d^2 p @f$
     * - @f$ m' \gets m_A + r_B d @f$
     * - @f$ M_2' \gets M_{2,A} + M_{2,B} + q @f$
     * - @f$ M_3' \gets M_{3,A} + M_{3,B} + q d (r_A - r_B) + 
     *         3 d (r_A M_{2,B} - r_B M_{2,A}) @f$
     * - @f$ M_4' \gets M_{4,A} + M_{4,B} + q d^2 (r_A^2 - r_A r_B + r_B^2) + 
     *         6 d^2 (r_A^2 M_{2,B} - r_B^2 M_{2,A}) + 
     *         4 d (r_A M_{3,B} - r_B M_{3,A}) @f$
     */
    running_stat operator+(const running_stat& b) const
    {
        const running_stat& a = *this;

        // Temporary terms.
        T d = b.m_[0] - a.m_[0];
        T ra = a.n_ / T(a.n_ + b.n_);
        T rb = b.n_ / T(a.n_ + b.n_);
        T p = 
            a.n_ > b.n_ ?
            b.n_ / (T(1) + T(b.n_) / T(a.n_)) :
            a.n_ / (T(1) + T(a.n_) / T(b.n_)); // = a.n_ * b.n_ / c.n_
        T q = d * d * p;

        // Update.
        return {
            a.n_ + b.n_,
            a.m_[0] + d * rb,
            a.m_[1] + b.m_[1] + q,
            a.m_[2] + b.m_[2] + 
            d * (q * (ra - rb) + 
            3 * (ra * b.m_[1] - rb * a.m_[1])),
            a.m_[3] + b.m_[3] + 
            d * (d * (q * (ra * ra - ra * rb + rb * rb) +
            6 * (ra * ra * b.m_[1] + rb * rb * a.m_[1])) + 
            4 * (ra * b.m_[2] + rb * a.m_[2]))
        };
    }

    /**
     * @brief Add negative terms.
     */
    running_stat operator-(const running_stat& b) const
    {
        return *this + -b;
    }

    /**
     * @brief Generic `operator+=`.
     */
    template <typename U>
    running_stat& operator+=(const U& any)
    {
        return *this = *this + any;
    }

    /**
     * @brief Generic `operator-=`.
     */
    template <typename U>
    running_stat& operator-=(const U& any)
    {
        return *this = *this - any;
    }

    /**
     * @brief Cast as different precision.
     */
    template <typename U>
    operator running_stat<U>() const
    {
        return {
            n_,
            static_cast<U>(m_[0]),
            static_cast<U>(m_[1]),
            static_cast<U>(m_[2]),
            static_cast<U>(m_[3])
        };
    }

private:

    /**
     * @brief Sample count @f$ n @f$.
     */
    long long n_ = 0;

    /**
     * @brief Sample mean and central moments.
     *
     * - 0: mean @f$ m @f$
     * - 1: central moment @f$ M_2 @f$
     * - 2: central moment @f$ M_3 @f$
     * - 3: central moment @f$ M_4 @f$
     */
    T m_[4] = {};
};

/**@}*/

} // namespace pr

#endif // #ifndef PREFORM_RUNNING_STAT_HPP
