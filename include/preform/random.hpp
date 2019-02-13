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
#error "preform/random.hpp requires >=C++14"
#endif // #if !(__cplusplus >= 201402L)
#endif // #if !DOXYGEN
#pragma once
#ifndef PREFORM_RANDOM_HPP
#define PREFORM_RANDOM_HPP

// for std::min, std::max, std::copy, ...
#include <algorithm>

// for std::allocator
#include <memory>

// for std::string
#include <string>

// for std::invalid_argument
#include <stdexcept>

// for std::vector
#include <vector>

// for pr::numeric_limits, pr::log2, ...
#include <preform/math.hpp>

// for pr::first1, pr::lcg_seek
#include <preform/int_helpers.hpp>

// for pr::neumaier_sum
#include <preform/neumaier_sum.hpp>

namespace pr {

/**
 * @defgroup random Random
 *
 * `<preform/random.hpp>`
 *
 * __C++ version__: >=C++14
 */
/**@{*/

/**
 * @brief Generate canonical random number.
 */
template <typename T, typename G>
inline std::enable_if_t<
       std::is_floating_point<T>::value, T> 
                        generate_canonical(G&& gen) 
{
    long double r = 
                static_cast<long double>(gen.max()) - 
                static_cast<long double>(gen.min()) + 1.0L;
    unsigned long long log2r = pr::log2(r);
    unsigned long long b = pr::numeric_limits<T>::digits;
    unsigned long long m = std::max(1ULL, 
            (log2r + b - 1ULL) / log2r);

    T s = T(0);
    T t = T(1);
    while (m-- != 0) {
        s += T(gen() - gen.min()) * t;
        t *= r;
	}
    s /= t;
    if (s >= T(1)) {
        s = pr::nextafter(T(1), T(0));
    }
    return s;
}

/**
 * @brief Uniform real distribution.
 */
template <typename T = double>
class uniform_real_distribution
{
public:

    // Sanity check.
    static_assert(
        std::is_floating_point<T>::value, 
        "T must be floating point");

    /**
     * @brief Value type.
     */
    typedef T value_type;

    /**
     * @brief Float type.
     */
    typedef T float_type;

    /**
     * @brief Default constructor.
     */
    uniform_real_distribution() = default;

    /**
     * @brief Constructor.
     *
     * @throw std::invalid_argument
     * Unless `a < b`.
     */
    uniform_real_distribution(T a, T b) : 
            a_(a), 
            b_(b)
    {
        if (!(a < b)) {
            throw std::invalid_argument(__PRETTY_FUNCTION__);
        }
    }

    /**
     * @brief Lower bound.
     *
     * @f[
     *      \min[X] = a
     * @f]
     */
    T lower_bound() const
    {
        return a_;
    }

    /**
     * @brief Upper bound.
     *
     * @f[
     *      \max[X] = b
     * @f]
     */
    T upper_bound() const
    {
        return b_;
    }

    /**
     * @brief Mean.
     *
     * @f[
     *      E[X] = \frac{1}{2}(a + b)
     * @f]
     */
    T mean() const
    {
        return T(0.5) * (a_ + b_);
    }

    /**
     * @brief Variance.
     *
     * @f[
     *      V[X] = \frac{1}{12}(b - a)^2
     * @f]
     */
    T variance() const
    {
        return (b_ - a_) * 
               (b_ - a_) / T(12);
    }

    /**
     * @brief Skewness.
     *
     * @f[
     *      \gamma_1[X] = 0
     * @f]
     */
    T skewness() const
    {
        return T(0);
    }

    /**
     * @brief Entropy.
     *
     * @f[
     *      h[X] = \log(b - a)
     * @f]
     */
    T entropy() const
    {
        return pr::log(b_ - a_);
    }

    /**
     * @brief Probability density function.
     *
     * @f[
     *      f(x) =
     *      \begin{cases}
     *          \frac{1}{b - a} & a \le x < b
     *      \\  0               & \text{otherwise}
     *      \end{cases}
     * @f]
     */
    T pdf(T x) const
    {
        if (!(x >= a_ &&
              x <  b_)) {
            return T(0);
        }
        else {
            return T(1) / (b_ - a_);
        }
    }

    /**
     * @brief Cumulative distribution function.
     *
     * @f[
     *      F(x) = 
     *          \begin{cases}
     *              0                  & x < a
     *          \\ \frac{x - a}{b - a} & a \le x < b
     *          \\  1                  & b \le x
     *          \end{cases}
     * @f]
     */
    T cdf(T x) const
    {
        return 
            pr::fmax(T(0),
            pr::fmin(T(1), (x - a_) / (b_ - a_)));
                
    }

    /**
     * @brief Cumulative distribution function inverse.
     *
     * @f[
     *      F^{-1}(u) =
     *          \begin{cases}
     *              (1 - u) a + u b & 0 \le u < 1
     *          \\  \text{NaN}      & \text{otherwise}
     *          \end{cases}
     * @f]
     */
    T cdfinv(T u) const
    {
        if (!(u >= T(0) && 
              u <  T(1))) {
            return pr::numeric_limits<T>::quiet_NaN();
        }
        else {
            return (T(1) - u) * a_ + u * b_;
        }
    }

    /**
     * @brief Generate number.
     */
    template <typename G> 
    T operator()(G&& gen) const
    {
        return cdfinv(pr::generate_canonical<T>(std::forward<G>(gen)));
    }

private:

    /**
     * @brief Lower bound @f$ a @f$.
     */
    T a_ = T(0);

    /**
     * @brief Upper bound @f$ b @f$.
     */
    T b_ = T(1);
};

/**
 * @brief Uniform int distribution.
 */
template <typename T = double>
class uniform_int_distribution
{
public:

    // Sanity check.
    static_assert(
        std::is_floating_point<T>::value, 
        "T must be floating point");

    /**
     * @brief Value type.
     */
    typedef int value_type;

    /**
     * @brief Float type.
     */
    typedef T float_type;

    /**
     * @brief Default constructor.
     */
    uniform_int_distribution() = default;

    /**
     * @brief Constructor.
     *
     * @throw std::invalid_argument
     * Unless `a < b`.
     */
    uniform_int_distribution(int a, int b) : 
            a_(a), 
            b_(b)
    {
        if (!(a < b)) {
            throw std::invalid_argument(__PRETTY_FUNCTION__);
        }
    }

    /**
     * @brief Lower bound.
     *
     * @f[
     *      \min[X] = a
     * @f]
     */
    T lower_bound() const
    {
        return a_;
    }

    /**
     * @brief Upper bound.
     *
     * @f[
     *      \max[X] = b
     * @f]
     *
     * @note
     * Just as in real distributions, the implementation follows the
     * convention that the upper bound is open such that @f$ f(b) = 0 @f$.
     */
    T upper_bound() const
    {
        return b_;
    }

    /**
     * @brief Mean.
     *
     * @f[
     *      E[X] = \frac{1}{2} (a + b - 1)
     * @f]
     */
    T mean() const
    {
        return T(0.5) * (a_ + b_ - 1);
    }

    /**
     * @brief Variance.
     *
     * @f[
     *      V[X] = \frac{1}{12} \left[(b - a)^2 - 1\right]
     * @f]
     */
    T variance() const
    {
        return ((b_ - a_) * 
                (b_ - a_) - 1) / T(12);
    }

    /**
     * @brief Skewness.
     *
     * @f[
     *      \gamma_1[X] = 0
     * @f]
     */
    T skewness() const
    {
        return T(0);
    }

    /**
     * @brief Shannon entropy.
     *
     * @f[
     *      H[X] = \log(b - a)
     * @f]
     */
    T entropy() const
    {
        return pr::log(T(b_ - a_));
    }

    /**
     * @brief Probability mass function.
     *
     * @f[
     *      f(k) =
     *          \begin{cases}
     *              \frac{1}{b - a} & a \le k < b
     *          \\  0               & \text{otherwise}
     *          \end{cases}
     * @f]
     */
    T pmf(int k) const
    {
        if (!(k >= a_ &&
              k <  b_)) {
            return 0;
        }
        else {
            return T(1) / T(b_ - a_);
        }
    }

    /**
     * @brief Cumulative distribution function.
     *
     * @f[
     *      F(x) = 
     *          \begin{cases}
     *              0                                 & k < a
     *          \\  \frac{\lceil k \rceil - a}{b - a} & a \le k < b
     *          \\  1                                 & b \le k
     *          \end{cases}
     * @f]
     */
    T cdf(T k) const
    {
        return 
            pr::fmax(T(0),
            pr::fmin(T(1), (pr::ceil(k) - a_) / (b_ - a_)));
    }

    /**
     * @brief Cumulative distribution function inverse.
     *
     * @f[
     *      F^{-1}(u) =
     *          \begin{cases}
     *              \lfloor (1 - u) a + u b \rfloor & 0 \le u < 1
     *          \\  \text{NaN}                      & \text{otherwise}
     *          \end{cases}
     * @f]
     */
    T cdfinv(T u) const
    {
        if (!(u >= T(0) && 
              u <  T(1))) {
            return pr::numeric_limits<T>::quiet_NaN();
        }
        else {
            return pr::floor((T(1) - u) * a_ + u * b_);
        }
    }

    /**
     * @brief Generate number.
     */
    template <typename G>
    int operator()(G&& gen) const
    {
        return int(cdfinv(pr::generate_canonical<T>(std::forward<G>(gen))));
    }

private:

    /**
     * @brief Lower bound @f$ a @f$.
     */
    int a_ = 0;

    /**
     * @brief Upper bound @f$ b @f$.
     */
    int b_ = 1;
};

/**
 * @brief Bernoulli distribution.
 */
template <typename T = double>
class bernoulli_distribution
{
public:

    // Sanity check.
    static_assert(
        std::is_floating_point<T>::value, 
        "T must be floating point");

    /**
     * @brief Value type.
     */
    typedef int value_type;

    /**
     * @brief Float type.
     */
    typedef T float_type;

    /**
     * @brief Default constructor.
     */
    bernoulli_distribution() = default;

    /**
     * @brief Constructor.
     *
     * @throw std::invalid_argument
     * Unless `p >= 0 && p <= 1`.
     */
    bernoulli_distribution(T p) : p_(p), q_(T(1) - p)
    {
        if (!(p >= T(0)) ||
            !(p <= T(1))) {
            throw std::invalid_argument(__PRETTY_FUNCTION__);
        }
    }

    /**
     * @brief Lower bound.
     *
     * @f[
     *      \min[X] = 0
     * @f]
     */
    T lower_bound() const
    {
        return T(0);
    }

    /**
     * @brief Upper bound.
     *
     * @f[
     *      \max[X] = 2
     * @f]
     *
     * @note
     * Just as in real distributions, the implementation follows the
     * convention that the upper bound is open such that @f$ f(b) = 0 @f$.
     */
    T upper_bound() const
    {
        return T(2);
    }

    /**
     * @brief Mean.
     *
     * @f[
     *      E[X] = p
     * @f]
     */
    T mean() const
    {
        return p_;
    }

    /**
     * @brief Variance.
     *
     * @f[
     *      V[X] = pq
     * @f]
     */
    T variance() const
    {
        return p_ * q_;
    }

    /**
     * @brief Skewness.
     *
     * @f[
     *      \gamma_1[X] = \frac{1 - 2p}{\sqrt{pq}}
     * @f]
     */
    T skewness() const
    {
        return (T(1) - T(2) * p_) / pr::sqrt(p_ * q_);
    }

    /**
     * @brief Shannon entropy.
     *
     * @f[
     *      H[X] = -p\log(p) - q\log(q)
     * @f]
     */
    T entropy() const
    {
        T pval = p_ * pr::log(p_);
        T qval = q_ * pr::log(q_);
        if (!pr::isfinite(pval) ||
            !pr::isfinite(qval)) {
            return T(1);
        }
        else {
            return -pval - qval;
        }
    }

    /**
     * @brief Probability mass function.
     *
     * @f[
     *      f(x) = 
     *          \begin{cases}
     *              q & k = 0
     *          \\  p & k = 1
     *          \\  0 & \text{otherwise}
     *          \end{cases}
     * @f]
     */
    T pmf(int k) const
    {
        if (k == 0) return q_;
        if (k == 1) return p_;
        return T(0);
    }

    /**
     * @brief Cumulative distribution function.
     *
     * @f[
     *      F(x) = 
     *          \begin{cases}
     *              0 & x < 0 
     *          \\  q & 0 \le x < 1
     *          \\  1 & 1 \le x
     *          \end{cases}
     * @f]
     */
    T cdf(T x) const
    {
        if (!(x >= T(0))) {
            return T(0);
        }
        else if (!(x < T(1))) {
            return T(1);
        }
        else {
            return q_;
        }
    }

    /**
     * @brief Cumulative distribution function inverse.
     *
     * @f[
     *      F^{-1}(u) = 
     *          \begin{cases}
     *              0 & 0 \le u < q
     *          \\  1 & q \le u < 1
     *          \\ \text{NaN} & \text{otherwise}
     *          \end{cases}
     * @f]
     */
    T cdfinv(T u) const
    {
        if (!(u >= T(0) &&
              u <  T(1))) {
            return pr::numeric_limits<T>::quiet_NaN();
        }
        else {
            if (u < q_) {
                return T(0);
            }
            else {
                return T(1);
            }
        }
    }

    /**
     * @brief Generate number.
     */
    template <typename G>
    int operator()(G&& gen) const
    {
        return int(cdfinv(pr::generate_canonical<T>(std::forward<G>(gen))));
    }

private:

    /**
     * @brief Probability of success @f$ p @f$.
     */
    T p_ = T(0.5);

    /**
     * @brief Probability of failure @f$ q = 1 - p @f$.
     */
    T q_ = T(0.5);
};

/**
 * @brief Exponential distribution.
 */
template <typename T = double>
class exponential_distribution
{
public:

    // Sanity check.
    static_assert(
        std::is_floating_point<T>::value, 
        "T must be floating point");

    /**
     * @brief Value type.
     */
    typedef T value_type;

    /**
     * @brief Float type.
     */
    typedef T float_type;

    /**
     * @brief Default constructor.
     */
    exponential_distribution() = default;

    /**
     * @brief Constructor.
     *
     * @throw std::invalid_argument
     * Unless `lambda > 0`.
     */
    exponential_distribution(T lambda) : lambda_(lambda)
    {
        if (!(lambda > T(0))) {
            throw std::invalid_argument(__PRETTY_FUNCTION__);
        }
    }

    /**
     * @brief Lower bound.
     *
     * @f[
     *      \min[X] = 0
     * @f]
     */
    T lower_bound() const
    {
        return T(0);
    }

    /**
     * @brief Upper bound.
     *
     * @f[
     *      \max[X] = \infty
     * @f]
     */
    T upper_bound() const
    {
        return pr::numeric_limits<T>::infinity();
    }

    /**
     * @brief Mean.
     *
     * @f[
     *      E[X] = \frac{1}{\lambda}
     * @f]
     */
    T mean() const
    {
        return T(1) / lambda_;
    }

    /**
     * @brief Variance.
     *
     * @f[
     *      V[X] = \frac{1}{\lambda^2}
     * @f]
     */
    T variance() const
    {
        return T(1) / (lambda_ * lambda_);
    }

    /**
     * @brief Skewness.
     *
     * @f[
     *      \gamma_1[X] = 2
     * @f]
     */
    T skewness() const
    {
        return T(2);
    }

    /**
     * @brief Differential Shannon entropy.
     *
     * @f[
     *      h[X] = 1 - \log(\lambda)
     * @f]
     */
    T entropy() const
    {
        return T(1) - pr::log(lambda_);
    }

    /**
     * @brief Probability density function.
     *
     * @f[
     *      f(x) =
     *          \begin{cases}
     *              \lambda e^{-\lambda x} & 0 \le x
     *          \\  0                      & \text{otherwise}
     *          \end{cases}
     * @f]
     */
    T pdf(T x) const
    {
        if (!(x >= T(0))) {
            return T(0);
        }
        else {
            return lambda_ * pr::exp(-lambda_ * x);
        }
    }

    /**
     * @brief Cumulative distribution function.
     *
     * @f[
     *      F(x) =
     *          \begin{cases}
     *              1 - e^{-\lambda x} & 0 \le x
     *          \\  0                  & \text{otherwise}
     *          \end{cases}
     * @f]
     */
    T cdf(T x) const
    {
        if (!(x >= T(0))) {
            return T(0);
        }
        else {
            return -pr::expm1(-lambda_ * x);
        }
    }

    /**
     * @brief Cumulative distribution function inverse.
     *
     * @f[
     *      F^{-1}(u) =
     *          \begin{cases}
     *             -\frac{1}{\lambda} \log(1 - u) & 0 \le u < 1
     *          \\  \text{NaN}                    & \text{otherwise}
     *          \end{cases}
     * @f]
     */
    T cdfinv(T u) const
    {
        if (!(u >= T(0) && 
              u <  T(1))) {
            return pr::numeric_limits<T>::quiet_NaN();
        }
        else {
            T log_term = 
                u < T(0.5) ?
                pr::log1p(-u) : // More accurate for small arguments?
                pr::log(T(1) - u);
            return -log_term / lambda_;
        }
    }

    /**
     * @brief Generate number.
     */
    template <typename G> 
    T operator()(G&& gen) const
    {
        return cdfinv(pr::generate_canonical<T>(std::forward<G>(gen)));
    }

private:

    /**
     * @brief Rate @f$ \lambda @f$.
     */
    T lambda_ = T(1);
};

/**
 * @brief Error function inverse.
 */
template <typename T>
inline std::enable_if_t<std::is_floating_point<T>::value, T> erfinv(T y)
{
    T w = -pr::log((T(1) - y) * (T(1) + y));
    T p;
    if (w < T(5)) {
        w = w - T(2.5);
        p = T(2.81022636e-08);
        p = pr::fma(p, w, T(3.43273939e-07));
        p = pr::fma(p, w, T(-3.5233877e-06));
        p = pr::fma(p, w, T(-4.39150654e-06));
        p = pr::fma(p, w, T(0.00021858087));
        p = pr::fma(p, w, T(-0.00125372503));
        p = pr::fma(p, w, T(-0.00417768164));
        p = pr::fma(p, w, T(0.246640727));
        p = pr::fma(p, w, T(1.50140941));
    }
    else {
        w = pr::sqrt(w) - 3;
        p = T(-0.000200214257);
        p = pr::fma(p, w, T(0.000100950558));
        p = pr::fma(p, w, T(0.00134934322));
        p = pr::fma(p, w, T(-0.00367342844));
        p = pr::fma(p, w, T(0.00573950773));
        p = pr::fma(p, w, T(-0.0076224613));
        p = pr::fma(p, w, T(0.00943887047));
        p = pr::fma(p, w, T(1.00167406));
        p = pr::fma(p, w, T(2.83297682));
    }
    return p * y;
}

/**
 * @brief Normal distribution.
 */
template <typename T = double>
class normal_distribution
{
public:

    // Sanity check.
    static_assert(
        std::is_floating_point<T>::value, 
        "T must be floating point");

    /**
     * @brief Value type.
     */
    typedef T value_type;

    /**
     * @brief Float type.
     */
    typedef T float_type;

    /**
     * @brief Default constructor.
     */
    normal_distribution() = default;

    /**
     * @brief Constructor.
     *
     * @throw std::invalid_argument
     * Unless `sigma > 0`.
     */
    normal_distribution(T mu, T sigma) : mu_(mu), sigma_(sigma)
    {
        if (!(sigma > T(0))) {
            throw std::invalid_argument(__PRETTY_FUNCTION__);
        }
    }

    /**
     * @brief Lower bound.
     *
     * @f[
     *      \min[X] = -\infty
     * @f]
     */
    T lower_bound() const
    {
        return -pr::numeric_limits<T>::infinity();
    }

    /**
     * @brief Upper bound.
     *
     * @f[
     *      \max[X] = \infty
     * @f]
     */
    T upper_bound() const
    {
        return pr::numeric_limits<T>::infinity();
    }

    /**
     * @brief Mean.
     *
     * @f[
     *      E[X] = \mu
     * @f]
     */
    T mean() const
    {
        return mu_;
    }

    /**
     * @brief Variance.
     *
     * @f[
     *      V[X] = \sigma^2
     * @f]
     */
    T variance() const
    {
        return sigma_ * sigma_;
    }

    /**
     * @brief Skewness.
     *
     * @f[
     *      \gamma_1[X] = 0
     * @f]
     */
    T skewness() const
    {
        return T(0);
    }

    /**
     * @brief Differential Shannon entropy.
     *
     * @f[
     *      h[X] = \frac{1}{2} \log\left(2\pi e \sigma^2\right)
     * @f]
     */
    T entropy() const
    {
        return
            pr::log(T(2) * 
            pr::numeric_constants<T>::M_pi() * 
            pr::numeric_constants<T>::M_e() * 
            sigma_ * sigma_) / T(2);
    }

    /**
     * @brief Probability density function.
     *
     * @f[
     *      f(x) = 
     *          \frac{1}{\sigma}
     *          \frac{1}{\sqrt{2\pi}}
     *          \exp\left[-\frac{1}{2}
     *              \left(\frac{x - \mu}{\sigma}
     *              \right)^2
     *              \right]
     * @f]
     */
    T pdf(T x) const
    {
        T xi = (x - mu_) / sigma_;
        return pr::numeric_constants<T>::M_2_sqrtpi() *
               pr::numeric_constants<T>::M_sqrt1_2() / T(2) *
                            pr::exp(-xi * xi / T(2)) / sigma_;
    }

    /**
     * @brief Cumulative distribution function.
     *
     * @f[
     *      F(x) = 
     *          \frac{1}{2}
     *          \erfc\left[-\sqrt{\frac{1}{2}}
     *               \left(\frac{x - \mu}{\sigma}
     *               \right)
     *               \right]
     * @f]
     */
    T cdf(T x) const 
    {
        T xi = (x - mu_) / sigma_;
        return pr::erfc(-pr::numeric_constants<T>::M_sqrt1_2() * xi) / T(2);
    }

    /**
     * @brief Cumulative distribution function inverse.
     *
     * @f[
     *      F^{-1}(u) = 
     *          \begin{cases}
     *              \mu + \sigma 
     *              \sqrt{2}\erf^{-1}(2 u - 1) & 0 \le u < 1
     *          \\  \text{NaN}                 & \text{otherwise}
     *          \end{cases}
     * @f]
     */
    T cdfinv(T u) const
    {
        if (!(u >= T(0) &&
              u <  T(1))) {
            return pr::numeric_limits<T>::quiet_NaN();
        }
        else {
            return mu_ + sigma_ *
                   pr::numeric_constants<T>::M_sqrt2() *
                   pr::erfinv(T(2) * u - T(1));
        }
    }

    /**
     * @brief Generate number.
     */
    template <typename G> 
    T operator()(G&& gen) const
    {
        return cdfinv(pr::generate_canonical<T>(std::forward<G>(gen)));
    }

protected:

    /**
     * @brief Mean @f$ \mu @f$.
     */
    T mu_ = T(0);

    /**
     * @brief Standard deviation @f$ \sigma @f$.
     */
    T sigma_ = T(1);
};

/**
 * @brief Log-normal distribution.
 */
template <typename T = double> 
class lognormal_distribution : public normal_distribution<T>
{
public:

    // Sanity check.
    static_assert(
        std::is_floating_point<T>::value, 
        "T must be floating point");

    /**
     * @brief Value type.
     */
    typedef T value_type;

    /**
     * @brief Float type.
     */
    typedef T float_type;

    // Inherit constructors.
    using normal_distribution<T>::normal_distribution;

    /**
     * @brief Lower bound.
     *
     * @f[
     *      \min[X] = 0
     * @f]
     */
    T lower_bound() const
    {
        return T(0);
    }

    /**
     * @brief Upper bound.
     *
     * @f[
     *      \max[X] = \infty
     * @f]
     */
    T upper_bound() const
    {
        return pr::numeric_limits<T>::infinity();
    }

    /**
     * @brief Mean.
     *
     * @f[
     *      E[X] = \exp\left(\mu + \frac{1}{2}\sigma^2\right)
     * @f]
     */
    T mean() const
    {
        return pr::exp(mu_ + T(0.5) * sigma_ * sigma_);
    }

    /**
     * @brief Variance.
     *
     * @f[
     *      V[X] = 
     *          \left[\exp\left(\sigma^2\right) - 1\right]
     *                \exp\left(2\mu + \sigma^2\right)
     * @f]
     */
    T variance() const
    {
        return pr::expm1(sigma_ * sigma_) *
               pr::exp(T(2) * mu_ + sigma_ * sigma_);
    }

    /**
     * @brief Skewness.
     *
     * @f[
     *      \gamma_1[X] = 
     *          \sqrt{\exp\left(\sigma^2\right) - 1} 
     *                    \left[\exp\left(\sigma^2\right) + 2\right]
     * @f]
     */
    T skewness() const
    {
        return pr::sqrt(pr::expm1(sigma_ * sigma_)) *
                       (pr::exp(sigma_ * sigma_) + T(2));
    }

    /**
     * @brief Differential Shannon entropy.
     *
     * @f[
     *      h[X] = 
     *          \log_2\left[\sigma \sqrt{2\pi}
     *          \exp\left(\mu + \frac{1}{2}\right)\right]
     * @f]
     */
    T entropy() const
    {
        return pr::log2(
               pr::numeric_constants<T>::M_2_sqrtpi() *
               pr::numeric_constants<T>::M_sqrt1_2() / T(2) *
                    sigma_ * pr::exp(mu_ + T(0.5)));
    }

    /**
     * @brief Probability density function.
     */
    T pdf(T x) const
    {
        if (!(x > T(0))) {
            return T(0);
        }
        else {
            return normal_distribution<T>::pdf(pr::log(x)) / x;
        }
    }

    /**
     * @brief Cumulative distribution function.
     */
    T cdf(T x) const
    {
        if (!(x > T(0))) {
            return T(0);
        }
        else {
            return normal_distribution<T>::cdf(pr::log(x));
        }
    }

    /**
     * @brief Cumulative distribution function inverse.
     */
    T cdfinv(T u) const
    {
        if (!(u >= T(0) &&
              u <  T(1))) {
            return pr::numeric_limits<T>::quiet_NaN();
        }
        else {
            return pr::exp(normal_distribution<T>::cdfinv(u));
        }
    }

    /**
     * @brief Generate number.
     */
    template <typename G> 
    T operator()(G&& gen) const
    {
        return cdfinv(pr::generate_canonical<T>(std::forward<G>(gen)));
    }

private:

    // Make member variable visible.
    using normal_distribution<T>::mu_;

    // Make member variable visible.
    using normal_distribution<T>::sigma_;
};

/**
 * @brief Logistic distribution.
 */
template <typename T = double>
class logistic_distribution
{
public:

    // Sanity check.
    static_assert(
        std::is_floating_point<T>::value, 
        "T must be floating point");

    /**
     * @brief Value type.
     */
    typedef T value_type;

    /**
     * @brief Float type.
     */
    typedef T float_type;

    /**
     * @brief Default constructor.
     */
    logistic_distribution() = default;

    /**
     * @brief Constructor.
     *
     * @throw std::invalid_argument
     * Unless `s > 0`.
     */
    logistic_distribution(T mu, T s) : mu_(mu), s_(s)
    {
        if (!(s > T(0))) {
            throw std::invalid_argument(__PRETTY_FUNCTION__);
        }
    }

    /**
     * @brief Lower bound.
     *
     * @f[
     *      \min[X] = -\infty
     * @f]
     */
    T lower_bound() const
    {
        return -pr::numeric_limits<T>::infinity();
    }

    /**
     * @brief Upper bound.
     *
     * @f[
     *      \max[X] = \infty
     * @f]
     */
    T upper_bound() const
    {
        return pr::numeric_limits<T>::infinity();
    }

    /**
     * @brief Mean.
     *
     * @f[
     *      E[X] = \mu
     * @f]
     */
    T mean() const
    {
        return mu_;
    }

    /**
     * @brief Variance.
     *
     * @f[
     *      V[X] = \frac{1}{3} \pi^2 s^2
     * @f]
     */
    T variance() const
    {
        return 
            pr::numeric_constants<T>::M_pi() *
            pr::numeric_constants<T>::M_pi() * 
            s_ * s_ / T(3);
    }

    /**
     * @brief Skewness.
     *
     * @f[
     *      \gamma_1[X] = 0
     * @f]
     */
    T skewness() const
    {
        return T(0);
    }

    /**
     * @brief Differential Shannon entropy.
     *
     * @f[
     *      h[X] = \log(s) + 2
     * @f]
     */
    T entropy() const
    {
        return pr::log(s_) + 2;
    }

    /**
     * @brief Probability density function.
     *
     * @f[
     *      f(x) = 
     *          \frac{1}{s}
     *          \frac{e^{-\xi}}{\left(1 + e^{-\xi}\right)^2}
     *          \Bigg|_{\xi \gets (x - \mu) / s}
     * @f]
     */
    T pdf(T x) const
    {
        T exp_term = pr::exp(-(x - mu_) / s_);
        return exp_term / (s_ * 
                        (T(1) + exp_term) * 
                        (T(1) + exp_term));
    }

    /**
     * @brief Cumulative distribution function.
     *
     * @f[
     *      F(x) = 
     *          \frac{1}{1 + e^{-\xi}}
     *          \Bigg|_{\xi \gets (x - \mu) / s}
     * @f]
     */
    T cdf(T x) const
    {
        return T(1) / (T(1) + pr::exp(-(x - mu_) / s_));
    }

    /**
     * @brief Cumulative distribution function inverse.
     *
     * @f[
     *      F^{-1}(u) = 
     *          \begin{cases}
     *              \mu - s \log(1/u - 1) & 0 \le u < 1
     *          \\  \text{NaN}            & \text{otherwise}
     *          \end{cases}
     * @f]
     */
    T cdfinv(T u) const
    {
        if (!(u >= T(0) &&
              u <  T(1))) {
            return pr::numeric_limits<T>::quiet_NaN();
        }
        else {
            return mu_ - s_ * pr::log(T(1) / u - T(1));
        }
    }

    /**
     * @brief Generate number.
     */
    template <typename G> 
    T operator()(G&& gen) const
    {
        return cdfinv(pr::generate_canonical<T>(std::forward<G>(gen)));
    }

private:

    /**
     * @brief Mean @f$ \mu @f$.
     */
    T mu_ = T(0);

    /**
     * @brief Scale (or spread) @f$ s @f$.
     */
    T s_ = T(1);
};

/**
 * @brief Tanh distribution.
 */
template <typename T = double>
class tanh_distribution
{
public:

    // Sanity check.
    static_assert(
        std::is_floating_point<T>::value, 
        "T must be floating point");

    /**
     * @brief Value type.
     */
    typedef T value_type;

    /**
     * @brief Float type.
     */
    typedef T float_type;

    /**
     * @brief Default constructor.
     */
    tanh_distribution() = default;

    /**
     * @brief Constructor.
     *
     * @throw std::invalid_argument
     * Unless `s > 0`.
     */
    tanh_distribution(T mu, T s) : mu_(mu), s_(s)
    {
        if (!(s > T(0))) {
            throw std::invalid_argument(__PRETTY_FUNCTION__);
        }
    }

    /**
     * @brief Lower bound.
     *
     * @f[
     *      \min[X] = -\infty
     * @f]
     */
    T lower_bound() const
    {
        return -pr::numeric_limits<T>::infinity();
    }

    /**
     * @brief Upper bound.
     *
     * @f[
     *      \max[X] = \infty
     * @f]
     */
    T upper_bound() const
    {
        return pr::numeric_limits<T>::infinity();
    }

    /**
     * @brief Mean.
     *
     * @f[
     *      E[X] = \mu
     * @f]
     */
    T mean() const
    {
        return mu_;
    }

    /**
     * @brief Variance.
     *
     * @f[
     *      V[X] = \frac{1}{12} \pi^2 s^2
     * @f]
     */
    T variance() const
    {
        return 
            pr::numeric_constants<T>::M_pi() * 
            pr::numeric_constants<T>::M_pi() * 
            s_ * s_ / T(12);
    }

    /**
     * @brief Skewness.
     *
     * @f[
     *      \gamma_1[X] = 0
     * @f]
     */
    T skewness() const
    {
        return T(0);
    }

    /**
     * @brief Differential Shannon entropy.
     *
     * @f[
     *      h[X] = \log\left(\frac{1}{2} e^2 s\right)
     * @f]
     */
    T entropy() const
    {
        return pr::log(T(0.5) * 
                    pr::numeric_constants<T>::M_e() * 
                    pr::numeric_constants<T>::M_e() * s_);
    }

    /**
     * @brief Probability density function.
     *
     * @f[
     *      f(x) = 
     *          \frac{1}{2s}
     *          \operatorname{sech}^{2}\left(\frac{x - \mu}{s}\right)
     * @f]
     */
    T pdf(T x) const
    {
        return T(0.5) / (s_ * pr::nthpow(pr::cosh((x - mu_) / s_), 2));
    }

    /**
     * @brief Cumulative distribution function.
     *
     * @f[
     *      F(x) = 
     *          \frac{1}{2}
     *          \operatorname{tanh}\left(\frac{x - \mu}{s}\right) + 
     *                  \frac{1}{2}
     * @f]
     */
    T cdf(T x) const
    {
        return T(0.5) * pr::tanh((x - mu_) / s_) + T(0.5);
    }

    /**
     * @brief Cumulative distribution function inverse.
     *
     * @f[
     *      F^{-1}(u) = 
     *          \begin{cases}
     *              \mu + s \operatorname{arctanh}(2u - 1)  & 0 \le u < 1
     *          \\  \text{NaN}                              & \text{otherwise}
     *          \end{cases}
     * @f]
     */
    T cdfinv(T u) const
    {
        if (!(u >= T(0) &&
              u <  T(1))) {
            return pr::numeric_limits<T>::quiet_NaN();
        }
        else {
            return pr::atanh(T(2) * u - T(1)) * s_ + mu_;
        }
    }

    /**
     * @brief Generate number.
     */
    template <typename G> 
    T operator()(G&& gen) const
    {
        return cdfinv(pr::generate_canonical<T>(std::forward<G>(gen)));
    }

private:

    /**
     * @brief Mean @f$ \mu @f$.
     */
    T mu_ = T(0);

    /**
     * @brief Scale (or spread) @f$ s @f$.
     */
    T s_ = T(1);
};

/**
 * @brief Weibull distribution.
 */
template <typename T = double>
class weibull_distribution
{
public:

    // Sanity check.
    static_assert(
        std::is_floating_point<T>::value, 
        "T must be floating point");

    /**
     * @brief Value type.
     */
    typedef T value_type;

    /**
     * @brief Float type.
     */
    typedef T float_type;

    /**
     * @brief Default constructor.
     */
    weibull_distribution() = default;

    /**
     * @brief Constructor.
     *
     * @throw std::invalid_argument
     * Unless `lambda > 0 && k > 0`.
     */
    weibull_distribution(T lambda, T k) : lambda_(lambda), k_(k)
    {
        if (!(lambda > T(0)) ||
            !(k > T(0))) {
            throw std::invalid_argument(__PRETTY_FUNCTION__);
        }
    }

    /**
     * @brief Lower bound.
     *
     * @f[
     *      \min[X] = 0
     * @f]
     */
    T lower_bound() const
    {
        return T(0);
    }

    /**
     * @brief Upper bound.
     *
     * @f[
     *      \max[X] = \infty
     * @f]
     */
    T upper_bound() const
    {
        return pr::numeric_limits<T>::infinity();
    }

    /**
     * @brief Mean.
     *
     * @f[
     *      E[X] = 
     *          \lambda 
     *          \Gamma\left(1 + \frac{1}{k}\right)
     * @f]
     */
    T mean() const
    {
        return lambda_ * pr::tgamma(T(1) + T(1) / k_);
    }

    /**
     * @brief Variance.
     *
     * @f[
     *      V[X] = 
     *          \lambda^2 \left[
     *          \Gamma\left(1 + \frac{2}{k}\right) -
     *          \Gamma\left(1 + \frac{1}{k}\right)^2
     *          \right]
     * @f]
     */
    T variance() const
    {
        T a0 = pr::tgamma(T(1) + T(2) / k_);
        T a1 = pr::tgamma(T(1) + T(1) / k_);
        return lambda_ * lambda_ * (a0 - a1 * a1);
    }

    /**
     * @brief Skewness.
     *
     * @f[
     *      \gamma_1[X] =
     *          \frac{1}{\sigma^3}
     *          \left[\lambda^3
     *          \Gamma\left(1 + \frac{3}{k}\right) - 
     *                  \mu^3 - 3\mu\sigma^2\right]
     * @f]
     */
    T skewness() const
    {
        T mu = mean();
        T sigma2 = variance();
        T sigma3 = sigma2 * pr::sqrt(sigma2);
        return (pr::nthpow(lambda_, 3) *
                pr::tgamma(T(1) + T(3) / k_) - pr::nthpow(mu, 3) -
                T(3) * mu * sigma2) / sigma3;
    }

    /**
     * @brief Differential Shannon entropy.
     *
     * @f[
     *      h[X] = 1 + 
     *          \gamma\frac{k - 1}{k} + 
     *          \log\frac{\lambda}{k}
     * @f]
     */
    T entropy() const
    {
        return T(1) + 
               pr::numeric_constants<T>::M_gamma() * (k_ - 1) / k_ +
               pr::log(lambda_ / k_);
    }

    /**
     * @brief Probability density function.
     *
     * @f[
     *      f(x) =
     *          \begin{cases}
     *              \frac{k}{\lambda}
     *              \left(\frac{x}{\lambda}
     *              \right)^{k-1}
     *              \exp\left[
     *                 -\left(\frac{x}{\lambda}
     *                      \right)^k
     *                      \right]         & 0 \le x
     *          \\  0                       & \text{otherwise}
     *          \end{cases}
     * @f]
     */
    T pdf(T x) const
    {
        if (!(x >= T(0))) {
            return T(0);
        }
        else {
            return k_ / lambda_ * 
                   pr::pow(x / lambda_, k_ - T(1)) *
                   pr::exp(-pr::pow(x / lambda_, k_));
        }
    }

    /**
     * @brief Cumulative distribution function.
     *
     * @f[
     *      F(x) =
     *          \begin{cases}
     *              1 - 
     *              \exp\left[
     *                 -\left(\frac{x}{\lambda}
     *                      \right)^k
     *                      \right]         & 0 \le x
     *          \\  0                       & \text{otherwise}
     *          \end{cases}
     * @f]
     */
    T cdf(T x) const
    {
        if (!(x >= T(0))) {
            return T(0);
        }
        else {
            return T(1) - pr::exp(-pr::pow(x / lambda_, k_));
        }
    }

    /**
     * @brief Cumulative distribution function inverse.
     *
     * @f[
     *      F^{-1}(u) =
     *          \begin{cases}
     *              \lambda[
     *                  -\log(1 - u)]^{1 / k}   & 0 \le u < 1
     *          \\  \text{NaN}                  & \text{otherwise}
     *          \end{cases}
     * @f]
     */
    T cdfinv(T u) const
    {
        if (!(u >= T(0) &&
              u <  T(1))) {
            return pr::numeric_limits<T>::quiet_NaN();
        }
        else {
            return lambda_ * pr::pow(-pr::log(T(1) - u), T(1) / k_);
        }
    }

    /**
     * @brief Generate number.
     */
    template <typename G> 
    T operator()(G&& gen) const
    {
        return cdfinv(pr::generate_canonical<T>(std::forward<G>(gen)));
    }

private:

    /**
     * @brief Scale @f$ \lambda @f$.
     */
    T lambda_ = T(1);

    /**
     * @brief Shape @f$ k @f$.
     */
    T k_ = T(1);
};


/**
 * @brief Subset distribution wrapper.
 */
template <typename Tbase>
class subset_distribution_wrapper : public Tbase
{
public:

    /**
     * @brief Base.
     */
    typedef Tbase base;

    /**
     * @brief Value type.
     */
    typedef typename base::value_type value_type;

    /**
     * @brief Float type.
     */
    typedef typename base::float_type float_type;

    // Sanity check.
    static_assert(
        std::is_same<
            value_type, 
            float_type>::value,
        "Tbase must be a real distribution");

    /**
     * @brief Constructor.
     *
     * @throw std::invalid_argument
     * If invalid bounds.
     */
    template <typename... Targs>
    subset_distribution_wrapper(
            float_type x0,
            float_type x1,
            Targs&&... args) :
                Tbase(std::forward<Targs>(args)...),
                x0_(x0),
                x1_(x1)
    {
        x0_ = pr::fmax(x0_, Tbase::lower_bound());
        x1_ = pr::fmin(x1_, Tbase::upper_bound());
        cdf_x0_ = Tbase::cdf(x0_);
        cdf_x1_ = Tbase::cdf(x1_);
        fac_ = float_type(1) / (cdf_x1_ - cdf_x0_);
        if (!(x0_ < x1_) ||
            !pr::isfinite(fac_)) {
            throw std::invalid_argument(__PRETTY_FUNCTION__);
        }
    }

    /**
     * @brief Lower bound.
     *
     * @f[
     *      \min[X] = x_0
     * @f]
     */
    float_type lower_bound() const { return x0_; }

    /**
     * @brief Upper bound.
     *
     * @f[
     *      \max[X] = x_1
     * @f]
     */
    float_type upper_bound() const { return x1_; }

    /**
     * @brief Mean.
     *
     * @note
     * This is not feasible to compute in general, so it is deleted.
     */
    float_type mean() const = delete;

    /**
     * @brief Variance.
     *
     * @note
     * This is not feasible to compute in general, so it is deleted.
     */
    float_type variance() const = delete;

    /**
     * @brief Skewness.
     *
     * @note
     * This is not feasible to compute in general, so it is deleted.
     */
    float_type skewness() const = delete;

    /**
     * @brief Entropy.
     *
     * @note
     * This is not feasible to compute in general, so it is deleted.
     */
    float_type entropy() const = delete;

    /**
     * @brief Probability density function.
     *
     * @f[
     *      g(x) = 
     *          \begin{cases}
     *              \frac{1}{F_1 - F_0} f(x) & x_0 \le x < x_1
     *          \\  0                        & \text{otherwise}
     *          \end{cases}
     * @f]
     */
    float_type pdf(float_type x) const
    {
        if (!(x >= x0_ &&
              x <  x1_)) {
            return float_type(0);
        }
        else {
            return fac_ * Tbase::pdf(x);
        }
    }

    /**
     * @brief Cumulative distribution function.
     *
     * @f[
     *      G(x) =
     *          \begin{cases}
     *              0                                & x < x_0
     *          \\  \frac{1}{F_1 - F_0} (F(x) - F_0) & x_0 \le x < x_1
     *          \\  1                                & x_1 \le x
     *          \end{cases}
     * @f]
     */
    float_type cdf(float_type x) const
    {
        if (!(x >= x0_)) {
            return float_type(0);
        }
        else if (!(x < x1_)) {
            return float_type(1);
        }
        else {
            return fac_ * (Tbase::cdf(x) - cdf_x0_);
        }
    }

    /**
     * @brief Cumulative distribution function inverse.
     *
     * @f[
     *      G^{-1}(u) = 
     *          \begin{cases}
     *              F^{-1}((1 - u) F_0 + u F_1) & 0 \le u < 1
     *          \\  \text{NaN}                  & \text{otherwise}
     *          \end{cases}
     * @f]
     */
    float_type cdfinv(float_type u) const
    {
        if (!(u >= float_type(0) &&
              u <  float_type(1))) {
            return pr::numeric_limits<float_type>::quiet_NaN();
        }
        else {
            return Tbase::cdfinv((1 - u) * cdf_x0_ + u * cdf_x1_);
        }
    }

    /**
     * @brief Generate number.
     */
    template <typename G>
    float_type operator()(G&& gen) const
    {
        return cdfinv(std::forward<G>(gen));
    }

private:

    /**
     * @brief Lower bound @f$ x_0 @f$.
     */
    float_type x0_ = -pr::numeric_limits<float_type>::infinity();

    /**
     * @brief Upper bound @f$ x_1 @f$.
     */
    float_type x1_ = +pr::numeric_limits<float_type>::infinity();

    /**
     * @brief Pre-computed term @f$ F_0 = F(x_0) @f$.
     */
    float_type cdf_x0_ = 0;

    /**
     * @brief Pre-computed term @f$ F_1 = F(x_1) @f$.
     */
    float_type cdf_x1_ = 1;

    /**
     * @brief Pre-computed term @f$ 1 / (F_1 - F_0) @f$.
     */
    float_type fac_ = 1;
};

/**
 * @name Subset distributions
 */
/**@{*/

/**
 * @brief Subset exponential distribution.
 */
template <typename T = double>
using subset_exponential_distribution = 
      subset_distribution_wrapper<exponential_distribution<T>>;

/**
 * @brief Subset normal distribution.
 */
template <typename T = double>
using subset_normal_distribution = 
      subset_distribution_wrapper<normal_distribution<T>>;

/**
 * @brief Subset lognormal distribution.
 */
template <typename T = double>
using subset_lognormal_distribution = 
      subset_distribution_wrapper<lognormal_distribution<T>>;

/**
 * @brief Subset logistic distribution.
 */
template <typename T = double>
using subset_logistic_distribution = 
      subset_distribution_wrapper<logistic_distribution<T>>;

/**
 * @brief Subset tanh distribution.
 */
template <typename T = double>
using subset_tanh_distribution = 
      subset_distribution_wrapper<tanh_distribution<T>>;

/**
 * @brief Subset Weibull distribution.
 */
template <typename T = double>
using subset_weibull_distribution = 
      subset_distribution_wrapper<weibull_distribution<T>>;

/**@}*/

// Under construction
#if 0
/**
 * @brief Piecewise linear distribution.
 */
template <typename T = double>
class piecewise_linear_distribution
{
public:

    // Sanity check.
    static_assert(
        std::is_floating_point<T>::value,
        "T must be floating point");

    /**
     * @brief Value type.
     */
    typedef T value_type;

    /**
     * @brief Float type.
     */
    typedef T float_type;

    /**
     * @brief Default constructor.
     */
    piecewise_linear_distribution() = default;

    /**
     * @brief Constructor.
     *
     * @throw std::invalid_argument
     * If `!(std::distance(xfrom, xto) > 1)`.
     */
    template <
        typename Tinput0, 
        typename Tinput1
        >
    piecewise_linear_distribution(
            Tinput0 xfrom, Tinput0 xto,
            Tinput1 yfrom)
    {
        // Invalid?
        if (!(std::distance(xfrom, xto) > 1)) {
            throw std::invalid_argument(__PRETTY_FUNCTION__);
        }

        // Resize.
        points_.resize(
            std::distance(xfrom, xto));

        // Copy.
        int n = points_.size();
        for (int k = 0; k < n; k++) {
            points_[k].x = *xfrom++;
            points_[k].pdf = *yfrom++; // TODO verify?
        }

        // Integrate by trapezoid rule.
        neumaier_sum<T> yint = 0;
        for (int k = 1; k < n; k++) {
            T x0 = points_[k - 1].x, y0 = points_[k - 1].pdf;
            T x1 = points_[k - 0].x, y1 = points_[k - 0].pdf;
            yint += 
                (x1 - x0) * 
                (y1 + y0) * T(0.5);
            points_[k - 1].m = (y1 - y0) / (x1 - x0);
            points_[k - 0].cdf = T(yint);
        }

        // Normalize.
        T fac = T(1) / points_[n - 1].cdf;
        for (int k = 0; k < n; k++) {
            points_[k].m *= fac;
            points_[k].pdf *= fac;
            points_[k].cdf *= fac;
        }
    }

    /**
     * @brief Probability density function.
     *
     * TODO
     *
     * @throw std::runtime_error
     * If `!(points_.size() > 1)`.
     */
    T pdf(T x) const
    {
        // Invalid?
        if (!(points_.size() > 1)) {
            throw std::runtime_error(__PRETTY_FUNCTION__);
        }

        // Lookup.
        auto itr = 
            std::lower_bound(
                points_.begin(),
                points_.end(),
                x,
                [](const point_type& point,
                   const T& otherx) {
                    return point.x < otherx;
                });
        if (itr == points_.begin() ||
            itr == points_.end()) {
            return T(0);
        }
        --itr;

        // Evaluate.
        T t0 = (itr[1].x - x) / (itr[1].x - itr[0].x);
        T t1 = (x - itr[0].x) / (itr[1].x - itr[0].x);
        return t0 * itr[0].pdf + t1 * itr[1].pdf;
    }

    /**
     * @brief Cumulative distribution function.
     *
     * TODO
     *
     * @throw std::runtime_error
     * If `!(points_.size() > 1)`.
     */
    T cdf(T x) const
    {
        // Invalid?
        if (!(points_.size() > 1)) {
            throw std::runtime_error(__PRETTY_FUNCTION__);
        }

        // Lookup.
        auto itr = 
            std::lower_bound(
                points_.begin(),
                points_.end(),
                x,
                [](const point_type& point,
                   const T& otherx) {
                    return point.x < otherx;
                });
        if (itr == points_.begin() ||
            itr == points_.end()) {
            return T(itr == points_.end());
        }
        --itr;

        // Evaluate.
        T y = T(0.5) * itr[0].m;
        y = pr::fma(y, x, itr[0].pdf - itr[0].x * itr[0].m);
        y = pr::fma(y, x, itr[0].cdf - itr[0].x * (itr[0].pdf - 
                          T(0.5) * itr[0].m * itr[0].x));
        return y;
    }

    /**
     * @brief Cumulative distribution function inverse.
     *
     * TODO
     *
     * @throw std::runtime_error
     * If `!(points_.size() > 1)`.
     */
    T cdfinv(T u) const
    {
        // Invalid?
        if (!(points_.size() > 1)) {
            throw std::runtime_error(__PRETTY_FUNCTION__);
        }

        if (!(u >= T(0) &&
              u <  T(1))) {
            return pr::numeric_limits<T>::quiet_NaN();
        }
        else {

            // Lookup.
            auto itr = 
                std::lower_bound(
                    points_.begin(),
                    points_.end(),
                    u,
                    [](const point_type& point,
                       const T& othercdf) {
                        return point.cdf < othercdf;
                    });
            if (itr == points_.begin() ||
                itr == points_.end()) {
                return T(0);
            }
            --itr;

            // Quadratic coefficients.
            T a2 = T(0.5) * itr[0].m;
            T a1 = itr[0].pdf - itr[0].x * itr[0].m;
            T a0 = itr[0].cdf - itr[0].x * (itr[0].pdf -
                   T(0.5) * itr[0].m * itr[0].x);

            // Invert.
            u /= a2;
            a0 /= a2;
            a1 /= a2;
            a1 *= T(0.5);
            return pr::sqrt(u - a0 + a1 * a1) - a1;
        }
    }

    /**
     * @brief Generate number.
     */
    template <typename G>
    T operator()(G&& gen) const
    {
        return cdfinv(pr::generate_canonical<T>(std::forward<G>(gen)));
    } 

private:

    /**
     * @brief Point type.
     */
    struct alignas(16) point_type
    {
        /**
         * @brief Abscissa @f$ x_k @f$.
         */
        T x = 0;

        /**
         * @brief Probability density slope @f$ m_k @f$.
         */
        T m = 0;

        /**
         * @brief Probability density ordinate @f$ f_k @f$.
         */
        T pdf = 0;

        /**
         * @brief Cumulative distribution ordinate @f$ F_k @f$.
         */
        T cdf = 0;
    };

    /**
     * @brief Points.
     */
    std::vector<point_type> points_;
};
#endif

/**
 * @brief PCG XSH-RR engine.
 *
 * Permuted congruential generator with XOR-shift plus 
 * random-rotate output transform. The implementation here 
 * is adapted as a special case of the [implementation 
 * by Melissa O'Neill][1] affiliated with the [PCG project][2].
 *
 * [1]: https://github.com/imneme/pcg-cpp
 * [2]: https://pcg-random.org
 */
template <
    typename T, 
    typename Tstate,
    Tstate Nmultiplier,
    Tstate Ndefault_increment
    >
class pcg_xsh_rr_engine
{
public:

    // Sanity check.
    static_assert(
        std::is_unsigned<T>::value,
        "T must be unsigned");

    // Sanity check.
    static_assert(
        std::is_unsigned<Tstate>::value,
        "Tstate must be unsigned");

    // Sanity check.
    static_assert(
        sizeof(T) <= sizeof(Tstate), 
        "T cannot be larger than Tstate");

    // Sanity check.
    static_assert(
        (Nmultiplier & 3) == 1 &&
        (Ndefault_increment & 1) == 1,
        "Invalid constants");

public:

    /**
     * @brief Result type.
     */
    typedef T result_type;

    /**
     * @brief State type.
     */
    typedef Tstate state_type;

    /**
     * @brief Multiplier.
     */
    static constexpr state_type multiplier = Nmultiplier;

    /**
     * @brief Default increment.
     */
    static constexpr state_type default_increment = Ndefault_increment;

public:

    /**
     * @brief Default constructor.
     */
    pcg_xsh_rr_engine()
    {
        // Advance.
        state_ *= multiplier;
        state_ += inc_;
    }

    /**
     * @brief Constructor.
     */
    pcg_xsh_rr_engine(state_type seed) : 
            state_(seed)
    {
        // Advance.
        state_ += inc_;
        state_ *= multiplier;
        state_ += inc_;
    }

    /**
     * @brief Constructor.
     */
    pcg_xsh_rr_engine(state_type seed, state_type seq) : 
            state_(seed)
    {
        // Set stream.
        set_stream(seq);

        // Advance.
        state_ += inc_;
        state_ *= multiplier;
        state_ += inc_;
    }

public:

    /**
     * @brief Increment.
     */
    state_type increment() const
    {
        return inc_;
    }

    /**
     * @brief Stream.
     */
    state_type stream() const
    {
        return inc_ >> 1;
    }

    /**
     * @brief Set stream.
     */
    void set_stream(state_type seq)
    {
        inc_ = (seq << 1) | 1;
    }

    /**
     * @brief Result minimum.
     */
    static constexpr result_type min() noexcept
    {
        return 0;
    }

    /**
     * @brief Result maximum.
     */
    static constexpr result_type max() noexcept
    {
        return pr::numeric_limits<result_type>::max();
    }

    /**
     * @brief Generate result.
     */
    result_type operator()()
    {
        // Current state.
        state_type state = state_;

        // Advance.
        state_ *= multiplier;
        state_ += inc_;

        // Output.
        return output(state);
    }

    /**
     * @brief Generate result in range.
     *
     * @param[in] b
     * Upper bound.
     *
     * @throw std::invalid_argument
     * Unless `b > 0`.
     */
    result_type operator()(result_type b)
    {
        if (!(b > 0)) {
            throw std::invalid_argument(__PRETTY_FUNCTION__);
        }

        for (;;) {
            result_type r = operator()() - 
                pcg_xsh_rr_engine::min();
            if (r >= (pcg_xsh_rr_engine::max() - 
                      pcg_xsh_rr_engine::min() + 1 - b) % b) {
                return r % b;
            }
        }
    }

    /**
     * @brief Discard.
     */
    void discard(state_type n)
    {
        // Advance.
        state_ = pr::lcg_seek<state_type>(state_, multiplier, inc_, n);
    }

    /**
     * @brief Compare `operator==`.
     */
    bool operator==(const pcg_xsh_rr_engine& other) const
    {
        return state_ == other.state_ && inc_ == other.inc_;
    }

    /**
     * @brief Compare `operator!=`.
     */
    bool operator!=(const pcg_xsh_rr_engine& other) const
    {
        return state_ != other.state_ || inc_ != other.inc_;
    }

private:

    /**
     * @brief State.
     */
    state_type state_ = 0;

    /**
     * @brief Increment.
     */
    state_type inc_ = default_increment;

    /**
     * @brief Output function.
     */
    static constexpr result_type output(state_type state)
    {
        // Result bits.
        constexpr std::size_t result_bits = sizeof(result_type) * 8;

        // State bits.
        constexpr std::size_t state_bits = sizeof(state_type) * 8;

        // Spare bits.
        constexpr std::size_t spare_bits = state_bits - result_bits;

        // Target operation bits.
        constexpr std::size_t target_op_bits = pr::first1(result_bits);

        // Actual operation bits.
        constexpr std::size_t op_bits = 
              spare_bits < target_op_bits 
            ? spare_bits : target_op_bits;

        // Amplifier.
        constexpr std::size_t amplifier = target_op_bits - op_bits;

        // Mask.
        constexpr std::size_t mask = (1 << op_bits) - 1;

        // Top spare bits
        constexpr std::size_t top_spare = op_bits;

        // Bottom spare bits.
        constexpr std::size_t bottom_spare = spare_bits - top_spare;

        // Shift.
        constexpr std::size_t shift = (result_bits + top_spare) >> 1;

        // Pivot.
        std::size_t pivot = 0;
        if (op_bits > 0) {
            pivot = state >> (state_bits - op_bits);
            pivot = pivot & mask;
        }
        pivot = pivot << amplifier;
        pivot = pivot & mask;

        // XOR shift.
        state ^= 
        state >> shift;

        // Random rotate.
        return pr::rotr(result_type(state >> bottom_spare), pivot);
    }
};

/**
 * @name PCGs
 */
/**@{*/

/**
 * @brief 8-bit PCG XSH-RR generator.
 */
typedef pcg_xsh_rr_engine<
            std::uint8_t,
            std::uint16_t,
            12829U,
            47989U> pcg8;

/**
 * @brief 16-bit PCG XSH-RR generator.
 */
typedef pcg_xsh_rr_engine<
            std::uint16_t,
            std::uint32_t,
            747796405UL,
            2891336453UL> pcg16;

/**
 * @brief 32-bit PCG XSH-RR generator.
 */
typedef pcg_xsh_rr_engine<
            std::uint32_t,
            std::uint64_t,
            6364136223846793005ULL,
            1442695040888963407ULL> pcg32;

/**
 * @brief 64-bit PCG XSH-RR generator.
 */
typedef pcg_xsh_rr_engine<
            std::uint64_t,
            std::uint64_t,
            6364136223846793005ULL,
            1442695040888963407ULL> pcg64;

/**@}*/

// TODO stratify1
// TODO stratify2
// TODO latin_hypercube

/**@}*/

} // namespace pr

#endif // #ifndef PREFORM_RANDOM_HPP
