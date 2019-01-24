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

// for pr::numeric_limits, pr::log2, ...
#include <preform/math.hpp>

#if 0
// for pr::table_lerp1, pr::table_int1, ...
#include <preform/tabulation.hpp>
#endif

// for pr::range
#include <preform/range.hpp>

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
                        generate_canonical(G& gen) 
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
     * @brief Support minimum.
     *
     * @f[
     *      \min[X] = a
     * @f]
     */
    T support_min() const
    {
        return a_;
    }

    /**
     * @brief Support maximum.
     *
     * @f[
     *      \max[X] = b
     * @f]
     */
    T support_max() const
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
    T operator()(G& gen) const
    {
        return cdfinv(pr::generate_canonical<T>(gen));
    }

private:

    /**
     * @brief Support minimum.@f$ a @f$.
     */
    T a_ = T(0);

    /**
     * @brief Support maximum.@f$ b @f$.
     */
    T b_ = T(1);
};

// TODO uniform_int_distribution

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
     * @brief Support minimum.
     *
     * @f[
     *      \min[X] = 0
     * @f]
     */
    T support_min() const
    {
        return T(0);
    }

    /**
     * @brief Support maximum.
     *
     * @f[
     *      \max[X] = \infty
     * @f]
     */
    T support_max() const
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
     * @brief Entropy.
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
    T operator()(G& gen) const
    {
        return cdfinv(pr::generate_canonical<T>(gen));
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
     * @brief Support minimum.
     *
     * @f[
     *      \min[X] = -\infty
     * @f]
     */
    T support_min() const
    {
        return -pr::numeric_limits<T>::infinity();
    }

    /**
     * @brief Support maximum.
     *
     * @f[
     *      \max[X] = \infty
     * @f]
     */
    T support_max() const
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
     * @brief Entropy.
     *
     * @f[
     *      h[X] = \frac{1}{2} \log\left(2 \pi e \sigma^2\right)
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
     *          \frac{1}{\sqrt{2 \pi}}
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
    T operator()(G& gen) const
    {
        return cdfinv(pr::generate_canonical<T>(gen));
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
     * @brief Support minimum.
     *
     * @f[
     *      \min[X] = 0
     * @f]
     */
    T support_min() const
    {
        return T(0);
    }

    /**
     * @brief Support maximum.
     *
     * @f[
     *      \max[X] = \infty
     * @f]
     */
    T support_max() const
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
     * @brief Entropy.
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
    T operator()(G& gen) const
    {
        return cdfinv(pr::generate_canonical<T>(gen));
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
     * @brief Support minimum.
     *
     * @f[
     *      \min[X] = -\infty
     * @f]
     */
    T support_min() const
    {
        return -pr::numeric_limits<T>::infinity();
    }

    /**
     * @brief Support maximum.
     *
     * @f[
     *      \max[X] = \infty
     * @f]
     */
    T support_max() const
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
     * @brief Entropy.
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
    T operator()(G& gen) const
    {
        return cdfinv(pr::generate_canonical<T>(gen));
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
     * Unless `lambda > 0` and `k > 0`.
     */
    weibull_distribution(T lambda, T k) : lambda_(lambda), k_(k)
    {
        if (!(lambda > T(0)) ||
            !(k > T(0))) {
            throw std::invalid_argument(__PRETTY_FUNCTION__);
        }
    }

    /**
     * @brief Support minimum.
     *
     * @f[
     *      \min[X] = 0
     * @f]
     */
    T support_min() const
    {
        return T(0);
    }

    /**
     * @brief Support maximum.
     *
     * @f[
     *      \max[X] = \infty
     * @f]
     */
    T support_max() const
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
     * @brief Entropy.
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
    T operator()(G& gen) const
    {
        return cdfinv(pr::generate_canonical<T>(gen));
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

// TODO piecewise_constant_distribution

// TODO piecewise_linear_distribution

/**@}*/

} // namespace pr

#endif // #ifndef PREFORM_RANDOM_HPP
