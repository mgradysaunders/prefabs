/* Copyright (c) 2018 M. Grady Saunders
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
#pragma once
#ifndef PREFABS_DISTR_HPP
#define PREFABS_DISTR_HPP

#include <prefabs/math.hpp>

namespace pr {

/**
 * @defgroup distr Distributions
 *
 * `<prefabs/distr.hpp>`
 */
/**@{*/

/**
 * @brief Generate canonical random number.
 */
template <typename T, typename Gen>
inline std::enable_if_t<
                std::is_floating_point<T>::value, T> 
                        generate_canonical(Gen& gen) 
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
 * @brief Uniform distribution.
 */
template <typename T = double>
struct uniform_distribution
{
    // Sanity check.
    static_assert(std::is_floating_point<T>::value, 
                                        "T must be floating point");

    /**
     * @brief @f$ x_0 @f$, lower bound.
     */
    T x0 = T(0);

    /**
     * @brief @f$ x_1 @f$, upper bound.
     */
    T x1 = T(1);

    /**
     * @brief Probability density function.
     *
     * @f[
     *      p(x) =
     *          \begin{cases}
     *              1 / (x_1 - x_0) & x \in    [x_0, x_1)
     *          \\  0               & x \notin [x_0, x_1)
     *          \end{cases}
     * @f]
     */
    T pdf(T x) const
    {
        if ((x0 <= x && x < x1)) {
            return T(1) / (x1 - x0);
        }
        else {
            return T(0);
        }
    }

    /**
     * @brief Cumulative density function.
     *
     * @f[
     *      c(x) =
     *          \max\parens{0,
     *          \min\parens{1,
     *                \frac{x - x_0}{x_1 - x_0}}}
     * @f]
     */
    T cdf(T x) const
    {
        return
            pr::fmax(T(0),
            pr::fmin(T(1), (x - x0) / (x1 - x0)));
    }

    /**
     * @brief Cumulative density inverse function.
     *
     * @f[
     *      c^{-1}(u) =
     *          \begin{cases}
     *              (1 - u)x_0 + ux_1 & u \in    [0, 1)
     *          \\  \text{qNaN}       & u \notin [0, 1)
     *          \end{cases}
     * @f]
     */
    T cdfinv(T u) const
    {
        if ((T(0) <= u && u < T(1))) {
            return (T(1) - u) * x0 + u * x1;
        }
        else {
            return pr::numeric_limits<T>::quiet_NaN();
        }
    }

    /**
     * @brief Generate number.
     */
    template <typename Gen> 
    T operator()(Gen& gen) const
    {
        return cdfinv(pr::generate_canonical<T>(gen));
    }
};

/**
 * @brief Exponential distribution.
 */
template <typename T = double>
struct exponential_distribution
{
    // Sanity check.
    static_assert(std::is_floating_point<T>::value, 
                                        "T must be floating point");

    /**
     * @brief @f$ \lambda @f$, rate.
     */
    T lambda = T(1);

    /**
     * @brief Probability density function.
     *
     * @f[
     *      p(x) =
     *          \begin{cases}
     *              \lambda e^{-\lambda x} & x \in    [0, \infty)
     *          \\  0                      & x \notin [0, \infty)
     *          \end{cases}
     * @f]
     */
    T pdf(T x) const
    {
        if (x < T(0)) {
            return T(0);
        }
        else {
            return lambda * pr::exp(-lambda * x);
        }
    }

    /**
     * @brief Cumulative density function.
     *
     * @f[
     *      c(x) =
     *          \begin{cases}
     *              1 - e^{-\lambda x} & x \in [0, \infty)
     *          \\  0                  & x \notin [0, \infty)
     *          \end{cases}
     * @f]
     */
    T cdf(T x) const
    {
        if (x < T(0)) {
            return T(0);
        }
        else {
            return -pr::expm1(-lambda * x);
        }
    }

    /**
     * @brief Cumulative density inverse function.
     *
     * @f[
     *      c^{-1}(u) =
     *          \begin{cases}
     *              -\log(1 - u) / \lambda & u \in    [0, 1)
     *          \\  \text{qNaN}            & u \notin [0, 1)
     *          \end{cases}
     * @f]
     */
    T cdfinv(T u) const
    {
        if ((T(0) <= u && u < T(1))) {
            return -pr::log1p(-u) / lambda;
        }
        else {
            return pr::numeric_limits<T>::quiet_NaN();
        }
    }

    /**
     * @brief Generate number.
     */
    template <typename Gen> 
    T operator()(Gen& gen) const
    {
        return cdfinv(pr::generate_canonical<T>(gen));
    }
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
struct normal_distribution
{
    /**
     * @brief @f$ \mu @f$, mean.
     */
    T mu = T(0);

    /**
     * @brief @f$ \sigma @f$, standard deviation.
     */
    T sigma = T(1);

    /**
     * @brief Probability density function.
     *
     * @f[
     *      p(x) =
     *          \frac{1}{\sqrt{2\pi}\sigma} e^{-\xi^2/2}
     *                   \Bigg|_{\xi \gets (x-\mu)/\sigma}
     * @f]
     */
    T pdf(T x) const
    {
        T xi = (x - mu) / sigma;
        return pr::numeric_constants<T>::M_2_sqrtpi() *
               pr::numeric_constants<T>::M_sqrt1_2() / T(2) *
                            pr::exp(-xi * xi / T(2)) / sigma;
    }

    /**
     * @brief Cumulative density function.
     *
     * @f[
     *      c(x) =
     *          \frac{1}{2}
     *          \erfc(-\xi/\sqrt{2})
     *                     \Bigg|_{\xi \gets (x-\mu)/\sigma}
     * @f]
     */
    T cdf(T x) const 
    {
        T xi = (x - mu) / sigma;
        return pr::erfc(-pr::numeric_constants<T>::M_sqrt1_2() * xi) / T(2);
    }

    /**
     * @brief Cumulative density inverse function.
     *
     * @f[
     *      c^{-1}(u) =
     *          \begin{cases}
     *              \sqrt{2}\sigma\erf^{-1}(2u - 1) + \mu & u \in    [0, 1)
     *          \\  \text{qNaN}                           & u \notin [0, 1)
     *          \end{cases}
     * @f]
     */
    T cdfinv(T u) const 
    {
        if ((T(0) <= u && u < T(1))) {
            return pr::numeric_constants<T>::M_sqrt2() * sigma * 
                   pr::erfinv(T(2) * u - T(1)) + mu;
        }
        else {
            return pr::numeric_limits<T>::quiet_NaN();

        }
    }

    /**
     * @brief Generate number.
     */
    template <typename Gen> 
    T operator()(Gen& gen) const
    {
        return cdfinv(pr::generate_canonical<T>(gen));
    }
};

/**
 * @brief Log-normal distribution.
 */
template <typename T = double>
struct lognormal_distribution : normal_distribution<T> 
{
    // Sanity check.
    static_assert(std::is_floating_point<T>::value, 
                                        "T must be floating point");

    /**
     * @brief Probability density function.
     *
     * @f[
     *      p(x) = p_{\text{normal}}(\log(x))/x
     * @f]
     */
    T pdf(T x) const
    {
        return normal_distribution<T>::pdf(pr::log(x)) / x;
    }

    /**
     * @brief Cumulative density function.
     *
     * @f[
     *      c(x) = c_{\text{normal}}(\log(x))
     * @f]
     */
    T cdf(T x) const
    {
        return normal_distribution<T>::cdf(pr::log(x));
    }

    /**
     * @brief Cumulative density inverse function.
     *
     * @f[
     *      c^{-1}(u) =
     *          \begin{cases}
     *              \exp(c_{\text{normal}}^{-1}(u)) & u \in    [0, 1)
     *          \\  \text{qNaN}                     & u \notin [0, 1)
     *          \end{cases}
     * @f]
     */
    T cdfinv(T u) const
    {
        if ((T(0) <= u && u < T(1))) {
            return pr::exp(normal_distribution<T>::cdfinv(u));
        }
        else {
            return pr::numeric_limits<T>::quiet_NaN();
        }
    }

    /**
     * @brief Generate number.
     */
    template <typename Gen> 
    T operator()(Gen& gen) const
    {
        return cdfinv(pr::generate_canonical<T>(gen));
    }
};

// TODO piecewise_constant_distribution

// TODO piecewise_linear_distribution

/**@}*/

} // namespace pr

#endif // #ifndef PREFABS_DISTR_HPP
