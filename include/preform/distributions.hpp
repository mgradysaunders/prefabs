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
#error "preform/distributions.hpp requires >=C++14"
#endif // #if !(__cplusplus >= 201402L)
#endif // #if !DOXYGEN
#pragma once
#ifndef PREFORM_DISTRIBUTIONS_HPP
#define PREFORM_DISTRIBUTIONS_HPP

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

// for pr::table_lerp1, pr::table_int1, ...
#include <preform/tabulation.hpp>

// for pr::range
#include <preform/range.hpp>

namespace pr {

/**
 * @defgroup distributions Distributions
 *
 * `<preform/distributions.hpp>`
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
 * @brief Uniform distribution.
 */
template <typename T = double>
struct uniform_distribution
{
    // Sanity check.
    static_assert(
        std::is_floating_point<T>::value, 
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
    template <typename G> 
    T operator()(G& gen) const
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
    static_assert(
        std::is_floating_point<T>::value, 
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
    template <typename G> 
    T operator()(G& gen) const
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
    // Sanity check.
    static_assert(
        std::is_floating_point<T>::value, 
        "T must be floating point");

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
    template <typename G> 
    T operator()(G& gen) const
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
    static_assert(
        std::is_floating_point<T>::value, 
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
    template <typename G> 
    T operator()(G& gen) const
    {
        return cdfinv(pr::generate_canonical<T>(gen));
    }
};

/**
 * @brief Logistic distribution.
 */
template <typename T = double>
struct logistic_distribution
{
    // Sanity check.
    static_assert(
        std::is_floating_point<T>::value, 
        "T must be floating point");

    /**
     * @brief @f$ \mu @f$, mean.
     */
    T mu = T(0);

    /**
     * @brief @f$ s @f$, spread.
     */
    T s = T(1);

    /**
     * @brief Probability density function.
     *
     * @f[
     *      p(x) = 
     *          \frac{1}{s}
     *          \frac{e^{-\xi}}{(1 + e^{-\xi})^2}
     *          \Bigg|_{\xi \gets (x-\mu)/s}
     * @f]
     */
    T pdf(T x) const
    {
        T xi = (x - mu) / s;
        T exp_nxi = pr::exp(-xi);
        return exp_nxi / (s * (T(1) + exp_nxi) * (T(1) + exp_nxi));
    }

    /**
     * @brief Cumulative density function.
     *
     * @f[
     *      c(x) = 
     *          \frac{1}{1 + e^{-\xi}}
     *          \Bigg|_{\xi \gets (x-\mu)/s}
     * @f]
     */
    T cdf(T x) const
    {
        T xi = (x - mu) / s;
        T exp_nxi = pr::exp(-xi);
        return T(1) / (T(1) + exp_nxi);
    }

    /**
     * @brief Cumulative density inverse function.
     *
     * @f[
     *      c^{-1}(u) = 
     *          \begin{cases}
     *              \mu - s\log(1/u - 1) & u \in    [0, 1)
     *          \\  \text{qNaN}          & u \notin [0, 1)
     *          \end{cases}
     * @f]
     */
    T cdfinv(T u) const
    {
        if ((T(0) <= u && u < T(1))) {
            return mu - s * pr::log(T(1) / u - T(1));
        }
        else {
            return pr::numeric_limits<T>::quiet_NaN();
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
};

/**
 * @brief Log-logistic distribution.
 */
template <typename T = double>
struct loglogistic_distribution : logistic_distribution<T>
{
    // Sanity check.
    static_assert(
        std::is_floating_point<T>::value, 
        "T must be floating point");

    /**
     * @brief Probability density function.
     *
     * @f[
     *      p(x) = p_{\text{logistic}}(\log(x))/x
     * @f]
     */
    T pdf(T x) const
    {
        return logistic_distribution<T>::pdf(pr::log(x)) / x;
    }

    /**
     * @brief Cumulative density function.
     *
     * @f[
     *      c(x) = c_{\text{logistic}}(\log(x))
     * @f]
     */
    T cdf(T x) const
    {
        return logistic_distribution<T>::cdf(pr::log(x));
    }

    /**
     * @brief Cumulative density inverse function.
     *
     * @f[
     *      c^{-1}(u) =
     *          \begin{cases}
     *              \exp(c_{\text{logistic}}^{-1}(u)) & u \in    [0, 1)
     *          \\  \text{qNaN}                       & u \notin [0, 1)
     *          \end{cases}
     * @f]
     */
    T cdfinv(T u) const
    {
        if ((T(0) <= u && u < T(1))) {
            return pr::exp(logistic_distribution<T>::cdfinv(u));
        }
        else {
            return pr::numeric_limits<T>::quiet_NaN();
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
};

/**
 * @brief Piecewise constant distribution.
 */
template <
    typename T = double,
    typename Talloc = std::allocator<T>
    >
class piecewise_constant_distribution
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
    piecewise_constant_distribution() = default;

    /**
     * @brief Copy constructor.
     */
    piecewise_constant_distribution(
            const piecewise_constant_distribution& other) : 
                alloc_(other.alloc_)
            
    {
        int n = other.pmf_.size();
        if (n > 0) {
            // Allocate.
            ptr_ = 
            std::allocator_traits<Talloc>::allocate(
                alloc_, 
                2 * n);

            // Copy.
            std::copy(
                other.ptr_, 
                other.ptr_ + 2 * n,
                ptr_);

            // Make ranges.
            T* pos0 = ptr_;
            T* pos1 = pos0 + n;
            T* pos2 = pos1 + n;
            pmf_ = pr::make_range(pos0, pos1);
            cdf_ = pr::make_range(pos1, pos2);
        }
    }

    /**
     * @brief Move constructor.
     */
    piecewise_constant_distribution(
            piecewise_constant_distribution&& other) : 
                alloc_(std::move(other.alloc_))
            
    {
        int n = other.pmf_.size();
        if (n > 0) {
            // Copy pointer.
            ptr_ = other.ptr_;

            // Make ranges.
            T* pos0 = ptr_;
            T* pos1 = pos0 + n;
            T* pos2 = pos1 + n;
            pmf_ = pr::make_range(pos0, pos1);
            cdf_ = pr::make_range(pos1, pos2);

            // Nullify other.
            other.ptr_ = nullptr;
            other.clear();
        }
    }

    /**
     * @brief Constructor.
     */
    piecewise_constant_distribution(const Talloc& alloc) : 
            alloc_(alloc)
    {
    }

    /**
     * @brief Constructor.
     */
    piecewise_constant_distribution(Talloc&& alloc) :
            alloc_(std::move(alloc))
    {
    }

    /**
     * @brief Destructor.
     */
    ~piecewise_constant_distribution()
    {
        clear();
    }

public:

    /**
     * @brief Copy assignment.
     */
    piecewise_constant_distribution& operator=(
            const piecewise_constant_distribution& other)
    {
        if (this != &other) {

            // Clear.
            clear();

            int n = other.pmf_.size();
            if (n > 0) {
                // Allocate.
                ptr_ = 
                std::allocator_traits<Talloc>::allocate(
                    alloc_, 
                    2 * n);

                // Copy.
                std::copy(
                    other.ptr_, 
                    other.ptr_ + 2 * n,
                    ptr_);

                // Make ranges.
                T* pos0 = ptr_;
                T* pos1 = pos0 + n;
                T* pos2 = pos1 + n;
                pmf_ = pr::make_range(pos0, pos1);
                cdf_ = pr::make_range(pos1, pos2);
            }
        }
        return *this;
    }

    /**
     * @brief Move assignment.
     */
    piecewise_constant_distribution& operator=(
            piecewise_constant_distribution&& other)
    {
        // Clear.
        clear();

        int n = other.pmf_.size();
        if (n > 0) {
            // Copy pointer.
            ptr_ = other.ptr_;

            // Make ranges.
            T* pos0 = ptr_;
            T* pos1 = pos0 + n;
            T* pos2 = pos1 + n;
            pmf_ = pr::make_range(pos0, pos1);
            cdf_ = pr::make_range(pos1, pos2);

            // Nullify other.
            other.ptr_ = nullptr;
            other.clear();
        }
        return *this;
    }

public:

    /**
     * @brief Initialize.
     *
     * @pre
     * - @f$ n \ge 1 @f$.
     * - @f$ w_{[j]} \ge 0 @f$ for all @f$ j \in [0, n) @f$.
     *
     * @throw std::invalid_argument
     * If any precondition is violated.
     */
    template <typename Tinput_itr>
    void init(
            Tinput_itr wfrom,
            Tinput_itr wto)
    {
        // Clear.
        clear();

        // Distance.
        int n = std::distance(wfrom, wto);
        if (n < 1) {
            throw 
                std::invalid_argument(
                std::string(__PRETTY_FUNCTION__)
                    .append(": invalid range"));
        }

        // Allocate.
        ptr_ = 
        std::allocator_traits<Talloc>::allocate(
            alloc_, 
            2 * n);

        // Make ranges.
        T* pos0 = ptr_;
        T* pos1 = pos0 + n;
        T* pos2 = pos1 + n;
        pmf_ = pr::make_range(pos0, pos1);
        cdf_ = pr::make_range(pos1, pos2);
        
        // Copy initialize.
        std::copy(wfrom, wto, pmf_.begin());

        // Test invalid.
        for (int j = 0; j < n; j++) {
            if (!(T(0) <= pmf_[j])) {
                throw
                    std::invalid_argument(
                    std::string(__PRETTY_FUNCTION__)
                        .append(": invalid value"));
            }
        }

        // Sum.
        pr::table_sum1(
                pmf_.begin(), pmf_.end(),
                cdf_.begin());

        // Normalize.
        if (cdf_[n - 1] != T(0)) {
            for (int j = 0; j < n; j++) { 
                pmf_[j] /= cdf_[n - 1];
            }
            for (int j = 0; j < n; j++) {
                cdf_[j] /= cdf_[n - 1];
            }
        }
    }

    /**
     * @brief Clear.
     */
    void clear()
    {
        if (ptr_) {
            // Deallocate.
            std::allocator_traits<Talloc>::deallocate(
                alloc_,
                ptr_,
                pmf_.size() +
                cdf_.size());
        }

        // Nullify.
        ptr_ = nullptr;
        pmf_ = pr::range<T*>{};
        cdf_ = pr::range<T*>{};
    }

public:

    /**
     * @name Distribution interface
     */
    /**@{*/

    /**
     * @brief Probability mass function.
     *
     * @f[
     *      p(k) = 
     *      \begin{cases}
     *          p_{[k]} & k \in    [0, n)
     *      \\  0       & k \notin [0, n)
     *      \end{cases}
     * @f]
     */
    T pmf(int k) const
    {
        if (!(k >= 0) ||
            !(k < int(pmf_.size()))) {
            return T(0);
        }
        else {
            return pmf_[k];
        }
    }

    /**
     * @brief Cumulative density function.
     *
     * @f[
     *      c(k) = 
     *      \begin{cases}
     *          c_{[k - 1]} & k \in [1, n]
     *      \\  0           & k \notin [1, n], k < 1
     *      \\  1           & k \notin [1, n], k > n 
     *      \end{cases}
     * @f]
     *
     * @note
     * This follows the convention that 
     * @f$ c(k) = P(X < k) @f$. Hence, @f$ c(0) = 0 @f$ even if 
     * @f$ p(0) > 0 @f$.
     */
    T cdf(int k) const
    {
        if (!(cdf_.size() >= 1) ||
            !(k >= 1)) {
            return T(0);
        }
        else if (!(k <= int(cdf_.size()))) {
            return T(1);
        }
        else {
            return cdf_[k - 1];
        }
    }

    /**
     * @brief Cumulative density inverse function.
     */
    int cdfinv(T u) const
    {
        if (!(cdf_.size() >= 1) ||
            !(T(0) <= u && u < T(1))) {
            return -1;
        }
        else {
            return std::distance(
                        cdf_.begin(),
                        std::upper_bound(cdf_.begin(), cdf_.end(), u));
        }
    }

    /**
     * @brief Generate number.
     */
    template <typename G>
    int operator()(G& gen) const
    {
        return cdfinv(pr::generate_canonical<T>(gen));
    }

    /**@}*/

private:

    /**
     * @brief Pointer.
     */
    T* ptr_ = nullptr;

    /**
     * @brief @f$ n @f$ probabilities @f$ p_{[j]} @f$.
     */
    mutable pr::range<T*> pmf_;

    /**
     * @brief @f$ n @f$ probability partial sums @f$ c_{[j]} @f$.
     *
     * - @f$ c_{[0]} = p_{[0]} @f$
     * - @f$ c_{[j]} = c_{[j - 1]} + p_{[j]} @f$
     */
    mutable pr::range<T*> cdf_;

    /**
     * @brief Allocator.
     */
    Talloc alloc_ = Talloc();
};

/**
 * @brief Piecewise linear distribution.
 */
template <
    typename T = double,
    typename Talloc = std::allocator<T>
    >
class piecewise_linear_distribution
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
    piecewise_linear_distribution() = default;

    /**
     * @brief Copy constructor.
     */
    piecewise_linear_distribution(
            const piecewise_linear_distribution& other) : 
                alloc_(other.alloc_)
            
    {
        int n = other.x_.size();
        if (n > 0) {
            // Allocate.
            ptr_ = 
            std::allocator_traits<Talloc>::allocate(
                alloc_, 
                3 * n);

            // Copy.
            std::copy(
                other.ptr_, 
                other.ptr_ + 3 * n,
                ptr_);

            // Make ranges.
            T* pos0 = ptr_;
            T* pos1 = pos0 + n;
            T* pos2 = pos1 + n;
            T* pos3 = pos2 + n;
            x_ = pr::make_range(pos0, pos1);
            pdf_ = pr::make_range(pos1, pos2);
            cdf_ = pr::make_range(pos2, pos3);
        }
    }

    /**
     * @brief Move constructor.
     */
    piecewise_linear_distribution(
            piecewise_linear_distribution&& other) : 
                alloc_(std::move(other.alloc_))
            
    {
        int n = other.x_.size();
        if (n > 0) {
            // Copy pointer.
            ptr_ = other.ptr_;

            // Make ranges.
            T* pos0 = ptr_;
            T* pos1 = pos0 + n;
            T* pos2 = pos1 + n;
            T* pos3 = pos2 + n;
            x_ = pr::make_range(pos0, pos1);
            pdf_ = pr::make_range(pos1, pos2);
            cdf_ = pr::make_range(pos2, pos3);

            // Nullify other.
            other.ptr_ = nullptr;
            other.clear();
        }
    }

    /**
     * @brief Constructor.
     */
    piecewise_linear_distribution(const Talloc& alloc) : 
            alloc_(alloc)
    {
    }

    /**
     * @brief Constructor.
     */
    piecewise_linear_distribution(Talloc&& alloc) :
            alloc_(std::move(alloc))
    {
    }

    /**
     * @brief Destructor.
     */
    ~piecewise_linear_distribution()
    {
        clear();
    }

public:

    /**
     * @brief Copy assignment.
     */
    piecewise_linear_distribution& operator=(
            const piecewise_linear_distribution& other)
    {
        if (this != &other) {

            // Clear.
            clear();

            int n = other.x_.size();
            if (n > 0) {
                // Allocate.
                ptr_ = 
                std::allocator_traits<Talloc>::allocate(
                    alloc_, 
                    3 * n);

                // Copy.
                std::copy(
                    other.ptr_, 
                    other.ptr_ + 3 * n,
                    ptr_);

                // Make ranges.
                T* pos0 = ptr_;
                T* pos1 = pos0 + n;
                T* pos2 = pos1 + n;
                T* pos3 = pos2 + n;
                x_ = pr::make_range(pos0, pos1);
                pdf_ = pr::make_range(pos1, pos2);
                cdf_ = pr::make_range(pos2, pos3);
            }
        }
        return *this;
    }

    /**
     * @brief Move assignment.
     */
    piecewise_linear_distribution& operator=(
            piecewise_linear_distribution&& other)
    {
        // Clear.
        clear();

        int n = other.x_.size();
        if (n > 0) {
            // Copy pointer.
            ptr_ = other.ptr_;

            // Make ranges.
            T* pos0 = ptr_;
            T* pos1 = pos0 + n;
            T* pos2 = pos1 + n;
            T* pos3 = pos2 + n;
            x_ = pr::make_range(pos0, pos1);
            pdf_ = pr::make_range(pos1, pos2);
            cdf_ = pr::make_range(pos2, pos3);

            // Nullify other.
            other.ptr_ = nullptr;
            other.clear();
        }
        return *this;
    }

public:

    /**
     * @brief Initialize.
     *
     * @pre
     * - @f$ n \ge 2 @f$.
     * - @f$ x_{[j - 1]} < x_{[j]} @f$ for all @f$ j \in [1, n) @f$.
     * - @f$ w_{[j]} \ge 0 @f$ for all @f$ j \in [0, n) @f$.
     *
     * @throw std::invalid_argument
     * If any precondition is violated.
     */
    template <typename Tinput_itr>
    void init(
            Tinput_itr xfrom, Tinput_itr xto,
            Tinput_itr wfrom)
    {
        // Clear.
        clear();

        // Distance.
        int n = std::distance(xfrom, xto);
        if (n < 2) {
            throw
                std::invalid_argument(
                std::string(__PRETTY_FUNCTION__)
                    .append(": invalid range"));
        }

        // Allocate.
        ptr_ = 
        std::allocator_traits<Talloc>::allocate(
            alloc_, 
            3 * n);

        // Make ranges.
        T* pos0 = ptr_;
        T* pos1 = pos0 + n;
        T* pos2 = pos1 + n;
        T* pos3 = pos2 + n;
        x_ = pr::make_range(pos0, pos1);
        pdf_ = pr::make_range(pos1, pos2);
        cdf_ = pr::make_range(pos2, pos3);
        
        // Copy initialize.
        std::copy(xfrom, xto, x_.begin());
        std::copy(wfrom, std::next(wfrom, n), pdf_.begin());

        // Test invalid.
        for (int j = 0; j < n; j++) {
            if (!(T(0) <= pdf_[j])) {
                throw 
                    std::invalid_argument(
                    std::string(__PRETTY_FUNCTION__)
                        .append(": invalid value"));
            }
        }

        // Test invalid.
        for (int j = 1; j < n; j++) {
            if (!(x_[j - 1] < x_[j])) {
                throw 
                    std::invalid_argument(
                    std::string(__PRETTY_FUNCTION__)
                        .append(": invalid value"));
            }
        }
            
        // Integrate.
        pr::table_int1(
                x_.begin(),
                x_.end(),
                pdf_.begin(),
                cdf_.begin());

        // Normalize.
        if (cdf_[n - 1] != T(0)) {
            for (int j = 0; j < n; j++) {
                pdf_[j] /= cdf_[n - 1];
            }
            for (int j = 0; j < n; j++) {
                cdf_[j] /= cdf_[n - 1];
            }
        }
    }

    /**
     * @brief Clear.
     */
    void clear()
    {
        if (ptr_) {
            // Deallocate.
            std::allocator_traits<Talloc>::deallocate(
                alloc_,
                ptr_,
                x_.size() +
                pdf_.size() +
                cdf_.size());
        }

        // Nullify.
        ptr_ = nullptr;
        x_ = pr::range<T*>{};
        pdf_ = pr::range<T*>{};
        cdf_ = pr::range<T*>{};
    }

public:

    /**
     * @name Distribution interface
     */
    /**@{*/

    /**
     * @brief Probability density function.
     *
     * @f[
     *      p(x) = 
     *      \begin{cases}
     *          \operatorname{lerp}(x;\{x_{[i]}\},\{p_{[i]}\}) 
     *              & x \in    [x_{[0]},x_{[n - 1]})
     *      \\  0   & x \notin [x_{[0]},x_{[n - 1]})
     *      \end{cases}
     * @f]
     */
    T pdf(T x) const
    {
        if (!(x_.size() >= 2) || 
            !(x_.front() <= x && 
              x < x_.back())) { // catches NaNs
            return T(0);
        }
        else {
            // Delegate.
            return pr::table_lerp1(
                        x,
                        x_.begin(), 
                        x_.end(),
                        pdf_.begin());
        }
    }

    /**
     * @brief Cumulative density function.
     *
     * @f[
     *      c(x) = 
     *      \begin{cases}
     *          \operatorname{lerp}(x;\{x_{[i]}\},\{c_{[i]}\}) 
     *              & x \in    [x_{[0]},x_{[n - 1]})
     *      \\  0   & x \notin [x_{[0]},x_{[n - 1]}), x <   x_{[0]}
     *      \\  1   & x \notin [x_{[0]},x_{[n - 1]}), x \ge x_{[n - 1]}
     *      \end{cases}
     * @f]
     */
    T cdf(T x) const
    {
        if (!(x_.size() >= 2) || 
            !(x_.front() <= x)) { // catches NaNs
            return T(0);
        }
        else if (!(x < x_.back())) {
            return T(1);
        }
        else {
            // Delegate.
            return pr::table_lerp1(
                        x,
                        x_.begin(), 
                        x_.end(),
                        cdf_.begin());
        }
    }

    /**
     * @brief Cumulative density inverse function.
     *
     * @f[
     *      c^{-1}(u) = 
     *      \begin{cases}
     *          \operatorname{lerp}(u; 
     *          \{c_{[i]}\},\{x_{[i]}\}) & u \in    [0, 1)
     *      \\  \text{qNaN}              & u \notin [0, 1)
     *      \end{cases}
     * @f]
     */
    T cdfinv(T u) const
    {
        if (!(x_.size() >= 2) ||
            !(T(0) <= u && u < T(1))) {
            return pr::numeric_limits<T>::quiet_NaN();
        }
        else {
            // Delegate.
            return pr::table_lerp1(
                        u,
                        cdf_.begin(), 
                        cdf_.end(),
                        x_.begin());
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

    /**@}*/

private:

    /**
     * @brief Pointer.
     */
    T* ptr_ = nullptr;

    /**
     * @brief @f$ n @f$ abscissas @f$ x_{[i]} @f$.
     */
    mutable pr::range<T*> x_;

    /**
     * @brief @f$ n @f$ samples @f$ p_{[i]} @f$.
     */
    mutable pr::range<T*> pdf_;

    /**
     * @brief @f$ n @f$ samples @f$ c_{[i]} @f$.
     */
    mutable pr::range<T*> cdf_;

    /**
     * @brief Allocator.
     */
    Talloc alloc_ = Talloc();
};

/**@}*/

} // namespace pr

#endif // #ifndef PREFORM_DISTRIBUTIONS_HPP
