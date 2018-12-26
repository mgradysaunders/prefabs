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
#if !DOXYGEN
#if !(__cplusplus >= 201703L)
#error "prefabs/float_bounds.hpp requires >=C++17"
#endif // #if !(__cplusplus >= 201703L)
#endif // #if !DOXYGEN
#pragma once
#ifndef PREFABS_FLOAT_BOUNDS_HPP
#define PREFABS_FLOAT_BOUNDS_HPP

// for assert
#include <cassert>

// for std::basic_istream
#include <istream>

// for std::basic_ostream
#include <ostream>

// for pr::numeric_limits
#include <prefabs/math.hpp>

// for pr::finc, pr::fdec
#include <prefabs/float_helpers.hpp>

namespace pr {

/**
 * @defgroup float_bounds Float bounds
 *
 * `<prefabs/float_bounds.hpp>`
 *
 * __C++ version__: >=C++17
 */
/**@{*/

/**
 * @brief Float bounds.
 */
template <typename T>
class float_bounds
{
public:

    // Sanity check.
    static_assert(
        std::is_floating_point<T>::value,
        "T must be floating point");

    /**
     * @name Constructors
     */
    /**@{*/

    /**
     * @brief Default constructor.
     */
    float_bounds() = default;

    /**
     * @brief Constructor.
     *
     * @param[in] x
     * @f$ x @f$, value.
     */
    float_bounds(T x) : 
            x_(x), 
            x0_(x), 
            x1_(x)
    {
    }

    /**
     * @brief Constructor.
     *
     * @param[in] x
     * @f$ x @f$, value.
     *
     * @param[in] x0
     * @f$ x_0 @f$, lower bound.
     *
     * @param[in] x1
     * @f$ x_1 @f$, upper bound.
     */
    float_bounds(T x, T x0, T x1) :
            x_(x),
            x0_(x0),
            x1_(x1)
    {
#if !NDEBUG
        if (pr::isfinite(x_) &&
            pr::isfinite(x0_) &&
            pr::isfinite(x1_)) {
            assert(x0_ <= x &&
                   x1_ >= x);
        }
#endif // #if !NDEBUG
    }

    /**@}*/

public:

    /**
     * @brief Value.
     */
    T value() const
    {
        return x_;
    }

    /**
     * @brief Lower bound.
     */
    T lower_bound() const
    {
        return x0_;
    }

    /**
     * @brief Upper bound.
     */
    T upper_bound() const
    {
        return x1_;
    }

    /**
     * @brief Absolute error.
     */
    T abs_error() const
    {
        return pr::finc(pr::fmax(pr::fabs(x_ - x0_), pr::fabs(x_ - x1_)));
    }

    /**
     * @brief Relative error.
     */
    T rel_error() const
    {
        return abs_error() / value();
    }

public:

    /**
     * @name Tests
     */
    /**@{*/

    /**
     * @brief Overlaps other?
     */
    template <
        bool inclusive0 = true,
        bool inclusive1 = false
        >
    bool overlaps(const float_bounds& oth) const
    {
        if constexpr (inclusive0 && inclusive1) {
            // Both inclusive.
            return lower_bound() <= oth.upper_bound() && 
                   upper_bound() >= oth.lower_bound();
        }
        else if constexpr (inclusive0 && !inclusive1) {
            // First inclusive, second exclusive.
            return lower_bound() <= oth.upper_bound() && 
                   upper_bound() > oth.lower_bound();
        }
        else if constexpr (!inclusive0 && inclusive1) {
            // First exclusive, second inclusive.
            return lower_bound() < oth.upper_bound() && 
                   upper_bound() >= oth.lower_bound();
        }
        else {
            // Both exclusive.
            return lower_bound() < oth.upper_bound() && 
                   upper_bound() > oth.lower_bound();
        }
    }

    /**
     * @brief Contains other?
     */
    template <
        bool inclusive0 = true,
        bool inclusive1 = false
        >
    bool contains(const float_bounds& oth) const
    {
        if constexpr (inclusive0 && inclusive1) {
            // Both inclusive.
            return lower_bound() <= oth.lower_bound() && 
                   upper_bound() >= oth.upper_bound();
        }
        else if constexpr (inclusive0 && !inclusive1) {
            // First inclusive, second exclusive.
            return lower_bound() <= oth.lower_bound() && 
                   upper_bound() > oth.upper_bound();
        }
        else if constexpr (!inclusive0 && inclusive1) {
            // First exclusive, second inclusive.
            return lower_bound() < oth.lower_bound() && 
                   upper_bound() >= oth.upper_bound();
        }
        else {
            // Both exclusive.
            return lower_bound() < oth.lower_bound() && 
                   upper_bound() > oth.upper_bound();
        }
    }

    /**@}*/

private:

    /**
     * @brief @f$ x @f$, value.
     */
    T x_ = T();

    /**
     * @brief @f$ x_0 @f$, lower bound.
     */
    T x0_ = T();

    /**
     * @brief @f$ x_1 @f$, upper bound.
     */
    T x1_ = T();

public:

    /**
     * @name Stream operators
     */
    /**@{*/

    /**
     * @brief Parse from `std::basic_istream`.
     *
     * Format is `(x,[x0,x1])` or `x`. Sets `std::ios_base::failbit` on error.
     */
    template <typename C, typename Ctraits>
    friend
    inline std::basic_istream<C, Ctraits>& operator>>(
           std::basic_istream<C, Ctraits>& is, float_bounds& b)
    {
        C ch;
        if (!(is >> ch) ||
            !Ctraits::eq(ch,
             Ctraits::to_char_type('('))) {
            T x;
            if (!(is >> x)) {
                is.setstate(std::ios_base::failbit);
                return is;
            }
            b = float_bounds(x);
            return is;
        }
        T x;
        is >> x;
        if (!(is >> ch) ||
            !Ctraits::eq(ch,
             Ctraits::to_char_type(','))) {
            is.setstate(std::ios_base::failbit);
            return is;
        }
        if (!(is >> ch) ||
            !Ctraits::eq(ch,
             Ctraits::to_char_type('['))) {
            is.setstate(std::ios_base::failbit);
            return is;
        }
        T x0;
        is >> x0;
        if (!(is >> ch) ||
            !Ctraits::eq(ch,
             Ctraits::to_char_type(','))) {
            is.setstate(std::ios_base::failbit);
            return is;
        }
        T x1;
        is >> x1;
        if (!(is >> ch) ||
            !Ctraits::eq(ch,
             Ctraits::to_char_type(']'))) {
            is.setstate(std::ios_base::failbit);
            return is;
        }
        if (!(is >> ch) ||
            !Ctraits::eq(ch,
             Ctraits::to_char_type(')'))) {
            is.setstate(std::ios_base::failbit);
            return is;
        }
        b = float_bounds(x, x0, x1);
        return is;
    }

    /**
     * @brief Write into `std::basic_ostream`.
     *
     * Format is `(x,[x0,x1])`.
     */
    template <typename C, typename Ctraits>
    friend
    inline std::basic_ostream<C, Ctraits>& operator<<(
           std::basic_ostream<C, Ctraits>& os, const float_bounds& b)
    {
        os << '(' << b.x_ << ',';
        os << '[' << b.x0_ << ',' << b.x1_ << ']';
        os << ')';
        return os;
    }

    /**@}*/
};

/**
 * @name Unary operators (float_bounds)
 */
/**@{*/

/**
 * @brief Bound `operator+`.
 *
 * @f[
 *      +[b_0, b_1] \subseteq [+b_0, +b_1]
 * @f]
 */
template <typename T>
__attribute__((always_inline))
inline float_bounds<T> operator+(const float_bounds<T>& b)
{
    return b;
}

/**
 * @brief Bound `operator-`.
 *
 * @f[
 *      -[b_0, b_1] \subseteq [-b_1, -b_0]
 * @f]
 */
template <typename T>
__attribute__((always_inline))
inline float_bounds<T> operator-(const float_bounds<T>& b)
{
    return {
        -b.value(),
        -b.upper_bound(),
        -b.lower_bound()
    };
}

/**@}*/

/**
 * @name Binary operators (float_bounds/float_bounds)
 */
/**@{*/

/**
 * @brief Bound `operator+`.
 *
 * @f[
 *      [b_{00}, b_{01}] \oplus
 *      [b_{10}, b_{11}] \subset
 *      [\operatorname{fdec}(b_{00} \oplus b_{10}), 
 *       \operatorname{finc}(b_{01} \oplus b_{11})]
 * @f]
 */
template <typename T>
__attribute__((always_inline))
inline float_bounds<T> operator+(
                const float_bounds<T>& b0,
                const float_bounds<T>& b1)
{
    return {
        b0.value() + b1.value(),
        pr::fdec(b0.lower_bound() + b1.lower_bound()),
        pr::finc(b0.upper_bound() + b1.upper_bound())
    };
}

/**
 * @brief Bound `operator-`.
 *
 * @f[
 *      [b_{00}, b_{01}] \ominus
 *      [b_{10}, b_{11}] \subset
 *      [\operatorname{fdec}(b_{00} \ominus b_{11}), 
 *       \operatorname{finc}(b_{01} \ominus b_{10})]
 * @f]
 */
template <typename T>
__attribute__((always_inline))
inline float_bounds<T> operator-(
                const float_bounds<T>& b0,
                const float_bounds<T>& b1)
{
    return {
        b0.value() - b1.value(),
        pr::fdec(b0.lower_bound() - b1.upper_bound()),
        pr::finc(b0.upper_bound() - b1.lower_bound())
    };
}

/**
 * @brief Bound `operator*`.
 *
 * @f[
 *      [b_{00}, b_{01}] \otimes
 *      [b_{10}, b_{11}] \subset
 *      [\operatorname{fdec}(\min\{t_{jk}\}), 
 *       \operatorname{finc}(\max\{t_{jk}\})]
 * @f]
 * where
 * @f[
 *      t_{jk} = b_{0j} \otimes b_{1k}
 * @f]
 */
template <typename T>
inline float_bounds<T> operator*(
                const float_bounds<T>& b0,
                const float_bounds<T>& b1)
{
    T tmp[] = {
        b0.lower_bound() * b1.lower_bound(),
        b0.lower_bound() * b1.upper_bound(),
        b0.upper_bound() * b1.lower_bound(),
        b0.upper_bound() * b1.upper_bound()
    };
    return {
        b0.value() * b1.value(),
        pr::fdec(pr::fmin(pr::fmin(tmp[0], tmp[1]), pr::fmin(tmp[2], tmp[3]))),
        pr::finc(pr::fmax(pr::fmax(tmp[0], tmp[1]), pr::fmax(tmp[2], tmp[3])))
    };
}

/**
 * @brief Bound `operator/`.
 *
 * @f[
 *      [b_{00}, b_{01}] \oslash
 *      [b_{10}, b_{11}] \subset
 *      [\operatorname{fdec}(\min\{t_{jk}\}), 
 *       \operatorname{finc}(\max\{t_{jk}\})]
 * @f]
 * where
 * @f[
 *      t_{jk} = b_{0j} \oslash b_{1k}
 * @f]
 */
template <typename T>
inline float_bounds<T> operator/(
                const float_bounds<T>& b0,
                const float_bounds<T>& b1)
{
    // Denominator contains zero?
    if (b1.lower_bound() <= T(0) &&
        b1.upper_bound() >= T(0)) {
        return {
            b0.value() / b1.value(),
            -pr::numeric_limits<T>::infinity(),
            +pr::numeric_limits<T>::infinity()
        };
    }

    T tmp[] = {
        b0.lower_bound() / b1.lower_bound(),
        b0.lower_bound() / b1.upper_bound(),
        b0.upper_bound() / b1.lower_bound(),
        b0.upper_bound() / b1.upper_bound()
    };
    return {
        b0.value() / b1.value(),
        pr::fdec(pr::fmin(pr::fmin(tmp[0], tmp[1]), pr::fmin(tmp[2], tmp[3]))),
        pr::finc(pr::fmax(pr::fmax(tmp[0], tmp[1]), pr::fmax(tmp[2], tmp[3])))
    };
}

/**@}*/

/**
 * @name Binary operators (float_bounds/float)
 */
/**@{*/

/**
 * @brief Wrap `operator+`.
 */
template <typename T>
__attribute__((always_inline))
inline float_bounds<T> operator+(const float_bounds<T>& b0, T b1)
{
    return b0 + float_bounds<T>(b1);
}

/**
 * @brief Wrap `operator-`.
 */
template <typename T>
__attribute__((always_inline))
inline float_bounds<T> operator-(const float_bounds<T>& b0, T b1)
{
    return b0 - float_bounds<T>(b1);
}

/**
 * @brief Wrap `operator*`.
 */
template <typename T>
__attribute__((always_inline))
inline float_bounds<T> operator*(const float_bounds<T>& b0, T b1)
{
    return b0 * float_bounds<T>(b1);
}

/**
 * @brief Wrap `operator/`.
 */
template <typename T>
__attribute__((always_inline))
inline float_bounds<T> operator/(const float_bounds<T>& b0, T b1)
{
    return b0 / float_bounds<T>(b1);
}

/**@}*/

/**
 * @name Binary operators (float/float_bounds)
 */
/**@{*/

/**
 * @brief Wrap `operator+`.
 */
template <typename T>
__attribute__((always_inline))
inline float_bounds<T> operator+(T b0, const float_bounds<T>& b1)
{
    return float_bounds<T>(b0) + b1;
}

/**
 * @brief Wrap `operator-`.
 */
template <typename T>
__attribute__((always_inline))
inline float_bounds<T> operator-(T b0, const float_bounds<T>& b1)
{
    return float_bounds<T>(b0) - b1;
}

/**
 * @brief Wrap `operator*`.
 */
template <typename T>
__attribute__((always_inline))
inline float_bounds<T> operator*(T b0, const float_bounds<T>& b1)
{
    return float_bounds<T>(b0) * b1;
}

/**
 * @brief Wrap `operator/`.
 */
template <typename T>
__attribute__((always_inline))
inline float_bounds<T> operator/(T b0, const float_bounds<T>& b1)
{
    return float_bounds<T>(b0) / b1;
}

/**@}*/

/**
 * @name Binary operators (float_bounds/any)
 */
/**@{*/

/**
 * @brief Generic `operator+=`.
 */
template <typename T, typename U>
__attribute__((always_inline))
inline float_bounds<T>& operator+=(float_bounds<T>& b, const U& any)
{
    return b = b + any;
}

/**
 * @brief Generic `operator-=`.
 */
template <typename T, typename U>
__attribute__((always_inline))
inline float_bounds<T>& operator-=(float_bounds<T>& b, const U& any)
{
    return b = b - any;
}

/**
 * @brief Generic `operator*=`.
 */
template <typename T, typename U>
__attribute__((always_inline))
inline float_bounds<T>& operator*=(float_bounds<T>& b, const U& any)
{
    return b = b * any;
}

/**
 * @brief Generic `operator/=`.
 */
template <typename T, typename U>
__attribute__((always_inline))
inline float_bounds<T>& operator/=(float_bounds<T>& b, const U& any)
{
    return b = b / any;
}

/**@}*/

/**
 * @name Math (float_bounds)
 */
/**@{*/

/**
 * @brief Bound `pr::fabs()`.
 */
template <typename T>
inline float_bounds<T> fabs(const float_bounds<T>& b)
{
    if (b.lower_bound() >= T(0)) {
        return +b;
    }
    if (b.upper_bound() <= T(0)) {
        return -b;
    }
    return {
        pr::fabs(b.value()),
        T(0),
        pr::fmax(
                -b.lower_bound(), 
                +b.upper_bound())
    };
}

/**
 * @brief Bound `pr::sqrt()`.
 */
template <typename T>
inline float_bounds<T> sqrt(const float_bounds<T>& b)
{
    return {
        pr::sqrt(b.value()),
        pr::fdec(pr::sqrt(b.lower_bound())),
        pr::finc(pr::sqrt(b.upper_bound()))
    };
}

/**@}*/

/**@}*/

} // namespace pr

#endif // #ifndef PREFABS_FLOAT_BOUNDS_HPP
