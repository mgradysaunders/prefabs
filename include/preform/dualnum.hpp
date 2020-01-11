/* Copyright (c) 2018-20 M. Grady Saunders
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
#error "preform/dualnum.hpp requires >=C++17"
#endif // #if !(__cplusplus >= 201703L)
#endif // #if !DOXYGEN
#pragma once
#ifndef PREFORM_DUALNUM_HPP
#define PREFORM_DUALNUM_HPP

// for std::basic_istream
#include <istream>

// for std::basic_ostream
#include <ostream>

#include <preform/math.hpp>

namespace pr {

/**
 * @defgroup dualnum Dual number
 *
 * `<preform/dualnum.hpp>`
 *
 * __C++ version__: >=C++17
 */
/**@{*/

#if !DOXYGEN

template <typename T>
class dualnum;

template <typename T>
struct is_dualnum : std::false_type
{
};

template <typename T>
struct is_dualnum<dualnum<T>> : std::true_type
{
};

template <typename T>
struct is_dualnum_param :
            std::integral_constant<bool,
            std::is_arithmetic<T>::value ||
            is_complex<T>::value>
{
};

template <typename T>
struct is_floating_point_dualnum : std::false_type
{
};

template <typename T>
struct is_floating_point_dualnum<dualnum<T>> : std::is_floating_point<T>
{
};

#endif // #if !DOXYGEN

/**
 * @brief Dual number.
 */
template <typename T>
class dualnum
{
public:

    // Sanity check.
    static_assert(
        is_dualnum_param<T>::value,
        "T must be arithmetic or complex");

    /**
     * @brief Value type.
     */
    typedef T value_type;

public:

    /**
     * @name Constructors
     */
    /**@{*/

    /**
     * @brief Default constructor.
     */
    constexpr dualnum() = default;

    /**
     * @brief Constructor.
     */
    constexpr dualnum(T a, T b = T()) : a_(a), b_(b)
    {
    }

    /**@}*/

public:

    /**
     * @name Accessors
     */
    /**@{*/

    /**
     * @brief Get real part.
     */
    constexpr const T& real() const
    {
        return a_;
    }

    /**
     * @brief Get dual part.
     */
    constexpr const T& dual() const
    {
        return b_;
    }

    /**
     * @brief Set real part, return previous real part.
     */
    constexpr T real(T val)
    {
        T a = a_; a_ = val; return a;
    }

    /**
     * @brief Set dual part, return previous dual part.
     */
    constexpr T dual(T val)
    {
        T b = b_; b_ = val; return b;
    }

    /**@}*/

private:

    /**
     * @brief Real part.
     */
    T a_ = T();

    /**
     * @brief Dual part.
     */
    T b_ = T();

public:

    /**
     * @name Stream operators
     */
    /**@{*/

    /**
     * @brief Parse from `std::basic_istream`.
     *
     * Format is `(a,b)`. Sets `std::ios_base::failbit` on error.
     */
    template <typename C, typename Ctraits>
    friend
    inline std::basic_istream<C, Ctraits>& operator>>(
           std::basic_istream<C, Ctraits>& is, dualnum& x)
    {
        C ch;
        if (!(is >> ch) ||
            !Ctraits::eq(ch,
             Ctraits::to_char_type('('))) {
            is.setstate(std::ios_base::failbit);
            return is;
        }
        is >> x.a_;
        if (!(is >> ch) ||
            !Ctraits::eq(ch,
             Ctraits::to_char_type(','))) {
            is.setstate(std::ios_base::failbit);
            return is;
        }
        is >> x.b_;
        if (!(is >> ch) ||
            !Ctraits::eq(ch,
             Ctraits::to_char_type(')'))) {
            is.setstate(std::ios_base::failbit);
            return is;
        }
        return is;
    }

    /**
     * @brief Write into `std::basic_ostream`.
     *
     * Format is `(a,b)`.
     */
    template <typename C, typename Ctraits>
    friend
    inline std::basic_ostream<C, Ctraits>& operator<<(
           std::basic_ostream<C, Ctraits>& os, const dualnum& x)
    {
        os << '(';
        os << x.a_ << ',';
        os << x.b_ << ')';
        return os;
    }

    /**@}*/
};

/**
 * @name Unary operators (dualnum)
 */
/**@{*/

/**
 * @brief Distribute `operator+`.
 */
template <typename T>
__attribute__((always_inline))
constexpr dualnum<T> operator+(const dualnum<T>& x)
{
    return {+x.real(), +x.dual()};
}

/**
 * @brief Distribute `operator-`.
 */
template <typename T>
__attribute__((always_inline))
constexpr dualnum<T> operator-(const dualnum<T>& x)
{
    return {-x.real(), -x.dual()};
}

/**@}*/

/**
 * @name Binary operators (dualnum/dualnum)
 */
/**@{*/

/**
 * @brief Distribute `operator+`.
 *
 * @f[
 *      (a_0 + \varepsilon b_0) +
 *      (a_1 + \varepsilon b_1) =
 *      (a_0 + a_1) + \varepsilon
 *      (b_0 + b_1)
 * @f]
 */
template <typename T, typename U>
__attribute__((always_inline))
constexpr dualnum<decltype(T() + U())> operator+(
                    const dualnum<T>& x0, const dualnum<U>& x1)
{
    return {x0.real() + x1.real(), x0.dual() + x1.dual()};
}

/**
 * @brief Distribute `operator-`.
 *
 * @f[
 *      (a_0 + \varepsilon b_0) -
 *      (a_1 + \varepsilon b_1) =
 *      (a_0 - a_1) + \varepsilon
 *      (b_0 - b_1)
 * @f]
 */
template <typename T, typename U>
__attribute__((always_inline))
constexpr dualnum<decltype(T() - U())> operator-(
                    const dualnum<T>& x0, const dualnum<U>& x1)
{
    return {x0.real() - x1.real(), x0.dual() - x1.dual()};
}

/**
 * @brief Distribute `operator*`.
 *
 * @f[
 *      (a_0 + \varepsilon b_0)
 *      (a_1 + \varepsilon b_1) =
 *       a_0 a_1 + \varepsilon
 *      (a_0 b_1 + b_0 a_1)
 * @f]
 */
template <typename T, typename U>
__attribute__((always_inline))
constexpr dualnum<decltype(T() * U())> operator*(
                    const dualnum<T>& x0, const dualnum<U>& x1)
{
    return {
        x0.real() * x1.real(),
        x0.real() * x1.dual() + x0.dual() * x1.real()
    };
}

/**
 * @brief Distribute `operator*`, inverting right hand side.
 *
 * @f[
 *      (a_0 + \varepsilon b_0)
 *      (a_1 + \varepsilon b_1)^{-1} =
 *       a_0 a_1^{-1} + \varepsilon
 *      (b_0 a_1 - a_0 b_1) a_1^{-2}
 * @f]
 */
template <typename T, typename U>
__attribute__((always_inline))
constexpr dualnum<decltype(T() / U())> operator/(
                    const dualnum<T>& x0, const dualnum<U>& x1)
{
    return {
        x0.real() / x1.real(),
        (x0.dual() * x1.real() - x0.real() * x1.dual()) /
        (x1.real() * x1.real())
    };
}

/**@}*/

/**
 * @name Binary operators (dualnum/num)
 */
/**@{*/

/**
 * @brief Distribute `operator+`.
 *
 * @f[
 *      (a_0 + \varepsilon b_0) + a_1 =
 *      (a_0 + a_1) + \varepsilon b_0
 * @f]
 */
template <typename T, typename U>
__attribute__((always_inline))
constexpr std::enable_if_t<
                        is_dualnum_param<U>::value,
                        dualnum<decltype(T() + U())>> operator+(
                            const dualnum<T>& x0, const U& x1)
{
    return {x0.real() + x1, x0.dual()};
}

/**
 * @brief Distribute `operator-`.
 *
 * @f[
 *      (a_0 + \varepsilon b_0) - a_1 =
 *      (a_0 - a_1) + \varepsilon b_0
 * @f]
 */
template <typename T, typename U>
__attribute__((always_inline))
constexpr std::enable_if_t<
                        is_dualnum_param<U>::value,
                        dualnum<decltype(T() - U())>> operator-(
                            const dualnum<T>& x0, const U& x1)
{
    return {x0.real() - x1, x0.dual()};
}

/**
 * @brief Distribute `operator*`.
 *
 * @f[
 *      (a_0 + \varepsilon b_0) a_1 =
 *       a_0 a_1 + \varepsilon b_0 a_1
 * @f]
 */
template <typename T, typename U>
__attribute__((always_inline))
constexpr std::enable_if_t<
                        is_dualnum_param<U>::value,
                        dualnum<decltype(T() * U())>> operator*(
                            const dualnum<T>& x0, const U& x1)
{
    return {x0.real() * x1, x0.dual() * x1};
}

/**
 * @brief Distribute `operator*`, inverting right hand side.
 *
 * @f[
 *      (a_0 + \varepsilon b_0) a_1^{-1} =
 *       a_0 a_1^{-1} + \varepsilon b_0 a_1^{-1}
 * @f]
 */
template <typename T, typename U>
__attribute__((always_inline))
constexpr std::enable_if_t<
                        is_dualnum_param<U>::value,
                        dualnum<decltype(T() / U())>> operator/(
                            const dualnum<T>& x0, const U& x1)
{
    return {x0.real() / x1, x0.dual() / x1};
}

/**@}*/

/**
 * @name Binary operators (num/dualnum)
 */
/**@{*/

/**
 * @brief Distribute `operator+`.
 *
 * @f[
 *       a_0 + (a_1 + \varepsilon b_1) =
 *      (a_0 + a_1) + \varepsilon b_1
 * @f]
 */
template <typename T, typename U>
__attribute__((always_inline))
constexpr std::enable_if_t<
                        is_dualnum_param<T>::value,
                        dualnum<decltype(T() + U())>> operator+(
                            const T& x0, const dualnum<U>& x1)
{
    return {x0 + x1.real(), x1.dual()};
}

/**
 * @brief Distribute `operator-`.
 *
 * @f[
 *       a_0 - (a_1 + \varepsilon b_1)  =
 *      (a_0 - a_1) - \varepsilon b_1
 * @f]
 */
template <typename T, typename U>
__attribute__((always_inline))
constexpr std::enable_if_t<
                        is_dualnum_param<T>::value,
                        dualnum<decltype(T() - U())>> operator-(
                            const T& x0, const dualnum<U>& x1)
{
    return {x0 - x1.real(), -x1.dual()};
}

/**
 * @brief Distribute `operator*`.
 *
 * @f[
 *      a_0 (a_1 + \varepsilon b_1) =
 *      a_0  a_1 + \varepsilon a_0 b_1
 * @f]
 */
template <typename T, typename U>
__attribute__((always_inline))
constexpr std::enable_if_t<
                        is_dualnum_param<T>::value,
                        dualnum<decltype(T() * U())>> operator*(
                            const T& x0, const dualnum<U>& x1)
{
    return {x0 * x1.real(), x0 * x1.dual()};
}

/**
 * @brief Distribute `operator*`, inverting right hand side.
 *
 * @f[
 *      a_0 (a_1 + \varepsilon b_1)^{-1} =
 *      a_0  a_1^{-1} - \varepsilon a_0 b_1 a_1^{-2}
 * @f]
 */
template <typename T, typename U>
__attribute__((always_inline))
constexpr std::enable_if_t<
                        is_dualnum_param<T>::value,
                        dualnum<decltype(T() / U())>> operator/(
                            const T& x0, const dualnum<U>& x1)
{
    return {
        x0 / x1.real(),
        -x0 * x1.dual() /
        (x1.real() * x1.real())
    };
}

/**@}*/

/**
 * @name Binary operators (dualnum/any)
 */
/**@{*/

/**
 * @brief Generic `operator+=`.
 */
template <typename T, typename U>
__attribute__((always_inline))
constexpr dualnum<T>& operator+=(dualnum<T>& x, const U& any)
{
    return x = x + any;
}

/**
 * @brief Generic `operator-=`.
 */
template <typename T, typename U>
__attribute__((always_inline))
constexpr dualnum<T>& operator-=(dualnum<T>& x, const U& any)
{
    return x = x - any;
}

/**
 * @brief Generic `operator*=`.
 */
template <typename T, typename U>
__attribute__((always_inline))
constexpr dualnum<T>& operator*=(dualnum<T>& x, const U& any)
{
    return x = x * any;
}

/**
 * @brief Generic `operator/=`.
 */
template <typename T, typename U>
__attribute__((always_inline))
constexpr dualnum<T>& operator/=(dualnum<T>& x, const U& any)
{
    return x = x / any;
}

/**@}*/

/**
 * @name Comparison operators (dualnum)
 */
/**@{*/

/**
 * @brief Compare `operator==`.
 */
template <typename T, typename U>
__attribute__((always_inline))
constexpr bool operator==(const dualnum<T>& x0, const dualnum<U>& x1)
{
    return x0.real() == x1.real() && x0.dual() == x1.dual();
}

/**
 * @brief Compare `operator==`.
 */
template <typename T, typename U>
__attribute__((always_inline))
constexpr std::enable_if_t<
                        is_dualnum_param<U>::value, bool> operator==(
                        const dualnum<T>& x0, const U& x1)
{
    return x0.real() == x1 && x0.dual() == T();
}

/**
 * @brief Compare `operator==`.
 */
template <typename T, typename U>
__attribute__((always_inline))
constexpr std::enable_if_t<
                        is_dualnum_param<T>::value, bool> operator==(
                        const T& x0, const dualnum<U>& x1)
{
    return x0 == x1.real() && U() == x1.dual();
}

/**
 * @brief Compare `operator!=`.
 */
template <typename T, typename U>
__attribute__((always_inline))
constexpr bool operator!=(const dualnum<T>& x0, const dualnum<U>& x1)
{
    return !(x0 == x1);
}

/**
 * @brief Compare `operator!=`.
 */
template <typename T, typename U>
__attribute__((always_inline))
constexpr std::enable_if_t<
                        is_dualnum_param<U>::value, bool> operator!=(
                        const dualnum<T>& x0, const U& x1)
{
    return !(x0 == x1);
}

/**
 * @brief Compare `operator!=`.
 */
template <typename T, typename U>
__attribute__((always_inline))
constexpr std::enable_if_t<
                        is_dualnum_param<T>::value, bool> operator!=(
                        const T& x0, const dualnum<U>& x1)
{
    return !(x0 == x1);
}

/**@}*/

/**
 * @name Accessors (dualnum)
 */
/**@{*/

/**
 * @brief Real part.
 *
 * @f[
 *      \real(a + \varepsilon b) = a
 * @f]
 */
template <typename T>
__attribute__((always_inline))
constexpr T real(const dualnum<T>& x) { return x.real(); }

/**
 * @brief Dual part.
 *
 * @f[
 *      \dual(a + \varepsilon b) = b
 * @f]
 */
template <typename T>
__attribute__((always_inline))
constexpr T dual(const dualnum<T>& x) { return x.dual(); }

/**
 * @brief Dual conjugate.
 *
 * @f[
 *      (a + \varepsilon b)^\circ =
 *       a - \varepsilon b
 * @f]
 */
template <typename T>
__attribute__((always_inline))
constexpr dualnum<T> dual_conj(const dualnum<T>& x)
{
    return {+x.real(), -x.dual()};
}

/**
 * @brief Norm square.
 *
 * @f[
 *      |(a + \varepsilon b)
 *       (a + \varepsilon b)^\circ| = |a^2|
 * @f]
 *
 * @note
 * If `T` is complex, returns a floating
 * point type.
 */
template <typename T>
__attribute__((always_inline))
constexpr decltype(pr::abs(T() * T())) norm(const dualnum<T>& x)
{
    return pr::abs(x.real() * x.real());
}

/**
 * @brief Absolute value.
 *
 * @f[
 *      |a + \varepsilon b| = |a|
 * @f]
 *
 * @note
 * If `T` is complex, returns a floating
 * point type.
 */
template <typename T>
__attribute__((always_inline))
inline decltype(pr::abs(T())) abs(const dualnum<T>& x)
{
    return pr::abs(x.real());
}

/**@}*/

/**
 * @name Float checks (dualnum)
 */
/**@{*/

/**
 * @brief Any Inf?
 */
template <typename T>
__attribute__((always_inline))
inline bool isinf(const dualnum<T>& x)
{
    return pr::isinf(x.real()) || pr::isinf(x.dual());
}

/**
 * @brief Any NaN?
 */
template <typename T>
__attribute__((always_inline))
inline bool isnan(const dualnum<T>& x)
{
    return pr::isnan(x.real()) || pr::isnan(x.dual());
}

/**
 * @brief All finite?
 */
template <typename T>
__attribute__((always_inline))
inline bool isfinite(const dualnum<T>& x)
{
    return pr::isfinite(x.real()) && pr::isfinite(x.dual());
}

/**
 * @brief All normal?
 */
template <typename T>
__attribute__((always_inline))
inline bool isnormal(const dualnum<T>& x)
{
    return pr::isnormal(x.real()) && pr::isnormal(x.dual());
}

/**@}*/

/**@}*/

} // namespace pr

#if !DOXYGEN
#include "dualnum.inl"
#endif // #if !DOXYGEN

#endif // #ifndef PREFORM_DUALNUM_HPP
