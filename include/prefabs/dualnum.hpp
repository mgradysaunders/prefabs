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
#ifndef PREFABS_DUALNUM_HPP
#define PREFABS_DUALNUM_HPP

// use c++17
#if !(__cplusplus >= 201703L) 
#error "prefabs/dualnum.hpp requires C++17"
#endif // #if !(__cplusplus >= 201703L) 

// for std::basic_istream
#include <istream>

// for std::basic_ostream
#include <ostream>

#include <prefabs/math.hpp>

namespace pr {

/**
 * @defgroup dualnum Dual numbers
 *
 * `<prefabs/dualnum.hpp>`
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

#endif // #if !DOXYGEN

/**
 * @brief Dual number.
 */
template <typename T>
class dualnum
{
public:

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
     * @brief Default copy constructor.
     */
    constexpr dualnum(const dualnum&) = default;

    /**
     * @brief Default move constructor.
     */
    constexpr dualnum(dualnum&&) = default;

    /**
     * @brief Constructor.
     */
    constexpr dualnum(T a, T b = T()) : a_(a), b_(b)
    {
    }

    /**@}*/

public:

    /**
     * @name Assign operators
     */
    /**@{*/

    /**
     * @brief Default copy assign.
     */
    constexpr dualnum& operator=(const dualnum&) noexcept = default;

    /**
     * @brief Default move assign.
     */
    constexpr dualnum& operator=(dualnum&&) noexcept = default;

    /**
     * @brief Real assign.
     */
    constexpr dualnum& operator=(T a) noexcept
    {
        return *this = dualnum(a);
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
    constexpr const T& real() const noexcept
    {
        return a_;
    }

    /**
     * @brief Get dual part.
     */
    constexpr const T& dual() const noexcept
    {
        return b_;
    }

    /**
     * @brief Set real part, return previous real part.
     */
    constexpr T real(T val) noexcept 
    { 
        T a = a_; a_ = val; return a; 
    }

    /**
     * @brief Set dual part, return previous dual part.
     */
    constexpr T dual(T val) noexcept
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
            !Traits::eq(ch, 
             Traits::to_char_type('('))) {
            is.setstate(std::ios_base::failbit);
            return is;
        }
        is >> x.a_;
        if (!(is >> ch) ||
            !Traits::eq(ch, 
             Traits::to_char_type(','))) {
            is.setstate(std::ios_base::failbit);
            return is;
        }
        is >> x.b_;
        if (!(is >> ch) ||
            !Traits::eq(ch, 
             Traits::to_char_type(')'))) {
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
    return {
        +x.real(),
        +x.dual()
    };
}

/**
 * @brief Distribute `operator-`.
 */
template <typename T>
__attribute__((always_inline))
constexpr dualnum<T> operator-(const dualnum<T>& x)
{
    return {
        -x.real(),
        -x.dual()
    };
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
 *      (\real(x_0) + \varepsilon \dual(x_0)) + 
 *      (\real(x_1) + \varepsilon \dual(x_1)) = 
 *      (\real(x_0) + \real(x_1)) + \varepsilon 
 *      (\dual(x_0) + \dual(x_1))
 * @f]
 */
template <typename T, typename U>
__attribute__((always_inline))
constexpr dualnum<decltype(T() + U())> operator+(
                    const dualnum<T>& x0, const dualnum<U>& x1)
{
    return {
        x0.real() + x1.real(),
        x0.dual() + x1.dual()
    };
}

/**
 * @brief Distribute `operator-`.
 *
 * @f[
 *      (\real(x_0) + \varepsilon \dual(x_0)) -
 *      (\real(x_1) + \varepsilon \dual(x_1)) = 
 *      (\real(x_0) - \real(x_1)) + \varepsilon 
 *      (\dual(x_0) - \dual(x_1))
 * @f]
 */
template <typename T, typename U>
__attribute__((always_inline))
constexpr dualnum<decltype(T() - U())> operator-(
                    const dualnum<T>& x0, const dualnum<U>& x1)
{
    return {
        x0.real() - x1.real(),
        x0.dual() - x1.dual()
    };
}

/**
 * @brief Distribute `operator*`.
 *
 * @f[
 *      (\real(x_0) + \varepsilon \dual(x_0))
 *      (\real(x_1) + \varepsilon \dual(x_1)) = 
 *       \real(x_0) \real(x_1) + \varepsilon 
 *      (\real(x_0) \dual(x_1) + \dual(x_0) \real(x_1))
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
 *      (\real(x_0) + \varepsilon \dual(x_0)) 
 *      (\real(x_1) + \varepsilon \dual(x_1))^{-1} = 
 *       \real(x_0)\real(x_1)^{-1} + \varepsilon 
 *      (\dual(x_0)\real(x_1) - \real(x_0)\dual(x_1)) \real(x_1)^{-2}
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
 *      (\real(x_0) + \varepsilon \dual(x_0)) + x_1 = 
 *      (\real(x_0) + x_1) + \varepsilon \dual(x_0)
 * @f]
 */
template <typename T, typename U>
__attribute__((always_inline))
constexpr std::enable_if_t<
                !is_dualnum<U>::value, 
                    dualnum<decltype(T() + U())>> operator+(
                            const dualnum<T>& x0, const U& x1)
{
    return {
        x0.real() + x1,
        x0.dual()
    };
}

/**
 * @brief Distribute `operator-`.
 *
 * @f[
 *      (\real(x_0) + \varepsilon \dual(x_0)) - x_1 = 
 *      (\real(x_0) - x_1) + \varepsilon \dual(x_0)
 * @f]
 */
template <typename T, typename U>
__attribute__((always_inline))
constexpr std::enable_if_t<
                !is_dualnum<U>::value,
                    dualnum<decltype(T() - U())>> operator-(
                            const dualnum<T>& x0, const U& x1)
{
    return {
        x0.real() - x1,
        x0.dual()
    };
}

/**
 * @brief Distribute `operator*`.
 *
 * @f[
 *      (\real(x_0) + \varepsilon \dual(x_0)) x_1 = 
 *       \real(x_0) x_1 + \varepsilon \dual(x_0) x_1
 * @f]
 */
template <typename T, typename U>
__attribute__((always_inline))
constexpr std::enable_if_t<
                !is_dualnum<U>::value,
                    dualnum<decltype(T() * U())>> operator*(
                            const dualnum<T>& x0, const U& x1)
{
    return {
        x0.real() * x1,
        x0.dual() * x1
    };
}

/**
 * @brief Distribute `operator*`, inverting right hand side.
 *
 * @f[
 *      (\real(x_0) + \varepsilon \dual(x_0)) x_1^{-1} = 
 *       \real(x_0) x_1^{-1} + \varepsilon \dual(x_0) x_1^{-1}
 * @f]
 */
template <typename T, typename U>
__attribute__((always_inline))
constexpr std::enable_if_t<
                !is_dualnum<U>::value,
                    dualnum<decltype(T() / U())>> operator/(
                            const dualnum<T>& x0, const U& x1)
{
    return {
        x0.real() / x1,
        x0.dual() / x1
    };
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
 *       x_0 + (\real(x_1) + \varepsilon \dual(x_1)) = 
 *      (x_0 + \real(x_1)) + \varepsilon \dual(x_1)
 * @f]
 */
template <typename T, typename U>
__attribute__((always_inline))
constexpr std::enable_if_t<
                !is_dualnum<T>::value, 
                    dualnum<decltype(T() + U())>> operator+(
                            const T& x0, const dualnum<U>& x1)
{
    return {
        x1.real() + x0,
        x1.dual()
    };
}

/**
 * @brief Distribute `operator-`.
 *
 * @f[
 *       x_0 - (\real(x_1) + \varepsilon \dual(x_1))  = 
 *      (x_0 - \real(x_1)) - \varepsilon \dual(x_1)
 * @f]
 */
template <typename T, typename U>
__attribute__((always_inline))
constexpr std::enable_if_t<
                !is_dualnum<T>::value,
                    dualnum<decltype(T() - U())>> operator-(
                            const T& x0, const dualnum<U>& x1)
{
    return {
        -x1.real() + x0,
        -x1.dual()
    };
}

/**
 * @brief Distribute `operator*`.
 *
 * @f[
 *      x_0 (\real(x_1) + \varepsilon \dual(x_1)) = 
 *      x_0  \real(x_1) + \varepsilon x_0 \dual(x_1)
 * @f]
 */
template <typename T, typename U>
__attribute__((always_inline))
constexpr std::enable_if_t<
                !is_dualnum<T>::value,
                    dualnum<decltype(T() * U())>> operator*(
                            const T& x0, const dualnum<U>& x1)
{
    return {
        x0 * x1.real(),
        x0 * x1.dual()
    };
}

/**
 * @brief Distribute `operator*`, inverting right hand side.
 *
 * @f[
 *      x_0 (\real(x_1) + \varepsilon \dual(x_1))^{-1} = 
 *      x_0  \real(x_1)^{-1} - \varepsilon x_0 \dual(x_1) \real(x_1)^{-2}
 * @f]
 */
template <typename T, typename U>
__attribute__((always_inline))
constexpr std::enable_if_t<
                !is_dualnum<T>::value,
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
 * @name Complex accessors (dualnum)
 */
/**@{*/

/**
 * @brief Conjugate operator.
 */
enum conj_op
{
    /**
     * @brief Conjugate with respect to @f$ i @f$.
     */
    conj_imag,

    /**
     * @brief Conjugate with respect to @f$ \varepsilon @f$.
     */
    conj_dual
};

/**
 * @brief Real part (with respect to given conjugate operator).
 */
template <conj_op op = conj_imag, typename T> 
constexpr auto real(const dualnum<T>& x)
{
    if constexpr (op == conj_imag) {
        return dualnum<T>{
            pr::real(x.real()), 
            pr::real(x.dual())
        };
    }
    else {
        return x.real();
    }
}

/**
 * @brief Imag part.
 */
template <typename T> 
constexpr auto imag(const dualnum<T>& x)
{
    return dualnum<T>{
        pr::imag(x.real()), 
        pr::imag(x.dual())
    };
}

/**
 * @brief Dual part.
 */
template <typename T> 
constexpr auto dual(const dualnum<T>& x)
{
    return x.dual();
}

/**
 * @brief Conjugate.
 */
template <conj_op op = conj_imag, typename T> 
constexpr auto conj(const dualnum<T>& x)
{
    if constexpr (op == conj_imag) {
        return dualnum<T>{
            pr::conj(x.real()), 
            pr::conj(x.dual())
        };
    }
    else {
        return dualnum<T>{x.real(), -x.dual()};
    }
}

/**
 * @brief Norm square.
 */
template <conj_op op = conj_imag, typename T> 
constexpr auto norm(const dualnum<T>& x)
{
    if constexpr (op == conj_imag) {
        return dualnum<decltype(pr::norm(T()))>{
            pr::norm(x.real()),
            pr::real(x.real() * pr::conj(x.dual())) * 2
        };
    }
    else {
        return x.real() * x.real();
    }
}

/**@}*/

/**@}*/

#include "dualnum.inl"

} // namespace pr

#endif // #ifndef PREFABS_DUALNUM_HPP
