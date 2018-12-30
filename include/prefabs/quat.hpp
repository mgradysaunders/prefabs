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
#error "prefabs/quat.hpp requires >=C++17"
#endif // #if !(__cplusplus >= 201703L)
#endif // #if !DOXYGEN
#pragma once
#ifndef PREFABS_QUAT_HPP
#define PREFABS_QUAT_HPP

// for pr::dualnum
#include <prefabs/dualnum.hpp>

// for pr::multi
#include <prefabs/multi.hpp>

// for pr::dot, pr::cross
#include <prefabs/multi_math.hpp>

namespace pr {

/**
 * @defgroup quat Quaternion
 *
 * `<prefabs/quat.hpp>`
 *
 * __C++ version__: >=C++17
 */
/**@{*/

#if !DOXYGEN

template <typename T>
class quat;

template <typename T>
struct is_quat : std::false_type
{
};

template <typename T>
struct is_quat<quat<T>> : std::true_type
{
};

#endif // #if !DOXYGEN

/**
 * @brief Quaternion.
 */
template <typename T>
class quat
{
public:

    /**
     * @brief Real type.
     */
    typedef T real_type;

    /**
     * @brief Imag type.
     */
    typedef multi<T, 3> imag_type;

public:

    /**
     * @name Constructors
     */
    /**@{*/

    /**
     * @brief Default constructor.
     */
    constexpr quat() = default;

    /**
     * @brief Constructor.
     *
     * @param[in] s
     * Real part.
     *
     * @param[in] v
     * Imag part.
     */
    constexpr quat(
            const real_type& s, 
            const imag_type& v = {}) :
        s_(s),
        v_(v)
    {
    }

    /**
     * @brief Constructor.
     *
     * @param[in] s
     * Real part.
     *
     * @param[in] v0
     * Imag part, 0th component.
     *
     * @param[in] v1
     * Imag part, 1st component.
     *
     * @param[in] v2
     * Imag part, 2nd component.
     */
    constexpr quat(
            const real_type& s, 
            const real_type& v0, 
            const real_type& v1, 
            const real_type& v2) :
        s_(s),
        v_{{v0, v1, v2}}
    {
    }

    /**
     * @brief Constructor.
     *
     * @param[in] x
     * Component array.
     */
    constexpr quat(const multi<T, 4>& x) :
        s_(x[0]),
        v_{{x[1], x[2], x[3]}}
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
    constexpr const real_type& real() const
    {
        return s_;
    }

    /**
     * @brief Get imag part.
     */
    constexpr const imag_type& imag() const
    {
        return v_;
    }

    /**
     * @brief Set real part, return previous real part.
     */
    constexpr real_type real(const real_type& val)
    {
        const real_type s = s_; s_ = val; return s;
    }

    /**
     * @brief Set imag part, return previous imag part.
     */
    constexpr imag_type imag(const imag_type& val)
    {
        const imag_type v = v_; v_ = val; return v;
    }

    /**@}*/

public:
    
    // TODO slerp

    constexpr quat inverse() const
    {
        const real_type fac = s_ * s_ + pr::dot(v_, v_);
        return {
            +s_ / fac, 
            -v_ / fac
        };
    }

public:

    /**
     * @name Cast operators
     */
    /**@{*/

    /**
     * @brief Cast as different real type.
     */
    template <typename U>
    constexpr operator quat<U>() const
    {
        return {
            static_cast<U>(s_),
            static_cast<U>(v_[0]),
            static_cast<U>(v_[1]),
            static_cast<U>(v_[2])
        };
    }

    /**
     * @brief Cast as component array.
     */
    template <typename U>
    constexpr operator multi<U, 4>() const
    {
        return {{
            static_cast<U>(s_),
            static_cast<U>(v_[0]),
            static_cast<U>(v_[1]),
            static_cast<U>(v_[2])
        }};
    }

    /**@}*/

private:

    /**
     * @brief Real part.
     */
    real_type s_ = {};

    /**
     * @brief Imag part.
     */
    imag_type v_ = {};

private:

    // TODO operator<<
    // TODO operator>>
};

/**
 * @name Unary operators (quat)
 */
/**@{*/

/**
 * @brief Distribute `operator+`.
 */
template <typename T>
__attribute__((always_inline))
constexpr quat<T> operator+(const quat<T>& q)
{
    return {+q.real(), +q.imag()};
}

/**
 * @brief Distribute `operator-`.
 */
template <typename T>
__attribute__((always_inline))
constexpr quat<T> operator-(const quat<T>& q)
{
    return {-q.real(), -q.imag()};
}

/**@}*/

/**
 * @name Binary operators (quat/quat)
 */
/**@{*/

/**
 * @brief Distribute `operator+`.
 *
 * @f[
 *      (s_0 + \mathbf{v}_0) +
 *      (s_1 + \mathbf{v}_1) =
 *      (s_0 + s_1) + (\mathbf{v}_0 + \mathbf{v}_1)
 * @f]
 */
template <typename T, typename U>
__attribute__((always_inline))
constexpr quat<decltype(T() + U())> operator+(
                        const quat<T>& q0, const quat<U>& q1)
{
    return {q0.real() + q1.real(), q0.imag() + q1.imag()};
}

/**
 * @brief Distribute `operator-`.
 *
 * @f[
 *      (s_0 + \mathbf{v}_0) -
 *      (s_1 + \mathbf{v}_1) =
 *      (s_0 - s_1) + (\mathbf{v}_0 - \mathbf{v}_1)
 * @f]
 */
template <typename T, typename U>
__attribute__((always_inline))
constexpr quat<decltype(T() - U())> operator-(
                        const quat<T>& q0, const quat<U>& q1)
{
    return {q0.real() - q1.real(), q0.imag() - q1.imag()};
}

/**
 * @brief Distribute `operator*`.
 *
 * @f[
 *      (s_0 + \mathbf{v}_0)
 *      (s_1 + \mathbf{v}_1) =
 *      (s_0 s_1 - \mathbf{v}_0 \cdot \mathbf{v}_1) +
 *      (s_0 \mathbf{v}_1 +
 *       s_1 \mathbf{v}_0 +
 *       \mathbf{v}_0 \times
 *       \mathbf{v}_1)
 * @f]
 */
template <typename T, typename U>
__attribute__((always_inline))
constexpr quat<decltype(T() * U())> operator*(
                        const quat<T>& q0, const quat<U>& q1)
{
    return {
        q0.real() * q1.real() - pr::dot(q0.imag(), q1.imag()),
        q0.real() * q1.imag() + 
        q0.imag() * q1.real() +
        pr::cross(q0.imag(), q1.imag())
    };
}

/**
 * @brief Distribute `operator*`, inverting right hand side.
 */
template <typename T, typename U>
__attribute__((always_inline))
constexpr quat<decltype(T() * U())> operator/(
                        const quat<T>& q0, const quat<U>& q1)
{
    return q0 * q1.inverse();
}

/**@}*/

/**
 * @name Binary operators (quat/num)
 */
/**@{*/

/**
 * @brief Distribute `operator+`.
 */
template <typename T, typename U>
__attribute__((always_inline))
constexpr std::enable_if_t<
                !is_quat<U>::value,
                    quat<decltype(T() + U())>> operator+(
                            const quat<T>& q0, const U& q1)
{
    return {q0.real() + q1, q0.imag()};
}

/**
 * @brief Distribute `operator-`.
 */
template <typename T, typename U>
__attribute__((always_inline))
constexpr std::enable_if_t<
                !is_quat<U>::value,
                    quat<decltype(T() - U())>> operator-(
                            const quat<T>& q0, const U& q1)
{
    return {q0.real() - q1, q0.imag()};
}

/**
 * @brief Distribute `operator*`.
 */
template <typename T, typename U>
__attribute__((always_inline))
constexpr std::enable_if_t<
                !is_quat<U>::value,
                    quat<decltype(T() * U())>> operator*(
                            const quat<T>& q0, const U& q1)
{
    return {q0.real() * q1, q0.imag() * q1};
}

/**
 * @brief Distribute `operator*`, inverting right hand side.
 */
template <typename T, typename U>
__attribute__((always_inline))
constexpr std::enable_if_t<
                !is_quat<U>::value,
                    quat<decltype(T() * U())>> operator/(
                            const quat<T>& q0, const U& q1)
{
    return {q0.real() / q1, q0.imag() / q1};
}

/**@}*/

/**
 * @name Binary operators (num/quat)
 */
/**@{*/

/**
 * @brief Distribute `operator+`.
 */
template <typename T, typename U>
__attribute__((always_inline))
constexpr std::enable_if_t<
                !is_quat<T>::value,
                    quat<decltype(T() + U())>> operator+(
                            const T& q0, const quat<U>& q1)
{
    return {q0 + q1.real(), q1.imag()};
}

/**
 * @brief Distribute `operator-`.
 */
template <typename T, typename U>
__attribute__((always_inline))
constexpr std::enable_if_t<
                !is_quat<T>::value,
                    quat<decltype(T() - U())>> operator-(
                            const T& q0, const quat<U>& q1)
{
    return {q0 - q1.real(), -q1.imag()};
}

/**
 * @brief Distribute `operator*`.
 */
template <typename T, typename U>
__attribute__((always_inline))
constexpr std::enable_if_t<
                !is_quat<T>::value,
                    quat<decltype(T() * U())>> operator*(
                            const T& q0, const quat<U>& q1)
{
    return {q0 * q1.real(), q0 * q1.imag()};
}

/**
 * @brief Distribute `operator*`, inverting right hand side.
 */
template <typename T, typename U>
__attribute__((always_inline))
constexpr std::enable_if_t<
                !is_quat<T>::value,
                    quat<decltype(T() * U())>> operator/(
                            const T& q0, const quat<U>& q1)
{
    return q0 * q1.inverse();
}

/**@}*/

/**
 * @name Binary operators (quat/any)
 */
/**@{*/

/**
 * @brief Generic `operator+=`.
 */
template <typename T, typename U>
__attribute__((always_inline))
constexpr quat<T>& operator+=(quat<T>& q, const U& any)
{
    return q = q + any;
}

/**
 * @brief Generic `operator-=`.
 */
template <typename T, typename U>
__attribute__((always_inline))
constexpr quat<T>& operator-=(quat<T>& q, const U& any)
{
    return q = q - any;
}

/**
 * @brief Generic `operator*=`.
 */
template <typename T, typename U>
__attribute__((always_inline))
constexpr quat<T>& operator*=(quat<T>& q, const U& any)
{
    return q = q * any;
}

/**
 * @brief Generic `operator/=`.
 */
template <typename T, typename U>
__attribute__((always_inline))
constexpr quat<T>& operator/=(quat<T>& q, const U& any)
{
    return q = q / any;
}

/**@}*/

/**
 * @name Complex accessors (quat)
 */
/**@{*/

/**
 * @brief Conjugate with respect to @f$ \mathbf{i] @f$.
 */
constexpr long conj_quat = 0b0100L;

// TODO real
// TODO imag
// TODO conj
// TODO norm

/**@}*/

// TODO dot
// TODO length
// TODO fast_length
// TODO normalize
// TODO fast_normalize
// TODO inverse

/**@}*/

} // namespace pr

#endif // #ifndef PREFABS_QUAT_HPP
