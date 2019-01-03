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

template <typename T>
struct is_quat_param : 
            std::integral_constant<bool,
            std::is_arithmetic<T>::value ||
            is_complex<T>::value ||
            is_dualnum<T>::value>
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

    // Sanity check.
    static_assert(
        is_quat_param<T>::value,
        "T must be arithmetic, complex, or dualnum");

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
            const T& s, 
            const multi<T, 3>& v = {}) :
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
            const T& s, 
            const T& v0, 
            const T& v1, 
            const T& v2) :
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
    constexpr const T& real() const
    {
        return s_;
    }

    /**
     * @brief Get imag part.
     */
    constexpr const multi<T, 3>& imag() const
    {
        return v_;
    }

    /**
     * @brief Set real part, return previous real part.
     */
    constexpr T real(const T& val)
    {
        const T s = s_; s_ = val; return s;
    }

    /**
     * @brief Set imag part, return previous imag part.
     */
    constexpr multi<T, 3> imag(const multi<T, 3>& val)
    {
        const multi<T, 3> v = v_; v_ = val; return v;
    }

    /**@}*/

public: 

    /**
     * @brief Multiplicative inverse.
     */
    constexpr quat inverse() const
    {
        const T fac = s_ * s_ + pr::dot(v_, v_);
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
    constexpr explicit operator multi<U, 4>() const
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
    T s_ = {};

    /**
     * @brief Imag part.
     */
    multi<T, 3> v_ = {};

public:

    /**
     * @name Stream operators
     */
    /**@{*/

    /**
     * @brief Parse from `std::basic_istream`.
     *
     * Format is `(s,v)`. Sets `std::ios_base::failbit` on error.
     */
    template <typename C, typename Ctraits>
    friend
    inline std::basic_istream<C, Ctraits>& operator>>(
           std::basic_istream<C, Ctraits>& is, quat& q)
    {
        C ch;
        if (!(is >> ch) ||
            !Ctraits::eq(ch, 
             Ctraits::to_char_type('('))) {
            is.setstate(std::ios_base::failbit);
            return is;
        }
        is >> q.s_;
        if (!(is >> ch) ||
            !Ctraits::eq(ch, 
             Ctraits::to_char_type(','))) {
            is.setstate(std::ios_base::failbit);
            return is;
        }
        is >> q.v_;
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
     * Format is `(s,v)`.
     */
    template <typename C, typename Ctraits>
    friend
    inline std::basic_ostream<C, Ctraits>& operator<<(
           std::basic_ostream<C, Ctraits>& os, const quat& q)
    {
        os << '(';
        os << q.s_ << ',';
        os << q.v_ << ')';
        return os;
    }

    /**@}*/

public:

    /**
     * @brief Rotation.
     *
     * @param[in] theta
     * Radians.
     *
     * @param[in] axis
     * Normalized axis.
     */
    template <bool B = std::is_floating_point<T>::value>
    static std::enable_if_t<B, quat> rotation(T theta, const multi<T, 3>& axis)
    {
        return {pr::cos(theta), pr::sin(theta) * axis};
    }

    // TODO translation
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
 *
 * @f[
 *      (s_0 + \mathbf{v}_0) + s_1 = (s_0 + s_1) + \mathbf{v}_0
 * @f]
 */
template <typename T, typename U>
__attribute__((always_inline))
constexpr std::enable_if_t<is_quat_param<U>::value,
                    quat<decltype(T() + U())>> operator+(
                            const quat<T>& q0, const U& q1)
{
    return {q0.real() + q1, q0.imag()};
}

/**
 * @brief Distribute `operator-`.
 *
 * @f[
 *      (s_0 + \mathbf{v}_0) - s_1 = (s_0 - s_1) + \mathbf{v}_0
 * @f]
 */
template <typename T, typename U>
__attribute__((always_inline))
constexpr std::enable_if_t<
                        is_quat_param<U>::value,
                        quat<decltype(T() - U())>> operator-(
                            const quat<T>& q0, const U& q1)
{
    return {q0.real() - q1, q0.imag()};
}

/**
 * @brief Distribute `operator*`.
 *
 * @f[
 *      (s_0 + \mathbf{v}_0) s_1 = s_0 s_1 + \mathbf{v}_0 s_1
 * @f]
 */
template <typename T, typename U>
__attribute__((always_inline))
constexpr std::enable_if_t<
                        is_quat_param<U>::value,
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
                        is_quat_param<U>::value,
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
 *
 * @f[
 *      s_0 + (s_1 + \mathbf{v_1}) = (s_0 + s_1) + \mathbf{v_1}
 * @f]
 */
template <typename T, typename U>
__attribute__((always_inline))
constexpr std::enable_if_t<
                        is_quat_param<T>::value,
                        quat<decltype(T() + U())>> operator+(
                            const T& q0, const quat<U>& q1)
{
    return {q0 + q1.real(), q1.imag()};
}

/**
 * @brief Distribute `operator-`.
 * 
 * @f[
 *      s_0 - (s_1 + \mathbf{v_1}) = (s_0 - s_1) - \mathbf{v_1}
 * @f]
 */
template <typename T, typename U>
__attribute__((always_inline))
constexpr std::enable_if_t<
                        is_quat_param<T>::value,
                        quat<decltype(T() - U())>> operator-(
                            const T& q0, const quat<U>& q1)
{
    return {q0 - q1.real(), -q1.imag()};
}

/**
 * @brief Distribute `operator*`.
 *
 * @f[
 *      s_0 (s_1 + \mathbf{v_1}) = s_0 s_1 + s_0 \mathbf{v_1}
 * @f]
 */
template <typename T, typename U>
__attribute__((always_inline))
constexpr std::enable_if_t<
                        is_quat_param<T>::value,
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
                        is_quat_param<T>::value,
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
 * @name Comparison operators (quat)
 */
/**@{*/

/**
 * @brief Compare `operator==`.
 */
template <typename T, typename U>
__attribute__((always_inline))
constexpr bool operator==(const quat<T>& q0, const quat<U>& q1)
{
    return q0.real() == q1.real() && q0.imag() == q1.imag();
}

/**
 * @brief Compare `operator==`.
 */
template <typename T, typename U>
__attribute__((always_inline))
constexpr std::enable_if_t<
                        is_quat_param<U>::value, bool> operator==(
                        const quat<T>& q0, const U& q1)
{
    return q0.real() == q1 && q0.imag() == T();
}

/**
 * @brief Compare `operator==`.
 */
template <typename T, typename U>
__attribute__((always_inline))
constexpr std::enable_if_t<
                        is_quat_param<T>::value, bool> operator==(
                        const T& q0, const quat<U>& q1)
{
    return q0 == q1.real() && U() == q1.imag();
}

/**
 * @brief Compare `operator!=`.
 */
template <typename T, typename U>
__attribute__((always_inline))
constexpr bool operator!=(const quat<T>& q0, const quat<U>& q1)
{
    return !(q0 == q1);
}

/**
 * @brief Compare `operator!=`.
 */
template <typename T, typename U>
__attribute__((always_inline))
constexpr std::enable_if_t<
                        is_quat_param<U>::value, bool> operator!=(
                        const quat<T>& q0, const U& q1)
{
    return !(q0 == q1);
}

/**
 * @brief Compare `operator!=`.
 */
template <typename T, typename U>
__attribute__((always_inline))
constexpr std::enable_if_t<
                        is_quat_param<T>::value, bool> operator!=(
                        const T& q0, const quat<U>& q1)
{
    return !(q0 == q1);
}

/**@}*/

/**
 * @name Accessors (quat)
 */
/**@{*/

/**
 * @brief Real part.
 *
 * @f[
 *      \real(s + \mathbf{v}) = s
 * @f]
 */
template <typename T>
__attribute__((always_inline))
constexpr T real(const quat<T>& q)
{
    return q.real();
}

/**
 * @brief Imag part.
 *
 * @f[
 *      \imag(s + \mathbf{v}) = \mathbf{v}
 * @f]
 */
template <typename T>
__attribute__((always_inline))
constexpr multi<T, 3> imag(const quat<T>& q)
{
    return q.imag();
}

/**
 * @brief Conjugate.
 *
 * @f[
 *      (s + \mathbf{v})^\dagger =
 *       s - \mathbf{v}
 * @f]
 */
template <typename T>
__attribute__((always_inline))
constexpr quat<T> conj(const quat<T>& q)
{
    return {+q.real(), -q.imag()};
}

/**
 * @brief Norm square.
 *
 * @f[
 *      |(s + \mathbf{v})
 *       (s + \mathbf{v})^\dagger| = |s^2 + v^2|
 * @f]
 *
 * @note
 * If `T` is complex or dualnum, returns floating
 * point type.
 */
template <typename T> 
__attribute__((always_inline))
constexpr auto norm(const quat<T>& q)
{
    return pr::abs(pr::dot(
                static_cast<multi<T, 4>>(q), 
                static_cast<multi<T, 4>>(q)));
}

/**
 * @brief Absolute value.
 *
 * @f[
 *      |s + \mathbf{v}| = 
 *      |\sqrt{s^2 + v^2}|
 * @f]
 *
 * @note
 * If `T` is complex or dualnum, returns floating
 * point type.
 */
template <typename T>
__attribute__((always_inline))
inline auto abs(const quat<T>& q)
{
    return pr::length(static_cast<multi<T, 4>>(q));
}

/**@}*/

/**
 * @name Geometry (quat)
 */
/**@{*/

/**
 * @brief Dot product.
 */
template <typename T, typename U>
__attribute__((always_inline))
constexpr decltype(T() * U()) dot(const quat<T>& q0, const quat<U>& q1)
{
    return pr::dot(
                static_cast<multi<T, 4>>(q0), 
                static_cast<multi<U, 4>>(q1));
}

/**
 * @brief @f$ L^2 @f$ length.
 *
 * @note
 * If `T` is complex or dualnum, returns floating
 * point type.
 */
template <typename T>
__attribute__((always_inline))
inline auto length(const quat<T>& q)
{
    return pr::length(static_cast<multi<T, 4>>(q));
}

/**
 * @brief @f$ L^2 @f$ length, fast (and somewhat unsafe) variant.
 *
 * @note
 * If `T` is complex or dualnum, returns floating
 * point type.
 */
template <typename T>
__attribute__((always_inline))
inline auto fast_length(const quat<T>& q)
{
    return pr::fast_length(static_cast<multi<T, 4>>(q));
}

/**
 * @brief @f$ L^2 @f$ normalize.
 */
template <typename T>
__attribute__((always_inline))
inline quat<T> normalize(const quat<T>& q)
{
    return pr::normalize(static_cast<multi<T, 4>>(q));
}

/**
 * @brief @f$ L^2 @f$ normalize, fast (and somewhat unsafe) variant.
 */
template <typename T>
__attribute__((always_inline))
inline quat<T> fast_normalize(const quat<T>& q)
{
    return pr::fast_normalize(static_cast<multi<T, 4>>(q));
}

/**
 * @brief Inverse.
 *
 * @f[
 *      (s + \mathbf{v})^{-1} = 
 *      (s - \mathbf{v}) / (s^2 + v^2)
 * @f]
 */
template <typename T>
__attribute__((always_inline))
constexpr quat<T> inverse(const quat<T>& q)
{
    return q.inverse();
}

#if 0
/**
 * @brief Spherical linear interpolation.
 */
template <typename T>
inline std::enable_if_t<
       std::is_floating_point<T>::value, 
       quat<T>> slerp(T mu, const quat<T>& q0, const quat<T>& q1)
{
}
#endif

/**@}*/

/**
 * @name Math wrappers (quat)
 */
/**@{*/

/**
 * @brief Exponential.
 *
 * @f[
 *      \exp(s + \mathbf{v}) = 
 *      \exp(s) (\cos(\sqrt{v^2}) + 
 *               \sin(\sqrt{v^2}) \mathbf{v} / 
 *                    \sqrt{v^2})
 * @f]
 *
 * @note
 * This implementation is valid for all quaternion
 * parameters.
 */
template <typename T>
inline quat<T> exp(const quat<T>& q)
{
    T lenv = pr::dot(q.imag(), q.imag());
    if (pr::abs(lenv) > 0) {
        lenv = pr::sqrt(lenv);
        return pr::exp(q.real()) * 
                quat<T>(
                pr::cos(lenv), 
                pr::sin(lenv) * (q.imag() / lenv));
    }
    else {
        return pr::exp(q.real());
    }
}

/**
 * @brief Natural logarithm.
 *
 * @f[
 *      \log(s + \mathbf{v}) = 
 *      \log(\sqrt{s^2 + v^2}) + \arccos(s / \sqrt{s^2 + v^2}) 
 *      \mathbf{v} / \sqrt{v^2}
 * @f]
 *
 * @note
 * This implementation is valid for all quaternion
 * parameters.
 */
template <typename T>
inline quat<T> log(const quat<T>& q)
{
    T lenq = pr::dot(q, q);
    T lenv = pr::dot(q.imag(), q.imag());
    if (pr::abs(lenv) > 0) {
        lenq = pr::sqrt(lenq);
        lenv = pr::sqrt(lenv);
        return quat<T>(
                pr::log(lenq),
                pr::acos(q.real() / lenq) * (q.imag() / lenv));
    }
    else {
        lenq = pr::sqrt(lenq);
        return pr::log(lenq);
    }
}

/**@}*/

// TODO isinf
// TODO isnan
// TODO isfinite
// TODO isnormal

/**@}*/

} // namespace pr

#endif // #ifndef PREFABS_QUAT_HPP
