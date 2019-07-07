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
#if !(__cplusplus >= 201703L)
#error "preform/quat.hpp requires >=C++17"
#endif // #if !(__cplusplus >= 201703L)
#endif // #if !DOXYGEN
#pragma once
#ifndef PREFORM_QUAT_HPP
#define PREFORM_QUAT_HPP

// for pr::dualnum
#include <preform/dualnum.hpp>

// for pr::multi
#include <preform/multi.hpp>

// for pr::multi wrappers
#include <preform/multi_math.hpp>

namespace pr {

/**
 * @defgroup quat Quaternion
 *
 * `<preform/quat.hpp>`
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
        /*  is_complex<T>::value ||  */
            is_dualnum<T>::value>
{
};

#endif // #if !DOXYGEN

/**
 * @brief Quaternion.
 *
 * @tparam T
 * Float type.
 */
template <typename T>
class quat
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
     */
    constexpr quat(const value_type& s) : s_(s)
    {
    }

    /**
     * @brief Constructor.
     */
    constexpr quat(
            const value_type& s,
            const multi<value_type, 3>& v) :
        s_(s),
        v_(v)
    {
    }

    /**
     * @brief Constructor.
     */
    constexpr quat(
            const value_type& s,
            const value_type& v0,
            const value_type& v1,
            const value_type& v2) :
        s_(s),
        v_{v0, v1, v2}
    {
    }

    /**
     * @brief Constructor.
     */
    constexpr quat(const multi<value_type, 4>& x) :
        s_(x[0]),
        v_{x[1], x[2], x[3]}
    {
    }

    // TODO
#if 0
    constexpr explicit quat(const multi<value_type, 3, 3>& x)
    {
    }
#endif

    /**@}*/

public:

    /**
     * @name Accessors
     */
    /**@{*/

    /**
     * @brief Get real part.
     */
    constexpr value_type real() const
    {
        return s_;
    }

    /**
     * @brief Get imag part.
     */
    constexpr multi<value_type, 3> imag() const
    {
        return v_;
    }

    /**
     * @brief Set real part.
     */
    constexpr value_type real(const value_type& val)
    {
        const value_type s = s_; s_ = val; return s;
    }

    /**
     * @brief Set imag part.
     */
    constexpr multi<value_type, 3> imag(const multi<value_type, 3>& val)
    {
        const multi<value_type, 3> v = v_; v_ = val; return v;
    }

    /**@}*/

public:

    /**
     * @name Cast operators
     */
    /**@{*/

    /**
     * @brief Cast as different value type.
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
        return {
            static_cast<U>(s_),
            static_cast<U>(v_[0]),
            static_cast<U>(v_[1]),
            static_cast<U>(v_[2])
        };
    }

    /**
     * @brief Cast as matrix.
     *
     * @f[
     *      \mathbf{M} =
     *          s^2 \mathbf{I} +
     *          2 s \mathbf{v}_{\times} +
     *              \mathbf{v}_{\times}^2 +
     *              \mathbf{v} \mathbf{v}^\top
     * @f]
     */
    constexpr explicit operator multi<value_type, 3, 3>() const
    {
        value_type s = s_;
        value_type x = v_[0];
        value_type y = v_[1];
        value_type z = v_[2];
        value_type s2 = s * s, sx = s * x, sy = s * y, sz = s * z;
        value_type x2 = x * x, xy = x * y, xz = x * z;
        value_type y2 = y * y, yz = y * z;
        value_type z2 = z * z;
        return {
            {(s2 + x2) - (y2 + z2), 2 * (xy - sz), 2 * (xz + sy)},
            {2 * (xy + sz), (s2 - x2) + (y2 - z2), 2 * (yz - sx)},
            {2 * (xz - sy), 2 * (yz + sx), (s2 - x2) - (y2 - z2)}
        };
    }

    /**@}*/

public:

    /**
     * @name Basic operations
     */
    /**@{*/

    /**
     * @brief Conjugate.
     *
     * @f[
     *      q^\dagger = s - \mathbf{v}
     * @f]
     */
    constexpr quat conj() const
    {
        return {
            +real(),
            -imag()
        };
    }

    /**
     * @brief Norm.
     *
     * @f[
     *      q q^\dagger = s^2 + \mathbf{v} \cdot \mathbf{v}
     * @f]
     */
    constexpr value_type norm() const
    {
        return real() * real() + dot(imag(), imag());
    }

    /**
     * @brief Multiplicative inverse.
     *
     * @f[
     *      q^{-1} = \frac{q^\dagger}{q q^\dagger}
     * @f]
     */
    constexpr quat inverse() const
    {
        value_type fac = norm();
        return {
            +real() / fac,
            -imag() / fac
        };
    }

    /**
     * @brief Apply transform operator.
     *
     * @f[
     *      q(\mathbf{u})
     *      = q \mathbf{u} q^\dagger
     *      = s^2 \mathbf{u} +
     *        2 s \mathbf{v} \times \mathbf{u} +
     *        \mathbf{v} \times
     *        \mathbf{v} \times \mathbf{u} +
     *        \mathbf{v} \mathbf{v}^\top \mathbf{u}
     * @f]
     */
    template <typename U>
    constexpr multi<U, 3> operator()(const multi<U, 3>& u) const
    {
        U s = s_;
        multi<U, 3> v = v_;
        multi<U, 3> w = cross(v, u);
        return (s * s) * u + (U(2) * s) * w + (cross(v, w) + v * dot(v, u));
    }

    /**@}*/

private:

    /**
     * @brief Real part.
     */
    value_type s_ = {};

    /**
     * @brief Imag part.
     */
    multi<value_type, 3> v_ = {};

public:

    /**
     * @brief Rotate counter-clockwise about arbitrary axis.
     *
     * @param[in] phi
     * Angle in radians.
     *
     * @param[in] hatv
     * Normalized rotation axis.
     */
    static quat rotate(
                value_type theta,
                const multi<value_type, 3>& hatv)
    {
        return {
            pr::cos(theta * value_type(0.5)),
            pr::sin(theta * value_type(0.5)) * hatv
        };
    }

    /**
     * @brief Spherical linear interpolation.
     *
     * @param[in] mu
     * Factor @f$ \mu \in [0, 1] @f$.
     *
     * @param[in] q0
     * Versor @f$ q_0 @f$ for @f$ \mu = 0 @f$.
     *
     * @param[in] q1
     * Versor @f$ q_1 @f$ for @f$ \mu = 1 @f$.
     *
     * @param[out] dq_dmu
     * Derivative @f$ dq/d\mu @f$. _Optional_.
     */
    static quat slerp(
                value_type mu,
                const quat& q0,
                const quat& q1,
                quat* dq_dmu = nullptr)
    {
        value_type cos_theta = dot(q0, q1);
        cos_theta = pr::fmax(cos_theta, value_type(-1));
        cos_theta = pr::fmin(cos_theta, value_type(+1));
        if (cos_theta > value_type(0.9999)) {
            quat q = (1 - mu) * q0 + mu * q1;
            value_type invlen2 = 1 / dot(q, q);
            value_type invlen1 = pr::sqrt(invlen2);
            quat hatq = q * invlen1;
            if (dq_dmu) {
                *dq_dmu =
                    invlen1 * (q1 - q0) +
                    invlen2 * (1 - 2 * mu) *
                              (1 - cos_theta) * hatq;
            }
            return hatq;
        }
        else {
            value_type theta = pr::acos(cos_theta);
            value_type sin_mutheta = pr::sin(mu * theta);
            value_type cos_mutheta = pr::cos(mu * theta);
            quat qperp = q1 - q0 * cos_theta;
            qperp *= 1 / pr::sqrt(qperp.norm());
            quat q =
                cos_mutheta * q0 +
                sin_mutheta * qperp;
            if (dq_dmu) {
                *dq_dmu =
                    theta * cos_mutheta * qperp -
                    theta * sin_mutheta * q0;
            }
            return q;
        }
    }
};

/**
 * @brief Dual quaternion.
 *
 * @tparam T
 * Float type.
 */
template <typename T>
class quat<dualnum<T>>
{
public:

    // Sanity check.
    static_assert(
        std::is_floating_point<T>::value,
        "T must be floating point");

    /**
     * @brief Value type.
     */
    typedef dualnum<T> value_type;

    /**
     * @brief Float type.
     */
    typedef T float_type;

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
     */
    constexpr quat(const float_type& s) : s_(s)
    {
    }

    /**
     * @brief Constructor.
     */
    constexpr quat(const value_type& s) : s_(s)
    {
    }

    /**
     * @brief Constructor.
     */
    constexpr quat(
            const value_type& s,
            const multi<value_type, 3>& v) :
        s_(s),
        v_(v)
    {
    }

    /**
     * @brief Constructor.
     */
    constexpr quat(
            const value_type& s,
            const value_type& v0,
            const value_type& v1,
            const value_type& v2) :
        s_(s),
        v_{v0, v1, v2}
    {
    }

    /**
     * @brief Constructor.
     */
    constexpr quat(const multi<value_type, 4>& x) :
        s_(x[0]),
        v_{x[1], x[2], x[3]}
    {
    }

    /**
     * @brief Constructor.
     */
    constexpr quat(
            const quat<float_type>& a,
            const quat<float_type>& b) :
        s_(value_type(a.real(), b.real())),
        v_{value_type(a.imag()[0], b.imag()[0]),
           value_type(a.imag()[1], b.imag()[1]),
           value_type(a.imag()[2], b.imag()[2])}
    {
    }

    // TODO
#if 0
    constexpr explicit quat(const multi<float_type, 4, 4>& x)
    {
    }
#endif

    /**@}*/

public:

    /**
     * @name Accessors
     */

    /**
     * @brief Get real part.
     */
    constexpr value_type real() const
    {
        return s_;
    }

    /**
     * @brief Get imag part.
     */
    constexpr multi<value_type, 3> imag() const
    {
        return v_;
    }

    /**
     * @brief Set real part.
     */
    constexpr value_type real(const value_type& val)
    {
        const value_type s = s_; s_ = val; return s;
    }

    /**
     * @brief Set imag part.
     */
    constexpr multi<value_type, 3> imag(const multi<value_type, 3>& val)
    {
        const multi<value_type, 3> v = v_; v_ = val; return v;
    }

    /**
     * @brief Get real quaternion part.
     */
    constexpr quat<float_type> realquat() const
    {
        return {
            s_.real(),
            v_[0].real(),
            v_[1].real(),
            v_[2].real()
        };
    }

    /**
     * @brief Get dual quaternion part.
     */
    constexpr quat<float_type> dualquat() const
    {
        return {
            s_.dual(),
            v_[0].dual(),
            v_[1].dual(),
            v_[2].dual()
        };
    }

    /**
     * @brief Set real quaternion part.
     */
    constexpr quat<float_type> realquat(const quat<float_type>& val)
    {
        return {
            s_.real(val.real()),
            v_[0].real(val.imag()[0]),
            v_[1].real(val.imag()[1]),
            v_[2].real(val.imag()[2])
        };
    }

    /**
     * @brief Set dual quaternion part.
     */
    constexpr quat<float_type> dualquat(const quat<float_type>& val)
    {
        return {
            s_.dual(val.real()),
            v_[0].dual(val.imag()[0]),
            v_[1].dual(val.imag()[1]),
            v_[2].dual(val.imag()[2])
        };
    }

    /**@}*/

public:

    /**
     * @name Basic operations
     */
    /**@{*/

    /**
     * @brief Conjugate.
     *
     * @f[
     *      q^\dagger = s - \mathbf{v}
     * @f]
     */
    constexpr quat conj() const
    {
        return {
            +real(),
            -imag()
        };
    }

    /**
     * @brief Dual conjugate.
     *
     * @f[
     *      q^\circ
     *      = s^\circ + \mathbf{v}^\circ
     *      = a - \varepsilon b
     * @f]
     */
    constexpr quat dualconj() const
    {
        return {
            +realquat(),
            -dualquat()
        };
    }

    /**
     * @brief Norm.
     *
     * @f[
     *      q q^\dagger = s^2 + \mathbf{v} \cdot \mathbf{v}
     * @f]
     */
    constexpr value_type norm() const
    {
        return real() * real() + dot(imag(), imag());
    }

    /**
     * @brief Multiplicative inverse.
     *
     * @f[
     *      q^{-1} = \frac{q^\dagger}{q q^\dagger}
     * @f]
     */
    constexpr quat inverse() const
    {
        value_type fac = norm();
        return {
            +real() / fac,
            -imag() / fac
        };
    }

    /**@}*/

public:

    /**
     * @name Cast operators
     */
    /**@{*/

    /**
     * @brief Cast as different value type.
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
        return {
            static_cast<U>(s_),
            static_cast<U>(v_[0]),
            static_cast<U>(v_[1]),
            static_cast<U>(v_[2])
        };
    }

    // TODO
#if 0
    /**
     * @brief Cast as matrix.
     */
    constexpr explicit operator multi<float_type, 4, 4>() const
    {
    }
#endif

    /**@}*/

private:

    /**
     * @brief Real part.
     */
    value_type s_ = {};

    /**
     * @brief Imag part.
     */
    multi<value_type, 3> v_ = {};

public:

    /**
     * @brief Rotation.
     *
     * @param[in] phi
     * Radians.
     *
     * @param[in] w
     * Normalized axis.
     */
    static quat rotation(
                float_type phi,
                const multi<float_type, 3>& w)
    {
        return {
            quat<float_type>::rotation(phi, w),
            quat<float_type>()
        };
    }

    /**
     * @brief Translation.
     *
     * @param[in] x
     * Offset.
     */
    static quat translation(const multi<float_type, 3>& x)
    {
        return {
            value_type(1, 0),
            value_type(0, x[0] * float_type(0.5)),
            value_type(0, x[1] * float_type(0.5)),
            value_type(0, x[2] * float_type(0.5))
        };
    }
};

/**
 * @name Stream operators (quat)
 */
/**@{*/

/**
 * @brief Parse from `std::basic_istream`.
 *
 * Format is `(s,v)`. Sets `std::ios_base::failbit` on error.
 */
template <typename T, typename C, typename Ctraits>
inline std::basic_istream<C, Ctraits>& operator>>(
       std::basic_istream<C, Ctraits>& is, quat<T>& q)
{
    C ch;
    if (!(is >> ch) ||
        !Ctraits::eq(ch,
         Ctraits::to_char_type('('))) {
        is.setstate(std::ios_base::failbit);
        return is;
    }
    T s;
    is >> s;
    if (!(is >> ch) ||
        !Ctraits::eq(ch,
         Ctraits::to_char_type(','))) {
        is.setstate(std::ios_base::failbit);
        return is;
    }
    multi<T, 3> v;
    is >> v;
    if (!(is >> ch) ||
        !Ctraits::eq(ch,
         Ctraits::to_char_type(')'))) {
        is.setstate(std::ios_base::failbit);
        return is;
    }
    q = {s, v};
    return is;
}

/**
 * @brief Write into `std::basic_ostream`.
 *
 * Format is `(s,v)`.
 */
template <typename T, typename C, typename Ctraits>
inline std::basic_ostream<C, Ctraits>& operator<<(
       std::basic_ostream<C, Ctraits>& os, const quat<T>& q)
{
    os << '(';
    os << q.real() << ',';
    os << q.imag() << ')';
    return os;
}

/**@}*/

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
    return {
        +q.real(),
        +q.imag()
    };
}

/**
 * @brief Distribute `operator-`.
 */
template <typename T>
__attribute__((always_inline))
constexpr quat<T> operator-(const quat<T>& q)
{
    return {
        -q.real(),
        -q.imag()
    };
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
    return {
        q0.real() + q1.real(),
        q0.imag() + q1.imag()
    };
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
    return {
        q0.real() - q1.real(),
        q0.imag() - q1.imag()
    };
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
        q0.real() * q1.real() - dot(q0.imag(), q1.imag()),
        q0.real() * q1.imag() +
        q0.imag() * q1.real() +
        cross(q0.imag(), q1.imag())
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
constexpr std::enable_if_t<
                      is_quat_param<U>::value,
                      quat<decltype(T() + U())>> operator+(
                                    const quat<T>& q0, const U& q1)
{
    return {
        q0.real() + q1,
        q0.imag()
    };
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
    return {
        q0.real() - q1,
        q0.imag()
    };
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
    return {
        q0.real() * q1,
        q0.imag() * q1
    };
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
    return {
        q0.real() / q1,
        q0.imag() / q1
    };
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
    return {
        q0 + q1.real(),
             q1.imag()
    };
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
    return {
        q0 - q1.real(),
           - q1.imag()
    };
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
    return {
        q0 * q1.real(),
        q0 * q1.imag()
    };
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
    return q0.real() == q1.real() && (q0.imag() == q1.imag()).all();
}

/**
 * @brief Compare `operator!=`.
 */
template <typename T, typename U>
__attribute__((always_inline))
constexpr bool operator!=(const quat<T>& q0, const quat<U>& q1)
{
    return q0.real() != q1.real() || (q0.imag() != q1.imag()).any();
}

/**@}*/

/**
 * @name Comparison operators (quat/num)
 */
/**@{*/

/**
 * @brief Compare `operator==`.
 */
template <typename T, typename U>
__attribute__((always_inline))
constexpr std::enable_if_t<
                      is_quat_param<U>::value, bool> operator==(
                                        const quat<T>& q0, const U& q1)
{
    return q0.real() == q1 && (q0.imag() == T()).all();
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
    return q0.real() != q1 || (q0.imag() != T()).any();
}

/**@}*/

/**
 * @name Comparison operators (num/quat)
 */
/**@{*/

/**
 * @brief Compare `operator==`.
 */
template <typename T, typename U>
__attribute__((always_inline))
constexpr std::enable_if_t<
                      is_quat_param<T>::value, bool> operator==(
                                        const T& q0, const quat<U>& q1)
{
    return q1 == q0;
}

/**
 * @brief Compare `operator!=`.
 */
template <typename T, typename U>
__attribute__((always_inline))
constexpr std::enable_if_t<
                      is_quat_param<U>::value, bool> operator!=(
                                        const T& q0, const quat<U>& q1)
{
    return q1 != q0;
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
    return dot(static_cast<multi<T, 4>>(q0),
               static_cast<multi<U, 4>>(q1));
}

/**@}*/

// TODO exp

// TODO log

/**
 * @name Float checks (quat)
 */
/**@{*/

/**
 * @brief Any Inf?
 */
template <typename T>
__attribute__((always_inline))
inline bool isinf(const quat<T>& q)
{
    return pr::isinf(q.real()) || pr::isinf(q.imag()).any();
}

/**
 * @brief Any NaN?
 */
template <typename T>
__attribute__((always_inline))
inline bool isnan(const quat<T>& q)
{
    return pr::isnan(q.real()) || pr::isnan(q.imag()).any();
}

/**
 * @brief All finite?
 */
template <typename T>
__attribute__((always_inline))
inline bool isfinite(const quat<T>& q)
{
    return pr::isfinite(q.real()) && pr::isfinite(q.imag()).all();
}

/**
 * @brief All normal?
 */
template <typename T>
__attribute__((always_inline))
inline bool isnormal(const quat<T>& q)
{
    return pr::isnormal(q.real()) && pr::isnormal(q.imag()).all();
}

/**@}*/

/**@}*/

} // namespace pr

#endif // #ifndef PREFORM_QUAT_HPP
