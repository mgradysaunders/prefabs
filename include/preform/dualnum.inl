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
// A ruby script generates this file, DO NOT EDIT

namespace pre {

/**
 * @addtogroup dualnum
 */
/**@{*/

/**
 * @name Math (dualnum)
 *
 * @f[
 *      f(a + \varepsilon b) = 
 *      f(a) + \varepsilon f'(a) b
 * @f]
 */
/**@{*/

/**
 * @brief Dual number implementation of `pre::exp()`.
 */
template <typename T>
__attribute__((always_inline))
inline dualnum<T> exp(const dualnum<T>& x)
{
    return {
        pre::exp(x.real()),
        pre::exp(x.real()) * x.dual()
    };
}

/**
 * @brief Dual number implementation of `pre::log()`.
 */
template <typename T>
__attribute__((always_inline))
inline dualnum<T> log(const dualnum<T>& x)
{
    return {
        pre::log(x.real()),
        x.dual() / x.real()
    };
}

/**
 * @brief Dual number implementation of `pre::exp2()`.
 */
template <typename T>
__attribute__((always_inline))
inline dualnum<T> exp2(const dualnum<T>& x)
{
    return {
        pre::exp2(x.real()),
        pre::numeric_constants<T>::M_ln2() * pre::exp2(x.real()) * x.dual()
    };
}

/**
 * @brief Dual number implementation of `pre::log2()`.
 */
template <typename T>
__attribute__((always_inline))
inline dualnum<T> log2(const dualnum<T>& x)
{
    return {
        pre::log2(x.real()),
        x.dual() / (pre::numeric_constants<T>::M_ln2() * x.real())
    };
}

/**
 * @brief Dual number implementation of `pre::log10()`.
 */
template <typename T>
__attribute__((always_inline))
inline dualnum<T> log10(const dualnum<T>& x)
{
    return {
        pre::log10(x.real()),
        x.dual() / (pre::numeric_constants<T>::M_ln10() * x.real())
    };
}

/**
 * @brief Dual number implementation of `pre::expm1()`.
 */
template <typename T>
__attribute__((always_inline))
inline dualnum<T> expm1(const dualnum<T>& x)
{
    return {
        pre::expm1(x.real()),
        pre::exp(x.real()) * x.dual()
    };
}

/**
 * @brief Dual number implementation of `pre::log1p()`.
 */
template <typename T>
__attribute__((always_inline))
inline dualnum<T> log1p(const dualnum<T>& x)
{
    return {
        pre::log1p(x.real()),
        x.dual() / (T(1) + x.real())
    };
}

/**
 * @brief Dual number implementation of `pre::sqrt()`.
 */
template <typename T>
__attribute__((always_inline))
inline dualnum<T> sqrt(const dualnum<T>& x)
{
    return {
        pre::sqrt(x.real()),
        x.dual() / (T(2) * pre::sqrt(x.real()))
    };
}

/**
 * @brief Dual number implementation of `pre::cbrt()`.
 */
template <typename T>
__attribute__((always_inline))
inline dualnum<T> cbrt(const dualnum<T>& x)
{
    return {
        pre::cbrt(x.real()),
        x.dual() / (T(3) * pre::nthpow(pre::cbrt(x.real()), 2))
    };
}

/**
 * @brief Dual number implementation of `pre::erf()`.
 */
template <typename T>
__attribute__((always_inline))
inline dualnum<T> erf(const dualnum<T>& x)
{
    return {
        pre::erf(x.real()),
        pre::numeric_constants<T>::M_2_sqrtpi() * pre::exp(-pre::nthpow(x.real(), 2)) * x.dual()
    };
}

/**
 * @brief Dual number implementation of `pre::erfc()`.
 */
template <typename T>
__attribute__((always_inline))
inline dualnum<T> erfc(const dualnum<T>& x)
{
    return {
        pre::erfc(x.real()),
        -pre::numeric_constants<T>::M_2_sqrtpi() * pre::exp(-pre::nthpow(x.real(), 2)) * x.dual()
    };
}

/**
 * @brief Dual number implementation of `pre::sin()`.
 */
template <typename T>
__attribute__((always_inline))
inline dualnum<T> sin(const dualnum<T>& x)
{
    return {
        pre::sin(x.real()),
        pre::cos(x.real()) * x.dual()
    };
}

/**
 * @brief Dual number implementation of `pre::cos()`.
 */
template <typename T>
__attribute__((always_inline))
inline dualnum<T> cos(const dualnum<T>& x)
{
    return {
        pre::cos(x.real()),
        -pre::sin(x.real()) * x.dual()
    };
}

/**
 * @brief Dual number implementation of `pre::tan()`.
 */
template <typename T>
__attribute__((always_inline))
inline dualnum<T> tan(const dualnum<T>& x)
{
    return {
        pre::tan(x.real()),
        x.dual() / pre::nthpow(pre::cos(x.real()), 2)
    };
}

/**
 * @brief Dual number implementation of `pre::asin()`.
 */
template <typename T>
__attribute__((always_inline))
inline dualnum<T> asin(const dualnum<T>& x)
{
    return {
        pre::asin(x.real()),
        x.dual() / pre::sqrt(T(1) - pre::nthpow(x.real(), 2))
    };
}

/**
 * @brief Dual number implementation of `pre::acos()`.
 */
template <typename T>
__attribute__((always_inline))
inline dualnum<T> acos(const dualnum<T>& x)
{
    return {
        pre::acos(x.real()),
        x.dual() / -pre::sqrt(T(1) - pre::nthpow(x.real(), 2))
    };
}

/**
 * @brief Dual number implementation of `pre::atan()`.
 */
template <typename T>
__attribute__((always_inline))
inline dualnum<T> atan(const dualnum<T>& x)
{
    return {
        pre::atan(x.real()),
        x.dual() / (T(1) + pre::nthpow(x.real(), 2))
    };
}

/**
 * @brief Dual number implementation of `pre::sinh()`.
 */
template <typename T>
__attribute__((always_inline))
inline dualnum<T> sinh(const dualnum<T>& x)
{
    return {
        pre::sinh(x.real()),
        pre::cosh(x.real()) * x.dual()
    };
}

/**
 * @brief Dual number implementation of `pre::cosh()`.
 */
template <typename T>
__attribute__((always_inline))
inline dualnum<T> cosh(const dualnum<T>& x)
{
    return {
        pre::cosh(x.real()),
        pre::sinh(x.real()) * x.dual()
    };
}

/**
 * @brief Dual number implementation of `pre::tanh()`.
 */
template <typename T>
__attribute__((always_inline))
inline dualnum<T> tanh(const dualnum<T>& x)
{
    return {
        pre::tanh(x.real()),
        x.dual() / pre::nthpow(pre::cosh(x.real()), 2)
    };
}

/**
 * @brief Dual number implementation of `pre::asinh()`.
 */
template <typename T>
__attribute__((always_inline))
inline dualnum<T> asinh(const dualnum<T>& x)
{
    return {
        pre::asinh(x.real()),
        x.dual() / pre::sqrt(pre::nthpow(x.real(), 2) + T(1))
    };
}

/**
 * @brief Dual number implementation of `pre::acosh()`.
 */
template <typename T>
__attribute__((always_inline))
inline dualnum<T> acosh(const dualnum<T>& x)
{
    return {
        pre::acosh(x.real()),
        x.dual() / pre::sqrt(pre::nthpow(x.real(), 2) - T(1))
    };
}

/**
 * @brief Dual number implementation of `pre::atanh()`.
 */
template <typename T>
__attribute__((always_inline))
inline dualnum<T> atanh(const dualnum<T>& x)
{
    return {
        pre::atanh(x.real()),
        x.dual() / (T(1) - pre::nthpow(x.real(), 2))
    };
}

/**@}*/

/**
 * @name Reciprocal trigonometric functions (dualnum)
 */
/**@{*/

/**
 * @brief Reciprocal of `pre::sin()`.
 */
template <typename T>
__attribute__((always_inline))
inline dualnum<T> csc(const dualnum<T>& x)
{
    return T(1) / pre::sin(x);
}

/**
 * @brief Reciprocal of `pre::cos()`.
 */
template <typename T>
__attribute__((always_inline))
inline dualnum<T> sec(const dualnum<T>& x)
{
    return T(1) / pre::cos(x);
}

/**
 * @brief Reciprocal of `pre::tan()`.
 */
template <typename T>
__attribute__((always_inline))
inline dualnum<T> cot(const dualnum<T>& x)
{
    return T(1) / pre::tan(x);
}

/**
 * @brief Reciprocal of `pre::sinh()`.
 */
template <typename T>
__attribute__((always_inline))
inline dualnum<T> csch(const dualnum<T>& x)
{
    return T(1) / pre::sinh(x);
}

/**
 * @brief Reciprocal of `pre::cosh()`.
 */
template <typename T>
__attribute__((always_inline))
inline dualnum<T> sech(const dualnum<T>& x)
{
    return T(1) / pre::cosh(x);
}

/**
 * @brief Reciprocal of `pre::tanh()`.
 */
template <typename T>
__attribute__((always_inline))
inline dualnum<T> coth(const dualnum<T>& x)
{
    return T(1) / pre::tanh(x);
}

/**
 * @brief Inverse of `pre::csc()`.
 */
template <typename T>
__attribute__((always_inline))
inline dualnum<T> acsc(const dualnum<T>& x)
{
    return pre::asin(T(1) / x);
}

/**
 * @brief Inverse of `pre::sec()`.
 */
template <typename T>
__attribute__((always_inline))
inline dualnum<T> asec(const dualnum<T>& x)
{
    return pre::acos(T(1) / x);
}

/**
 * @brief Inverse of `pre::cot()`.
 */
template <typename T>
__attribute__((always_inline))
inline dualnum<T> acot(const dualnum<T>& x)
{
    return pre::atan(T(1) / x);
}

/**
 * @brief Inverse of `pre::csch()`.
 */
template <typename T>
__attribute__((always_inline))
inline dualnum<T> acsch(const dualnum<T>& x)
{
    return pre::asinh(T(1) / x);
}

/**
 * @brief Inverse of `pre::sech()`.
 */
template <typename T>
__attribute__((always_inline))
inline dualnum<T> asech(const dualnum<T>& x)
{
    return pre::acosh(T(1) / x);
}

/**
 * @brief Inverse of `pre::coth()`.
 */
template <typename T>
__attribute__((always_inline))
inline dualnum<T> acoth(const dualnum<T>& x)
{
    return pre::atanh(T(1) / x);
}

/**@}*/

/**@}*/

} // namespace pre

