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
#ifndef PREFABS_BIT_HELPERS_HPP
#define PREFABS_BIT_HELPERS_HPP

// for std::enable_if, std::is_integral
#include <type_traits>

namespace pr {

/**
 * @defgroup bit_helpers Bit helpers
 */
/**@{*/

/**
 * @brief Enable type `U` if type `T` is integral.
 */
template <typename T, typename U = T>
using enable_int = std::enable_if<std::is_integral<T>::value, U>;

/**
 * @brief Greatest common divisor.
 */
template <typename T> 
constexpr typename enable_int<T>::type gcd(T p, T q)
{
    T r = T();
    p = (p >= 0) ? p : -p;
    q = (q >= 0) ? q : -q;
    while (p) {
        r = q % p;
        q = p;
        p = r;
    }
    return q;
}

/**
 * @brief Least common multiple.
 */
template <typename T> 
constexpr typename enable_int<T>::type lcm(T p, T q)
{
    p = (p >= 0) ? p : -p;
    q = (q >= 0) ? q : -q;
    if (!p || !q)
        return 0;
    else
        return p * q / gcd(p, q);
}

/**
 * @brief Is odd?
 */
template <typename T> 
constexpr typename enable_int<T, bool>::type isodd(T n)
{
    return (n & 1) != 0;
}

/**
 * @brief Is even?
 */
template <typename T>
constexpr typename enable_int<T, bool>::type iseven(T n)
{
    return (n & 1) == 0;
}

/**
 * @brief Is power of 2?
 */
template <typename T> 
constexpr typename enable_int<T, bool>::type ispow2(T n)
{
    return n > 0 && (n & (n - 1)) == 0;
}

/**
 * @brief Round up to nearest power of 2.
 */
template <typename T> 
constexpr typename enable_int<T>::type roundpow2(T n)
{
    if (n <= 0)
        return 1;

    n--;
    T p = 1;
    while (n) {
        n >>= 1;
        p <<= 1;
    }
    return p;
}

/**
 * @brief Find first bit equal to 1.
 *
 * @note This is equivalent to the binary logarithm if the
 * input integer is a positive power of 2.
 *
 * @note Uses [`__builtin_ctz`][1] when compiling with GCC.
 * [1]: https://gcc.gnu.org/onlinedocs/gcc/Other-Builtins.html
 */
template <typename T> 
constexpr typename enable_int<T>::type first1(T n)
{
#if __GNUC__
    if (n == 0)
        return 0; // error?

    if (sizeof(T) <= sizeof(int)) {
        return __builtin_ctz(n);
    }
    else if (sizeof(T) == sizeof(long)) {
        return __builtin_ctzl(n);
    }
    else if (sizeof(T) == sizeof(long long)) {
        return __builtin_ctzll(n);
    }
    else {
        // error
    }
#else
    T j = 0;
    while (!(n & 1)) {
        n >>= 1;
        j++;
    }
    return j;
#endif // #if __GNUC__
}

/**
 * @brief Cyclical bit rotate left.
 */
template <typename T> 
constexpr typename enable_int<T>::type rotl(T val, int rot)
{
    return (val << rot) | (val >> ((-rot) & (sizeof(T) * 8 - 1)));
}

/**
 * @brief Cyclical bit rotate right.
 */
template <typename T> 
constexpr typename enable_int<T>::type rotr(T val, int rot)
{
    return (val >> rot) | (val << ((-rot) & (sizeof(T) * 8 - 1)));
}

/**@}*/

} // namespace pr

#endif // #ifndef PREFABS_BIT_HELPERS_HPP
