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
#if !(__cplusplus >= 201402L)
#error "preform/misc_int.hpp requires >=C++14"
#endif // #if !(__cplusplus >= 201402L)
#endif // #if !DOXYGEN
#pragma once
#ifndef PREFORM_MISC_INT_HPP
#define PREFORM_MISC_INT_HPP

// for std::enable_if_t, std::is_integral
#include <type_traits>

namespace pr {

/**
 * @defgroup misc_int Miscellaneous int
 *
 * `<preform/misc_int.hpp>`
 *
 * __C++ version__: >=C++14
 */
/**@{*/

/**
 * @brief Greatest common divisor.
 */
template <typename T>
constexpr std::enable_if_t<
          std::is_integral<T>::value, T> gcd(T a, T b)
{
    T r = 0;
    while (a) {
        r = b % a;
        b = a;
        a = r;
    }
    return b;
}

/**
 * @brief Greatest common divisor with Bezout coefficients.
 */
template <typename T>
constexpr std::enable_if_t<
          std::is_integral<T>::value, T> gcd_bezout(T a, T b, T* x, T* y)
{
    T s1 = 0, s0 = 1;
    T t1 = 1, t0 = 0;
    T r1 = b, r0 = a;
    while (r1 != 0) {
        T q = r0 / r1;
        T k;
        k = r1, r1 = r0 - q * r1, r0 = k;
        k = s1, s1 = s0 - q * s1, s0 = k;
        k = t1, t1 = t0 - q * t1, t0 = k;
    }
    if (x) *x = s0;
    if (y) *y = t0;
    return r0;
}

/**
 * @brief Least common multiple.
 */
template <typename T>
constexpr std::enable_if_t<
          std::is_integral<T>::value, T> lcm(T a, T b)
{
    if (!a || !b)
        return 0;
    else
        return a * b / gcd(a, b);
}

/**
 * @brief Is odd?
 */
template <typename T>
constexpr std::enable_if_t<
          std::is_integral<T>::value, bool> isodd(T n)
{
    return (n & 1) != 0;
}

/**
 * @brief Is even?
 */
template <typename T>
constexpr std::enable_if_t<
          std::is_integral<T>::value, bool> iseven(T n)
{
    return (n & 1) == 0;
}

/**
 * @brief Is power of 2?
 */
template <typename T>
constexpr std::enable_if_t<
          std::is_integral<T>::value, bool> ispow2(T n)
{
    return n > 0 && (n & (n - 1)) == 0;
}

/**
 * @brief Round up to power of 2.
 */
template <typename T>
constexpr std::enable_if_t<
          std::is_integral<T>::value, T> roundpow2(T n)
{
    if (n <= 0) {
        return 1;
    }
    else {
        n--;
        T p = 1;
        while (n) {
            n >>= 1;
            p <<= 1;
        }
        return p;
    }
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
constexpr std::enable_if_t<
          std::is_integral<T>::value, T> first1(T n)
{
    if (n == 0) {
        return 0; // Error?
    }

#if __GNUC__
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
        // Error.
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
constexpr std::enable_if_t<
          std::is_integral<T>::value, T> rotl(T val, unsigned rot)
{
    return (val << rot) | (val >> ((-rot) & (sizeof(T) * 8 - 1)));
}

/**
 * @brief Cyclical bit rotate right.
 */
template <typename T>
constexpr std::enable_if_t<
          std::is_integral<T>::value, T> rotr(T val, unsigned rot)
{
    return (val >> rot) | (val << ((-rot) & (sizeof(T) * 8 - 1)));
}

/**
 * @brief Bit swap.
 */
template <typename T>
constexpr std::enable_if_t<
          std::is_integral<T>::value, T> bit_swap(T val, int pos0, int pos1)
{
    T mask = ((val >> pos0) ^ (val >> pos1)) & 1;
    return
        ((mask << pos0) |
         (mask << pos1)) ^ val;
}

#if !DOXYGEN
template <typename>
struct bit_reverse_impl;

// Reverse 8-bit unsigned integer.
template <>
struct bit_reverse_impl<std::uint8_t>
{
    __attribute__((always_inline))
    static constexpr std::uint8_t reverse(std::uint8_t val)
    {
        std::uint8_t mask[3] = {
            0xaaU,
            0xccU,
            0xf0U
        };
        for (int k = 0; k < 3; k++) {
            val = ((val &  mask[k]) >> (1 << k)) |
                  ((val & ~mask[k]) << (1 << k));
        }
        return val;
    }
};

// Reverse 16-bit unsigned integer.
template <>
struct bit_reverse_impl<std::uint16_t>
{
    __attribute__((always_inline))
    static constexpr std::uint16_t reverse(std::uint16_t val)
    {
        std::uint16_t mask[4] = {
            0xaaaaU,
            0xccccU,
            0xf0f0U,
            0xff00U
        };
#if __GNUC__
        val = __builtin_bswap16(val);
        for (int k = 0; k < 3; k++) {
#else
        for (int k = 0; k < 4; k++) {
#endif // #if __GNUC__
            val = ((val &  mask[k]) >> (1 << k)) |
                  ((val & ~mask[k]) << (1 << k));
        }
        return val;
    }
};

// Reverse 32-bit unsigned integer.
template <>
struct bit_reverse_impl<std::uint32_t>
{
    __attribute__((always_inline))
    static constexpr std::uint32_t reverse(std::uint32_t val)
    {
        std::uint32_t mask[5] = {
            0xaaaaaaaaUL,
            0xccccccccUL,
            0xf0f0f0f0UL,
            0xff00ff00UL,
            0xffff0000UL
        };
#if __GNUC__
        val = __builtin_bswap32(val);
        for (int k = 0; k < 3; k++) {
#else
        for (int k = 0; k < 5; k++) {
#endif // #if __GNUC__
            val = ((val &  mask[k]) >> (1 << k)) |
                  ((val & ~mask[k]) << (1 << k));
        }
        return val;
    }
};

// Reverse 64-bit unsigned integer.
template <>
struct bit_reverse_impl<std::uint64_t>
{
    __attribute__((always_inline))
    static constexpr std::uint64_t reverse(std::uint64_t val)
    {
        std::uint64_t mask[6] = {
            0xaaaaaaaaaaaaaaaaULL,
            0xccccccccccccccccULL,
            0xf0f0f0f0f0f0f0f0ULL,
            0xff00ff00ff00ff00ULL,
            0xffff0000ffff0000ULL,
            0xffffffff00000000ULL
        };
#if __GNUC__
        val = __builtin_bswap64(val);
        for (int k = 0; k < 3; k++) {
#else
        for (int k = 0; k < 6; k++) {
#endif // #if __GNUC__
            val = ((val &  mask[k]) >> (1 << k)) |
                  ((val & ~mask[k]) << (1 << k));
        }
        return val;
    }
};
#endif // #if !DOXYGEN

/**
 * @brief Bit reverse.
 */
template <typename T>
constexpr std::enable_if_t<
          std::is_integral<T>::value, T> bit_reverse(T val)
{
    return bit_reverse_impl<std::make_unsigned_t<T>>::reverse(val);
}

#if !DOXYGEN
template <typename>
struct bit_interleave_zero_impl;

// Interleave 8-bit unsigned integer with zero.
template <>
struct bit_interleave_zero_impl<std::uint8_t>
{
    __attribute__((always_inline))
    static constexpr std::uint8_t interleave(std::uint8_t val)
    {
        std::uint8_t mask[3] = {
            0xaaU,
            0xccU,
            0xf0U
        };
        for (int k = 2; k >= 0; k--) {
            val = (val ^ (val << (1 << k))) & ~mask[k];
        }
        return val;
    }
};

// Interleave 16-bit unsigned integer with zero.
template <>
struct bit_interleave_zero_impl<std::uint16_t>
{
    __attribute__((always_inline))
    static constexpr std::uint16_t interleave(std::uint16_t val)
    {
        std::uint16_t mask[4] = {
            0xaaaaU,
            0xccccU,
            0xf0f0U,
            0xff00U
        };
        for (int k = 3; k >= 0; k--) {
            val = (val ^ (val << (1 << k))) & ~mask[k];
        }
        return val;
    }
};

// Interleave 32-bit unsigned integer with zero.
template <>
struct bit_interleave_zero_impl<std::uint32_t>
{
    __attribute__((always_inline))
    static constexpr std::uint32_t interleave(std::uint32_t val)
    {
        std::uint32_t mask[5] = {
            0xaaaaaaaaUL,
            0xccccccccUL,
            0xf0f0f0f0UL,
            0xff00ff00UL,
            0xffff0000UL
        };
        for (int k = 4; k >= 0; k--) {
            val = (val ^ (val << (1 << k))) & ~mask[k];
        }
        return val;
    }
};

// Interleave 64-bit unsigned integer with zero.
template <>
struct bit_interleave_zero_impl<std::uint64_t>
{
    __attribute__((always_inline))
    static constexpr std::uint64_t interleave(std::uint64_t val)
    {
        std::uint64_t mask[6] = {
            0xaaaaaaaaaaaaaaaaULL,
            0xccccccccccccccccULL,
            0xf0f0f0f0f0f0f0f0ULL,
            0xff00ff00ff00ff00ULL,
            0xffff0000ffff0000ULL
        };
        for (int k = 5; k >= 0; k--) {
            val = (val ^ (val << (1 << k))) & ~mask[k];
        }
        return val;
    }
};
#endif // #if !DOXYGEN

/**
 * @brief Bit interleave.
 *
 * Form an integer @f$ z @f$ from the
 * bits of @f$ x @f$ and @f$ y @f$ such that
 * - bit @f$ k @f$ of @f$ x @f$ is bit @f$ 2k @f$ of @f$ z @f$,
 * - bit @f$ k @f$ of @f$ y @f$ is bit @f$ 2k + 1 @f$ of @f$ z @f$.
 *
 * @note
 * If the input integer type has @f$ 2^n @f$ bits, the top
 * @f$ 2^{n - 1} @f$ bits of the input integers do not appear in
 * the output.
 */
template <typename T>
constexpr std::enable_if_t<
          std::is_integral<T>::value, T> bit_interleave(T val0, T val1)
{
    val0 = bit_interleave_zero_impl<std::make_unsigned_t<T>>::interleave(val0);
    val1 = bit_interleave_zero_impl<std::make_unsigned_t<T>>::interleave(val1);
    return val0 | (val1 << 1);
}

/**
 * @brief Clamp integer in range.
 *
 * - If @f$ n > 0 @f$, clamp @f$ k @f$ to @f$ [0, n) @f$.
 * - If @f$ n < 0 @f$, clamp @f$ k @f$ to @f$ (n, 0] @f$.
 */
template <typename T>
constexpr std::enable_if_t<
          std::is_integral<T>::value, T> clamp(T k, T n)
{
    if (n < T(0)) {
        return -clamp(-k, -n);
    }
    else {
        return
            std::max<T>(0,
            std::min<T>(k, n - 1));
    }
}

/**
 * @brief Wrap integer in range.
 *
 * - If @f$ n > 0 @f$, wrap @f$ k @f$ to @f$ [0, n) @f$.
 * - If @f$ n < 0 @f$, wrap @f$ k @f$ to @f$ (n, 0] @f$.
 *
 * @f[
 *      \begin{aligned}
 *          \operatorname{repeat}(k; n) &= \operatorname{repeat}(k + mn; n)
 *      \\ -\operatorname{repeat}(k; n) &= \operatorname{repeat}(-k; -n)
 *      \end{aligned}
 * @f]
 *
 * @note
 * For @f$ k > 0 @f$, @f$ n > 0 @f$, this operation is
 * equivalent to @f$ k \% n @f$.
 */
template <typename T>
constexpr std::enable_if_t<
          std::is_integral<T>::value, T> repeat(T k, T n)
{
#if (__cplusplus >= 201703L)
    if constexpr
#else
    if
#endif // #if (__cplusplus >= 201703L)
       (std::is_unsigned<T>::value) {
        return k % n;
    }
    else {
        if (n < T(0)) {
            return -repeat(-k, -n);
        }
        else {
            if (k >= T(0)) {
                return k % n;
            }
            else {
                k = n + k % n;
                if (k == n) {
                    k = T(0);
                }
                return k;
            }
        }
    }
}

/**
 * @brief Wrap integer in range, mirror with each repeat.
 *
 * - If @f$ n > 0 @f$, wrap @f$ k @f$ to @f$ [0, n) @f$.
 * - If @f$ n < 0 @f$, wrap @f$ k @f$ to @f$ (n, 0] @f$.
 *
 * @f[
 *      \begin{aligned}
 *          \operatorname{mirror}(k; n) &=
 *          \operatorname{mirror}(k + mn; n),
 *          \, m = 2\ell
 *      \\  \operatorname{mirror}(k; n) &= n - 1 -
 *          \operatorname{mirror}(k + mn; n),
 *          \, m = 2\ell + 1
 *      \\ -\operatorname{mirror}(k; n) &= \operatorname{mirror}(-k; -n)
 *      \end{aligned}
 * @f]
 */
template <typename T>
constexpr std::enable_if_t<
          std::is_integral<T>::value, T> mirror(T k, T n)
{
    if (n < T(0)) {
        return -mirror(-k, -n);
    }
    else {
        T r = k % n;
        T q = k / n;
        if (r < T(0)) {
            r += n;
            q++;
        }
        if (q & T(1)) {
            r = n - r - T(1);
        }
        return r;
    }
}

/**
 * @brief Linear congruential generator seek.
 *
 * @param[in] x
 * State.
 *
 * @param[in] a
 * State multiplier.
 *
 * @param[in] b
 * State increment.
 *
 * @param[in] n
 * Step count.
 *
 * @note
 * If @f$ a \% 4 = 1 @f$ and @f$ b @f$ is odd, such that
 * the generator has a full period of @f$ 2^{m} @f$ where @f$ m @f$ is
 * the bit depth of the unsigned integral type, then seeking by negative
 * numbers works as expected.
 */
template <typename T>
constexpr std::enable_if_t<
          std::is_unsigned<T>::value, T> lcg_seek(T x, T a, T b, T n)
{
    // Fast power by squaring.
    T an = 1;
    T bn = 0;
    while (n > 0) {
        if (n & 1) {
            an *= a;
            bn *= a;
            bn += b;
        }
        b *= a + 1;
        a *= a;
        n >>= 1;
    }
    x *= an;
    x += bn;
    return x;
}

/**
 * @brief Linear congruential generator distance.
 *
 * @param[in] x
 * State.
 *
 * @param[in] a
 * State multiplier.
 *
 * @param[in] b
 * State increment.
 *
 * @param[in] xn
 * Target state.
 *
 * @throw std::invalid_argument
 * Unless @f$ a \% 4 = 1 @f$ and @f$ b @f$ is odd, such that
 * the generator has a full period of @f$ 2^{m} @f$ where @f$ m @f$ is
 * the bit depth of the unsigned integral type.
 */
template <typename T>
constexpr std::enable_if_t<
          std::is_unsigned<T>::value, T> lcg_distance(T x, T a, T b, T xn)
{
    // Ensure full period.
    if (!((a & 3) == 1) ||
        !((b & 1) == 1)) {
        throw std::invalid_argument(__PRETTY_FUNCTION__);
    }

    T p = 1;
    T r = 0;
    while (x != xn) {
        if ((x & p) != (xn & p)) {
            x *= a;
            x += b;
            r |= p;
        }
        p <<= 1;
        b *= a + 1;
        a *= a;
    }
    return r;
}

/**
 * @brief Cantor pairing function.
 *
 * @f[
 *      \pi(x, y) =
 *      \frac{1}{2}(x + y)(x + y + 1) + y
 * @f]
 */
template <typename T>
constexpr std::enable_if_t<
          std::is_integral<T>::value, T> cantor(T x, T y)
{
    return ((x + y) * (x + y + 1)) / 2 + y;
}

/**
 * @brief Cantor tuple pairing function.
 *
 * @f[
 *      \pi(x, y, z, \ldots) = \pi(\pi(x, y), z, \ldots)
 * @f]
 */
template <typename T, typename... Ts>
constexpr std::enable_if_t<
          std::is_integral<T>::value, T> cantor(T x, T y, T z, Ts... ws)
{
    return cantor(cantor(x, y), z, ws...);
}

/**
 * @brief Sized int type selector.
 *
 * Defines member type `type` to be the smallest signed integral
 * type occupying at least `Nbytes`, or `void` if no such type exists.
 *
 * @tparam Nbytes
 * Least number of bytes.
 */
template <std::size_t Nbytes>
struct sized_int;

/**
 * @brief Sized unsigned int type selector.
 *
 * Defines member type `type` to be the smallest unsigned integral
 * type occupying at least `Nbytes`, or `void` if no such type exists.
 *
 * @tparam Nbytes
 * Least number of bytes.
 */
template <std::size_t Nbytes>
struct sized_uint;

#if !DOXYGEN

// Implementation.
template <std::size_t Nbytes>
struct sized_int {
    typedef std::conditional_t<
                sizeof(std::int_least8_t) >= Nbytes,
                std::int_least8_t,
                std::conditional_t<
                    sizeof(std::int_least16_t) >= Nbytes,
                    std::int_least16_t,
                    std::conditional_t<
                        sizeof(std::int_least32_t) >= Nbytes,
                        std::int_least32_t,
                        std::conditional_t<
                            sizeof(std::int_least64_t) >= Nbytes,
                            std::int_least64_t,
                            void>>>> type;
};

// Implementation.
template <std::size_t Nbytes>
struct sized_uint {
    typedef std::conditional_t<
                sizeof(std::uint_least8_t) >= Nbytes,
                std::uint_least8_t,
                std::conditional_t<
                    sizeof(std::uint_least16_t) >= Nbytes,
                    std::uint_least16_t,
                    std::conditional_t<
                        sizeof(std::uint_least32_t) >= Nbytes,
                        std::uint_least32_t,
                        std::conditional_t<
                            sizeof(std::uint_least64_t) >= Nbytes,
                            std::uint_least64_t,
                            void>>>> type;
};

#endif // #if !DOXYGEN

/**
 * @brief Bayer index for ordered dithering.
 *
 * Let @f$ \mathbf{M} \in \mathbb{R}^{n,n} @f$ denote the Bayer index
 * matrix for dimension @f$ n = 2^B @f$, where
 * @f[
 *      M_{i,j} =
 *          \frac{1}{n^2}
 *          \operatorname{reverse}_{2B}(
 *          \operatorname{interleave}(\operatorname{xor}(j, i), i))
 * @f]
 * This routine calculates @f$ n^2 M_{i,j} @f$, which is always an
 * integer.
 */
template <std::size_t B, typename T>
constexpr std::enable_if_t<
          std::is_integral<T>::value, T> bayer_index(T i, T j)
{
    static_assert(B > 0, "B must be greater than 0");
    static_assert(sizeof(T) * 8 > B * 2, "T does not have enough bits");
    i = i & ((1 << B) - 1);
    j = j & ((1 << B) - 1);
    return
        bit_reverse<T>(
        bit_interleave<T>(j ^ i, i)) >> (sizeof(T) * 8 - B * 2);
}

/**@}*/

} // namespace pr

#endif // #ifndef PREFORM_MISC_INT_HPP
