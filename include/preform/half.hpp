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
#error "preform/half.hpp requires >=C++14"
#endif // #if !(__cplusplus >= 201402L)
#endif // #if !DOXYGEN
#pragma once
#ifndef PREFORM_HALF_HPP
#define PREFORM_HALF_HPP

// for std::uint16_t, std::uint32_t
#include <cstdint>

// for std::memcpy
#include <cstring>

// for std::numeric_limits
#include <limits>

namespace pr {

/**
 * @defgroup half Half-precision float
 *
 * `<preform/half.hpp>`
 *
 * __C++ version__: >=C++14
 */
/**@{*/

/**
 * @brief Half-precision float.
 *
 * Half-precision float, an adaptation of [the ILM
 * implementation][1] by Florian Kains and Rod Bogart. Notably, the
 * implementation here requires neither pre-calculations nor lookup tables.
 * [1]: https://github.com/openexr/openexr/tree/develop/IlmBase/half
 */
class half
{
public:

    /**
     * @name Constructors
     */
    /**@{*/

    /**
     * @brief Default constructor.
     */
    half() noexcept = default;

    /**
     * @brief Constructor.
     */
    half(float f) noexcept;

    /**@}*/

public:

    /**
     * @name Cast operators
     */
    /**@{*/

    /**
     * @brief Cast to float.
     */
    operator float() const noexcept;

    /**
     * @brief Cast to bit pattern.
     */
    explicit operator std::uint16_t&() noexcept
    {
        return b_;
    }

    /**
     * @brief Cast to bit pattern.
     */
    explicit operator const std::uint16_t&() const noexcept
    {
        return b_;
    }

    /**@}*/

#if 0
public:

    /**
     * @name Arithmetic operators
     */
    /**@{*/

    /**
     * @brief In-place addition.
     */
    half& operator+=(half h) noexcept
    {
        return *this = float(*this) + float(h);
    }

    /**
     * @brief In-place subtraction.
     */
    half& operator-=(half h) noexcept
    {
        return *this = float(*this) - float(h);
    }

    /**
     * @brief In-place multiplication.
     */
    half& operator*=(half h) noexcept
    {
        return *this = float(*this) * float(h);
    }

    /**
     * @brief In-place division.
     */
    half& operator/=(half h) noexcept
    {
        return *this = float(*this) / float(h);
    }

    /**@}*/
#endif

public:

    /**
     * @name Miscellaneous
     */
    /**@{*/

    /**
     * @brief Absolute value.
     */
    half abs() const noexcept
    {
        half h = *this;
        h.b_ = h.b_ & ~0x8000;
        return h;
    }

    /**
     * @brief Is negative? (possibly `-0.0`)
     */
    bool signbit() const noexcept
    {
        return (b_ & 0x8000);
    }

    /**
     * @brief Is infinite?
     */
    bool isinf() const noexcept
    {
        return (b_ & 0x7FFF) == 0x7C00;
    }

    /**
     * @brief Is not-a-number?
     */
    bool isnan() const noexcept
    {
        return (b_ & 0x7FFF) != 0x7C00 && (b_ & 0x7C00) == 0x7C00;
    }

    /**
     * @brief Is neither infinite nor not-a-number?
     */
    bool isfinite() const noexcept
    {
        return (b_ & 0x7C00) != 0x7C00;
    }

    /**
     * @brief Is neither infinite nor not-a-number nor subnormal?
     */
    bool isnormal() const noexcept
    {
        return (b_ & 0x7C00) != 0x7C00 && (b_ & 0x7C00) != 0;
    }

    /**@}*/

private:

    /**
     * @brief Bit pattern.
     */
    std::uint16_t b_ = 0;

    // Sanity check.
    static_assert(
        std::numeric_limits<float>::is_iec559,
        "half requires IEC-559/IEEE-754 single precision floats");
};

#if !DOXYGEN

inline half::half(float f) noexcept
{
    std::uint32_t u; std::memcpy(&u, &f, sizeof(f));
    if (f == 0) {
        b_ = u >> 16;
    }
    else {
        std::int32_t e = (u >> 23) & 0x01FF;
        std::int32_t s = (u >> 31) & 0x0001;
        std::int32_t m =  u & 0x007FFFFF;
        if (e > 0x0070 &&
            e < 0x008E) {
            e = (e - 0x0070) * 0x0400;
            b_ = e + ((m + ((m >> 13) & 1) + 0x0FFF) >> 13);
        }
        else if (e > 0x0170 &&
                 e < 0x018E) {
            e = (e - 0x0170) * 0x0400 + 0x8000;
            b_ = e + ((m + ((m >> 13) & 1) + 0x0FFF) >> 13);
        }
        else {
            s <<= 15;
            e &= 255;
            e -= 112;
            if (e < 1) {
                if (e < -10) {
                    // Zero.
                    b_ = s;
                }
                else {
                    // Subnormal.
                    m |= 0x00800000;
                    b_ = s | ((m +
                        ((1 << (13 - e)) +
                        ((m >> (14 - e)) & 1) - 1))
                            >> (14 - e));
                }
            }
            else if (e == 0x8F) {
                if (m == 0) {
                    // Inf.
                    b_ = s | 0x7C00;
                }
                else {
                    // NaN.
                    m >>= 13;
                    b_ = s | 0x7C00 | m | (m == 0);
                }
            }
            else {
                m += ((m >> 13) & 1) + 0x0FFF;
                if (m & 0x00800000) {
                    m = 0;
                    e++;
                }
                // Overflow?
                if (e > 30) {
                    // Inf.
                    b_ = s | 0x7C00;
                }
                else {
                    // Normal.
                    b_ = s | (e << 10) | (m >> 13);
                }
            }
        }
    }
}

inline half::operator float() const noexcept
{
    float f;
    std::uint32_t u;
    std::int32_t e = (b_ >> 10) & 0x001F;
    std::int32_t s = (b_ >> 15) & 0x0001;
    std::int32_t m =  b_ & 0x03FF;
    if (e == 0) {
        if (m == 0) {
            // Zero.
            u = s << 31;
        }
        else {
            // Subnormal.
            e += 112;
            while (!(m & 0x0400))
                e--, m <<= 1;
            e++;
            m &= ~0x0400;
            u = (s << 31) | (e << 23) | (m << 13);
        }
    }
    else if (e == 31) {
        if (m == 0) {
            // Inf.
            u = (s << 31) | 0x7F800000;
        }
        else {
            // NaN.
            u = (s << 31) | 0x7F800000 | (m << 13);
        }
    }
    else {
        // Normal.
        e += 112;
        u = (s << 31) | (e << 23) | (m << 13);
    }
    std::memcpy(&f, &u, sizeof(f));
    return f;
}

#endif // #if !DOXYGEN

/**@}*/

} // namespace pr

#endif // #ifndef PREFORM_HALF_HPP
