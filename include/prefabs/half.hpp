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
#ifndef PREFABS_HALF_HPP
#define PREFABS_HALF_HPP

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
 * `<prefabs/half.hpp>`
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
    constexpr half() noexcept = default;

    /**
     * @brief Default copy constructor.
     */
    constexpr half(const half&) noexcept = default;

    /**
     * @brief Default move constructor.
     */
    constexpr half(half&&) noexcept = default;

    /**
     * @brief Constructor.
     */
    half(float f) noexcept;

    /**@}*/

public:

    /**
     * @name Assign operators
     */
    /**@{*/

    /**
     * @brief Default copy assign.
     */
    constexpr half& operator=(const half&) noexcept = default;

    /**
     * @brief Default move assign.
     */
    constexpr half& operator=(half&&) noexcept = default;

    /**
     * @brief Assign float.
     */
    half& operator=(float f) noexcept {
        return *this = half(f);
    }

    /**@}*/

public:

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

public:

    /**
     * @name Arithmetic operators
     */
    /**@{*/

    /**
     * @brief In-place half addition.
     */
    half& operator+=(half h)
    {
        return *this = float(*this) + float(h);
    }

    /**
     * @brief In-place half subtraction.
     */
    half& operator-=(half h)
    {
        return *this = float(*this) - float(h);
    }

    /**
     * @brief In-place half multiplication.
     */
    half& operator*=(half h)
    {
        return *this = float(*this) * float(h);
    }

    /**
     * @brief In-place half division.
     */
    half& operator/=(half h)
    {
        return *this = float(*this) / float(h);
    }

    /**@}*/

public:

    /**
     * @name Miscellaneous
     */
    /**@{*/

    /**
     * @brief Absolute value.
     */
    half abs() const
    {
        half h = *this;
        h.b_ = h.b_ & ~0x8000;
        return h;
    }

    /**
     * @brief Is negative? (possibly `-0.0`)
     */
    bool signbit() const
    {
        return (b_ & 0x8000);
    }

    /**
     * @brief Is infinite?
     */
    bool isinf() const
    {
        return (b_ & 0x7FFF) == 0x7C00;
    }

    /**
     * @brief Is not-a-number?
     */
    bool isnan() const
    {
        return (b_ & 0x7FFF) != 0x7C00 && (b_ & 0x7C00) == 0x7C00;
    }

    /**
     * @brief Is neither infinite nor not-a-number?
     */
    bool isfinite() const
    {
        return (b_ & 0x7C00) != 0x7C00;
    }

    /**
     * @brief Is neither infinite nor not-a-number nor subnormal?
     */
    bool isnormal() const
    {
        return (b_ & 0x7C00) != 0x7C00 && (b_ & 0x7C00) != 0;
    }

    /**@}*/

private:

    /**
     * @brief Bit pattern.
     */
    std::uint16_t b_ = 0;

    // sanity check
    static_assert(std::numeric_limits<float>::is_iec559,
        "Requires IEC-559/IEEE-754 single precision floats");
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
                // zero
                if (e < -10) {
                    b_ = s;
                }
                // subnormal
                else {
                    m |= 0x00800000;
                    b_ = s | ((m +
                        ((1 << (13 - e)) +
                        ((m >> (14 - e)) & 1) - 1))
                            >> (14 - e));
                }
            }
            else if (e == 0x8F) {
                // inf
                if (m == 0) {
                    b_ = s | 0x7C00;
                }
                // nan
                else {
                    m >>= 13;
                    b_ = s | 0x7C00 | m | (m == 0);
                }
            }
            else {
                m += ((m >> 13) & 1) + 0x0FFF;
                if (m & 0x0080000) {
                    m = 0;
                    e++;
                }
                // overflow
                if (e > 30) {
                    b_ = s | 0x7C00;
                }
                // normal
                else {
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
        // zero
        if (m == 0) {
            u = s << 31;
        }
        // subnormal
        else {
            e += 112;
            while (!(m & 0x0400))
                e--, m <<= 1;
            e++;
            m &= ~0x0400;
            u = (s << 31) | (e << 23) | (m << 13);
        }
    }
    else if (e == 31) {
        // inf
        if (m == 0) {
            u = (s << 31) | 0x7F800000;
        }
        // nan
        else {
            u = (s << 31) | 0x7F800000 | (m << 13);
        }
    }
    else {
        // normal
        e += 112;
        u = (s << 31) | (e << 23) | (m << 13);
    }
    std::memcpy(&f, &u, sizeof(f));
    return f;
}

#endif // #if !DOXYGEN

/**@}*/

} // namespace pr

#endif // #ifndef PREFABS_HALF_HPP
