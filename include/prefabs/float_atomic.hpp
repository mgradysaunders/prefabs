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
#if !(__cplusplus >= 201103L)
#error "prefabs/float_atomic.hpp requires >=C++11"
#endif // #if !(__cplusplus >= 201103L)
#endif // #if !DOXYGEN
#pragma once
#ifndef PREFABS_FLOAT_ATOMIC_HPP
#define PREFABS_FLOAT_ATOMIC_HPP

// for std::memcpy
#include <cstring>

// for std::atomic
#include <atomic>

// for std::is_floating_point, std::is_integral
#include <type_traits>

namespace pr {

/**
 * @defgroup float_atomic Float atomic
 *
 * `<prefabs/float_atomic.hpp>`
 *
 * __C++ version__: >=C++11
 */
/**@{*/

/**
 * @brief Floating point atomic.
 */
template <
    typename Tfloat, 
    typename Tfloat_bits
    >
class float_atomic
{
public:

    // Sanity check.
    static_assert(
        std::is_floating_point<Tfloat>::value,
        "Tfloat must be floating point");

    // Sanity check.
    static_assert(
        std::is_unsigned<Tfloat_bits>::value,
        "Tfloat_bits must be unsigned");

    // Sanity check.
    static_assert(
        sizeof(Tfloat) == sizeof(Tfloat_bits),
        "Tfloat and Tfloat_bits must be the same size");

    /**
     * @name Type conversion
     */
    /**@{*/

    /**
     * @brief Mem-copy `Tfloat` to `Tfloat_bits`.
     */
    static inline Tfloat_bits convert(Tfloat val) noexcept
    {
        Tfloat_bits res; std::memcpy(&res, &val, sizeof(val)); 
        return res;
    }

    /**
     * @brief Mem-copy `Tfloat_bits` to `Tfloat`.
     */
    static inline Tfloat convert(Tfloat_bits val) noexcept
    {
        Tfloat res; std::memcpy(&res, &val, sizeof(val)); 
        return res;
    }

    /**@}*/

public:

    /**
     * @name Constructors
     */
    /**@{*/

    /**
     * @brief Default constructor.
     */
    constexpr float_atomic() noexcept : bits_(0)
    {
    }

    /**
     * @brief Constructor.
     *
     * @param[in] bits
     * Unsigned integral bits.
     */
    constexpr float_atomic(Tfloat_bits bits) noexcept : bits_(bits)
    {
    }

    /**
     * @brief Constructor.
     *
     * @param[in] val
     * Floating point value.
     */
    float_atomic(Tfloat val) noexcept : bits_(convert(val))
    {
    }

    /**
     * @brief Copy constructor.
     *
     * @note
     * Copying is _not_ atomic.
     */
    float_atomic(const float_atomic& oth) noexcept : bits_(oth.load()) 
    {
    }

    /**@}*/

public:

    /**
     * @name Unsigned integral operations
     */
    /**@{*/

    /**
     * @brief Store bits.
     */
    void store(
            Tfloat_bits bits, 
            std::memory_order order =
            std::memory_order_seq_cst) noexcept
    {
        bits_.store(bits, order);
    }

    /**
     * @brief Store bits, volatile variant.
     */
    void store(
            Tfloat_bits bits, 
            std::memory_order order =
            std::memory_order_seq_cst) volatile noexcept
    {
        bits_.store(bits, order);
    }

    /**
     * @brief Load bits.
     */
    Tfloat_bits load(
            std::memory_order order =
            std::memory_order_seq_cst) const noexcept
    {
        return bits_.load(order);
    }

    /**
     * @brief Load bits, volatile variant.
     */
    Tfloat_bits load(
            std::memory_order order =
            std::memory_order_seq_cst) const volatile noexcept
    {
        return bits_.load(order);
    }

    /**
     * @brief Fetch _then_ add.
     */
    Tfloat_bits fetch_add(
            Tfloat_bits bits, 
            std::memory_order order =
            std::memory_order_seq_cst) noexcept
    {
        return bits_.fetch_add(bits, order);
    }

    /**
     * @brief Fetch _then_ add, volatile variant.
     */
    Tfloat_bits fetch_add(
            Tfloat_bits bits, 
            std::memory_order order =
            std::memory_order_seq_cst) volatile noexcept
    {
        return bits_.fetch_add(bits, order);
    }

    /**
     * @brief Fetch _then_ subtract.
     */
    Tfloat_bits fetch_sub(
            Tfloat_bits bits, 
            std::memory_order order =
            std::memory_order_seq_cst) noexcept
    {
        return bits_.fetch_sub(bits, order);
    }

    /**
     * @brief Fetch _then_ subtract, volatile variant.
     */
    Tfloat_bits fetch_sub(
            Tfloat_bits bits, 
            std::memory_order order =
            std::memory_order_seq_cst) volatile noexcept
    {
        return bits_.fetch_sub(bits, order);
    }

    /**
     * @brief Fetch _then_ or.
     */
    Tfloat_bits fetch_or(
            Tfloat_bits bits, 
            std::memory_order order =
            std::memory_order_seq_cst) noexcept
    {
        return bits_.fetch_or(bits, order);
    }

    /**
     * @brief Fetch _then_ or, volatile variant.
     */
    Tfloat_bits fetch_or(
            Tfloat_bits bits, 
            std::memory_order order =
            std::memory_order_seq_cst) volatile noexcept
    {
        return bits_.fetch_or(bits, order);
    }

    /**
     * @brief Fetch _then_ xor.
     */
    Tfloat_bits fetch_xor(
            Tfloat_bits bits, 
            std::memory_order order =
            std::memory_order_seq_cst) noexcept
    {
        return bits_.fetch_xor(bits, order);
    }

    /**
     * @brief Fetch _then_ xor, volatile variant.
     */
    Tfloat_bits fetch_xor(
            Tfloat_bits bits, 
            std::memory_order order =
            std::memory_order_seq_cst) volatile noexcept
    {
        return bits_.fetch_xor(bits, order);
    }

    /**
     * @brief Fetch _then_ and.
     */
    Tfloat_bits fetch_and(
            Tfloat_bits bits, 
            std::memory_order order =
            std::memory_order_seq_cst) noexcept
    {
        return bits_.fetch_and(bits, order);
    }

    /**
     * @brief Fetch _then_ and, volatile variant.
     */
    Tfloat_bits fetch_and(
            Tfloat_bits bits, 
            std::memory_order order =
            std::memory_order_seq_cst) volatile noexcept
    {
        return bits_.fetch_and(bits, order);
    }

    /**
     * @brief Atomic compare/exchange.
     */
    bool compare_exchange_weak(
            Tfloat_bits& expect,
            Tfloat_bits  desire,
            std::memory_order success,
            std::memory_order failure) noexcept
    {
        return bits_.compare_exchange_weak(expect, desire, success, failure);
    }

    /**
     * @brief Atomic compare/exchange, volatile variant.
     */
    bool compare_exchange_weak(
            Tfloat_bits& expect,
            Tfloat_bits  desire,
            std::memory_order success,
            std::memory_order failure) volatile noexcept
    {
        return bits_.compare_exchange_weak(expect, desire, success, failure);
    }

    /**
     * @brief Atomic compare/exchange.
     */
    bool compare_exchange_strong(
            Tfloat_bits& expect,
            Tfloat_bits  desire,
            std::memory_order success,
            std::memory_order failure) noexcept
    {
        return bits_.compare_exchange_strong(expect, desire, success, failure);
    }

    /**
     * @brief Atomic compare/exchange, volatile variant.
     */
    bool compare_exchange_strong(
            Tfloat_bits& expect,
            Tfloat_bits  desire,
            std::memory_order success,
            std::memory_order failure) volatile noexcept
    {
        return bits_.compare_exchange_strong(expect, desire, success, failure);
    }

    /**
     * @brief Is lock free?
     */
    bool is_lock_free() const noexcept
    {
        return std::atomic_is_lock_free(&bits_);
    }

    /**
     * @brief Is lock free? volatile variant.
     */
    bool is_lock_free() const volatile noexcept
    {
        return std::atomic_is_lock_free(&bits_);
    }

    /**@}*/

public:

    /**
     * @name Floating point operations
     */
    /**@{*/

    /**
     * @brief Store floating point value.
     */
    void storef(
            Tfloat val, 
            std::memory_order order =
            std::memory_order_seq_cst) noexcept
    {
        bits_.store(convert(val), order);
    }

    /**
     * @brief Store floating point value, volatile variant.
     */
    void storef(
            Tfloat val, 
            std::memory_order order =
            std::memory_order_seq_cst) volatile noexcept
    {
        bits_.store(convert(val), order);
    }

    /**
     * @brief Load floating point value.
     */
    Tfloat loadf(
            std::memory_order order =
            std::memory_order_seq_cst) const noexcept
    {
        return convert(bits_.load(order));
    }

    /**
     * @brief Load floating point value, volatile variant.
     */
    Tfloat loadf(
            std::memory_order order =
            std::memory_order_seq_cst) const volatile noexcept
    {
        return convert(bits_.load(order));
    }

    /**
     * @brief Fetch _then_ add floating point value.
     */
    Tfloat fetch_addf(Tfloat val) noexcept
    {
        Tfloat_bits prev = load(std::memory_order_acquire);
        Tfloat_bits next;
        do { next = convert(convert(prev) + val); } 
            while (!compare_exchange_weak(
                    prev, next,
                    std::memory_order_release,
                    std::memory_order_relaxed));
        return convert(prev);
    }

    /**
     * @brief Fetch _then_ add floating point value, volatile variant.
     */
    Tfloat fetch_addf(Tfloat val) volatile noexcept
    {
        Tfloat_bits prev = load(std::memory_order_acquire);
        Tfloat_bits next;
        do { next = convert(convert(prev) + val); } 
            while (!compare_exchange_weak(
                    prev, next,
                    std::memory_order_release,
                    std::memory_order_relaxed));
        return convert(prev);
    }

    /**
     * @brief Fetch _then_ subtract floating point value.
     */
    Tfloat fetch_subf(Tfloat val) noexcept
    {
        Tfloat_bits prev = load(std::memory_order_acquire);
        Tfloat_bits next;
        do { next = convert(convert(prev) - val); } 
            while (!compare_exchange_weak(
                    prev, next,
                    std::memory_order_release,
                    std::memory_order_relaxed));
        return convert(prev);
    }

    /**
     * @brief Fetch _then_ subtract floating point value, volatile variant.
     */
    Tfloat fetch_subf(Tfloat val) volatile noexcept
    {
        Tfloat_bits prev = load(std::memory_order_acquire);
        Tfloat_bits next;
        do { next = convert(convert(prev) - val); } 
            while (!compare_exchange_weak(
                    prev, next,
                    std::memory_order_release,
                    std::memory_order_relaxed));
        return convert(prev);
    }

    /**@}*/

private:

    /**
     * @brief Atomic integral type.
     */
    std::atomic<Tfloat_bits> bits_;
};

/**@}*/

} // namespace pr

#endif // #ifndef PREFABS_FLOAT_ATOMIC_HPP
