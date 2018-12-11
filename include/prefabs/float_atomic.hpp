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
 * @defgroup float_atomic Floating point atomic
 */
/**@{*/

/**
 * @brief Floating point atomic.
 *
 * @tparam T Floating type.
 *
 * @tparam U Unsigned integral type.
 */
template <
    typename T, 
    typename U
    >
class float_atomic
{
public:

    // sanity check
    static_assert(
            std::is_floating_point<T>::value &&
            std::is_unsigned<U>::value);
    static_assert(
            sizeof(T) == sizeof(U));

    /**
     * @name Type conversion
     */
    /**@{*/

    /**
     * @brief Mem-copy `T` to `U`.
     */
    static inline U convert(T val) noexcept
    {
        U res; std::memcpy(&res, &val, sizeof(val)); return res;
    }

    /**
     * @brief Mem-copy `U` to `T`.
     */
    static inline T convert(U val) noexcept
    {
        T res; std::memcpy(&res, &val, sizeof(val)); return res;
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
    constexpr float_atomic(U bits) noexcept : bits_(bits)
    {
    }

    /**
     * @brief Constructor.
     *
     * @param[in] val
     * Floating point value.
     */
    float_atomic(T val) noexcept : bits_(convert(val))
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
    void store(U bits, 
            std::memory_order order =
            std::memory_order_seq_cst) noexcept
    {
        bits_.store(bits, order);
    }

    /**
     * @brief Store bits, volatile variant.
     */
    void store(U bits, 
            std::memory_order order =
            std::memory_order_seq_cst) volatile noexcept
    {
        bits_.store(bits, order);
    }

    /**
     * @brief Load bits.
     */
    U load(std::memory_order order =
           std::memory_order_seq_cst) const noexcept
    {
        return bits_.load(order);
    }

    /**
     * @brief Load bits, volatile variant.
     */
    U load(std::memory_order order =
           std::memory_order_seq_cst) const volatile noexcept
    {
        return bits_.load(order);
    }

    /**
     * @brief Fetch _then_ add.
     */
    U fetch_add(U bits, 
            std::memory_order order =
            std::memory_order_seq_cst) noexcept
    {
        return bits_.fetch_add(bits, order);
    }

    /**
     * @brief Fetch _then_ add, volatile variant.
     */
    U fetch_add(U bits, 
            std::memory_order order =
            std::memory_order_seq_cst) volatile noexcept
    {
        return bits_.fetch_add(bits, order);
    }

    /**
     * @brief Fetch _then_ subtract.
     */
    U fetch_sub(U bits, 
            std::memory_order order =
            std::memory_order_seq_cst) noexcept
    {
        return bits_.fetch_sub(bits, order);
    }

    /**
     * @brief Fetch _then_ subtract, volatile variant.
     */
    U fetch_sub(U bits, 
            std::memory_order order =
            std::memory_order_seq_cst) volatile noexcept
    {
        return bits_.fetch_sub(bits, order);
    }

    /**
     * @brief Fetch _then_ or.
     */
    U fetch_or(U bits, 
            std::memory_order order =
            std::memory_order_seq_cst) noexcept
    {
        return bits_.fetch_or(bits, order);
    }

    /**
     * @brief Fetch _then_ or, volatile variant.
     */
    U fetch_or(U bits, 
            std::memory_order order =
            std::memory_order_seq_cst) volatile noexcept
    {
        return bits_.fetch_or(bits, order);
    }

    /**
     * @brief Fetch _then_ xor.
     */
    U fetch_xor(U bits, 
            std::memory_order order =
            std::memory_order_seq_cst) noexcept
    {
        return bits_.fetch_xor(bits, order);
    }

    /**
     * @brief Fetch _then_ xor, volatile variant.
     */
    U fetch_xor(U bits, 
            std::memory_order order =
            std::memory_order_seq_cst) volatile noexcept
    {
        return bits_.fetch_xor(bits, order);
    }

    /**
     * @brief Fetch _then_ and.
     */
    U fetch_and(U bits, 
            std::memory_order order =
            std::memory_order_seq_cst) noexcept
    {
        return bits_.fetch_and(bits, order);
    }

    /**
     * @brief Fetch _then_ and, volatile variant.
     */
    U fetch_and(U bits, 
            std::memory_order order =
            std::memory_order_seq_cst) volatile noexcept
    {
        return bits_.fetch_and(bits, order);
    }

    /**
     * @brief Atomic compare/exchange.
     */
    bool compare_exchange_weak(
            U& expect,
            U  desire,
            std::memory_order success,
            std::memory_order failure) noexcept
    {
        return bits_.compare_exchange_weak(expect, desire, success, failure);
    }

    /**
     * @brief Atomic compare/exchange, volatile variant.
     */
    bool compare_exchange_weak(
            U& expect,
            U  desire,
            std::memory_order success,
            std::memory_order failure) volatile noexcept
    {
        return bits_.compare_exchange_weak(expect, desire, success, failure);
    }

    /**
     * @brief Atomic compare/exchange.
     */
    bool compare_exchange_strong(
            U& expect,
            U  desire,
            std::memory_order success,
            std::memory_order failure) noexcept
    {
        return bits_.compare_exchange_strong(expect, desire, success, failure);
    }

    /**
     * @brief Atomic compare/exchange, volatile variant.
     */
    bool compare_exchange_strong(
            U& expect,
            U  desire,
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
    void storef(T val, 
            std::memory_order order =
            std::memory_order_seq_cst) noexcept
    {
        bits_.store(convert(val), order);
    }

    /**
     * @brief Store floating point value, volatile variant.
     */
    void storef(T val, 
            std::memory_order order =
            std::memory_order_seq_cst) volatile noexcept
    {
        bits_.store(convert(val), order);
    }

    /**
     * @brief Load floating point value.
     */
    T loadf(std::memory_order order =
            std::memory_order_seq_cst) const noexcept
    {
        return convert(bits_.load(order));
    }

    /**
     * @brief Load floating point value, volatile variant.
     */
    T loadf(std::memory_order order =
            std::memory_order_seq_cst) const volatile noexcept
    {
        return convert(bits_.load(order));
    }

    /**
     * @brief Fetch _then_ add floating point value.
     */
    T fetch_addf(T val) noexcept
    {
        U prev = load(std::memory_order_acquire);
        U next;
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
    T fetch_addf(T val) volatile noexcept
    {
        U prev = load(std::memory_order_acquire);
        U next;
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
    T fetch_subf(T val) noexcept
    {
        U prev = load(std::memory_order_acquire);
        U next;
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
    T fetch_subf(T val) volatile noexcept
    {
        U prev = load(std::memory_order_acquire);
        U next;
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
    std::atomic<U> bits_;
};

/**@}*/

} // namespace pr

#endif // #ifndef PREFABS_FLOAT_ATOMIC_HPP
