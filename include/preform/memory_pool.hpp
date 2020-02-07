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
#if !(__cplusplus >= 201103L)
#error "preform/memory_pool.hpp requires >=C++11"
#endif // #if !(__cplusplus >= 201103L)
#endif // #if !DOXYGEN
#pragma once
#ifndef PREFORM_MEMORY_POOL_HPP
#define PREFORM_MEMORY_POOL_HPP

// for std::size_t
#include <cstddef>

// for std::memcpy
#include <cstring>

// for std::allocator
#include <memory>

namespace pr {

/**
 * @defgroup memory_pool Memory pool
 *
 * `<preform/memory_pool.hpp>`
 *
 * __C++ version__: >=C++11
 */
/**@{*/

/**
 * @brief Memory pool.
 *
 * @tparam Talloc
 * Internal allocator type.
 */
template <typename Talloc = std::allocator<char>>
class memory_pool
{
private:

    /**
     * @brief Byte type.
     */
    typedef char byte_type;

    /**
     * @brief Pool type.
     */
    struct pool_type {

        /**
         * @brief Pointer to next pool.
         */
        pool_type* next;

        /**
         * @brief Pointer to elements.
         */
        byte_type* begin;

        /**
         * @brief Pointer to first free element.
         */
        byte_type* first_free;

    };

    /**
     * @brief Byte allocator type.
     */
    typedef typename std::allocator_traits<Talloc>::
            template rebind_alloc<byte_type> byte_allocator;

    /**
     * @brief Pool allocator type.
     */
    typedef typename std::allocator_traits<Talloc>::
            template rebind_alloc<pool_type> pool_allocator;

public:

    /**
     * @brief Constructor.
     */
    memory_pool(
            std::size_t elem_size,
            std::size_t elems_per_pool,
            const Talloc& alloc = Talloc()) :
                elem_size_(elem_size),
                elems_per_pool_(elems_per_pool),
                byte_alloc_(alloc),
                pool_alloc_(alloc)
    {
        if (elem_size_ < sizeof(void*)) {
            elem_size_ = sizeof(void*);
        }
        if (elems_per_pool_ < 1) {
            elems_per_pool_ = 1;
        }
    }

    /**
     * @brief Non-copyable.
     */
    memory_pool(const memory_pool&) = delete;

    /**
     * @brief Destructor.
     */
    ~memory_pool()
    {
        reset();
    }

public:

    /**
     * @brief Allocate element.
     */
    void* allocate()
    {
        pool_type* pool = head_;
        while (pool && 
               pool->first_free == nullptr) {
            pool = pool->next;
        }

        // Pool with free element not found?
        if (pool == nullptr) {
            pool = alloc_pool_();
        }

        // First free element.
        byte_type* elem = pool->first_free;

        // Increment.
        std::memcpy(
                &pool->first_free, 
                 pool->first_free, sizeof(void*));

        return static_cast<void*>(elem);
    }

    /**
     * @brief Allocate elements.
     *
     * @param[in] count
     * Element count.
     */
    void* allocate(std::size_t count)
    {
        // Validate.
        if (count > elems_per_pool_) {
            throw std::logic_error(__PRETTY_FUNCTION__);
        }

        // Consider special cases.
        if (count == 0) {
            return nullptr;
        }
        if (count == 1) {
            return allocate();
        }

        // TODO

#if 0
        if (first_free_ == nullptr) {
            throw std::runtime_error(__PRETTY_FUNCTION__);
        }
        else {

            char* pos = first_free_;
            char* pos_next = nullptr;
            while (1) {

                // Length of contiguous free element block.
                std::size_t len = 0;
                char* tmp = pos;
                while (len != n) {
                    std::memcpy(&pos_next, tmp, sizeof(void*));
                    if (pos_next == nullptr ||
                        pos_next - tmp != std::ptrdiff_t(elem_stride_)) {
                        break;
                    }
                    tmp = pos_next;
                    len++;
                }

                // Sufficient length?
                if (len == n) {
                    // Position is head of free element list?
                    if (pos == first_free_) {
                        // Move head.
                        first_free_ = pos_next;
                    }
                    else {
                        // Find previous position.
                        char* pos_prev = first_free_;
                        char* pos_prev_next;
                        while (1) {
                            std::memcpy(
                               &pos_prev_next, pos_prev, sizeof(void*));
                            if (pos_prev_next == pos) {
                                break;
                            }
                            pos_prev = pos_prev_next;
                        }
                        // Link.
                        std::memcpy(pos_prev, &pos_next, sizeof(void*));
                    }

                    return pos;
                }
                else {
                    // No suitable block found?
                    if (pos_next == nullptr) {
                        throw std::runtime_error(__PRETTY_FUNCTION__);
                    }
                    pos = pos_next;
                }
            }
        }
#endif

        // Unreachable.
        return nullptr;
    }

    /**
     * @brief Deallocate element.
     *
     * @param[in] ptr
     * Pointer.
     *
     * @note
     * If `ptr == nullptr`, implementation is a no-op.
     * 
     * @throw std::logic_error
     * If `ptr` is neither `nullptr` nor a pointer returned
     * by `allocate()`.
     */
    void deallocate(void* ptr)
    {
        if (ptr == nullptr) {
            return;
        }

        byte_type* elem = static_cast<byte_type*>(ptr);
        pool_type* pool = head_;
        while (pool) {
            // Element in range of pool?
            std::ptrdiff_t pool_size = elem_size_ * elems_per_pool_;
            std::ptrdiff_t elem_diff = elem - pool->begin;
            if (elem_diff >= 0 && elem_diff < pool_size) {

                // Element in range of pool, but pointer is not 
                // aligned to element boundary?
                if (elem_diff % std::ptrdiff_t(elem_size_) != 0) {

                    // User passed a garbage pointer.
                    throw std::logic_error(__PRETTY_FUNCTION__);
                }

                break;
            }
            else {
                // Search next pool.
                pool = pool->next; 
            }
        }

        // Element not in range of any pool?
        if (!pool) {

            // User passed a garbage pointer.
            throw std::logic_error(__PRETTY_FUNCTION__);
        }

        // Insert so that free element list is sorted by address.
        if (pool->first_free == nullptr ||
            pool->first_free > elem) {

            // Prepend.
            std::memcpy(elem, &pool->first_free, sizeof(void*));
            pool->first_free = elem;
        }
        else {

            // Find insertion.
            byte_type* free_elem = pool->first_free;
            byte_type* free_elem_next = nullptr;
            while (1) {
                std::memcpy(&free_elem_next, free_elem, sizeof(void*));
                if (free_elem_next == nullptr ||
                    free_elem_next > elem) {
                    break;
                }
                free_elem = free_elem_next;
            }

            // Insert.
            std::memcpy(free_elem, &elem, sizeof(void*));
            std::memcpy(elem, &free_elem_next, sizeof(void*));
        }
    }

    /**
     * @brief Deallocate elements.
     *
     * @param[in] ptr
     * Pointer.
     *
     * @param[in] count
     * Element count.
     *
     * @note
     * If `ptr == nullptr` or `count == 0`, implementation is a no-op.
     */
    void deallocate(void* ptr, std::size_t count)
    {
        if (ptr == nullptr) {
            return;
        }
        
        byte_type* elem = static_cast<byte_type*>(ptr);
        for (std::size_t index = 0;
                         index < count; index++) {

            // Delegate.
            deallocate(elem + index * elem_size_);
        }
    }

    /**
     * @brief Clear.
     */
    void clear()
    {
        // Loop through all pools.
        for (pool_type* pool = head_; pool; pool = pool->next) {

            // Clear.
            clear_pool_(pool);
        }
    }

    /**
     * @brief Reset.
     */
    void reset()
    {
        for (pool_type* pool = head_; pool;) {

            // Deallocate.
            pool_type* next = pool->next;
            byte_alloc_.deallocate(pool->begin, elem_size_ * elems_per_pool_);
            pool_alloc_.deallocate(pool, 1);
            pool = next; // Increment.
        }

        // Nullify.
        head_ = nullptr;
    }

private:

    /**
     * @brief Element size in bytes.
     */
    std::size_t elem_size_ = 0;

    /**
     * @brief Elements per pool.
     */
    std::size_t elems_per_pool_ = 0;

    /**
     * @brief Pool list.
     */
    pool_type* head_ = nullptr;

    /**
     * @brief Byte allocator.
     */
    byte_allocator byte_alloc_;

    /**
     * @brief Pool allocator.
     */
    pool_allocator pool_alloc_;

#if !DOXYGEN 
private:

    // Allocate pool and append to list.
    pool_type* alloc_pool_()
    {
        // Allocate pool structure.
        pool_type* pool = 
        pool_alloc_.allocate(1);
        pool->next = nullptr;

        // Allocate pool memory.
        pool->begin = 
        byte_alloc_.allocate(elem_size_ * elems_per_pool_);

        // Clear.
        clear_pool_(pool);

        // First pool?
        if (head_ == nullptr) {
            head_ = pool; // Set head.
        }
        else {

            // Append.
            pool_type* tail = head_;
            while (tail->next) { tail = tail->next; }
            tail->next = pool;
        }

        return pool;
    }

    // Clear pool.
    void clear_pool_(pool_type* pool)
    {
        // Link all elements sequentially.
        pool->first_free = pool->begin;
        for (std::size_t index = 0;
                         index + 1 < elems_per_pool_; index++) {

            byte_type* elem0 = pool->begin + (index + 0) * elem_size_;
            byte_type* elem1 = pool->begin + (index + 1) * elem_size_;
            std::memcpy(elem0, &elem1, sizeof(void*));
        }

        byte_type* elem0 = pool->begin + (elems_per_pool_ - 1) * elem_size_;
        byte_type* elem1 = nullptr;
        std::memcpy(elem0, &elem1, sizeof(void*));
    }

#endif // #if !DOXYGEN
};

/**@}*/

} // namespace pr

#endif // #ifndef PREFORM_MEMORY_POOL_HPP
