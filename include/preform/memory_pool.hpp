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
 * @tparam Tbyte_alloc
 * Underlying byte allocator type.
 */
template <typename Tbyte_alloc = std::allocator<char>>
class memory_pool
{
public:

    /**
     * @brief Constructor.
     */
    memory_pool(
            std::size_t elem_stride,
            std::size_t elem_count,
            const Tbyte_alloc& byte_alloc = Tbyte_alloc()) :
                elem_stride_(elem_stride),
                elem_count_(elem_count),
                byte_alloc_(byte_alloc)
    {
        if (elem_stride_ < sizeof(void*)) {
            elem_stride_ = sizeof(void*);
        }
        if (elem_count_ < 1) {
            elem_count_ = 1;
        }

        // Allocate pool.
        begin_ = 
            byte_alloc_.allocate(
            elem_stride_ * elem_count_);

        // Initialize free element list.
        clear();
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
        // Deallocate.
        byte_alloc_.deallocate(begin_, elem_stride_ * elem_count_);

        // Nullify.
        begin_ = nullptr;

        // Nullify.
        first_free_ = nullptr;
    }

public:

    /**
     * @brief Allocate element.
     *
     * @throw std::runtime_error
     * If no free elements.
     */
    void* allocate()
    {
        if (first_free_ == nullptr) {
            throw std::runtime_error(__PRETTY_FUNCTION__);
        }
        else {
            // Result.
            void* ptr = first_free_;

            // Increment.
            std::memcpy(
                    &first_free_,
                    first_free_,
                    sizeof(void*));

            return ptr;
        }
    }

    /**
     * @brief Allocate elements.
     *
     * @param[in] n
     * Number of elements.
     *
     * @throw std::runtime_error
     * If no contiguous block of `n` free elements.
     *
     * @note
     * If `n == 0`, returns null.
     *
     * @note
     * If the pool is fragmented, this function may throw even
     * if `n` or more free elements exist.
     */
    void* allocate(std::size_t n)
    {
        // Consider special cases.
        if (n == 0) {
            return nullptr;
        }
        if (n == 1) {
            return allocate();
        }

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
     */
    void deallocate(void* ptr)
    {
        if (ptr != nullptr) {

            // Validate.
            std::ptrdiff_t off = static_cast<char*>(ptr) - begin_;
            if (!(off >= 0 &&
                  off < std::ptrdiff_t(elem_stride_ * elem_count_) &&
                  off % std::ptrdiff_t(elem_stride_) == 0)) {
                throw std::invalid_argument(__PRETTY_FUNCTION__);
            }

            if (first_free_ == nullptr ||
                first_free_ > static_cast<char*>(ptr)) {

                // Prepend.
                std::memcpy(ptr, &first_free_, sizeof(void*));
                first_free_ = static_cast<char*>(ptr);
            }
            else {

                // Insert so that free element list is sorted by address.
                char* pos = first_free_;
                char* pos_next = nullptr;
                while (1) {
                    std::memcpy(&pos_next, pos, sizeof(void*));
                    if (pos_next == nullptr ||
                        pos_next > static_cast<char*>(ptr)) {
                        break;
                    }
                    pos = pos_next;
                }
                std::memcpy(pos, &ptr, sizeof(void*));
                std::memcpy(ptr, &pos_next, sizeof(void*));
            }
        }
    }

    /**
     * @brief Deallocate elements.
     *
     * @param[in] ptr
     * Pointer.
     *
     * @param[in] n
     * Number of elements.
     *
     * @note
     * If `ptr == nullptr` or `n == 0`, implementation is a no-op.
     */
    void deallocate(void* ptr, std::size_t n)
    {
        if (ptr != nullptr) {
            char* pos = static_cast<char*>(ptr);
            while (n > 0) {
                deallocate(pos);
                pos += elem_stride_;
                n--;
            }
        }
    }

    /**
     * @brief Clear, deallocate all elements.
     */
    void clear()
    {
        // Reset free element list.
        first_free_ = begin_;
        for (std::size_t elem_index = 0; 
                         elem_index + 1 < elem_count_; 
                         elem_index++) {
            char* elem0 = begin_ + (elem_index + 0) * elem_stride_;
            char* elem1 = begin_ + (elem_index + 1) * elem_stride_;
            std::memcpy(elem0, &elem1, sizeof(void*));
        }
        char* elem0 = begin_ + (elem_count_ - 1) * elem_stride_;
        char* elem1 = nullptr;
        std::memcpy(elem0, &elem1, sizeof(void*));
    }

private:

    /**
     * @brief Element size in bytes.
     */
    std::size_t elem_stride_ = 0;

    /**
     * @brief Element count.
     */
    std::size_t elem_count_ = 0;

    /**
     * @brief Pointer to elements.
     */
    char* begin_ = nullptr;

    /**
     * @brief Pointer to first free element.
     */
    char* first_free_ = nullptr;

    /**
     * @brief Byte allocator.
     */
    Tbyte_alloc byte_alloc_;
};

/**@}*/

} // namespace pr

#endif // #ifndef PREFORM_MEMORY_POOL_HPP
