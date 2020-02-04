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
#error "preform/aligned_allocator.hpp requires >=C++11"
#endif // #if !(__cplusplus >= 201103L)
#endif // #if !DOXYGEN
#pragma once
#ifndef PREFORM_ALIGNED_ALLOCATOR_HPP
#define PREFORM_ALIGNED_ALLOCATOR_HPP

// for std::size_t
#include <cstddef>

// for std::uintptr_t
#include <cstdint>

// for std::aligned_alloc if C++17
#include <cstdlib>

// for std::memcpy
#include <cstring>

// for std::invalid_argument
#include <stdexcept>

namespace pr {

/**
 * @defgroup aligned_allocator Aligned allocator
 *
 * `<preform/aligned_allocator.hpp>`
 *
 * __C++ version__: >=C++11
 */
/**@{*/

/**
 * @brief Convert pointer to pointer address.
 *
 * @param[in] ptr
 * Pointer.
 */
__attribute__((always_inline))
inline std::uintptr_t pointer_to_address(void* ptr)
{
    std::uintptr_t ptr_addr = 0;
    std::memcpy(
            &ptr_addr,
            &ptr,
            sizeof(void*));
    return ptr_addr;
}

/**
 * @brief Convert pointer address to pointer.
 *
 * @param[in] ptr_addr
 * Pointer address.
 */
__attribute__((always_inline))
inline void* address_to_pointer(std::uintptr_t ptr_addr)
{
    void* ptr = nullptr;
    std::memcpy(
            &ptr,
            &ptr_addr,
            sizeof(void*));
    return ptr;
}

/**
 * @brief Aligned new.
 *
 * @param[in] alignment
 * Alignment boundary.
 *
 * @param[in] size
 * Size of allocation in bytes.
 *
 * @throw std::invalid_argument
 * If `alignment` is not a power of 2.
 *
 * @note
 * If compiling with C++17 or newer, this delegates to 
 * `std::aligned_alloc()`. Otherwise, this over-allocates and
 * aligns manually.
 */
inline void* aligned_new(std::size_t alignment, std::size_t size)
{
    // Is alignment not a power of 2?
    if (!(alignment > 0 && 
         (alignment & (alignment - 1)) == 0)) {
        throw std::invalid_argument(__PRETTY_FUNCTION__);
    }

    if (size == 0) {
        return nullptr;
    }
    else {
#if (__cplusplus >= 201703L)

        // Round size up to multiple of alignment,
        // which is required by std::aligned_alloc().
        if (size & (alignment - 1)) {
            size += alignment - (size & (alignment - 1));
        }

        // Delegate.
        return std::aligned_alloc(alignment, size);

#else

        // Make sure we have enough space to align the pointer,
        // then save the original pointer in the sizeof(void*) bytes
        // before the aligned pointer.
        size += alignment + sizeof(void*);

        // Original pointer.
        void* ptr = ::operator new(size);
 
        // Original pointer address.
        std::uintptr_t ptr_addr = pointer_to_address(ptr);

        // Aligned pointer address, after shifting by sizeof(void*).
        std::uintptr_t ptr_aligned_addr = 
                (ptr_addr + sizeof(void*) + 
                (alignment - 1)) & ~(alignment - 1);

        // Aligned pointer.
        void* ptr_aligned = address_to_pointer(ptr_aligned_addr);

        // Save original pointer.
        std::memcpy(
                static_cast<char*>(ptr_aligned) - 
                sizeof(void*), 
                &ptr, sizeof(void*));

        return ptr_aligned;

#endif // #if (__cplusplus >= 201703L)
    }
}

/**
 * @brief Aligned delete.
 *
 * @param[in] ptr_aligned
 * Pointer.
 */
inline void aligned_delete(void* ptr_aligned)
{
#if (__cplusplus >= 201703L) 

    // Delegate.
    std::free(ptr_aligned);

#else

    if (ptr_aligned != nullptr) {

        // Load original pointer.
        void* ptr = nullptr;
        std::memcpy(&ptr, 
                static_cast<char*>(ptr_aligned) - sizeof(void*), 
                sizeof(void*));

        // Delete.
        ::operator delete(ptr);
    }

#endif // #if (__cplusplus >= 201703L)
}

/**
 * @brief Aligned allocator.
 */
template <typename T>
class aligned_allocator
{
public:

    /**
     * @brief Value type.
     */
    typedef T value_type;

    /**
     * @brief Propagate on container copy assignment.
     */
    typedef std::true_type propagate_on_container_copy_assignment;

    /**
     * @brief Propagate on container move assignment.
     */
    typedef std::true_type propagate_on_container_move_assignment;

    /**
     * @brief Propagate on container swap.
     */
    typedef std::true_type propagate_on_container_swap;

    /**
     * @brief Is _not_ always equal.
     */
    typedef std::false_type is_always_equal;

public:

    /**
     * @brief Constructor.
     *
     * @param[in] alignment
     * Alignment boundary.
     */
    aligned_allocator(std::size_t alignment = 1) :
            alignment_(alignment)
    {
    }

    /**
     * @brief Copy constructor.
     */
    template <typename U>
    aligned_allocator(const aligned_allocator<U>& other) :
            alignment_(other.alignment_)
    {
    }

    /**
     * @brief Move constructor.
     */
    template <typename U>
    aligned_allocator(aligned_allocator<U>&& other) :
            alignment_(std::move(other.alignment_))
    {
    }

    /**
     * @brief Copy assignment.
     */
    template <typename U>
    aligned_allocator& operator=(const aligned_allocator<U>& other)
    {
        if (this != &other) {
            this->alignment_ = other.alignment_;
        }
        return *this;
    }

    /**
     * @brief Move assignment.
     */
    template <typename U>
    aligned_allocator& operator=(aligned_allocator<U>&& other)
    {
        this->alignment_ = std::move(other.alignment_);
        return *this;
    }

    /**
     * @brief Allocate.
     *
     * @param[in] n
     * Number of objects.
     */
    [[nodiscard]]
    T* allocate(std::size_t n)
    {
        return static_cast<T*>(pr::aligned_new(alignment_, sizeof(T) * n));
    }

    /**
     * @brief Deallocate.
     *
     * @param[in] ptr
     * Pointer.
     *
     * @param[in] n
     * Number of objects.
     */
    void deallocate(T* ptr, std::size_t n)
    {
        (void) n;
        pr::aligned_delete(ptr);
    }

    /**
     * @brief Equal?
     */
    template <typename U>
    bool operator==(const aligned_allocator<U>& other) const
    {
        return alignment_ == other.alignment_;
    }

    /**
     * @brief Not equal?
     */
    template <typename U>
    bool operator!=(const aligned_allocator<U>& other) const
    {
        return alignment_ != other.alignment_;
    }

private:

    /**
     * @brief Alignment.
     */
    std::size_t alignment_;

    // Declare friend.
    template <typename>
    friend class aligned_allocator;
};

/**@}*/

} // namespace pr

#endif // #ifndef PREFORM_ALIGNED_ALLOCATOR_HPP
