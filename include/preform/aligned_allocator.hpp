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

// for std::memcpy
#include <cstring>

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
 * @brief Align pointer.
 *
 * @param[in] ptr
 * Pointer.
 *
 * @param[in] alignment
 * Alignment boundary.
 *
 * @throw std::invalid_argument
 * Unless `alignment` is a power of 2.
 */
template <typename T>
inline T* align_pointer(T* ptr, std::size_t alignment)
{
    if (!(alignment > 0 && 
         (alignment & (alignment - 1)) == 0)) {
        throw std::invalid_argument(__PRETTY_FUNCTION__);
    }
    std::uintptr_t ptr_addr = 0;
    std::memcpy(
            &ptr_addr, 
            &ptr, 
            sizeof(ptr));
    // Align.
    std::size_t alignment_mask = alignment - 1;
    ptr_addr += alignment_mask;
    ptr_addr &= ~alignment_mask;
    std::memcpy(
            &ptr, 
            &ptr_addr, 
            sizeof(ptr));
    return ptr;
}

/**
 * @brief Aligned new.
 *
 * @param[in] size
 * Size of allocation in bytes.
 *
 * @param[in] alignment
 * Alignment boundary.
 *
 * @throw std::invalid_argument
 * Unless `alignment` is a power of 2.
 */
inline void* aligned_new(std::size_t size, std::size_t alignment)
{
    if (size == 0) {
        return nullptr;
    }
    else {
        std::size_t size_actual = size + alignment + sizeof(void*);
        char* mem = static_cast<char*>(::operator new(size_actual));
        char* mem_shifted = mem + sizeof(void*);
        char* mem_aligned = align_pointer(mem_shifted, alignment);
        std::memcpy(mem_aligned - sizeof(void*), &mem, sizeof(void*));
        return static_cast<void*>(mem_aligned);
    }
}

/**
 * @brief Aligned delete.
 *
 * @param[in] ptr
 * Pointer.
 */
inline void aligned_delete(void* ptr)
{
    if (ptr != nullptr) {
        char* mem_aligned = static_cast<char*>(ptr);
        char* mem = nullptr;
        std::memcpy(&mem, mem_aligned - sizeof(void*), sizeof(void*));
        ::operator delete(mem);
    }
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
        return static_cast<T*>(pr::aligned_new(sizeof(T) * n, alignment_));
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
