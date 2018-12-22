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
#ifndef PREFABS_MEMORY_ARENA_ALLOCATOR_HPP
#define PREFABS_MEMORY_ARENA_ALLOCATOR_HPP

#include <prefabs/memory_arena.hpp>

namespace pr {

/**
 * @defgroup memory_arena_allocator Memory arena allocator
 *
 * `<prefabs/memory_arena_allocator.hpp>`
 */
/**@{*/

/**
 * @brief Memory arena allocator.
 */
template <typename T>
class memory_arena_allocator
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
     * @param[in] block_size
     * Block size for underlying memory arena.
     */
    memory_arena_allocator(
            std::size_t block_size = 0) : 
            arena_(new memory_arena(block_size))
    {
    }

    /**
     * @brief Copy constructor.
     */
    template <typename U>
    memory_arena_allocator(
            const memory_arena_allocator<U>& other) :
            arena_(other.arena_)
    {
    }

    /**
     * @brief Move constructor.
     */
    template <typename U>
    memory_arena_allocator(
            memory_arena_allocator<U>&& other) :
            arena_(std::move(other.arena_))
    {
    }

    /**
     * @brief Copy assignment.
     */
    template <typename U>
    memory_arena_allocator& operator=(
                            const memory_arena_allocator<U>& other)
    {
        if (this != &other) {
            this->arena_ = other.arena_;
        }
        return *this;
    }

    /**
     * @brief Move assignment.
     */
    template <typename U>
    memory_arena_allocator& operator=(
                            memory_arena_allocator<U>&& other)
    {
        this->arena_ = std::move(other.arena_);
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
        // Delegate.
        return static_cast<T*>(arena_->allocate(sizeof(T) * n));
    }

    /**
     * @brief Deallocate.
     *
     * @param[in] p
     * Pointer.
     *
     * @param[in] n
     * Number of objects.
     */
    void deallocate(T* p, std::size_t n)
    {
        // Do nothing.
        (void) p;
        (void) n;
    }

    /**
     * @brief Equal?
     */
    template <typename U>
    bool operator==(const memory_arena_allocator<U>& other) const
    {
        return arena_.get() == other.arena_.get();
    }

    /**
     * @brief Not equal?
     */
    template <typename U>
    bool operator!=(const memory_arena_allocator<U>& other) const
    {
        return arena_.get() != other.arena_.get();
    }

private:

    /**
     * @brief Memory arena.
     */
    std::shared_ptr<memory_arena> arena_;
};

/**@}*/

} // namespace pr

#endif // #ifndef PREFABS_MEMORY_ARENA_ALLOCATOR_HPP
