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
#error "preform/memory_arena.hpp requires >=C++11"
#endif // #if !(__cplusplus >= 201103L)
#endif // #if !DOXYGEN
#pragma once
#ifndef PREFORM_MEMORY_ARENA_HPP
#define PREFORM_MEMORY_ARENA_HPP

// for std::allocator
#include <memory>

// for std::vector
#include <vector>

namespace pre {

/**
 * @defgroup memory_arena Memory arena
 *
 * `<preform/memory_arena.hpp>`
 *
 * __C++ version__: >=C++11
 */
/**@{*/

/**
 * @brief Memory arena.
 *
 * @tparam Tbyte_alloc
 * Underlying byte allocator type.
 */
template <typename Tbyte_alloc = std::allocator<char>>
class memory_arena
{
public:

    /**
     * @brief Constructor.
     */
    memory_arena(
            std::size_t block_size = 0,
            const Tbyte_alloc& byte_alloc = Tbyte_alloc()) :
                block_size_(block_size),
                byte_alloc_(byte_alloc)
    {
        // Round up to 256 byte interval.
        block_size_ +=  255u;
        block_size_ &= ~255u;
        if (block_size_ == 0) {
            block_size_ = 65536;
        }

        // Allocate initial block.
        block_.size = block_size_;
        block_.begin = byte_alloc_.allocate(block_.size);
        block_.offset = 0;

        // Reserve blocks.
        free_blocks_.reserve(4);
        full_blocks_.reserve(4);
    }

    /**
     * @brief Non-copyable.
     */
    memory_arena(const memory_arena&) = delete;

    /**
     * @brief Destructor.
     */
    ~memory_arena()
    {
        // Deallocate current block.
        byte_alloc_.deallocate(block_.begin, block_.size);

        // Deallocate free blocks.
        for (block& free_block : free_blocks_) {
            byte_alloc_.deallocate(free_block.begin, free_block.size);
        }

        // Deallocate full blocks.
        for (block& full_block : full_blocks_) {
            byte_alloc_.deallocate(full_block.begin, full_block.size);
        }
    }

public:

    /**
     * @brief Allocate bytes.
     */
    void* allocate(std::size_t size)
    {
        // Round up to 16 byte interval.
        size +=  15u;
        size &= ~15u;
        if (size == 0) {
            return nullptr;
        }

        if (block_.size < block_.offset + size) {
            full_blocks_.emplace_back(block_);
            if (free_blocks_.size() == 0 ||
                free_blocks_.back().size < size) {
                // Allocate block.
                block_.size = std::max(block_size_, size);
                block_.begin = byte_alloc_.allocate(block_.size);
                block_.offset = 0;
            }
            else {
                // Use free block.
                block_ =
                free_blocks_.back();
                free_blocks_.pop_back();
            }
        }

        char* pos =
            block_.begin +
            block_.offset;
        block_.offset += size;
        return static_cast<void*>(pos);
    }

    /**
     * @brief Clear memory arena.
     */
    void clear()
    {
        // Clear current block.
        block_.offset = 0;

        // Convert full blocks to free blocks.
        free_blocks_.reserve(
                free_blocks_.size() +
                full_blocks_.size());
        for (block& full_block : full_blocks_) {
            free_blocks_.push_back(full_block);
            free_blocks_.back().offset = 0;
        }
        full_blocks_.clear();
    }

    /**
     * @brief Clear memory arena and free blocks.
     */
    void reset()
    {
        // Clear current block.
        block_.offset = 0;

        // Deallocate free blocks.
        for (block& free_block : free_blocks_) {
            byte_alloc_.deallocate(free_block.begin, free_block.size);
        }
        free_blocks_.clear();
        free_blocks_.shrink_to_fit();

        // Deallocate full blocks.
        for (block& full_block : full_blocks_) {
            byte_alloc_.deallocate(full_block.begin, full_block.size);
        }
        full_blocks_.clear();
        full_blocks_.shrink_to_fit();
    }

private:

    /**
     * @brief Memory block.
     */
    struct block
    {
        /**
         * @brief Bytes.
         */
        char* begin;

        /**
         * @brief Offset.
         */
        std::size_t offset;

        /**
         * @brief Size.
         */
        std::size_t size;
    };

    /**
     * @brief Block size.
     */
    std::size_t block_size_;

    /**
     * @brief Current block.
     */
    block block_;

    /**
     * @brief Free blocks.
     */
    std::vector<block> free_blocks_;

    /**
     * @brief Full blocks.
     */
    std::vector<block> full_blocks_;

    /**
     * @brief Byte allocator.
     */
    Tbyte_alloc byte_alloc_;
};

/**@}*/

} // namespace pre

#if !DOXYGEN

// operator new
template <typename Tbyte_alloc>
inline void* operator new(
                std::size_t size,
                pre::memory_arena<Tbyte_alloc>& arena)
{
    return arena.allocate(size);
}

// operator new[]
template <typename Tbyte_alloc>
inline void* operator new[](
                std::size_t size,
                pre::memory_arena<Tbyte_alloc>& arena)
{
    return arena.allocate(size);
}

#endif // #if !DOXYGEN

#endif // #ifndef PREFORM_MEMORY_ARENA_HPP
