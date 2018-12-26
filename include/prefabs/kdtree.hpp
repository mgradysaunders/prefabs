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
#if !(__cplusplus >= 201703L)
#error "prefabs/kdtree.hpp requires >=C++17"
#endif // #if !(__cplusplus >= 201703L)
#endif // #if !DOXYGEN
#pragma once
#ifndef PREFABS_KDTREE_HPP
#define PREFABS_KDTREE_HPP

// for assert
#include <cassert>

// for std::allocator, std::allocator_traits
#include <memory>

// for std::vector
#include <vector>

// for std::forward, std::pair, ...
#include <utility>

// for std::logic_error, std::invalid_argument
#include <stdexcept>

// for pr::range
#include <prefabs/range.hpp>

// for pr::multi
#include <prefabs/multi.hpp>

// for pr::aabb
#include <prefabs/aabb.hpp>

namespace pr {

/**
 * @defgroup kdtree k-d tree
 *
 * `<prefabs/kdtree.hpp>`
 *
 * __C++ version__: >=C++17
 */
/**@{*/

/**
 * @brief k-d tree.
 */
template <
    typename Tfloat, std::size_t N,
    typename Tvalue_data,
    typename Tcell_alloc = std::allocator<char>
    >
class kdtree
{
public:

    // Sanity check.
    static_assert(
        std::is_floating_point<Tfloat>::value,
        "Tfloat must be floating point");

    // Sanity check.
    static_assert(
        std::is_default_constructible<Tvalue_data>::value &&
        std::is_copy_assignable<Tvalue_data>::value, 
        "Tvalue_data must be default constructible and copy assignable");

public:

    /**
     * @name Container typedefs
     */
    /**@{*/

    /**
     * @brief Axis-aligned bounding box type.
     */
    typedef aabb<Tfloat, N> aabb_type;

    /**
     * @brief Multi-dimensional array type.
     */
    typedef multi<Tfloat, N> multi_type;

    /**
     * @brief Floating point type.
     */
    typedef Tfloat float_type;

    /**
     * @brief Value data type.
     */
    typedef Tvalue_data value_data_type;

    /**
     * @brief Value type.
     */
    typedef std::pair<
                multi_type, 
                value_data_type> value_type;
    
    /**
     * @brief Size type.
     */
    typedef std::size_t size_type;

#if !DOXYGEN
    // prototype
    class cell_type;
#endif // #if !DOXYGEN
    
    /**
     * @brief Cell allocator type.
     */
    typedef typename std::allocator_traits<Tcell_alloc>::
            template rebind_alloc<cell_type> cell_allocator_type;

    /**
     * @brief Cell allocator traits.
     */
    typedef typename std::allocator_traits<Tcell_alloc>::
            template rebind_traits<cell_type> cell_allocator_traits;

    /**@}*/

public:

};

/**@}*/

} // namespace pr

#endif // #ifndef PREFABS_KDTREE_HPP
