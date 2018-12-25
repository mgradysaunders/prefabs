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
#ifndef PREFABS_KCTREE_HPP
#define PREFABS_KCTREE_HPP

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
 * @defgroup kctree k-c tree
 *
 * `<prefabs/kctree.hpp>`
 */
/**@{*/

/**
 * @brief k-c tree.
 */
template <
    typename Tfloat, std::size_t N,
    typename Tvalue_data,
    typename Tcell_alloc = std::allocator<char>,
    std::size_t Ncell_capacity = 4
    >
class kctree
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

    // Sanity check.
    static_assert(
        Ncell_capacity > 0, 
        "Ncell_capacity must be positive");

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

    /**
     * @brief Constructor.
     *
     * @param[in] box
     * Box.
     *
     * @param[in] cell_alloc
     * Cell allocator.
     */
    kctree(
        const aabb_type& box,
        const cell_allocator_type& cell_alloc = 
              cell_allocator_type()) :
                    cell_alloc_(cell_alloc),
                    root_(box, cell_alloc_)
    {
    }

    /**
     * @brief Constructor.
     *
     * @param[in] box
     * Box.
     *
     * @param[in] cell_alloc
     * Cell allocator.
     */
    kctree(
        const aabb_type& box,
        cell_allocator_type&& cell_alloc) :
                    cell_alloc_(std::move(cell_alloc)),
                    root_(box, cell_alloc_)
    {
    }

    /**
     * @brief Non-copyable.
     */
    kctree(const kctree&) = delete;

public:

    /**
     * @brief Insert.
     *
     * @param[in] loc
     * Location.
     *
     * @param[in] val_data
     * Value data.
     *
     * @throw std::invalid_argument
     * If `loc` is outside root cell.
     */
    void insert(
            const multi_type& loc, 
            const value_data_type& val_data)
    {
        root_.insert(std::make_pair(loc, val_data));
    }

    /**
     * @brief Query region.
     *
     * @param[in] reg
     * Region.
     *
     * @param[in] fun
     * Function.
     *
     * @note 
     * Function must have signature 
     * equivalent to 
     * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{cpp}
     * void(const multi_type&, const value_data_type&)
     * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
     */
    template <typename Fun>
    void query(const aabb_type& reg, Fun&& fun) const
    {
        root_.query(reg, std::forward<Fun>(fun));
    }

    /**
     * @brief Root cell.
     */
    const cell_type& root() const
    {
        return root_;
    }

public:

    /**
     * @brief Cell type.
     */
    class cell_type
    {
    public:

        /**
         * @brief Constructor.
         *
         * @param[in] box
         * Box.
         *
         * @param[in] cell_alloc
         * Cell allocator.
         */
        cell_type(
                const aabb_type& box,
                cell_allocator_type& cell_alloc) :
            box_(box),
            box_center_(box.center()),
            cell_alloc_(cell_alloc)
        {
        }

        /**
         * @brief Destructor.
         */
        ~cell_type()
        {
            if (cells_) {
                // Destroy.
                for (size_type k = 0; k < 1 << N; k++) {
                    cell_allocator_traits::destroy(
                    cell_alloc_, cells_ + k);
                }

                // Deallocate.
                cell_allocator_traits::deallocate(
                cell_alloc_, cells_, 1 << N);
            }
        }

    public:

        /**
         * @brief Insert.
         */
        void insert(const value_type& val)
        {
            if (!box_.contains(val.first)) {
                throw std::invalid_argument(__PRETTY_FUNCTION__);
            }

            // Is tree?
            if (cells_) {
                cell_lookup(val).insert(val);
            }
            else {

                // Is leaf full?
                if (stack_top_ >= Ncell_capacity) {

                    // Allocate.
                    cells_ =
                    cell_allocator_traits::allocate(
                    cell_alloc_, 1 << N);

                    for (size_type k = 0; k < 1 << N; k++) {
            
                        // Subdivide.
                        aabb_type box;
                        for (size_type j = 0; j < N; j++) {
                            if (k & (1 << j)) {
                                box[0][j] = box_[0][j];
                                box[1][j] = box_center_[j];
                            }
                            else {
                                box[0][j] = box_center_[j];
                                box[1][j] = box_[1][j];
                            }
                        }

                        // Construct.
                        cell_allocator_traits::construct(
                        cell_alloc_, 
                        cells_ + k,
                            box, 
                            cell_alloc_);
                    }

                    // Delegate.
                    cell_lookup(val).insert(val);
                }
                else {

                    // Push value.
                    stack_[
                    stack_top_++] = val;
                }
            }
        }

        /**
         * @brief Query region.
         *
         * @param[in] reg
         * Region.
         *
         * @param[in] fun
         * Function.
         *
         * @note 
         * Function must have signature 
         * equivalent to 
         * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{cpp}
         * void(const multi_type&, const value_data_type&)
         * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
         */
        template <typename Fun>
        void query(const aabb_type& reg, Fun&& fun) const
        {
            if (box_.overlaps(reg)) {
                for (auto itval = &stack_[0];
                          itval < &stack_[0] + stack_top_; ++itval) {
                    if (reg.contains(itval->first)) {
                        std::forward<Fun>(fun)(itval->first, itval->second);
                    }
                }

                if (cells_) {
                    for (size_type k = 0; k < 1 << N; k++) {
                        cells_[k].query(reg, std::forward<Fun>(fun));
                    }
                }
            }
        }

    public:

        /**
         * @name Traversal
         */
        /**@{*/

        /**
         * @brief Box.
         */
        const aabb_type& box() const
        {
            return box_;
        }

        /**
         * @brief Box center.
         */
        const multi_type& box_center() const
        {
            return box_center_;
        }

        /**
         * @brief Is leaf?
         */
        bool is_leaf() const
        {
            return cells_ == nullptr;
        }

        /**
         * @brief Is tree?
         */
        bool is_tree() const
        {
            return cells_ != nullptr;
        }

        /**
         * @brief Iterable range of cells.
         */
        range<const cell_type*> cells() const
        {
            if (cells_) {
                return {
                    &cells_[0], 
                    &cells_[0] + (1 << N)
                };
            }
            else {
                return {};
            }
        }

        /**
         * @brief Iterable range of values.
         */
        range<const value_type*> values() const
        {
            return {
                &stack_[0], 
                &stack_[0] + stack_top_
            };
        }

        /**@}*/

    private:

        /**
         * @brief Box.
         */
        aabb_type box_;

        /**
         * @brief Box center.
         */
        multi_type box_center_;

        /**
         * @brief If tree, cell allocator.
         */
        cell_allocator_type& cell_alloc_;

        /**
         * @brief If tree, cells.
         */
        cell_type* cells_ = nullptr;

        /**
         * @brief Value stack.
         */
        value_type stack_[Ncell_capacity] = {};

        /**
         * @brief Value stack top.
         */
        size_type stack_top_ = 0;

    private:

        /**
         * @brief If tree, cell lookup helper.
         */
        cell_type& cell_lookup(const value_type& val)
        {
            assert(cells_);
            return cells_[multitoindex(val.first < box_center_)];
        }
    };

private:

    /**
     * @brief Cell allocator.
     */
    cell_allocator_type cell_alloc_;

    /**
     * @brief Root.
     */
    cell_type root_;
};

/**@}*/

} // namespace pr

#endif // #ifndef PREFABS_KCTREE_HPP
