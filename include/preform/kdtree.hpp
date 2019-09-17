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
#if !(__cplusplus >= 201703L)
#error "preform/kdtree.hpp requires >=C++17"
#endif // #if !(__cplusplus >= 201703L)
#endif // #if !DOXYGEN
#pragma once
#ifndef PREFORM_KDTREE_HPP
#define PREFORM_KDTREE_HPP

// for std::nth_element
#include <algorithm>

// for std::allocator
#include <memory>

// for std::vector
#include <vector>

// for pr::multi
#include <preform/multi.hpp>

// for pr::multi wrappers
#include <preform/multi_math.hpp>

// for pr::aabb
#include <preform/aabb.hpp>

// for pr::iterator_range
#include <preform/iterator_range.hpp>

namespace pr {

/**
 * @defgroup kdtree Kd tree
 *
 * `<preform/kdtree.hpp>`
 */
/**@{*/

/**
 * @brief Kd tree.
 *
 * @tparam Tfloat
 * Float type.
 *
 * @tparam N
 * Dimension.
 *
 * @tparam Tvalue
 * Value type.
 *
 * @tparam Talloc
 * Allocator type.
 */
template <
    typename Tfloat, std::size_t N, 
    typename Tvalue,
    typename Talloc = std::allocator<char>
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
        N < 256,
        "N must be less than 256");

    // Sanity check.
    static_assert(
        std::is_default_constructible<Tvalue>::value &&
        std::is_copy_constructible<Tvalue>::value &&
        std::is_copy_assignable<Tvalue>::value &&
        std::is_destructible<Tvalue>::value,
        "Tvalue must be default constructible, copy constructible, "
        "copy assignable, and destructible");

#if !DOXYGEN

    struct node_type;

#endif // #if !DOXYGEN

    /**
     * @name Container typedefs
     */
    /**@{*/

    /**
     * @brief Size type.
     */
    typedef std::size_t size_type;

    /**
     * @brief Float type.
     */
    typedef Tfloat float_type;

    /**
     * @brief Point type.
     */
    typedef multi<Tfloat, N> point_type;

    /**
     * @brief Value type.
     */
    typedef std::pair<multi<Tfloat, N>, Tvalue> value_type;

    /**
     * @brief Node allocator type.
     */
    typedef typename std::allocator_traits<Talloc>::
            template rebind_alloc<node_type> node_allocator_type;

    /**@}*/

public:

    /**
     * @brief Node type.
     */
    struct node_type
    {
        /**
         * @brief Value.
         */
        value_type value;

        /**
         * @brief Left child.
         */
        node_type* left;

        /**
         * @brief Right child.
         */
        node_type* right;

        /**
         * @brief Split dimension.
         */
        std::uint8_t split_dim;
    };
    
public:

    /**
     * @brief Constructor.
     *
     * @param[in] alloc
     * Allocator.
     */
    kdtree(const Talloc& alloc = Talloc()) : node_alloc_(alloc)
    {
    }

    /**
     * @brief Non-copyable.
     */
    kdtree(const kdtree&) = delete;

    /**
     * @brief Destructor.
     */
    ~kdtree()
    {
        clear();
    }

public:

    /**
     * @brief Initialize.
     *
     * @param[in] from
     * Input from.
     *
     * @param[in] to
     * Input to.
     *
     * @param[in] func
     * Function constructing an instance of `value_type` for each 
     * input element.
     *
     * @note
     * Function must have signature equivalent to
     * ~~~~~~~~~~~~~~~~~~~~~~~~~{cpp}
     * value_type(const Tinput&)
     * ~~~~~~~~~~~~~~~~~~~~~~~~~
     * where `Tinput` is the value type corresponding to
     * `Tinput_itr`.
     */
    template <typename Tinput_itr, typename Tfunc>
    void init(
            Tinput_itr from,
            Tinput_itr to,
            Tfunc&& func)
    {
        // Clear.
        clear();

        // Count.
        typename 
        std::iterator_traits<Tinput_itr>::difference_type 
            count = std::distance(from, to);
        if (count < decltype(count)(1)) {
            return;
        }

        std::vector<value_type> values;
        values.reserve(count);
        while (from != to) {
            values.emplace_back(std::forward<Tfunc>(func)(*from));
            from++;
        }

        root_ = 
        init_recursive({
                &values[0], 
                &values[0] + values.size()
            });
    }

    /**
     * @brief Clear.
     */
    void clear()
    {
        deallocate_recursive(root_);
        root_ = nullptr;
        node_count_ = 0;
    }

public:

    /**
     * @name Accessors
     */
    /**@{*/

    /**
     * @brief Node allocator.
     */
    const node_allocator_type& node_alloc() const
    {
        return node_alloc_;
    }

    /**
     * @brief Node count.
     */
    size_type node_count() const
    {
        return node_count_;
    }

    /**
     * @brief Root.
     */
    const node_type* root() const
    {
        return root_;
    }

    /**@}*/

private:

    /**
     * @brief Node allocator.
     */
    node_allocator_type node_alloc_;

    /**
     * @brief Node count.
     */
    size_type node_count_ = 0;

    /**
     * @brief Root.
     */
    node_type* root_ = nullptr;

private:

    /**
     * @brief Allocate.
     */
    node_type* allocate()
    {
        node_count_++;
        return node_alloc_.allocate(1);
    }

    /**
     * @brief Deallocate.
     */
    void deallocate(node_type* node)
    {
        if (node) {
            node->value.~value_type();
            node_count_++;
            node_alloc_.deallocate(node, 1);
        }
    }

    /**
     * @brief Deallocate recursively.
     */
    void deallocate_recursive(node_type* node)
    {
        if (node) {
            // Recurse.
            deallocate_recursive(node->left);
            deallocate_recursive(node->right);

            // Deallocate.
            deallocate(node);
        }
    }

    /**
     * @brief Initialize recursively.
     */
    node_type* init_recursive(iterator_range<value_type*> values)
    {
        if (values.empty()) {
            return nullptr;
        }

        aabb<Tfloat, N> box;
        for (const value_type& value : values) {
            box |= value.first;
        }

        // Split dimension.
        size_type split_dim = box.diag().argmax();

        // Split element.
        value_type* split =
                values.begin() +
                values.size() / 2;

        if (values.size() > 1) {

            // Partition.
            std::nth_element(
                    values.begin(), split,
                    values.end(),
                    [=](const value_type& value0,
                        const value_type& value1) -> bool {
                        return value0.first[split_dim] <
                               value1.first[split_dim];
                    });

        }

        // Allocate node.
        node_type* node = allocate();

        // Initialize node. 
        *node = {
            *split,
            nullptr,
            nullptr,
            std::uint8_t(split_dim)
        };

        // Recurse to initialize left child.
        node->left = 
        init_recursive({values.begin(), split});

        // Recurse to initialize right child.
        node->right = 
        init_recursive({split + 1, values.end()});

        return node;
    }

public:

    /**
     * @brief Nearest neighbor.
     *
     * @param[in] point
     * Reference point.
     */
    const node_type* nearest(const point_type& point) const
    {
        const node_type* near = nullptr;
        float_type near_dist2 = pr::numeric_limits<float_type>::infinity();
        if (root_) {
            nearest_recursive(
                    point, 
                    root_, 
                    near, near_dist2);
        }
        return near;
    }

private:

#if !DOXYGEN

    /**
     * @brief Nearest neighbor, recursive algorithm.
     */
    void nearest_recursive(
                const point_type& point,
                const node_type* node,
                const node_type*& near, float_type& near_dist2) const
    {
        point_type diff = node->value.first - point;

        // Possibly update nearest node.
        if (near_dist2 > pr::dot(diff, diff)) {
            near_dist2 = pr::dot(diff, diff);
            near = node;
        }

        // Signed distance to split plane.
        float_type min_dist = diff[node->split_dim];
        if (node->left && 
                !(min_dist < float_type(0) &&
                  min_dist * min_dist > near_dist2)) {

            // Recurse only if necessary.
            nearest_recursive(
                    point, 
                    node->left, 
                    near, near_dist2);
        }
        if (node->right && 
                !(min_dist > float_type(0) &&
                  min_dist * min_dist > near_dist2)) {

            // Recurse only if necessary.
            nearest_recursive(
                    point, 
                    node->right, 
                    near, near_dist2);
        }
    }
#endif // #if !DOXYGEN

public:

    /**
     * @brief Visit nearby nodes.
     *
     * TODO
     */
    template <typename Tfunc>
    bool nearby(
            const point_type& point,
            float_type cutoff_dist,
            Tfunc&& func) const
    {
        bool result = true;
        float_type cutoff_dist2 = cutoff_dist * cutoff_dist;
        if (root_) {
            result = 
            nearby_recursive(
                    point, cutoff_dist2,
                    std::forward<Tfunc>(func),
                    root_);
        }
        return result;
    }

private:

#if !DOXYGEN

    /**
     * @brief Visit nearby nodes, recursive algorithm.
     */
    template <typename Tfunc>
    bool nearby_recursive(
            const point_type& point, 
            float_type cutoff_dist2,
            Tfunc&& func,
            const node_type* node) const
    {
        point_type diff = node->value.first - point;

        // Process node if within cutoff distance.
        if (cutoff_dist2 > 
            pr::dot(diff, diff)) {
            if (!std::forward<Tfunc>(func)(node)) {
                return false;
            }
        }

        // Signed distance to split plane.
        float_type min_dist = diff[node->split_dim];
        if (node->left && 
                !(min_dist < float_type(0) &&
                  min_dist * min_dist > cutoff_dist2)) {

            // Recurse only if necessary.
            if (!nearby_recursive(
                    point, 
                    cutoff_dist2, 
                    std::forward<Tfunc>(func),
                    node->left)) {
                return false;
            }
        }
        if (node->right && 
                !(min_dist > float_type(0) &&
                  min_dist * min_dist > cutoff_dist2)) {

            // Recurse only if necessary.
            if (!nearby_recursive(
                    point, 
                    cutoff_dist2,
                    std::forward<Tfunc>(func),
                    node->right)) {
                return false;
            }
        }

        return true;
    }

#endif // #if !DOXYGEN
};

/**@}*/

} // namespace pr

#endif // #ifndef PREFORM_KDTREE_HPP
