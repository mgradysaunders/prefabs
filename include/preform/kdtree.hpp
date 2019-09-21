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

    /**
     * @brief Node/distance-squared pair type.
     */
    typedef std::pair<const node_type*, float_type> node_dist2_pair_type;

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
        std::uint32_t split_dim;
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
            std::uint32_t(split_dim)
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
    node_dist2_pair_type nearest(const point_type& point) const
    {
        nearest_recursive_info1 info;
        info.point = point;
        info.near.first = nullptr;
        info.near.second = pr::numeric_limits<float_type>::infinity();
        if (root_) {
            nearest_recursive(info, root_);
        }
        return info.near;
    }

private:

#if !DOXYGEN

    /**
     * @brief Nearest neighbor, recursive algorithm info.
     */
    struct nearest_recursive_info1
    {
        /**
         * @brief Reference point.
         */
        point_type point;

        /**
         * @brief Nearest node/distance-squared pair.
         */
        node_dist2_pair_type near;
    };

    /**
     * @brief Nearest neighbor, recursive algorithm.
     *
     * @param[inout] info
     * Info.
     *
     * @param[in] node
     * Current node.
     */
    static
    void nearest_recursive(
         nearest_recursive_info1& info, const node_type* node)
    {
        // Difference.
        point_type diff = node->value.first - info.point;

        // Squared distance to reference point.
        float_type dist2 = dot(diff, diff);

        // Possibly update nearest node.
        if (info.near.second > dist2) {
            info.near.second = dist2;
            info.near.first = node;
        }

        // Signed distance to split plane.
        float_type min_dist = diff[node->split_dim];
        float_type min_dist2 = min_dist * min_dist; 

        // If point is on left, process left child first.
        // If point is on right, process right child first.
        const node_type* child0 = node->left;
        const node_type* child1 = node->right;
        if (pr::signbit(min_dist)) {
            min_dist = -min_dist;
            std::swap(child0, child1);
        }

        if (child0) {
            // Recurse only if necessary. 
            if (!(min_dist < 0 &&
                  min_dist2 > info.near.second)) {
                nearest_recursive(info, child0);
            }
        }

        if (child1) {
            // Recurse only if necessary.
            if (!(min_dist > 0 &&
                  min_dist2 > info.near.second)) {
                nearest_recursive(info, child1);
            }
        }
    }

#endif // #if !DOXYGEN

public:

    /**
     * @brief Nearest neighbors.
     *
     * @param[in] point
     * Reference point.
     *
     * @param[out] near
     * Nearest node/distance-squared pairs range.
     *
     * @param[out] near_end
     * Nearest node/distance-squared pairs range end.
     *
     * @return
     * Returns the effective end of the nearest 
     * node/distance-squared pair range,
     * - if `near_end - near <= node_count_`, returns `near_end`,
     * - if `near_end - near > node_count_`, returns `near + node_count_`.
     *
     * @note
     * The implementation sorts node/distance-squared pairs in 
     * ascending order by distance-squared to the reference point.
     *
     * @throw std::invalid_argument
     * Unless 
     * `near` is non-null, `near_end` is non-null, and
     * `near` is strictly less than `near_end`.
     */
    node_dist2_pair_type* nearest(
            const point_type& point,
            node_dist2_pair_type* near,
            node_dist2_pair_type* near_end) const
    {
        if (near == nullptr || near_end == nullptr ||
            near >= near_end) {
            throw std::invalid_argument(__PRETTY_FUNCTION__);
        }

        nearest_recursive_info2 info;
        info.point = point;
        info.near = near;
        info.near_end = near_end;
        info.near_top = near;
        if (root_) {
            nearest_recursive(info, root_);
        }
        if (info.near_top > 
            info.near) {
            std::sort_heap(
                    info.near,
                    info.near_top,
                    [](const node_dist2_pair_type& lhs,
                       const node_dist2_pair_type& rhs) -> bool {
                        return lhs.second < rhs.second;
                    });
        }
        return info.near_top;
    }

private:

#if !DOXYGEN

    /**
     * @brief Nearest neighbors, recursive algorithm info.
     */
    struct nearest_recursive_info2 {

        /**
         * @brief Reference point.
         */
        point_type point;

        /**
         * @brief Nearest node/distance-squared pairs heap.
         */
        node_dist2_pair_type* near;

        /**
         * @brief Nearest node/distance-squared pairs heap end.
         */
        node_dist2_pair_type* near_end;

        /**
         * @brief Nearest node/distance-squared pairs heap top.
         */
        node_dist2_pair_type* near_top;
    };

    /**
     * @brief Nearest neighbors, recursive algorithm.
     *
     * @param[inout] info
     * Info.
     *
     * @param[in] node
     * Current node.
     */
    static
    void nearest_recursive(
         nearest_recursive_info2& info, const node_type* node)
    {
        // Compare operator.
        constexpr auto near_cmp = 
        [](const node_dist2_pair_type& lhs,
           const node_dist2_pair_type& rhs) -> bool {
            return lhs.second < rhs.second;
        };

        // Difference.
        point_type diff = node->value.first - info.point;

        // Squared distance to reference point.
        float_type dist2 = dot(diff, diff);

        // Signed distance to split plane.
        float_type min_dist = diff[node->split_dim];
        float_type min_dist2 = min_dist * min_dist;

        // If point is on left, process left child first.
        // If point is on right, process right child first.
        const node_type* child0 = node->left;
        const node_type* child1 = node->right;
        if (pr::signbit(min_dist)) {
            min_dist = -min_dist;
            std::swap(child0, child1);
        }

        if (info.near_top != info.near_end) {
            
            // Push.
            *info.near_top++ = std::make_pair(node, dist2);
            std::push_heap(
                    info.near, 
                    info.near_top,
                    near_cmp);
        }
        else if (info.near->second > dist2) {

            // Replace furthest node.
            std::pop_heap(
                    info.near,
                    info.near_top,
                    near_cmp);
            *(info.near_top - 1) = std::make_pair(node, dist2);
            std::push_heap(
                    info.near,
                    info.near_top,
                    near_cmp);
        }

        if (child0) {
            // Recurse only if
            // 1. the pairs heap is not full;
            // 2. the pairs heap is full, and we cannot cull the subtree
            // because a) the point is on the same side as child0 or
            // b) the minimum distance to the split plane is not greater 
            // than the distance to the furthest node.
            if (info.near_top != 
                info.near_end ||
                    !(min_dist < 0 &&
                      min_dist2 > info.near->second)) {

                nearest_recursive(info, child0);
            }
        }

        if (child1) {
            // Recurse only if
            // 1. the pairs heap is not full;
            // 2. the pairs heap is full, and we cannot cull the subtree
            // because a) the point is on the same side as child1 or
            // b) the minimum distance to the split plane is not greater 
            // than the distance to the furthest node.
            if (info.near_top != 
                info.near_end ||
                    !(min_dist > 0 && 
                      min_dist2 > info.near->second)) {

                nearest_recursive(info, child1);
            }
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
        // Difference.
        point_type diff = node->value.first - point;

        // Squared distance to reference point.
        float_type dist2 = dot(diff, diff);

        // Process node if within cutoff distance.
        if (cutoff_dist2 > dist2) {
            if (!std::forward<Tfunc>(func)(node)) {
                return false;
            }
        }

        // Signed distance to split plane.
        float_type min_dist = diff[node->split_dim];
        float_type min_dist2 = min_dist * min_dist; 

        // If point is on left, process left child first.
        // If point is on right, process right child first.
        const node_type* child0 = node->left;
        const node_type* child1 = node->right;
        if (min_dist < 0) {
            min_dist = -min_dist;
            std::swap(child0, child1);
        }

        if (child0) {
            // Recurse only if necessary. 
            if (!(min_dist < 0 &&
                  min_dist2 > cutoff_dist2)) {

                if (!nearby_recursive(
                        point, 
                        cutoff_dist2, 
                        std::forward<Tfunc>(func),
                        child0)) {
                    return false;
                }
            }
        }
        if (child1) {
            // Recurse only if necessary. 
            if (!(min_dist > 0 &&
                  min_dist2 > cutoff_dist2)) {

                // Recurse only if necessary.
                if (!nearby_recursive(
                        point, 
                        cutoff_dist2,
                        std::forward<Tfunc>(func),
                        child1)) {
                    return false;
                }
            }
        }

        return true;
    }

#endif // #if !DOXYGEN
};

/**@}*/

} // namespace pr

#endif // #ifndef PREFORM_KDTREE_HPP
