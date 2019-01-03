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
#error "prefabs/aabbtree.hpp requires >=C++17"
#endif // #if !(__cplusplus >= 201703L)
#endif // #if !DOXYGEN
#pragma once
#ifndef PREFABS_AABBTREE_HPP
#define PREFABS_AABBTREE_HPP

// for assert
#include <cassert>

// for std::array
#include <array>

// for std::allocator, std::allocator_traits
#include <memory>

// for std::vector
#include <vector>

// for std::pair
#include <utility>

// for std::atomic
#include <atomic>

// for std::thread
#include <thread>

// for std::mutex
#include <mutex>

// for std::future, std::async
#include <future>

// for pr::range
#include <prefabs/range.hpp>

// for pr::aabb, pr::multi
#include <prefabs/aabb.hpp>

namespace pr {

/**
 * @defgroup aabbtree Axis-aligned bounding box tree
 *
 * `<prefabs/aabbtree.hpp>`
 *
 * __C++ version__: >=C++17
 */
/**@{*/

/**
 * @brief Axis-aligned bounding box tree.
 */
template <
    typename Tfloat, std::size_t N,
    typename Tsplit_policy,
    typename Tnode_alloc = std::allocator<char>
    >
class aabbtree
{
public:

    // Sanity check.
    static_assert(
        std::is_floating_point<Tfloat>::value,
        "Tfloat must be floating point");

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
     * @brief Size type.
     */
    typedef std::size_t size_type;

#if !DOXYGEN
    // prototype
    struct node_type;
#endif // #if !DOXYGEN

    /**
     * @brief Node allocator type.
     */
    typedef typename std::allocator_traits<Tnode_alloc>::
            template rebind_alloc<node_type> node_allocator_type;

    /**
     * @brief Node allocator traits.
     */
    typedef typename std::allocator_traits<Tnode_alloc>::
            template rebind_traits<node_type> node_allocator_traits;

    /**@}*/

public:

    /**
     * @brief Proxy type.
     */
    struct proxy_type
    {
    public:

        /**
         * @brief Box.
         */
        aabb_type box;

        /**
         * @brief Box center.
         */
        multi_type box_center;

        /**
         * @brief Value index.
         */
        size_type value_index;
    };

    /**
     * @brief Node type.
     */
    struct node_type
    {
    public:

        /**
         * @brief Box.
         */
        aabb_type box;

        /**
         * @brief If branch, left child. 
         */
        node_type* left;

        /**
         * @brief If branch, right child.
         */
        node_type* right;

        /**
         * @brief If branch, split dimension.
         */
        size_type split_dim;

        /**
         * @brief If leaf, first index.
         */
        size_type first_index;

        /**
         * @brief If leaf, count.
         */
        size_type count;
    };

public:

    /**
     * @brief Constructor.
     */
    aabbtree(
        size_type leaf_cutoff = 8,
        const node_allocator_type& node_alloc = 
              node_allocator_type()) :
            leaf_cutoff_(leaf_cutoff),
            node_alloc_(node_alloc)
    {
    }

    /**
     * @brief Non-copyable.
     */
    aabbtree(const aabbtree&) = delete;

    /**
     * @brief Destructor.
     */
    ~aabbtree()
    {
        clear();
    }

public:

    /**
     * @brief Initialize.
     */
    template <typename Tforward_itr, typename Tfunc>
    void init(
            Tforward_itr from,
            Tforward_itr to,
            Tfunc&& func)
    {
        // Clear.
        clear();

        // Count.
        size_type count = std::distance(from, to);
        if (count < 1) {
            return;
        }

        // Initialize proxies.
        size_type value_index = 0;
        while (from != to) {
            aabb_type box = std::forward<Tfunc>(func)(*from);
            proxies_.emplace_back(
            proxy_type{
                box,
                box.center(),
                value_index
            });
            ++from;
            ++value_index;
        }

        // Initialize.
        std::atomic<size_type> total_branches = 0;
        std::atomic<size_type> total_leaves = 0;
        size_type first_index = 0;
        root_ = 
        init_recursive(
            total_branches,
            total_leaves,
            first_index,
            {&proxies_[0],
             &proxies_[0] + proxies_.size()});
        assert(first_index == proxies_.size());
        // TODO Output info?
    }

    /**
     * @brief Clear.
     */
    void clear()
    {
        // Destroy root.
        deallocate_recursive(root_);
        root_ = nullptr;

        // Destroy proxies.
        proxies_.clear();
        proxies_.shrink_to_fit();
    }

public:

    /**
     * @brief Root.
     */
    const node_type* root() const
    {
        return root_;
    }

    /**
     * @brief Proxies.
     */
    const std::vector<proxy_type>& proxies() const
    {
        return proxies_;
    }

private:

    /**
     * @brief Leaf cutoff.
     */
    size_type leaf_cutoff_ = 8;

    /**
     * @brief Node allocator.
     */
    node_allocator_type node_alloc_;

    /**
     * @brief Node allocator mutual exclusion lock.
     */
    std::mutex node_alloc_mutex_;

    /**
     * @brief Root.
     */
    node_type* root_ = nullptr;

    /**
     * @brief Proxies.
     */
    std::vector<proxy_type> proxies_;

private:

    /**
     * @brief Allocate.
     */
    node_type* allocate()
    {
        std::unique_lock<std::mutex> lock(node_alloc_mutex_);
        return 
            node_allocator_traits::allocate(
            node_alloc_, 1);
    }

    /**
     * @brief Deallocate.
     */
    void deallocate(node_type* node) 
    {
        if (node) {
            std::unique_lock<std::mutex> lock(node_alloc_mutex_);
            node_allocator_traits::deallocate(
            node_alloc_, node, 1);
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
    node_type* init_recursive(
            std::atomic<size_type>& total_branches,
            std::atomic<size_type>& total_leaves,
            size_type& first_index,
            range<proxy_type*> proxies)
    {
        // Allocate.
        node_type* node = allocate();

        // Surround boxes and box centers.
        aabb_type box;
        aabb_type box_center;
        for (const proxy_type& proxy : proxies) {
            box |= proxy.box;
            box_center |= proxy.box_center;
        }

        // Proxies count.
        size_type count = proxies.size();
        assert(count);

        if (count <= leaf_cutoff_) {
            // Initialize leaf.
            *node = node_type{
                box,
                nullptr,
                nullptr,
                size_type(0),
                first_index,
                count
            };

            // Shift first index.
            first_index += count;

            // Increment.
            total_leaves++;
        }
        else {

            // Split dimension.
            size_type split_dim = 
                        box_center.diag().argmax();

            // Split.
            proxy_type* split = Tsplit_policy()(
                    box,
                    box_center,
                    split_dim,
                    proxies);
            assert(split && 
                   proxies.begin() <= split &&
                   proxies.end() > split);

            // Children.
            node_type* left;
            node_type* right;

            // Count sufficiently small?
            if (count <= 1024) {

                // Recurse.
                left = 
                init_recursive(
                    total_branches,
                    total_leaves,
                    first_index,
                    {proxies.begin(), split});

                // Recurse.
                right = 
                init_recursive(
                    total_branches,
                    total_leaves,
                    first_index,
                    {split, proxies.end()});
            }
            else {

                // Recurse async.
                size_type first_index_left = first_index;
                first_index += std::distance(proxies.begin(), split);
                std::future<node_type*> future_left =
                std::async([&]() {
                    return init_recursive(
                                total_branches,
                                total_leaves,
                                first_index_left,
                                {proxies.begin(), split});
                });

                // Recurse.
                right = 
                init_recursive(
                    total_branches,
                    total_leaves,
                    first_index,
                    {split, proxies.end()});

                // Wait.
                left = future_left.get();
            }

            // Initialize.
            *node = node_type{
                box,
                left,
                right,
                split_dim,
                size_type(0),
                size_type(0)
            };

            // Increment.
            total_branches++;
        }
        return node;
    }
};

/**
 * @brief Split by equal counts.
 */
struct aabbtree_split_equal_counts
{
    /**
     * @brief Split.
     */
    template <
        typename Tfloat, std::size_t N,
        typename Tproxy
        >
    Tproxy* operator()(
            const aabb<Tfloat, N>& box,
            const aabb<Tfloat, N>& box_center,
            const std::size_t split_dim,
            range<Tproxy*> proxies) const
    {
        (void) box;
        (void) box_center;

        Tproxy* split = 
                proxies.begin() + 
                proxies.size() / 2;

        // Partition.
        std::nth_element(
                proxies.begin(), split,
                proxies.end(),
                [=](const Tproxy& proxy0,
                    const Tproxy& proxy1) -> bool {
                    return proxy0.box_center[split_dim] <
                           proxy1.box_center[split_dim];
                });

        return split;        
    }
};

/**
 * @brief Split by equal dimensions.
 */
struct aabbtree_split_equal_dimensions
{
    /**
     * @brief Split.
     */
    template <
        typename Tfloat, std::size_t N,
        typename Tproxy
        >
    Tproxy* operator()(
            const aabb<Tfloat, N>& box,
            const aabb<Tfloat, N>& box_center,
            const std::size_t split_dim,
            range<Tproxy*> proxies) const
    {
        Tfloat cen = 
            (box_center[0][split_dim] +
             box_center[1][split_dim]) / 2;

        // Partition.
        Tproxy* split =
            std::partition(
                proxies.begin(),
                proxies.end(),
                [=](const Tproxy& proxy) {
                    return proxy.box_center[split_dim] < cen;
                });

        // Partition successful?
        if (split != proxies.begin() &&
            split != proxies.end()) {
            return split;
        }
        else {
            // Default to equal counts.
            return aabbtree_split_equal_counts()(
                        box, 
                        box_center, 
                        split_dim, 
                        proxies);
        }
    }
};

/**
 * @brief Split by surface area heuristic.
 */
template <std::size_t Nbins = 8>
struct aabbtree_split_surface_area
{
    // Sanity check.
    static_assert(
        Nbins > 1,
        "Nbins must be greater than 1");

    /**
     * @brief Split.
     */
    template <
        typename Tfloat, std::size_t N,
        typename Tproxy
        >
    Tproxy* operator()(
            const aabb<Tfloat, N>& box,
            const aabb<Tfloat, N>& box_center,
            const std::size_t split_dim,
            range<Tproxy*> proxies) const
    {
        // Degenerate?
        if (box_center[0][split_dim] == 
            box_center[1][split_dim]) {
            // Default to equal counts.
            return aabbtree_split_equal_counts()(
                        box,
                        box_center,
                        split_dim,
                        proxies);
        }

        // Bin type.
        typedef std::pair<aabb<Tfloat, N>, std::size_t> bin;

        // Initialize bins.
        std::array<bin, Nbins> bins = {};
        for (const Tproxy& proxy : proxies) {
            Tfloat cen = proxy.box_center[split_dim];
            Tfloat cenmin = box_center[0][split_dim];
            Tfloat cenmax = box_center[1][split_dim];
            std::size_t pos =
            std::min<std::size_t>(
                Nbins * ((cen - cenmin) / (cenmax - cenmin)),
                Nbins - 1);

            bins[pos].first |= proxy.box;
            bins[pos].second++;
        }

        // Initialize sweeps.
        std::array<bin, Nbins - 1> lsweep;
        std::array<bin, Nbins - 1> rsweep; {
            auto itrlsweep = lsweep.begin(), itrlbins = bins.begin();
            auto itrrsweep = rsweep.rbegin(), itrrbins = bins.rbegin();
            *itrlsweep++ = *itrlbins++;
            *itrrsweep++ = *itrrbins++;
            for (; itrlsweep < lsweep.end();
                    ++itrlsweep, ++itrlbins,
                    ++itrrsweep, ++itrrbins) {
                itrlsweep->first = (itrlsweep - 1)->first | itrlbins->first;
                itrrsweep->first = (itrrsweep - 1)->first | itrrbins->first;
                itrlsweep->second = (itrlsweep - 1)->second + itrlbins->second;
                itrrsweep->second = (itrrsweep - 1)->second + itrrbins->second;
            }
        }

        // Compute costs.
        std::array<Tfloat, Nbins - 1> costs; {
            auto itrcosts = costs.begin();
            auto itrlsweep = lsweep.begin();
            auto itrrsweep = rsweep.begin();
            for (; itrrsweep < rsweep.end(); 
                    ++itrcosts,
                    ++itrlsweep,
                    ++itrrsweep) {
                *itrcosts =
                    (itrlsweep->first.surface_area() * itrlsweep->second +
                     itrrsweep->first.surface_area() * itrrsweep->second); 
            }
        }

        // Compute costs argmin.
        std::size_t costs_argmin = 
            std::distance(
                costs.begin(),
                std::min_element(
                    costs.begin(),
                    costs.end()));

        // Partition.
        Tproxy* split = 
            std::partition(
                proxies.begin(),
                proxies.end(),
                [=](const Tproxy& proxy) {
                    Tfloat cen = proxy.box_center[split_dim];
                    Tfloat cenmin = box_center[0][split_dim];
                    Tfloat cenmax = box_center[1][split_dim];
                    std::size_t pos =
                    std::min<std::size_t>(
                        Nbins * ((cen - cenmin) / (cenmax - cenmin)),
                        Nbins - 1);
                    return pos <= costs_argmin;
                });

        // Partition successful?
        if (split != proxies.begin() &&
            split != proxies.end()) {
            return split;
        }
        else {
            // Default to equal counts.
            return aabbtree_split_equal_counts()(
                        box,
                        box_center,
                        split_dim,
                        proxies);
        }
    }
};

/**@}*/

} // namespace pr

#endif // #ifndef PREFABS_AABBTREE_HPP
