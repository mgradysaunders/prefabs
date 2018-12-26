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
#ifndef PREFABS_AABBTREE_HPP
#define PREFABS_AABBTREE_HPP

// for assert
#include <cassert>

// for std::allocator, std::allocator_traits
#include <memory>

// for std::vector
#include <vector>

// for pr::range
#include <prefabs/range.hpp>

// for pr::multi
#include <prefabs/multi.hpp>

// for pr::aabb
#include <prefabs/aabb.hpp>

namespace pr {

/**
 * @defgroup aabbtree Axis-aligned bounding box tree
 *
 * `<prefabs/aabbtree.hpp>`
 */
/**@{*/

/**
 * @brief Axis-aligned bounding box tree split mode.
 */
enum class aabbtree_split_mode
{
    /**
     * @brief Split by equal dimensions.
     */
    equal_dimensions,

    /**
     * @brief Split by equal counts.
     */
    equal_counts,

    /**
     * @brief Split by surface area heuristic.
     */
    surface_area_heuristic
};

/**
 * @brief Axis-aligned bounding box tree.
 */
template <
    typename Tfloat, std::size_t N,
    typename Tnode_alloc = std::allocator<char>
    >
class aabbtree
{
public:

    // Sanity check.
    static_assert(
        std::is_floating_point<Tfloat>::value,
        "Tfloat must be floating point");

public:

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
    class node_type;
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

    /**
     * @brief Node type.
     */
    class node_type
    {
    public:

        /**
         * @brief Constructor.
         *
         * @param[in] box
         * Box.
         *
         * @param[in] node_alloc
         * Node allocator.
         *
         * @param[in] node0
         * If tree, node.
         *
         * @param[in] node1
         * If tree, node.
         *
         * @param[in] split_dim
         * If tree, split dimension.
         *
         * @param[in] first_index
         * If leaf, first value index.
         *
         * @param[in] count
         * If leaf, value count.
         */
        node_type(
            const aabb_type& box,
            node_allocator_type& node_alloc,
            node_type* node0,
            node_type* node1,
            size_type split_dim,
            size_type first_index,
            size_type count) :
                box_(box),
                node_alloc_(node_alloc),
                nodes_{
                    node0, 
                    node1},
                split_dim_(split_dim),
                first_index_(first_index),
                count_(count)
        {
        #if !NDEBUG
            bool is_valid_tree = 
                // Nodes non-null.
                nodes_[0] &&
                nodes_[1] &&
                // Split dimension is valid.
                split_dim_ < N &&
                // Count zero.
                !count_;

            bool is_valid_leaf =
                // Nodes null.
                !nodes_[0] &&
                !nodes_[1] &&
                // Count non-zero.
                count_;

            assert(is_valid_tree != 
                   is_valid_leaf);
        #endif // #if !NDEBUG
        }

        /**
         * @brief Destructor.
         */
        ~node_type() 
        {
            if (nodes_[0]) {
                assert(
                    nodes_[0] && 
                    nodes_[1]);
                for (size_type i = 0; i < 2; i++) {
                    // Destroy.
                    node_allocator_traits::destroy(
                    node_alloc_, nodes_[i]);

                    // Deallocate.
                    node_allocator_traits::deallocate(
                    node_alloc_, nodes_[i], 1);
                }
            }
        }

    public:

        /**
         * @name Traversal
         */
        /**@{*/

        /**
         * @brief Is leaf?
         */
        bool is_leaf() const noexcept
        {
            return count_ != 0;
        }

        /**
         * @brief Is tree?
         */
        bool is_tree() const noexcept
        {
            return count_ == 0;
        }

        /**
         * @brief Box.
         */
        const aabb_type& box() const noexcept
        {
            return box_;
        }

        /**
         * @brief If tree, node.
         *
         * @throw std::runtime_error
         * If `!is_tree()`.
         */
        const node_type& operator[](size_type pos) const
        {
            if (!is_tree()) {
                throw std::runtime_error(__PRETTY_FUNCTION__);
            }
            if (!(pos < 2)) {
                throw std::invalid_argument(__PRETTY_FUNCTION__);
            }
            return *nodes_[pos];
        }

        /**
         * @brief If tree, split dimension.
         *
         * @throw std::runtime_error
         * If `!is_tree()`.
         */
        size_type split_dim() const
        {
            if (!is_tree()) {
                throw std::runtime_error(__PRETTY_FUNCTION__);
            }
            return split_dim_;
        }

        /**
         * @brief If leaf, first value index.
         *
         * @throw std::runtime_error
         * If `!is_leaf()`.
         */
        size_type first_index() const
        {
            if (!is_leaf()) {
                throw std::runtime_error(__PRETTY_FUNCTION__);
            }
            return first_index_;
        }

        /**
         * @brief If leaf, value count.
         *
         * @throw std::runtime_error
         * If `!is_leaf()`.
         */
        size_type count() const
        {
            if (!is_leaf()) {
                throw std::runtime_error(__PRETTY_FUNCTION__);
            }
            return count_;
        }

        /**@}*/

    private:

        /**
         * @brief Box.
         */
        aabb_type box_;

        /**
         * @brief Node allocator.
         */
        node_allocator_type& node_alloc_;

        /**
         * @brief If tree, nodes.
         */
        node_type* nodes_[2];

        /**
         * @brief If tree, split dimension.
         */
        size_type split_dim_;

        /**
         * @brief If leaf, first value index.
         */
        size_type first_index_;

        /**
         * @brief If leaf, value count.
         */
        size_type count_;
    };

    /**
     * @brief Proxy type.
     */
    class proxy_type
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

public:

    /**
     * @brief Constructor.
     *
     * @param[in] leaf_max
     * Leaf max.
     *
     * @param[in] split_mode
     * Split mode.
     *
     * @param[in] node_alloc
     * Node allocator.
     */
    aabbtree(
        size_type leaf_max = 4,
        aabbtree_split_mode split_mode = 
        aabbtree_split_mode::equal_counts,
        const node_allocator_type& node_alloc =
              node_allocator_type()) :
            leaf_max_(leaf_max),
            split_mode_(split_mode),
            node_alloc_(node_alloc)
    {
    }

    /**
     * @brief Destructor.
     */
    ~aabbtree() 
    {
        // Clear.
        clear();
    }

public:

    /**
     * @brief Clear.
     */
    void clear()
    {
        // Clear.
        proxies_.clear();
        proxies_.shrink_to_fit();
        if (root_) {
            // Destroy.
            node_allocator_traits::destroy(
            node_alloc_, root_);

            // Deallocate.
            node_allocator_traits::deallocate(
            node_alloc_, root_, 1);
            root_ = nullptr;
        }
    }

    /**
     * @brief Initialize.
     *
     * @param[in] values_begin
     * Values begin.
     *
     * @param[in] values_end
     * Values end.
     *
     * @param[in] func
     * Function.
     *
     * @note
     * Function must have signature 
     * equivalent to
     * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~{cpp}
     * aabb_type(const value_type&)
     * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~
     */
    template <typename Tforward_itr, typename Tfunc>
    size_type init(
            Tforward_itr values_begin,
            Tforward_itr values_end,
            Tfunc&& func)
    {
        // Clear.
        clear();

        // Count.
        size_type values_size = 
        std::distance(
                values_begin, 
                values_end);
        if (values_size == 0) {
            return 0;
        }
        proxies_.reserve(values_size);

        // Initialize proxies.
        size_type value_index = 0;
        while (values_begin != values_end) {
            aabb_type box = 
                std::forward<Tfunc>(func)(*values_begin);
            proxies_.emplace_back(
            proxy_type{
                box,
                box.center(),
                value_index
            });
            ++values_begin;
            ++value_index;
        }

        // Build.
        size_type total = 0;
        size_type first_index = 0;
        root_ = 
        init_recursive(
            total,
            first_index,
            {&proxies_[0], 
             &proxies_[0] + proxies_.size()});
        assert(first_index == proxies_.size());
        return total;
    }

    /**
     * @brief Root.
     *
     * @throw std::runtime_error
     * If `root_` is not properly initialized.
     */
    const node_type& root() const
    {
        if (!root_) {
            throw std::runtime_error(__PRETTY_FUNCTION__);
        }
        return *root_;
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
     * @name Build parameters
     */
    /**@{*/

    /**
     * @brief Leaf max.
     */
    size_type leaf_max_;

    /**
     * @brief Split mode.
     */
    aabbtree_split_mode split_mode_;

    /**@}*/

    /**
     * @brief Node allocator.
     */
    node_allocator_type node_alloc_;

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
     * @brief Initialize, recursive helper function.
     *
     * @param[inout] total
     * Total nodes.
     *
     * @param[inout] first_index
     * First index.
     *
     * @param[in] proxies
     * Proxies.
     */
    node_type* init_recursive(
            size_type& total,
            size_type& first_index,
            range<proxy_type*> proxies)
    {
        // Allocate.
        node_type* node = 
        node_allocator_traits::allocate(
        node_alloc_, 1);

        // Increment total.
        total++;

        // Surround boxes.
        aabb_type box;
        for (const proxy_type& proxy : proxies) {
            box |= proxy.box;
        }

        // Leaf?
        assert(proxies.size() > 0);
        if (size_type(proxies.size()) <= leaf_max_) {
            
            // Construct.
            node_allocator_traits::construct(
            node_alloc_, node, 
                box,
                node_alloc_,
                nullptr,
                nullptr,
                0,
                first_index,
                proxies.size());

            // Increment first index.
            first_index += proxies.size();
        }
        else {

            // Surround box centers.
            aabb_type box_center;
            for (const proxy_type& proxy : proxies) {
                box_center |= proxy.box_center;
            }

            // Split dimension.
            size_type split_dim = 
                        box_center.diag().argmax();

            // Split.
            proxy_type* split = 
                proxies.begin() +
                proxies.size() / 2;
            switch (split_mode_) {

                default:
                case aabbtree_split_mode::equal_dimensions: {
                    // Partition by equal dimensions.
                    float_type cen = 
                        (box_center[0][split_dim] + 
                         box_center[1][split_dim]) / 2;
                    split = 
                    std::partition(
                        proxies.begin(),
                        proxies.end(),
                        [=](const proxy_type& proxy) {
                            return proxy.box_center[split_dim] < cen;
                        });

                    // Partition successful?
                    if (!(split == proxies.begin() ||
                          split == proxies.end())) {
                        break;
                    }

                    // Fall through.
                    [[fallthrough]];
                }

                case aabbtree_split_mode::equal_counts: {
                    // Paritition by equal counts.
                    split = proxies.begin() + 
                            proxies.size() / 2;
                    std::nth_element(
                        proxies.begin(), split,
                        proxies.end(),
                        [=](const proxy_type& proxy0,
                            const proxy_type& proxy1) {
                            return proxy0.box_center[split_dim] <
                                   proxy1.box_center[split_dim];
                        });
                    break;
                }

                case aabbtree_split_mode::surface_area_heuristic: {
                    constexpr size_type Nbins = 8;
                    struct bin {
                        aabb_type box = {};
                        size_type count = 0;
                    };
                    bin bins[Nbins] = {};

                    // Initialize bins.
                    for (const proxy_type& proxy : proxies) {
                        float_type cen = proxy.box_center[split_dim];
                        float_type cenmin = box_center[0][split_dim];
                        float_type cenmax = box_center[1][split_dim];
                        size_type pos = Nbins * 
                                 (cen - cenmin) / (cenmax - cenmin);
                        if (pos == Nbins) {
                            pos =  Nbins - 1;
                        }
                        bins[pos].box |= proxy.box;
                        bins[pos].count++;
                    }

                    // Compute costs.
                    float_type costs[Nbins - 1];
                    for (size_type pos = 0; 
                                   pos < Nbins - 1; pos++) {
                        bin bin0 = {};
                        for (size_type pos0 = 0; 
                                       pos0 < pos + 1; pos0++) {
                            bin0.box |= bins[pos0].box;
                            bin0.count += bins[pos0].count;
                        }
                        bin bin1 = {};
                        for (size_type pos1 = pos + 1; 
                                       pos1 < Nbins; pos1++) {
                            bin1.box |= bins[pos1].box;
                            bin1.count += bins[pos1].count;
                        }
                        costs[pos] = 
                            (bin0.box.surface_area() * bin0.count +
                             bin1.box.surface_area() * bin1.count);
                    }

                    // Compute costs argmin.
                    size_type costs_argmin =
                        std::distance(
                            &costs[0],
                            std::min_element(
                                &costs[0],
                                &costs[0] + Nbins - 1));

                    // Partition by surface area heuristic.
                    split = 
                    std::partition(
                        proxies.begin(),
                        proxies.end(),
                        [=](const proxy_type& proxy) {
                            float_type cen = proxy.box_center[split_dim];
                            float_type cenmin = box_center[0][split_dim];
                            float_type cenmax = box_center[1][split_dim];
                            size_type pos = Nbins * 
                                     (cen - cenmin) / (cenmax - cenmin);
                            if (pos == Nbins) {
                                pos =  Nbins - 1;
                            }
                            return pos <= costs_argmin;
                        });

                    // Partition unsuccessful?
                    if (split == proxies.begin() ||
                        split == proxies.end()) {
                        // Paritition by equal counts.
                        split = proxies.begin() + 
                                proxies.size() / 2;
                        std::nth_element(
                            proxies.begin(), split,
                            proxies.end(),
                            [=](const proxy_type& proxy0,
                                const proxy_type& proxy1) {
                                return proxy0.box_center[split_dim] <
                                       proxy1.box_center[split_dim];
                            });
                    }

                    break;
                }
            }

            // Construct.
            node_allocator_traits::construct(
            node_alloc_, node,
                box,
                node_alloc_,
                init_recursive(
                    total, 
                    first_index, 
                    {proxies.begin(), split}),
                init_recursive(
                    total, 
                    first_index,
                    {split, proxies.end()}),
                split_dim,
                size_type(-1),
                size_type(0));
        }
        return node;
    }
};

/**@}*/


} // namespace pr

#endif // #ifndef PREFABS_AABBTREE_HPP
