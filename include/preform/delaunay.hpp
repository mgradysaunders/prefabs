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
#error "preform/delaunay.hpp requires >=C++17"
#endif // #if !(__cplusplus >= 201703L)
#endif // #if !DOXYGEN
#pragma once
#ifndef PREFORM_DELAUNAY_HPP
#define PREFORM_DELAUNAY_HPP

#include <cassert>
#include <vector>
#include <map>
#include <set>
#include <preform/float_interval.hpp>
#include <preform/memory_arena.hpp>
#include <preform/memory_arena_allocator.hpp>
#include <preform/multi.hpp>
#include <preform/multi_math.hpp>

namespace pr {

/**
 * @defgroup delaunay Delaunay triangulation (2-dimensional)
 *
 * `<preform/delaunay.hpp>`
 *
 * __C++ version__: >=C++17
 */
/**@{*/

// TODO doc
/**
 * @brief Delaunay triangulation (2-dimensional).
 *
 * @tparam T
 * Float type.
 *
 * @tparam Talloc
 * Allocator type.
 */
template <typename T, typename Talloc = std::allocator<char>>
class delaunay_triangulation
{
public:

    // Sanity check.
    static_assert(
        std::is_floating_point<T>::value,
        "T must be floating point");

    /**
     * @brief Float type.
     */
    typedef T float_type;

    /**
     * @brief Float interval type.
     */
    typedef float_interval<T> float_interval_type;

    /**
     * @brief Size type.
     */
    typedef std::size_t size_type;

    /**
     * @brief Difference type.
     */
    typedef std::ptrdiff_t difference_type;

    /**
     * @brief Index type.
     */
    typedef difference_type index_type;

    /**
     * @brief Bad index.
     */
    static constexpr index_type bad_index = -1;

    // TODO doc
    /**
     * @brief Edge type.
     */
    class edge_type
    {
    public:

        /**
         * @brief Index of point @f$ A @f$.
         */
        index_type a = bad_index;

        /**
         * @brief Index of point @f$ B @f$.
         */
        index_type b = bad_index;

        // TODO doc
        /**
         * @brief Compare less-than.
         */
        __attribute__((always_inline))
        bool operator<(const edge_type& other) const
        {
            auto [a0, b0] = std::minmax(a, b);
            auto [a1, b1] = std::minmax(other.a, other.b);
            return a0 < a1 || (a0 == a1 && b0 < b1);
        }

        // TODO doc
        /**
         * @brief Compare equal.
         */
        __attribute__((always_inline))
        bool operator==(const edge_type& other) const
        {
            auto [a0, b0] = std::minmax(a, b);
            auto [a1, b1] = std::minmax(other.a, other.b);
            return a0 == a1 && b0 == b1;
        }

        /**
         * @brief Reverse orientation.
         */
        edge_type reverse() const
        {
            return {b, a};
        }
    };

    /**
     * @brief Triangle type.
     */
    class triangle_type
    {
    public:

        /**
         * @brief Index of point @f$ A @f$.
         */
        index_type a = bad_index;

        /**
         * @brief Index of point @f$ B @f$.
         */
        index_type b = bad_index;

        /**
         * @brief Index of point @f$ C @f$.
         */
        index_type c = bad_index;

        /**
         * @brief Access operator.
         */
        index_type operator[](size_type k) const
        {
            // TODO test sizeof structure
            return (&a)[k];
        }
    };

    /**
     * @brief Edge triangle pair type.
     */
    class edge_triangle_pair_type
    {
    public:

        /**
         * @brief Index of triangle @f$ T_1 @f$.
         */
        index_type t1 = bad_index;

        /**
         * @brief Index of triangle @f$ T_2 @f$.
         */
        index_type t2 = bad_index;

        /**
         * @brief Is full?
         */
        bool is_full() const
        {
            return t1 != bad_index && t2 != bad_index;
        }

        /**
         * @brief Push triangle.
         */
        void push(index_type t)
        {
            assert(
                t1 == bad_index ||
                t2 == bad_index);

            if (t1 == bad_index) {
                t1 = t;
            }
            else { 
                t2 = t;
            }
        }

        /**
         * @brief Replace triangle.
         *
         * @param[in] tfrom
         * Index of triangle to replace, must be either
         * equal to either `t1` or `t2`.
         *
         * @param[in] tto
         * Index of replacement triangle.
         */
        void replace(index_type tfrom, index_type tto)
        {
            assert(
                t1 == tfrom ||
                t2 == tfrom);

            if (t1 == tfrom) {
                t1 = tto;
            }
            else { 
                t2 = tto;
            }
        }
    };

    // TODO allocator types

public:

    // TODO constructors

    // TODO doc
    template <typename Tinput_itr>
    void init(
            Tinput_itr from,
            Tinput_itr to)
    {
        // TODO clean-up this mess

        // Initialize points.
        points_.insert(
        points_.begin(), from, to);

        // Compute point center.
        multi<float_type, 2> point_center;
        for (const multi<float_type, 2>& point : points_) {
            point_center += point;
        }
        point_center /= 
        float_type(points_.size());

        // Compute square distances to center.
        std::vector<std::pair<float_type, index_type>> point_dists;
        point_dists.reserve(points_.size());
        for (const multi<float_type, 2>& point : points_) {
            point_dists.push_back({
                    dot(point - point_center,
                        point - point_center),
                    index_type(&point - &points_[0])
                });
        }

        // Sort.
        std::sort(
                point_dists.begin(),
                point_dists.end());

        // Erase points until first 3 form a valid triangle.
        while (point_dists.size() > 2) {
            if (compute_area(
                        point_dists[0].second, 
                        point_dists[1].second,
                        point_dists[2].second).contains(0)) {
                // TODO track discarded points?
                point_dists.erase(point_dists.begin());
            }
            else {
                break;
            }
        }

        if (point_dists.size() < 3) {
            // TODO error
        }

        triangle_type triangle = {
            point_dists[0].second,
            point_dists[1].second,
            point_dists[2].second
        };

        if (compute_area(
                    triangle.a, 
                    triangle.b, 
                    triangle.c).upper_bound() < 0) {
            std::swap(
                    triangle.b, 
                    triangle.c);
        }

        edge_type edge0 = {triangle.a, triangle.b};
        edge_type edge1 = {triangle.b, triangle.c};
        edge_type edge2 = {triangle.c, triangle.a};

        edge_to_triangles_[edge0] = {0, -1};
        edge_to_triangles_[edge1] = {0, -1};
        edge_to_triangles_[edge2] = {0, -1};

        boundary_edges_.insert(edge0);
        boundary_edges_.insert(edge1);
        boundary_edges_.insert(edge2);

        pr::memory_arena_allocator<char> alloc1;
        pr::memory_arena_allocator<char> alloc2;
        for (size_type k = 3; k < point_dists.size(); k++) {
            add_point(point_dists[k].second, alloc1, alloc2);
        }

        // TODO all triangles ccw
    }

    // TODO accessors

private:

    // TODO doc explain
    /**
     * @brief Compute signed area of triangle.
     *
     * @param[in] a
     * Index of point @f$ A @f$.
     *
     * @param[in] b
     * Index of point @f$ B @f$.
     *
     * @param[in] c
     * Index of point @f$ C @f$.
     */
    float_interval_type compute_area(
                index_type a,
                index_type b,
                index_type c) const
    {
        multi<float_interval_type, 2> pa = points_[a];
        multi<float_interval_type, 2> pb = points_[b];
        multi<float_interval_type, 2> pc = points_[c];
        return pr::cross(pb - pa, pc - pa);
    }

    // TODO doc explain
    /**
     * @brief Is edge visible from point?
     *
     * @param[in] p
     * Index of point @f$ P @f$.
     *
     * @param[in] e
     * Indices of edge points @f$ E = (A, B) @f$.
     */
    bool is_edge_visible(index_type p, edge_type e) const
    {
        return compute_area(p, e.a, e.b).upper_bound() < 0;
    }

    /**
     * @brief Add point.
     *
     * @param[in] p
     * Index of point @f$ P @f$.
     *
     * @param[in] alloc1
     * Arena allocator for temporary allocations.
     *
     * @param[in] alloc2
     * Arena allocator for temporary allocations.
     */
    void add_point(
                index_type p, 
                pr::memory_arena_allocator<char> alloc1,
                pr::memory_arena_allocator<char> alloc2)
    {
        {

        // Boundary edges to remove.
        std::vector<edge_type, 
        pr::memory_arena_allocator<edge_type>> 
                boundary_edges_to_remove(alloc1); // Use allocator 1.

        // Boundary edges to add.
        std::set<edge_type, 
        std::less<edge_type>,
        pr::memory_arena_allocator<edge_type>> 
                boundary_edges_to_add(alloc1); // Use allocator 1.

        // Iterate over boundary edges.
        for (const edge_type& edge : boundary_edges_) {

            // Is edge visible from point?
            if (is_edge_visible(p, edge)) {

                // Form (counterclockwise) triangle.
                triangles_.push_back({p, edge.b, edge.a});

                // Form edges.
                index_type t = 
                index_type(triangles_.size()) - 1;
                edge_type edge1 = {p, edge.a};
                edge_type edge2 = {edge.b, p};
                edge_to_triangles_[edge].push(t);
                edge_to_triangles_[edge1].push(t);
                edge_to_triangles_[edge2].push(t);

                // Remember boundary updates.
                boundary_edges_to_remove.push_back(edge);
                boundary_edges_to_add.insert(edge1.reverse());
                boundary_edges_to_add.insert(edge2.reverse());
            }
        }

        // Remove boundary edges.
        for (const edge_type& edge : boundary_edges_to_remove) {
            boundary_edges_.erase(
            boundary_edges_.find(edge));
        }

        // Add boundary edges.
        for (const edge_type& edge : boundary_edges_to_add) {
            // Boundary edge to add ended up being an interior edge?
            if (edge_to_triangles_[edge].is_full()) {
                // Disregard.
                continue;
            }
            else {
                // Add.
                boundary_edges_.insert(edge);
            }
        }

        // Leave scope.
        }

        // Clear allocations.
        alloc1.clear();

        {

        // Set of edges to flip.
        std::set<edge_type, 
        std::less<edge_type>, 
        pr::memory_arena_allocator<edge_type>> 
                edges_to_flip(alloc1); // Use allocator 1.
        // Initialize with all edges.
        for (const auto& kv : edge_to_triangles_) {
            edges_to_flip.insert(kv.first);
        }

        // Forever...
        for (;;) {

            // Set of edges to flip next.
            std::set<edge_type,
            std::less<edge_type>,
            pr::memory_arena_allocator<edge_type>>
                    edges_to_flip_next(alloc2); // Use allocator 2.

            for (const edge_type& edge : edges_to_flip) {
                maybe_flip_edge(
                        edge, 
                        edges_to_flip_next);
            }

            // No edges to flip next?
            if (edges_to_flip_next.empty()) {
                // Done.
                break;
            }

            // Clear edges.
            edges_to_flip.clear();
            alloc1.clear(); // Clear allocator 1.

            // Copy edges to flip next.
            // Note, copy assignment cannot be used here because
            // pr::memory_arena_allocator propagates on copy and move.
            // Here, we only want to copy the contents, not the allocator
            // itself.
            edges_to_flip.insert(
                    edges_to_flip_next.begin(),
                    edges_to_flip_next.end());

            // Clear edges to flip next.
            edges_to_flip_next.clear();
            alloc2.clear(); // Clear allocator 2.
        }

        // Leave scope.
        }

        // Clear allocations.
        alloc1.clear();
        alloc2.clear();
    }

    /**
     * @brief Maybe flip edge to satisfy Delaunay condition.
     *
     * @param[in] edge
     * Edge.
     *
     * @param[inout] edges_to_flip_next
     * Edges to flip on next iteration.
     */
    template <typename... Targs>
    void maybe_flip_edge(
                edge_type edge, 
                std::set<edge_type, Targs...>& edges_to_flip_next)
    {
        edge_triangle_pair_type 
        edge_triangles = edge_to_triangles_[edge];
        // Is boundary edge?
        if (!edge_triangles.is_full()) {
            // Return.
            return;
        }

        // Triangles.
        index_type t1 = edge_triangles.t1;
        index_type t2 = edge_triangles.t2;
        triangle_type& triangle1 = triangles_[t1];
        triangle_type& triangle2 = triangles_[t2];

        // Find indices of opposite points.
        index_type q1 = bad_index;
        index_type q2 = bad_index;
        for (int k = 0; k < 3; k++) {
            if (triangle1[k] != edge.a &&
                triangle1[k] != edge.b) {
                q1 = triangle1[k];
            }
            if (triangle2[k] != edge.a &&
                triangle2[k] != edge.b) {
                q2 = triangle2[k];
            }
        }
        assert(q1 != bad_index);
        assert(q2 != bad_index);

        // Points.
        const multi<float_type, 2>& pa = points_[edge.a];
        const multi<float_type, 2>& pb = points_[edge.b];
        const multi<float_type, 2>& pq1 = points_[q1];
        const multi<float_type, 2>& pq2 = points_[q2];

        // Angles.
        multi<float_type, 2> vq1a = pa - pq1;
        multi<float_type, 2> vq1b = pb - pq1;
        multi<float_type, 2> vq2a = pa - pq2;
        multi<float_type, 2> vq2b = pb - pq2;
        float_type phi1 = pr::atan2(cross(vq1a, vq1b), dot(vq1a, vq1b));
        float_type phi2 = pr::atan2(cross(vq2b, vq2a), dot(vq2a, vq2b));
        phi1 = pr::abs(phi1);
        phi2 = pr::abs(phi2);

        // Is Delaunay condition already satisfied?
        if (!(phi1 + phi2 > pr::numeric_constants<float_type>::M_pi() *
                (1 + float_type(1e-7)))) {
            // Return.
            return;
        }

        // Update triangles.
        triangle1 = {q1, edge.a, q2};
        triangle2 = {q1, q2, edge.b};

        // Erase edge.
        edge_to_triangles_.erase(
        edge_to_triangles_.find(edge));

        // Insert flipped edge.
        edge_type flipped_edge = {q1, q2};
        edge_to_triangles_[flipped_edge] = {t1, t2};

        // Update affected edges.
        edge_type affected_edge1 = {q1, edge.b};
        edge_type affected_edge2 = {q2, edge.a};
        edge_to_triangles_[affected_edge1].replace(t1, t2);
        edge_to_triangles_[affected_edge2].replace(t2, t1);

        // Maybe flip all neighboring edges on next iteration.
        edges_to_flip_next.insert(affected_edge1);
        edges_to_flip_next.insert(affected_edge2);
        edges_to_flip_next.insert(edge_type{q1, edge.a});
        edges_to_flip_next.insert(edge_type{q2, edge.b});
    }
    
private:
public:

    // TODO use user-specified allocator
    /**
     * @brief Points.
     */
    std::vector<multi<float_type, 2>> points_;

    // TODO use user-specified allocator
    /**
     * @brief Triangles.
     */
    std::vector<triangle_type> triangles_;

    // TODO use user-specified allocator
    // TODO doc
    /**
     * @brief Edge-to-triangles map.
     */
    std::map<
        edge_type,
        edge_triangle_pair_type> edge_to_triangles_;

    // TODO use user-specified allocator
    // TODO doc
    /**
     * @brief Boundary edges.
     */
    std::set<edge_type> boundary_edges_;

};

/**@}*/

} // namespace pr

#endif // #ifndef PREFORM_DELAUNAY_HPP
