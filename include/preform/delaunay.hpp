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
#include <memory>
#include <vector>
#include <map>
#include <set>
#include <preform/float_interval.hpp>
#include <preform/memory_arena.hpp>
#include <preform/memory_arena_allocator.hpp>
#include <preform/iterator_range.hpp>
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
 * @tparam Tfloat
 * Float type.
 *
 * @tparam Talloc
 * Allocator type.
 */
template <
    typename Tfloat, 
    typename Talloc = std::allocator<char>
    >
class delaunay_triangulation
{
public:

    // Sanity check.
    static_assert(
        std::is_floating_point<Tfloat>::value,
        "Tfloat must be floating point");

    /**
     * @brief Float type.
     */
    typedef Tfloat float_type;

    /**
     * @brief Float interval type.
     */
    typedef float_interval<Tfloat> float_interval_type;

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

public:

    /**
     * @brief Point type.
     */
    typedef multi<Tfloat, 2> point_type;

    /**
     * @brief Triangle type.
     *
     * This represents a triangle formed by three points as a tuple
     * of indices @f$ (a, b, c) @f$.
     */
    class triangle_type
    {
    public:

        /**
         * @brief Index of point @f$ a @f$.
         */
        index_type a = bad_index;

        /**
         * @brief Index of point @f$ b @f$.
         */
        index_type b = bad_index;

        /**
         * @brief Index of point @f$ c @f$.
         */
        index_type c = bad_index;

        /**
         * @brief Access operator.
         */
        index_type operator[](size_type k) const
        {
            if constexpr (sizeof(*this) == 3 * sizeof(index_type)) {
                return (&a)[k];
            }
            else {
                return k == 0 ? a : 
                       k == 1 ? b : c;
            }
        }
    };

    /**
     * @brief Edge type.
     *
     * This represents an edge between two points as a pair of
     * indices @f$ (a, b) @f$. For the purposes of comparison, the
     * pair is unordered. This means that @f$ (a, b) @f$ and @f$ (b, a) @f$
     * identfy the same edge. However, the order is relevant in determining 
     * whether a border edge is visible to a point, and so it is
     * preserved.
     */
    class edge_type
    {
    public:

        /**
         * @brief Index of point @f$ a @f$.
         */
        index_type a = bad_index;

        /**
         * @brief Index of point @f$ b @f$.
         */
        index_type b = bad_index;

        /**
         * @brief Compare less-than.
         *
         * After sorting the indices of each edge, compare less-than 
         * lexicographically. This ensures that each pair @f$ (a, b) @f$
         * behaves identically to @f$ (b, a) @f$.
         */
        __attribute__((always_inline))
        bool operator<(const edge_type& other) const
        {
            auto [a0, b0] = std::minmax(a, b);
            auto [a1, b1] = std::minmax(other.a, other.b);
            return a0 < a1 || (a0 == a1 && b0 < b1);
        }

        /**
         * @brief Compare equal.
         *
         * After sorting the indices of each edge, compare equal. 
         * lexicographically. This ensures that each pair @f$ (a, b) @f$
         * behaves identically to @f$ (b, a) @f$.
         */
        __attribute__((always_inline))
        bool operator==(const edge_type& other) const
        {
            auto [a0, b0] = std::minmax(a, b);
            auto [a1, b1] = std::minmax(other.a, other.b);
            return a0 == a1 && b0 == b1;
        }

        /**
         * @brief Reverse ordering @f$ (a, b) \to (b, a) @f$.
         */
        edge_type reverse() const
        {
            return {b, a};
        }
    };

private:

    /**
     * @brief Edge triangle pair type.
     */
    class edge_triangles_type
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
         *
         * @note
         * A triangle pair is full once both indices have been
         * assigned.
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

public:

    /**
     * @brief Default constructor.
     */
    delaunay_triangulation() = default;

    /**
     * @brief Constructor.
     *
     * @param[in] alloc
     * Allocator.
     */
    delaunay_triangulation(const Talloc& alloc) :
            points_(alloc),
            triangles_(alloc),
            edge_triangles_(alloc),
            boundary_edges_(alloc)
    {
    }

    /**
     * @brief Non-copyable.
     */
    delaunay_triangulation(const delaunay_triangulation&) = delete;

public:

    /**
     * @brief Initialize.
     *
     * @param[in] from
     * Iterator range from.
     *
     * @param[in] to
     * Iterator range to.
     *
     * @tparam Tinput_itr
     * Input iterator whose value type is suitable to 
     * initialize `point_type`.
     */
    template <typename Tinput_itr>
    void init(
            Tinput_itr from,
            Tinput_itr to)
    {
        // Initialize points.
        points_.insert(
        points_.begin(), from, to);
        if (points_.empty()) {
            return;
        }

        // Compute point centroid.
        point_type point_centroid = {};
        for (const point_type& point : points_) {
            if (pr::isfinite(point).all()) {
                point_centroid += point;
            }
        }
        point_centroid /= 
        float_type(points_.size());

        // Compute point square distances to point centroid.
        std::vector<std::pair<float_type, index_type>> 
        point_distances;
        point_distances.reserve(points_.size());
        for (const point_type& point : points_) {
            point_distances.push_back({
                    dot(point - point_centroid,
                        point - point_centroid),
                    index_type(&point - &points_[0])
                });
        }

        // Sort by square distance.
        std::sort(
                point_distances.begin(),
                point_distances.end());

        // Initial triangle.
        triangle_type initial_triangle;

        // Initial triangle area.
        float_interval_type initial_triangle_area;

        // Form initial triangle from points nearest to centroid.
        while (point_distances.size() > 2) {
            initial_triangle = triangle_type{
                point_distances[0].second,
                point_distances[1].second,
                point_distances[2].second
            };
            initial_triangle_area = 
            signed_area<float_interval_type>(
                        initial_triangle.a,
                        initial_triangle.b,
                        initial_triangle.c);

            // Is degenerate?
            if (!(initial_triangle_area.upper_bound() < 0 ||
                  initial_triangle_area.lower_bound() > 0)) {

                // Erase first point, try again.
                point_distances.erase(
                point_distances.begin());
            }
            else {
                break;
            }
        }

        // Failed to form initial triangle?
        if (point_distances.size() < 3) {
            throw std::runtime_error(__PRETTY_FUNCTION__);
        }

        // Initial triangle area negative?
        if (initial_triangle_area.upper_bound() < 0) {
            // Make counter-clockwise.
            std::swap(
                    initial_triangle.b, 
                    initial_triangle.c);
        }

        // Add initial triangle.
        triangles_.reserve(points_.size());
        triangles_.push_back(initial_triangle);

        // Initial edges.
        edge_type initial_edges[3] = {
            {initial_triangle.a, initial_triangle.b},
            {initial_triangle.b, initial_triangle.c},
            {initial_triangle.c, initial_triangle.a}
        };

        // Map initial edges to initial triangle.
        edge_triangles_[initial_edges[0]].push(0);
        edge_triangles_[initial_edges[1]].push(0);
        edge_triangles_[initial_edges[2]].push(0);
            
        // Insert initial edges to boundary edge set.
        boundary_edges_.insert(initial_edges[0]);
        boundary_edges_.insert(initial_edges[1]);
        boundary_edges_.insert(initial_edges[2]);

        {
            // Temporary allocators.
            pr::memory_arena_allocator<char> alloc1;
            pr::memory_arena_allocator<char> alloc2;

            // Add remaining points.
            for (size_type k = 3; 
                           k < point_distances.size(); k++) {
                add_point(point_distances[k].second, alloc1, alloc2);
            }
        }

        // Iterate triangles.
        for (triangle_type& triangle : triangles_) {

            // Triangle area negative?
            if (signed_area<float_type>(
                            triangle.a,
                            triangle.b,
                            triangle.c) < 0) {
                // Make counter-clockwise.
                std::swap(
                        triangle.b, 
                        triangle.c);
            }
        }
    }

public:

    /**
     * @name Accessors
     */
    /**@{*/

    /**
     * @brief Iterator range of points.
     */
    auto points() const
    {
        return make_iterator_range(points_);
    }

    /**
     * @brief Iterator range of triangles.
     */
    auto triangles() const
    {
        return make_iterator_range(triangles_);
    }

    /**
     * @brief Iterator range of boundary edges.
     */
    auto boundary_edges() const
    {
        return make_iterator_range(boundary_edges_);
    }

    /**@}*/

private:

    /**
     * @brief Signed area of parallelogram.
     *
     * This computes the signed area of the parallelogram 
     * corresponding to the triangle formed by three points with indices
     * @f$ (a, b, c) @f$. This is given by
     * @f[
     *      A = \mathbf{v}_{ab} \times \mathbf{v}_{ac}
     * @f]
     * where
     * - @f$ \mathbf{v}_{ab} = \mathbf{P}_b - \mathbf{P}_a @f$,
     * - @f$ \mathbf{v}_{ac} = \mathbf{P}_c - \mathbf{P}_a @f$.
     *
     * @param[in] a
     * Index of point.
     *
     * @param[in] b
     * Index of point.
     *
     * @param[in] c
     * Index of point.
     *
     * @tparam Twhich_float
     * Which float type to use, either `float_type` or `float_interval_type`.
     */
    template <typename Twhich_float>
    Twhich_float signed_area(
                index_type a,
                index_type b,
                index_type c) const
    {
        static_assert(
            std::is_same<Twhich_float, float_type>::value ||
            std::is_same<Twhich_float, float_interval_type>::value,
            "Invalid usage");

        // Compute.
        multi<Twhich_float, 2> pa = points_[a];
        multi<Twhich_float, 2> pb = points_[b];
        multi<Twhich_float, 2> pc = points_[c];
        return cross(pb - pa, pc - pa);
    }

    /**
     * @brief Add point.
     *
     * @param[in] p
     * Index of point.
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
        if (!pr::isfinite(points_[p]).all()) {
            return; // Ignore.
        }

        {

        // Boundary edges to remove.
        std::vector<edge_type, 
        pr::memory_arena_allocator<edge_type>> 
                boundary_edges_to_remove(alloc1); // Use allocator 1.
        boundary_edges_to_remove.reserve(
        boundary_edges_.size() / 2);

        // Boundary edges to add.
        std::set<edge_type, 
        std::less<edge_type>,
        pr::memory_arena_allocator<edge_type>> 
                boundary_edges_to_add(alloc1); // Use allocator 1.

        // Iterate over boundary edges.
        for (const edge_type& edge : boundary_edges_) {

            // Is edge visible from point?
            if (signed_area<float_interval_type>(
                        p, edge.a, edge.b).upper_bound() < 0) {

                // Form (counterclockwise) triangle.
                index_type t = 
                index_type(triangles_.size());
                triangles_.push_back({p, edge.b, edge.a});

                // Form edges.
                edge_type edge1 = {p, edge.a};
                edge_type edge2 = {edge.b, p};
                edge_triangles_[edge].push(t);
                edge_triangles_[edge1].push(t);
                edge_triangles_[edge2].push(t);

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
            if (edge_triangles_[edge].is_full()) {
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
        for (const auto& kv : edge_triangles_) {
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
        edge_triangles_type 
        edge_triangles = edge_triangles_[edge];
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
        edge_triangles_.erase(
        edge_triangles_.find(edge));

        // Insert flipped edge.
        edge_type flipped_edge = {q1, q2};
        edge_triangles_[flipped_edge] = {t1, t2};

        // Update affected edges.
        edge_type affected_edge1 = {q1, edge.b};
        edge_type affected_edge2 = {q2, edge.a};
        edge_triangles_[affected_edge1].replace(t1, t2);
        edge_triangles_[affected_edge2].replace(t2, t1);

        // Maybe flip all neighboring edges on next iteration.
        edges_to_flip_next.insert(affected_edge1);
        edges_to_flip_next.insert(affected_edge2);
        edges_to_flip_next.insert(edge_type{q1, edge.a});
        edges_to_flip_next.insert(edge_type{q2, edge.b});
    }
    
private:

    /**
     * @brief Points.
     *
     * @note
     * This uses the user-specified allocator type `Talloc` rebound to 
     * `point_type`. For brevity, this is not shown in the documentation 
     * type signature.
     */
    std::vector<
        point_type,
#if !DOXYGEN
        typename std::allocator_traits<Talloc>::
        template rebind_alloc<point_type>
#else
        ...
#endif // #if !DOXYGEN
        > points_;

    /**
     * @brief Triangles.
     *
     * @note
     * This uses the user-specified allocator type `Talloc` rebound to 
     * `triangle_type`. For brevity, this is not shown in the documentation 
     * type signature.
     */
    std::vector<
        triangle_type,
#if !DOXYGEN
        typename std::allocator_traits<Talloc>::
        template rebind_alloc<triangle_type>
#else
        ...
#endif // #if !DOXYGEN
        > triangles_;

    /**
     * @brief Edge triangles.
     *
     * This maps an unordered edge @f$ (a, b) @f$ to the indices of the
     * 1 or 2 triangles containing it.
     *
     * @note
     * This uses the user-specified allocator type `Talloc` rebound to 
     * `std::pair<const edge_type, edge_triangles_type>`. For brevity, this is
     * not shown in the documentation type signature.
     */
    std::map<
        edge_type,
        edge_triangles_type,
#if !DOXYGEN
        std::less<edge_type>,
        typename std::allocator_traits<Talloc>::
        template rebind_alloc<std::pair<
                 const edge_type, edge_triangles_type>>
#else
        ...
#endif // #if !DOXYGEN
        > edge_triangles_;

    /**
     * @brief Boundary edges.
     *
     * This is the set of boundary edges, which forms the convex hull,
     * oriented counter-clockwise around the point set.
     *
     * @note
     * This uses the user-specified allocator type `Talloc` rebound to 
     * `edge_type`. For brevity, this is not shown in the documentation 
     * type signature.
     */
    std::set<
        edge_type,
#if !DOXYGEN
        std::less<edge_type>,
        typename std::allocator_traits<Talloc>::
        template rebind_alloc<edge_type>
#else
        ...
#endif // #if !DOXYGEN
        > boundary_edges_;

};

/**@}*/

} // namespace pr

#endif // #ifndef PREFORM_DELAUNAY_HPP
