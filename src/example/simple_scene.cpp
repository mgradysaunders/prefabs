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
#include <cassert>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <preform/multi.hpp>
#include <preform/multi_math.hpp>
#include <preform/multi_random.hpp>
#include <preform/float_interval.hpp>
#include <preform/neumaier_sum.hpp>
#include <preform/color.hpp>
#include <preform/image2.hpp>
#include <preform/image_filters.hpp>
#include <preform/aabb.hpp>
#include <preform/aabbtree.hpp>
#include <preform/microsurface.hpp>
#include <preform/bash_format.hpp>
#include <preform/thread_pool.hpp>
#include <preform/static_stack.hpp>
#include <preform/memory_arena.hpp>
#include <preform/memory_arena_allocator.hpp>
#include <preform/option_parser.hpp>

// Float.
typedef float Float;

// Float interval.
typedef pr::float_interval<Float> FloatInterval;

// Neumaier sum.
typedef pr::neumaier_sum<Float> NeumaierSum;

// 2-dimensional vector of floats.
typedef pr::vec2<Float> Vec2f;

// 3-dimensional vector of floats.
typedef pr::vec3<Float> Vec3f;

// 3-dimensional vector of floats (32-bit).
typedef pr::vec3<float> Vec3f32;

// 3-dimensional vector of float intervals.
typedef pr::vec3<FloatInterval> Vec3fInterval;

// 2-dimensional vector of ints.
typedef pr::vec2<int> Vec2i;

// 3-dimensional vector of ints.
typedef pr::vec3<int> Vec3i;

// 3-dimensional bounding box.
typedef pr::aabb3<Float> Aabb3f;

// 3-dimensional bounding box (32-bit).
typedef pr::aabb3<float> Aabb3f32;

// 3-dimensional bounding box tree (32-bit).
typedef pr::aabbtree3<float,
        pr::aabbtree_split_surface_area<16>,
        pr::memory_arena_allocator<char>> AabbTree3;

// 3-dimensional linear-storage bounding box tree (32-bit).
typedef pr::linear_aabbtree3<float> LinearAabbTree3;

// Ray.
struct Ray
{
    // Position.
    Vec3f pos;

    // Direction.
    Vec3f dir;

    // Parameter minimum.
    Float tmin = 0;

    // Parameter maximum.
    mutable 
    Float tmax = pr::numeric_limits<Float>::infinity();
};

// Hit.
struct Hit
{
    // Position.
    Vec3f pos;

    // Position absolute error.
    Vec3f pos_abs_error;

    // Normal.
    Vec3f normal;

    // Normal (shading).
    Vec3f normal_shading;
};

// Triangle mesh.
struct TriangleMesh
{
public:

    // Vertex.
    struct Vertex
    {
        // Position.
        Vec3f32 pos;

        // Normal (shading).
        Vec3f32 normal_shading;
    };

    // Triangle.
    struct Triangle
    {
        // Vertices.
        Vertex vertices[3];

        // Normal, unnormalized.
        Vec3f normal() const
        {
            return pr::cross(
                        vertices[1].pos - vertices[0].pos,
                        vertices[2].pos - vertices[0].pos);
        }

        // Surface area.
        Float surfaceArea() const
        {
            return pr::length(normal()) * Float(0.5);
        }
    };

    // Initialize from OBJ file.
    void initFromObj(const char* filename);

    // Initialize.
    void init(const std::vector<Triangle>& triangles);

    // Bounding box.
    Aabb3f boundingBox() const
    {
        assert(triangles_tree_.begin());
        return triangles_tree_.begin()->box;
    }

    // Surface area.
    Float surfaceArea() const
    {
        return surface_area_;
    }

    // Surface area sample.
    Hit surfaceAreaSample(Vec2f samp) const;

    // Intersect.
    bool intersect(const Ray& ray, Hit* hit = nullptr) const;

private:

    // Triangles.
    std::vector<Triangle> triangles_;

    // Triangles linear bounding box tree.
    LinearAabbTree3 triangles_tree_;

    // Triangles PMF values.
    std::vector<Float> triangles_pmf_;

    // Triangles CDF values.
    std::vector<Float> triangles_cdf_;

    // Surface area.
    Float surface_area_ = 0;
};

// TODO ... many things

int main(int argc, char** argv)
{
    Vec2i image_dim = {512, 512};
    TriangleMesh meshes[3];
    meshes[0].initFromObj("data/simple_scene/happy.obj");
    meshes[1].initFromObj("data/simple_scene/armadillo.obj");
    meshes[2].initFromObj("data/simple_scene/floor.obj");

    pr::image2<Float, Float, 3> image;
    image.resize(image_dim);
    for (int i = 0; i < image_dim[0]; i++)
    for (int j = 0; j < image_dim[1]; j++) {
        Float u = i / Float(image_dim[0]);
        Float v = j / Float(image_dim[1]);
        Vec3f p0 = {
            0, 5,
            -100
        };
        Vec3f p1 = {
            +15 * (u - Float(0.5)),
            -15 * (v - Float(0.5)) + 5,
            0
        };
        Ray ray;
        ray.pos = p0;
        ray.dir = pr::normalize(p1 - p0);
        ray.tmin = 0;
        ray.tmax = pr::numeric_limits<Float>::infinity();
        Hit hit;
        bool res = false;
        for (const TriangleMesh& mesh : meshes) {
            if (mesh.intersect(ray, &hit)) {
                res = true;
            }
        }
        if (res) {
            image(i, j) = 
                Vec3f{1,1,1} *
                pr::abs(pr::dot(hit.normal_shading,
                        pr::normalize(Vec3f{1, 1, 1})));
        }
    }

#if 1
    std::cout << "P3\n";
    std::cout << image_dim[0] << ' ';
    std::cout << image_dim[1] << '\n';
    std::cout << "255\n";
    for (int j = 0; j < image_dim[1]; j++)
    for (int i = 0; i < image_dim[0]; i++) {
        Vec3i val = pr::pack_uint8(image(i, j));
        std::cout << val[0] << ' ';
        std::cout << val[1] << ' ';
        std::cout << val[2] << '\n';
        std::cout.flush();
    }
#endif
    return EXIT_SUCCESS;
}

// Ray/box intersector.
struct RayBoxIntersector
{
public:

    // Constructor.
    RayBoxIntersector(const Ray* ray) :
            ray_(*ray),
            dir_neg_(pr::signbit(ray->dir)),
            dir_rcp_(1 / ray->dir)
    {
    }

    // Intersect.
    bool intersect(Aabb3f box) const
    {
        Float tmin, tkmin;
        Float tmax, tkmax;
        tmin = (box[    dir_neg_[0]][0] - ray_.pos[0]) * dir_rcp_[0];
        tmax = (box[1 - dir_neg_[0]][0] - ray_.pos[0]) * dir_rcp_[0];
        tmin *= 1 - 2 * pr::numeric_limits<Float>::echelon(3);
        tmax *= 1 + 2 * pr::numeric_limits<Float>::echelon(3);
        for (int k = 1; k < 3; k++) {
            tkmin = (box[    dir_neg_[k]][k] - ray_.pos[k]) * dir_rcp_[k];
            tkmax = (box[1 - dir_neg_[k]][k] - ray_.pos[k]) * dir_rcp_[k];
            tkmin *= 1 - 2 * pr::numeric_limits<Float>::echelon(3);
            tkmax *= 1 + 2 * pr::numeric_limits<Float>::echelon(3);
            if (!(tmin < tkmax &&
                  tmax > tkmin)) {
                return false;
            }
            tmin = pr::fmax(tmin, tkmin);
            tmax = pr::fmin(tmax, tkmax);
        }
        return tmin < ray_.tmax && 
               tmax > ray_.tmin;
    }

private:

    // Ray.
    const Ray& ray_;

    // Ray direction component negative?
    Vec3i dir_neg_;

    // Ray direction component reciprocal.
    Vec3f dir_rcp_;
};

// Ray/triangle intersector.
struct RayTriangleIntersector
{
public:

    // Constructor.
    RayTriangleIntersector(const Ray* ray) : 
            ray_(*ray),
            pos_interval_(ray->pos),
            dir_interval_(ray->dir)
    {
        // Permutation.
        kr_[2] = pr::fabs(ray_.dir).argmax();
        kr_[0] = (kr_[2] + 1) % 3;
        kr_[1] = (kr_[2] + 2) % 3;

        // Shear.
        hr_interval_[2] = Float(1) / dir_interval_[kr_[2]];
        hr_interval_[0] = -dir_interval_[kr_[0]] * hr_interval_[2];
        hr_interval_[1] = -dir_interval_[kr_[1]] * hr_interval_[2];
    }

    // Intersect.
    bool intersect(
            Vec3f pos0,
            Vec3f pos1,
            Vec3f pos2,
            Vec3f* bhit = nullptr, 
            Float* thit = nullptr) const;

private:

    // Ray.
    const Ray& ray_;

    // Unpermuted position interval.
    Vec3fInterval pos_interval_;

    // Unpermuted direction interval.
    Vec3fInterval dir_interval_;

    // Permutation.
    Vec3i kr_;
    
    // Permuted shear coefficients.
    Vec3fInterval hr_interval_;
};

// Intersect.
bool RayTriangleIntersector::intersect(
            Vec3f pos0,
            Vec3f pos1,
            Vec3f pos2,
            Vec3f* bhit, 
            Float* thit) const
{
    // Shear in XY.
    Vec3fInterval 
        pos0_interval = 
       (pos0 - pos_interval_).swizzle(kr_);
    Vec3fInterval h0_interval;
    h0_interval[0] = pos0_interval[0] + hr_interval_[0] * pos0_interval[2];
    h0_interval[1] = pos0_interval[1] + hr_interval_[1] * pos0_interval[2];

    // Shear in XY.
    Vec3fInterval
        pos1_interval = 
       (pos1 - pos_interval_).swizzle(kr_);
    Vec3fInterval h1_interval;
    h1_interval[0] = pos1_interval[0] + hr_interval_[0] * pos1_interval[2];
    h1_interval[1] = pos1_interval[1] + hr_interval_[1] * pos1_interval[2];

    // Shear in XY.
    Vec3fInterval
        pos2_interval =
       (pos2 - pos_interval_).swizzle(kr_);
    Vec3fInterval h2_interval;
    h2_interval[0] = pos2_interval[0] + hr_interval_[0] * pos2_interval[2];
    h2_interval[1] = pos2_interval[1] + hr_interval_[1] * pos2_interval[2];

    // Any unnormalized barycentric coordinates certainly negative?
    bool any_negative = false;

    // Any unnormalized barycentric coordinates certainly positive?
    bool any_positive = false;

    // Unnormalized barycentric coordinate.
    FloatInterval
        b0_interval = 
        h1_interval[1] * h2_interval[0] - 
        h1_interval[0] * h2_interval[1];
    any_negative |= !(b0_interval.upper_bound() >= 0);
    any_positive |= !(b0_interval.lower_bound() <= 0);

    // Unnormalized barycentric coordinate.
    FloatInterval 
        b1_interval = 
        h2_interval[1] * h0_interval[0] - 
        h2_interval[0] * h0_interval[1];
    any_negative |= !(b1_interval.upper_bound() >= 0);
    any_positive |= !(b1_interval.lower_bound() <= 0);

    // Unnormalized barycentric coordinate.
    FloatInterval 
        b2_interval = 
        h0_interval[1] * h1_interval[0] - 
        h0_interval[0] * h1_interval[1];
    any_negative |= !(b2_interval.upper_bound() >= 0);
    any_positive |= !(b2_interval.lower_bound() <= 0);

    // Reject certain misses.
    if (any_negative && 
        any_positive) {
        return false;
    }

    // Reject uncertain hits.
    FloatInterval q_interval = 
            b0_interval + 
            b1_interval + 
            b2_interval;
    if (q_interval.contains(0)) {
        return false;
    }

    // Normalize barycentric coordinates.
    if (q_interval.abs_lower_bound() <
        pr::numeric_limits<Float>::min_invertible()) {

        // Divide.
        b0_interval /= q_interval;
        b1_interval /= q_interval;
        b2_interval /= q_interval;
    }
    else {

        // Multiply by reciprocal.
        q_interval = Float(1) / q_interval;
        b0_interval *= q_interval;
        b1_interval *= q_interval;
        b2_interval *= q_interval;
    }

    // Shear in Z.
    h0_interval[2] = hr_interval_[2] * pos0_interval[2];
    h1_interval[2] = hr_interval_[2] * pos1_interval[2];
    h2_interval[2] = hr_interval_[2] * pos2_interval[2];

    // Parametric value.
    FloatInterval t_interval = 
        b0_interval * h0_interval[2] + 
        b1_interval * h1_interval[2] + 
        b2_interval * h2_interval[2];

    // Reject uncertain hits.
    if (!(t_interval.upper_bound() < ray_.tmax &&
          t_interval.lower_bound() > ray_.tmin)) {
        return false;
    }

    if (bhit) {
        *bhit = {
            b0_interval.value(),
            b1_interval.value(),
            b2_interval.value()
        };
    }
    if (thit) {
        *thit = t_interval.value();
    }

    return true;
}

// Initialize from OBJ file.
void TriangleMesh::initFromObj(const char* filename)
{
    std::vector<Vec3i> objf;
    std::vector<Vec3f32> objv, objvn;
    std::ifstream ifs(filename);
    if (!ifs.is_open()) {
        throw std::runtime_error("cannot open ifstream");
    }
    while (!ifs.eof()) {
        std::string ln;
        std::getline(ifs, ln);
        std::stringstream lnss(ln);
        char ch;
        lnss >> ch;
        switch (ch) {
            case 'v':
                objv.emplace_back();
                lnss >> objv.back()[0];
                lnss >> objv.back()[1];
                lnss >> objv.back()[2];
                break;
            case 'f':
                objf.emplace_back();
                lnss >> objf.back()[0];
                lnss >> objf.back()[1];
                lnss >> objf.back()[2];
                objf.back() -= 1; // OBJ indices start at 1
                break;
            default:
                assert(false);
                break;
        }
    }
    ifs.close();

    // Compute smooth vertex normals.
    objvn.resize(objv.size());
    for (Vec3i f : objf) {
        Vec3f32 e1 = objv[f[1]] - objv[f[0]];
        Vec3f32 e2 = objv[f[2]] - objv[f[0]];
        Vec3f32 vn = pr::cross(e1, e2);
        objvn[f[0]] += vn;
        objvn[f[1]] += vn;
        objvn[f[2]] += vn;
    }
    for (Vec3f32& vn : objvn) {
        vn = pr::normalize_safe(vn);
    }

    // Assemble triangles.
    std::vector<Triangle> triangles;
    triangles.reserve(objf.size());
    for (Vec3i f : objf) {
        Triangle triangle;
        triangle.vertices[0] = Vertex{objv[f[0]], objvn[f[0]]};
        triangle.vertices[1] = Vertex{objv[f[1]], objvn[f[1]]};
        triangle.vertices[2] = Vertex{objv[f[2]], objvn[f[2]]};
        triangles.push_back(triangle);
    }

    // Deallocate OBJ variables.
    objf = std::move(std::vector<Vec3i>());
    objv = std::move(std::vector<Vec3f32>());
    objvn = std::move(std::vector<Vec3f32>());

    // Delegate.
    init(triangles);
}

// Initialize.
void TriangleMesh::init(const std::vector<Triangle>& triangles)
{
    // Sanity check.
    assert(triangles_.empty());

    // Copy initialize non-degenerate triangles.
    triangles_.reserve(triangles.size());
    for (auto triangle_itr  = triangles.begin();
              triangle_itr != triangles.end(); triangle_itr++) {
        Vec3f normal = triangle_itr->normal();
        if (pr::dot(normal, normal) > 0) {
            triangles_.push_back(*triangle_itr);
        }
    }

    // Sanity check.
    assert(!triangles_.empty());

    {
        // Build triangles tree.
        AabbTree3 tree(8);
        tree.init(
                triangles_.begin(),
                triangles_.end(),
                [](const Triangle& triangle) {
                    Aabb3f32 box = triangle.vertices[0].pos;
                    box |= triangle.vertices[1].pos;
                    box |= triangle.vertices[2].pos;
                    return box;
                });
        tree.sort(
                triangles_.begin(),
                triangles_.end());
        triangles_tree_ = LinearAabbTree3(tree);
    }

    // Initialize probabilities with surface areas.
    NeumaierSum surface_area_sum;
    triangles_pmf_.reserve(triangles_.size());
    triangles_cdf_.reserve(triangles_.size() + 1);
    triangles_cdf_.push_back(0); 
    for (const Triangle& triangle : triangles_) {
        Float surface_area = triangle.surfaceArea();
        triangles_pmf_.push_back(surface_area);
        triangles_cdf_.push_back(Float(surface_area_sum += surface_area));
    }

    // Normalize.
    surface_area_ = Float(surface_area_sum);
    if (surface_area_ < pr::numeric_limits<Float>::min_invertible()) {

        // Divide.
        for (Float& pmf : triangles_pmf_) { pmf /= surface_area_; }
        for (Float& cdf : triangles_cdf_) { cdf /= surface_area_; }
    }
    else {

        // Multiply by reciprocal.
        Float surface_area_rcp = 1 / surface_area_;
        for (Float& pmf : triangles_pmf_) { pmf *= surface_area_rcp; }
        for (Float& cdf : triangles_cdf_) { cdf *= surface_area_rcp; }
    }
}

// Surface area sample.
Hit TriangleMesh::surfaceAreaSample(Vec2f samp) const
{
    Hit hit;

    Float u0 = samp[0];
    for (std::size_t k = 0; 
                     k < triangles_pmf_.size(); k++) {
        if (u0 < triangles_cdf_[k + 1] || 
            k == triangles_pmf_.size() - 1) {

            // Stretch to [0, 1).
            u0 -= triangles_cdf_[k];
            u0 /= triangles_pmf_[k];
            
            // Just to be safe.
            u0 = pr::max(u0, Float(0));
            u0 = pr::min(u0, Float(1) - 
                 pr::numeric_limits<Float>::machine_epsilon());

            // Triangle.
            const Triangle& triangle = triangles_[k];
            const Vertex& vertex0 = triangle.vertices[0];
            const Vertex& vertex1 = triangle.vertices[1];
            const Vertex& vertex2 = triangle.vertices[2];

            // Barycentric coordinates.
            Float sqrt_u0 = pr::sqrt(u0);
            Float b0 = 1 - sqrt_u0;
            Float b1 = sqrt_u0 * samp[1];
            Float b2 = sqrt_u0 * (1 - samp[1]);

            // Set position.
            hit.pos = 
                b0 * vertex0.pos + 
                b1 * vertex1.pos + 
                b2 * vertex2.pos;
            hit.pos_abs_error = 
                (pr::abs(b0 * vertex0.pos) +
                 pr::abs(b1 * vertex1.pos) +
                 pr::abs(b2 * vertex2.pos)) * 
                 pr::numeric_limits<Float>::echelon(6);

            // Set normal.
            hit.normal = pr::normalize(triangle.normal());
            hit.normal_shading = 
                pr::normalize(
                    b0 * vertex0.normal_shading + 
                    b1 * vertex1.normal_shading + 
                    b2 * vertex2.normal_shading);
            if (pr::dot(hit.normal, hit.normal_shading) < 0) {
                hit.normal = -hit.normal;
            }
        }
    }

    return hit;
}

// Intersect.
bool TriangleMesh::intersect(const Ray& ray, Hit* hit) const
{
    // Result.
    bool res = false;

    // Associated triangle.
    const Triangle* res_triangle = nullptr;

    // Associated barycentric coordinates.
    Vec3f res_bhit;

    // Nodes stack.
    pr::static_stack<
    const LinearAabbTree3::node_type*, 64> nodes;
    nodes.push(triangles_tree_.begin());

    // Box intersector.
    RayBoxIntersector box_intersector(&ray);

    // Triangle intersector.
    RayTriangleIntersector triangle_intersector(&ray);

    while (!nodes.empty()) {
        // Node.
        const LinearAabbTree3::node_type* node = nodes.pop();

        // Intersect?
        if (box_intersector.intersect(node->box)) {

            // Is branch?
            if (node->is_branch()) {

                // Push children.
                nodes.push(node->left_child());
                nodes.push(node->right_child());
                if (pr::signbit(ray.dir[node->split_dim])) {
                    // Traverse in opposite order.
                    std::swap(
                            nodes[-1], 
                            nodes[-2]);
                }
            }
            else {

                // Iterate triangles.
                for (std::uint32_t 
                        index = node->first_index;
                        index < node->first_index + node->count;
                        index++) {

                    Vec3f tmp_bhit;
                    Float tmp_thit;
                    if (triangle_intersector.intersect(
                        triangles_[index].vertices[0].pos,
                        triangles_[index].vertices[1].pos,
                        triangles_[index].vertices[2].pos, 
                            &tmp_bhit, 
                            &tmp_thit)) {
                        res = true;
                        res_triangle = &triangles_[index];
                        res_bhit = tmp_bhit;
                        ray.tmax = tmp_thit; // Overwrite tmax.

                        // Quit if no hit requested.
                        if (!hit) {
                            return true;
                        }
                    }
                }
            }
        }
    }

    if (res) {
        const Vertex& vertex0 = res_triangle->vertices[0];
        const Vertex& vertex1 = res_triangle->vertices[1];
        const Vertex& vertex2 = res_triangle->vertices[2];
        Float b0 = res_bhit[0];
        Float b1 = res_bhit[1];
        Float b2 = res_bhit[2];

        // Set position.
        hit->pos = 
            b0 * vertex0.pos +
            b1 * vertex1.pos +
            b2 * vertex2.pos;
        hit->pos_abs_error = 
            (pr::abs(b0 * vertex0.pos) +
             pr::abs(b1 * vertex1.pos) +
             pr::abs(b2 * vertex2.pos)) * 
             pr::numeric_limits<Float>::echelon(6);

        // Set normal.
        hit->normal = pr::normalize(res_triangle->normal());
        hit->normal_shading = 
            pr::normalize(
                b0 * vertex0.normal_shading + 
                b1 * vertex1.normal_shading + 
                b2 * vertex2.normal_shading);
        if (pr::dot(hit->normal, hit->normal_shading) < 0) {
            hit->normal = -hit->normal;
        }
    }

    return res;
}
