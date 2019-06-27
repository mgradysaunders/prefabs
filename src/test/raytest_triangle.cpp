#include <iostream>
#include <random>
#include <preform/random.hpp>
#include <preform/multi.hpp>
#include <preform/multi_math.hpp>
#include <preform/raytest_triangle.hpp>
#include <preform/option_parser.hpp>

// Float type.
typedef float Float;

// 2-dimensional vector type.
typedef pr::vec2<Float> Vec2f;

// 3-dimensional vector type.
typedef pr::vec3<Float> Vec3f;

// Hit-test (triangle).
typedef pr::raytest_triangle<Float> RaytestTriangle;

// Permuted congruential generator.
pr::pcg32 pcg;

// Generate canonical random number.
Float generateCanonical()
{
    return pr::generate_canonical<Float>(pcg);
}

// Generate canonical random 2-dimensional vector.
Vec2f generateCanonical2()
{
    return {
        pr::generate_canonical<Float>(pcg),
        pr::generate_canonical<Float>(pcg)
    };
}

// Generate canonical random 3-dimensional vector.
Vec3f generateCanonical3()
{
    return {
        pr::generate_canonical<Float>(pcg),
        pr::generate_canonical<Float>(pcg),
        pr::generate_canonical<Float>(pcg)
    };
}

// Test epsilon spawning.
void testEpsilonSpawning(const RaytestTriangle& triangle)
{
    std::cout << "Testing epsilon spawning for RaytestTriangle:\n";
    std::cout << "This test fires rays away from random surface points. By\n";
    std::cout << "construction, none of the rays should intersect with the\n";
    std::cout << "surface. If a ray does intersect with the surface, this\n";
    std::cout << "test fails, indicating a bug in the ray-epsilon code.\n";
    std::cout.flush();

    for (int k = 0; k < 262144; k++) {
        Vec2f u0 = generateCanonical2();
        Vec2f u1 = generateCanonical2();
        Vec3f wi = pr::uniform_sphere_pdf_sample(u0);
        Vec3f wierr = {};
        RaytestTriangle::hit_type hit = triangle.surface_area_pdf_sample(u1);
        RaytestTriangle::ray_type ray = {
            hit.p,
            hit.perr,
            wi,
            wierr
        };
        Float t = triangle.intersect(ray);
        if (!pr::isnan(t)) {
            std::cerr << "Failure!\n";
            std::cerr << "ray.o = " << ray.o << "\n";
            std::cerr << "ray.d = " << ray.d << "\n";
            std::cerr << "ray.oerr = " << ray.oerr << "\n";
            std::cerr << "ray.derr = " << ray.derr << "\n";
            std::cerr << "t = " << t << "\n\n";
            std::exit(EXIT_FAILURE);
        }
    }

    std::cout << "Success (262144 tests).\n\n";
    std::cout.flush();
}

// Test epsilon shadowing.
void testEpsilonShadowing(const RaytestTriangle& triangle)
{
    std::cout << "Testing epsilon shadowing for RaytestTriangle:\n";
    std::cout << "This test fires rays toward random surface points with\n";
    std::cout << "tmax equal to the distance to the surface point. By\n";
    std::cout << "construction, none of the rays should intersect with the\n";
    std::cout << "surface. If a ray does intersect with the surface, this\n";
    std::cout << "test fails, indicating a bug in the ray-epsilon code.\n";
    std::cout.flush();

    for (int k = 0; k < 262144; k++) {
        Vec2f u0 = generateCanonical2();
        Vec2f u1 = generateCanonical2();
        Vec3f wi = pr::uniform_sphere_pdf_sample(u0);
        Vec3f vi = wi * (generateCanonical() * 400 + Float(0.0001));
        RaytestTriangle::hit_type hit = triangle.surface_area_pdf_sample(u1);
        RaytestTriangle::ray_type ray = {
            hit.p - vi,
            vi,
            Float(0),
            Float(1)
        };
        Float t = triangle.intersect(ray);
        if (!pr::isnan(t)) {
            std::cerr << "Failure!\n";
            std::cerr << "ray.o = " << ray.o << "\n";
            std::cerr << "ray.d = " << ray.d << "\n";
            std::cerr << "ray.oerr = " << ray.oerr << "\n";
            std::cerr << "ray.derr = " << ray.derr << "\n";
            std::cerr << "ray.tmax = " << ray.tmax << "\n";
            std::cerr << "t = " << t << "\n\n";
            std::exit(EXIT_FAILURE);
        }
    }

    std::cout << "Success (262144 tests).\n\n";
    std::cout.flush();
}

int main(int argc, char** argv)
{
    int seed = 0;

    // Option parser.
    pr::option_parser opt_parser("[OPTIONS]");

    // Specify seed.
    opt_parser.on_option(
    "-s", "--seed", 1,
    [&](char** argv) {
        try {
            seed = std::stoi(argv[0]);
        }
        catch (const std::exception&) {
            throw
                std::runtime_error(
                std::string("-s/--seed expects 1 integer ")
                    .append("(can't parse ").append(argv[0])
                    .append(")"));
        }
    })
    << "Specify seed. By default, random.\n";

    // Display help.
    opt_parser.on_option(
    "-h", "--help", 0,
    [&](char**) {
        std::cout << opt_parser << std::endl;
        std::exit(EXIT_SUCCESS);
    })
    << "Display this help and exit.\n";

    try {
        // Parse args.
        opt_parser.parse(argc, argv);
    }
    catch (const std::exception& exception) {
        std::cerr << "Unhandled exception!\n";
        std::cerr << "exception.what(): " << exception.what() << "\n";
        std::exit(EXIT_FAILURE);
    }

    // Seed.
    if (seed == 0) {
        seed = std::random_device()();
    }
    std::cout << "seed = " << seed << "\n";
    pcg = pr::pcg32(seed);

    // Triangle vertices.
    Vec3f p0 = generateCanonical3() * 2 - Vec3f{10, 3, 2};
    Vec3f p1 = generateCanonical3() * 5 + Vec3f{15, 10, -10};
    Vec3f p2 = generateCanonical3() * 3 + Vec3f{-8, 19, 5};
    std::cout << "p0 = " << p0 << "\n";
    std::cout << "p1 = " << p1 << "\n";
    std::cout << "p2 = " << p2 << "\n\n";

    // Test.
    RaytestTriangle triangle(p0, p1, p2);
    testEpsilonSpawning(triangle);
    testEpsilonShadowing(triangle);
    return EXIT_SUCCESS;
}
