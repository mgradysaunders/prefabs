#include <iostream>
#include <random>
#include <preform/random.hpp>
#include <preform/sampling.hpp>
#include <preform/raytest_disk.hpp>
#include <preform/option_parser.hpp>

// Float type.
typedef float Float;

// 2-dimensional vector type.
typedef pr::vec2<Float> Vec2f;

// 3-dimensional vector type.
typedef pr::vec3<Float> Vec3f;

// Ray-test (disk).
typedef pr::raytest_disk<Float> RaytestDisk;

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

// Test spawning.
void testEpsilonSpawning(const RaytestDisk& disk)
{
    std::cout << "Testing epsilon spawning for RaytestDisk:\n";
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
        RaytestDisk::hit_info hit = disk.surface_area_pdf_sample(u1);
        RaytestDisk::ray_info ray = {
            hit.p,
            hit.perr,
            wi,
            wierr
        };
        Float t = disk.intersect(ray);
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
void testEpsilonShadowing(const RaytestDisk& disk)
{
    std::cout << "Testing epsilon shadowing for RaytestDisk:\n";
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
        RaytestDisk::hit_info hit = disk.surface_area_pdf_sample(u1);
        RaytestDisk::ray_info ray = {
            hit.p - vi,
            wi,
            Float(0),
            Float(pr::length(vi))
        };
        Float t = disk.intersect(ray);
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

    // Disk parameters.
    Float rmin = generateCanonical() * Float(0.5);
    Float rmax = generateCanonical() * Float(2.5) + 1;
    Float h = generateCanonical() * 100 - 50;
    std::cout << "rmin = " << rmin << "\n";
    std::cout << "rmax = " << rmax << "\n";
    std::cout << "h = " << h << "\n\n";

    // Test.
    RaytestDisk disk(rmin, rmax, h);
    testEpsilonSpawning(disk);
    testEpsilonShadowing(disk);
    return EXIT_SUCCESS;
}
