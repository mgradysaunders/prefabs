#include <iostream>
#include <random>
#include <preform/random.hpp>
#include <preform/sampling.hpp>
#include <preform/raytest_sphere.hpp>
#include <preform/option_parser.hpp>

// Float type.
typedef float Float;

// 2-dimensional vector type.
typedef pr::vec2<int> Vec2i;

// 2-dimensional vector type.
typedef pr::vec2<Float> Vec2f;

// 3-dimensional vector type.
typedef pr::vec3<Float> Vec3f;

// Ray-test (sphere).
typedef pr::raytest_sphere<Float> RaytestSphere;

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
    Float r = generateCanonical() * Float(10) + 1;
    std::cout << "r = " << r << "\n\n";

    std::cout << "Testing RaytestSphere:\n";
    std::cout << "This test fires rays away from random surface points. By\n";
    std::cout << "construction, none of the rays should intersect with the\n";
    std::cout << "surface. If a ray does intersect with the surface, this\n";
    std::cout << "test fails, indicating a bug in the ray-epsilon code.\n";
    std::cout.flush();
    RaytestSphere sphere(r);

    for (int k = 0; k < 262144; k++) {
        Vec2f u0 = generateCanonical2();
        Vec2f u1 = generateCanonical2();
        Vec3f wi = pr::uniform_hemisphere_pdf_sample(u0);
        Vec3f wierr = {};
        RaytestSphere::hit_info hit = sphere.surface_area_pdf_sample(u1);
        wi = pr::dot(
             pr::build_onb(pr::normalize(hit.p)), wi);
        RaytestSphere::ray_info ray = {
            hit.p,
            hit.perr,
            wi,
            wierr
        };
        Float t = sphere.intersect(ray);
        if (!pr::isnan(t)) {
            std::cerr << "Failure!\n";
            std::cerr << "ray.o = " << ray.o << '\n';
            std::cerr << "ray.d = " << ray.d << '\n';
            std::cerr << "ray.oerr = " << ray.oerr << '\n';
            std::cerr << "ray.derr = " << ray.derr << '\n';
            std::cerr << "t = " << t << '\n';
            std::exit(EXIT_FAILURE);
        }
    }

    std::cout << "Success (262144 tests).\n\n";
    std::cout.flush();
    return EXIT_SUCCESS;
}
