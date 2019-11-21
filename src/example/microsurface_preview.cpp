#include <iostream>
#include <fstream>
#include <random>
#include <preform/random.hpp>
#include <preform/multi_math.hpp>
#include <preform/multi_random.hpp>
#include <preform/microsurface.hpp>
#include <preform/color.hpp>
#include <preform/image2.hpp>
#include <preform/image_filters.hpp>
#include <preform/float_interval.hpp>
#include <preform/option_parser.hpp>

// Float type.
typedef float Float;

// Float interval type.
typedef pr::float_interval<Float> FloatInterval;

// 1-dimensional vector type.
typedef pr::vec1<Float> Vec1f;

// 2-dimensional vector type.
typedef pr::vec2<int> Vec2i;

// 2-dimensional vector type.
typedef pr::vec2<Float> Vec2f;

// 3-dimensional vector type.
typedef pr::vec3<Float> Vec3f;

// 3-by-3 matrix type.
typedef pr::mat3<Float> Mat3f;

// Image.
typedef pr::image2<Float, Float, 1> Image2x1;

// Image Mitchell filter.
typedef pr::mitchell_filter2<Float> MitchellFilter2;

// Microsurface diffuse with Trowbridge-Reitz slope distribution.
typedef pr::microsurface_lambertian_brdf<
        Float,
        pr::microsurface_trowbridge_reitz_slope,
        pr::microsurface_uniform_height>
            MicrosurfaceLambertianTrowbridgeReitz;

// Microsurface diffuse with Beckmann slope distribution.
typedef pr::microsurface_lambertian_brdf<
        Float,
        pr::microsurface_beckmann_slope,
        pr::microsurface_uniform_height>
            MicrosurfaceLambertianBeckmann;

// Microsurface dielectric with Trowbridge-Reitz slope distribution.
typedef pr::microsurface_dielectric_bsdf<
        Float,
        pr::microsurface_trowbridge_reitz_slope,
        pr::microsurface_uniform_height>
            MicrosurfaceDielectricTrowbridgeReitz;

// Microsurface dielectric with Beckmann slope distribution.
typedef pr::microsurface_dielectric_bsdf<
        Float,
        pr::microsurface_beckmann_slope,
        pr::microsurface_uniform_height>
            MicrosurfaceDielectricBeckmann;

// Oren-Nayar diffuse.
typedef pr::oren_nayar_diffuse_brdf<Float>
            OrenNayarDiffuse;

// Disney diffuse.
typedef pr::disney_diffuse_brdf<Float>
            DisneyDiffuse;

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

struct Ray
{
    // Origin.
    Vec3f o;

    // Direction.
    Vec3f d;

    Float tmin = 0;

    Float tmax = pr::numeric_limits<Float>::infinity();
};

struct Hit
{
    // Position.
    Vec3f p;

    // Normal.
    Vec3f n;
};

bool intersectSphere(Ray ray, Hit& hit)
{
    FloatInterval t0;
    FloatInterval t1;
    FloatInterval::solve_poly2(
            pr::dot(ray.o, ray.o) - 1,
            pr::dot(ray.d, ray.o) * 2,
            pr::dot(ray.d, ray.d),
            t0, t1);
    if (!(t0.upper_bound() < ray.tmax &&
          t1.lower_bound() > ray.tmin)) {
        return false;
    }

    // Select root.
    FloatInterval t = t0;
    if (!(t.upper_bound() < ray.tmax &&
          t.lower_bound() > ray.tmin)) {
        t = t1;
        if (!(t.upper_bound() < ray.tmax &&
              t.lower_bound() > ray.tmin)) {
            return false;
        }
    }

    // Initialize hit.
    hit.p = 
    hit.n = pr::normalize_fast(ray.o + ray.d * t.value());
    return true;
}

int main(int argc, char** argv)
{
    // Configurable options.
    int seed = 0;
    Vec2i image_dim = {512, 512};
    Vec2f image_filter_rad = {1, 1};
    std::string ofs_name = "sphere.pgm";

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
    << "Specify seed. By default, 0.\n";

    // Specify image dimensions.
    opt_parser.on_option(
    nullptr, "--image-dim", 2,
    [&](char** argv) {
        try {
            image_dim[0] = std::stoi(argv[0]);
            image_dim[1] = std::stoi(argv[1]);
            if (!(image_dim > 0).all()) {
                throw std::exception();
            }
        }
        catch (const std::exception&) {
            throw
                std::runtime_error(
                std::string("--image-dim expects 2 positive integers ")
                    .append("(can't parse ")
                    .append(argv[0]).append(" ")
                    .append(argv[1]).append(")"));
        }
    })
    << "Specify image dimensions. By default, 512x512.\n";

    // Specify image filter radii.
    opt_parser.on_option(
    nullptr, "--image-filter-rad", 2,
    [&](char** argv) {
        try {
            image_filter_rad[0] = std::stof(argv[0]);
            image_filter_rad[1] = std::stof(argv[1]);
            if (!(image_filter_rad > 0).all()) {
                throw std::exception();
            }
        }
        catch (const std::exception&) {
            throw
                std::runtime_error(
                std::string("--image-filter-rad expects 2 positive floats ")
                    .append("(can't parse ")
                    .append(argv[0]).append(" ")
                    .append(argv[1]).append(")"));
        }
    })
    << "Specify image reconstruction filter radii. By default, 1x1.\n";

    // Specify output filename.
    opt_parser.on_option(
    "-o", "--output", 1,
    [&](char** argv) {
        ofs_name = argv[0];
    })
    << "Specify output filename. By default, sphere.pgm.\n";

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

    Image2x1 image;
    image.resize(image_dim);
    MitchellFilter2 image_filter;

    Vec3f l0 = {5, -1, -4};
    Vec3f l1 = {-1, -2, 2};
    l0 = pr::normalize(l0);
    l1 = pr::normalize(l1);

    for (int i = 0; i < image_dim[0]; i++)
    for (int j = 0; j < image_dim[1]; j++) {
        // Supersample.
        for (int k = 0; k < 3; k++)
        for (int l = 0; l < 3; l++) {
            Vec2f image_loc = {
                Float(i) + Float(k) / 3,
                Float(j) + Float(l) / 3
            };
            Vec3f p0 = {0, 0, -4};
            Vec3f p1 = {
                3 * (image_loc[0] / image_dim[0] - Float(0.5)),
                3 * (image_loc[1] / image_dim[1] - Float(0.5)),
                0
            };
            Ray ray;
            ray.o = p0;
            ray.d = pr::normalize(p1 - p0);
            ray.tmin = 0;
            ray.tmax = pr::numeric_limits<Float>::infinity();
            Hit hit;
            if (intersectSphere(ray, hit)) {
                Mat3f tbn = Mat3f::build_onb(hit.n);
                Vec3f wo = pr::dot(pr::transpose(tbn), -ray.d);
                Vec3f wi0 = pr::dot(pr::transpose(tbn), l0);
                Vec3f wi1 = pr::dot(pr::transpose(tbn), l1);
                Vec1f f = {
                    DisneyDiffuse(1.0).fs(wo, wi0) + 
                    DisneyDiffuse(1.0).fs(wo, wi1)
                };
                image.reconstruct(
                        f / 9,
                        image_loc,
                        image_filter_rad,
                        image_filter);
            }
        }
    }

    try {
        std::ofstream ofs(ofs_name);
        ofs << "P2\n";
        ofs << image_dim[0] << ' ';
        ofs << image_dim[1] << '\n';
        ofs << "255\n";
        for (int j = 0; j < image_dim[1]; j++)
        for (int i = 0; i < image_dim[0]; i++) {
            ofs << int(pr::pack_uint8(
                       pr::srgbenc(image(i, j)[0]))) << ' ';
        }
    }
    catch (const std::exception& exception) {
        std::cerr << "Unhandled exception!\n";
        std::cerr << "exception.what(): " << exception.what() << "\n";
        std::exit(EXIT_FAILURE);
    }
    return EXIT_SUCCESS;
}

