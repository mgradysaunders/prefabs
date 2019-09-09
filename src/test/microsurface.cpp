#include <iostream>
#include <random>
#include <preform/random.hpp>
#include <preform/multi_random.hpp>
#include <preform/microsurface.hpp>
#include <preform/neumaier_sum.hpp>
#include <preform/option_parser.hpp>

// Float type.
typedef float Float;

// 2-dimensional vector type.
typedef pr::vec2<int> Vec2i;

// 2-dimensional vector type.
typedef pr::vec2<Float> Vec2f;

// 3-dimensional vector type.
typedef pr::vec3<Float> Vec3f;

// Diffuse microsurface with Trowbridge-Reitz slope distribution.
typedef pr::diffuse_brdf_microsurface_adapter<
        pr::trowbridge_reitz_microsurface_slope<Float>,
        pr::uniform_microsurface_height<Float>>
            DiffuseTrowbridgeReitz;

// Diffuse microsurface with Beckmann slope distribution.
typedef pr::diffuse_brdf_microsurface_adapter<
        pr::beckmann_microsurface_slope<Float>,
        pr::uniform_microsurface_height<Float>>
            DiffuseBeckmann;

// Dielectric microsurface with Trowbridge-Reitz slope distribution.
typedef pr::dielectric_bsdf_microsurface_adapter<
        pr::trowbridge_reitz_microsurface_slope<Float>,
        pr::uniform_microsurface_height<Float>>
            DielectricTrowbridgeReitz;

// Dielectric microsurface with Beckmann slope distribution.
typedef pr::dielectric_bsdf_microsurface_adapter<
        pr::beckmann_microsurface_slope<Float>,
        pr::uniform_microsurface_height<Float>>
            DielectricBeckmann;

// Neumaier sum.
typedef pr::neumaier_sum<Float> NeumaierSum;

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

// Test full-sphere scattering.
template <typename Microsurface>
void testFullSphere(const char* name, const Microsurface& microsurface)
{
    Vec2i n = {512, 512};
    std::cout << "Testing full-sphere scattering for ";
    std::cout << name << "::fm():\n";
    std::cout <<
        "This test uses Monte Carlo integration to estimate the full-sphere\n"
        "scattering integral, which is equal to 4 pi for an energy-conserving "
        "BSDF.\n";
    std::cout.flush();

    // Stratify samples.
    Vec2f* u0 = new Vec2f[n.prod()];
    Vec2f* u1 = new Vec2f[n.prod()];
    pr::stratify(&u0[0], n, pcg);
    pr::stratify(&u1[0], n, pcg);

    // Monte Carlo integration.
    NeumaierSum f = 0;
    for (int k = 0; k < n.prod(); k++) {

        // Directions.
        Vec3f wo = Vec3f::cosine_hemisphere_pdf_sample(u0[k]);
        Vec3f wi = Vec3f::cosine_hemisphere_pdf_sample(u1[k]);
        Float wo_pdf = Vec3f::cosine_hemisphere_pdf(wo[2]) / 2;
        Float wi_pdf = Vec3f::cosine_hemisphere_pdf(wi[2]) / 2;
        wo = pcg(2) == 0 ? +wo : -wo;
        wi = pcg(2) == 0 ? +wi : -wi;

        // Integrand.
        if (wo_pdf > 0 &&
            wi_pdf > 0) {
            Float fk = microsurface.fm(generateCanonical, wo, wi);
            fk /= wi_pdf;
            fk /= wo_pdf;
            fk /= n.prod();
            f += fk;
        }
    }

    std::cout << "Result: " << Float(f) << " (should be close to 12.56)\n";
    std::cout << "\n\n";
    std::cout.flush();

    delete[] u0;
    delete[] u1;
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

    // Generate refractive indices.
    Float eta0 = 1;
    Float eta1 = 1.1 + generateCanonical();
    std::cout << "eta0 = " << eta0 << "\n";
    std::cout << "eta1 = " << eta1 << "\n";

    // Generate roughness.
    Vec2f alpha = {
        generateCanonical() * 2 + Float(0.1),
        generateCanonical() * 2 + Float(0.1)
    };
    std::cout << "alpha = " << alpha << "\n\n";
    std::cout.flush();

    // Test full-sphere scattering.
    testFullSphere(
        "DiffuseTrowbridgeReitz",
         DiffuseTrowbridgeReitz(alpha));
    testFullSphere(
        "DiffuseBeckmann",
         DiffuseBeckmann(alpha));
    testFullSphere(
        "DielectricTrowbridgeReitz",
         DielectricTrowbridgeReitz(eta0 / eta1, alpha));
    testFullSphere(
        "DielectricBeckmann",
         DielectricBeckmann(eta0 / eta1, alpha));

    return EXIT_SUCCESS;
}
