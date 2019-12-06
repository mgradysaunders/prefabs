#include <iostream>
#include <random>
#include <preform/multi.hpp>
#include <preform/multi_math.hpp>
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

// Lambertian with Trowbridge-Reitz slope distribution.
typedef pr::microsurface_lambertian_brdf<
        Float,
        pr::microsurface_trowbridge_reitz_slope,
        pr::microsurface_uniform_height>
            LambertianTrowbridgeReitz;

// Lambertian with Beckmann slope distribution.
typedef pr::microsurface_lambertian_brdf<
        Float,
        pr::microsurface_beckmann_slope,
        pr::microsurface_uniform_height>
            LambertianBeckmann;

// Dielectric with Trowbridge-Reitz slope distribution.
typedef pr::microsurface_dielectric_bsdf<
        Float,
        pr::microsurface_trowbridge_reitz_slope,
        pr::microsurface_uniform_height>
            DielectricTrowbridgeReitz;

// Dielectric with Beckmann slope distribution.
typedef pr::microsurface_dielectric_bsdf<
        Float,
        pr::microsurface_beckmann_slope,
        pr::microsurface_uniform_height>
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
    return pr::generate_canonical<Float, 2>(pcg);
}

// Test full-sphere scattering.
template <typename Microsurface>
void testFullSphere(const char* name, const Microsurface& microsurface)
{
    Vec2i n = {512, 512};
    std::cout << "Testing full-sphere scattering for ";
    std::cout << name << "::fs():\n";
    std::cout <<
        "This test uses Monte Carlo integration to estimate the full-sphere\n"
        "scattering integral, which is equal to 4 pi for an energy-conserving "
        "BSDF.\n";
    std::cout.flush();

    // Stratify samples.
    Vec2f* u0 = new Vec2f[n.prod()];
    Vec2f* u1 = new Vec2f[n.prod()];
    pr::stratify(u0, n, pcg);
    pr::stratify(u1, n, pcg);

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
            Float fk = microsurface.fs(generateCanonical, wo, wi);
            fk /= wi_pdf;
            fk /= wo_pdf;
            fk /= n.prod();
            if (pr::isinf(fk)) {
                continue;
            }
            f += fk;
        }
    }

    std::cout << "Result: " << Float(f) << " (should be close to 12.56)\n";
    std::cout << "\n\n";
    std::cout.flush();

    delete[] u0;
    delete[] u1;
}

// Test multi-scatter phase function.
template <typename Microsurface, typename Pred>
void testPhase(const char* name, const Microsurface& microsurface, Pred&& pred)
{
    Vec2i n = {512, 512};
    std::cout << "Testing multi-scatter phase function for ";
    std::cout << name << "::ps():\n";
    std::cout <<
        "This test uses Monte Carlo integration to estimate the multiple\n"
        "scattering phase function integral, which should equal 1 for an\n"
        "arbitrary viewing direction.\n";
    std::cout.flush();

    // Stratify samples.
    Vec2f* u0 = new Vec2f[n.prod()];
    pr::stratify(u0, n, pcg);
    
    // Generate random viewing direction.
    Vec3f wo = 
    Vec3f::uniform_sphere_pdf_sample(generateCanonical2());

    // Monte Carlo integration.
    NeumaierSum f = 0;
    for (int k = 0; k < n.prod(); k++) {

        // Inident direction.
        Vec3f wi = Vec3f::uniform_sphere_pdf_sample(u0[k]);
        Float wi_pdf = Vec3f::uniform_sphere_pdf();

        // Integrand.
        if (wi_pdf > 0) {
            Float fk = std::forward<Pred>(pred)(microsurface, wo, wi);
            fk /= wi_pdf;
            fk /= n.prod();
            if (pr::isinf(fk)) {
                continue;
            }
            f += fk;
        }
    }

    std::cout << "Result: " << Float(f) << " (should be close to 1)\n";
    std::cout << "\n\n";
    std::cout.flush();

    delete[] u0;
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
        "LambertianTrowbridgeReitz",
         LambertianTrowbridgeReitz(1, alpha));
    testFullSphere(
        "LambertianBeckmann",
         LambertianBeckmann(1, alpha));
    testFullSphere(
        "DielectricTrowbridgeReitz",
         DielectricTrowbridgeReitz(1, 1, eta0 / eta1, alpha));
    testFullSphere(
        "DielectricBeckmann",
         DielectricBeckmann(1, 1, eta0 / eta1, alpha));

    // Test phase function normalization.
    auto lambertian_pred = 
    [=](const auto& microsurface, 
        const Vec3f& wo, 
        const Vec3f& wi) {
        return microsurface.ps(generateCanonical2(), wo, wi);
    };
    auto dielectric_pred = 
    [=](const auto& microsurface, 
        const Vec3f& wo, 
        const Vec3f& wi) {
        return microsurface.ps(wo, wi, wo[2] > 0, true) +
               microsurface.ps(wo, wi, wo[2] > 0, false);
    };
    testPhase(
        "LambertianTrowbridgeReitz",
         LambertianTrowbridgeReitz(1, alpha),
         lambertian_pred);
    testPhase(
        "LambertianBeckmann",
         LambertianBeckmann(1, alpha),
         lambertian_pred);
    testPhase(
        "DielectricTrowbridgeReitz",
         DielectricTrowbridgeReitz(1, 1, eta0 / eta1, alpha),
         dielectric_pred);
    testPhase(
        "DielectricBeckmann",
         DielectricBeckmann(1, 1, eta0 / eta1, alpha),
         dielectric_pred);
    return EXIT_SUCCESS;
}
