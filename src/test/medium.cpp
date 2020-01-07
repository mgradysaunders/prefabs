#include <iostream>
#include <random>
#include <preform/multi.hpp>
#include <preform/multi_math.hpp>
#include <preform/multi_random.hpp>
#include <preform/quat.hpp>
#include <preform/medium.hpp>
#include <preform/neumaier_sum.hpp>
#include <preform/option_parser.hpp>

// Float.
typedef double Float;

// 2-dimensional vector.
typedef pr::vec2<int> Vec2i;

// 2-dimensional vector.
typedef pr::vec2<Float> Vec2f;

// 3-dimensional vector.
typedef pr::vec3<Float> Vec3f;

// 3-dimensional matrix.
typedef pr::mat3<Float> Mat3f;

// Quaternion.
typedef pr::quat<Float> Quatf;

// Henyey-Greenstein phase.
typedef pr::hg_phase<Float>
        HgPhase;

// Henyey-Greenstein phase stack.
typedef pr::hg_phase_stack<Float, 4>
        HgPhaseStack;

// Rayleigh phase.
typedef pr::rayleigh_phase<Float>
        RayleighPhase;

// Microvolume SGGX specular phase.
typedef pr::microvolume_sggx_specular_phase<Float>
        MicrovolumeSggxSpecularPhase;

// Microvolume SGGX diffuse phase.
typedef pr::microvolume_sggx_diffuse_phase<Float>
        MicrovolumeSggxDiffusePhase;

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

// Test phase function normalization.
template <typename Func, typename Pred>
void testPhase(const char* name, const Func& func, Pred&& pred)
{
    Vec2i n = {512, 512};
    std::cout << "Testing phase function for ";
    std::cout << name << "::ps():\n";
    std::cout <<
        "This test uses Monte Carlo integration to estimate the\n"
        "phase function integral over the sphere, which should equal 1 for\n"
        "an arbitrary viewing direction.\n";
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

        // Incident direction.
        Vec3f wi = Vec3f::uniform_sphere_pdf_sample(u0[k]);
        Float wi_pdf = Vec3f::uniform_sphere_pdf();

        // Integrand.
        if (wi_pdf > 0) {
            Float fk = std::forward<Pred>(pred)(func, wo, wi);
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

// Test phase function sampling.
template <typename Func, typename Pred>
void testPhaseSampling(const char* name, const Func& func, Pred&& pred)
{
    Vec2i n = {512, 512};
    std::cout << "Testing phase function sampling for ";
    std::cout << name << "::ps_sample():\n";
    std::cout <<
        "This test uses Monte Carlo integration to estimate the\n"
        "phase function integral over the sphere, which should equal 1 for\n"
        "an arbitrary viewing direction.\n";
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

        // Incident direction.
        Vec3f wi;
        Float wi_pdf = 0;
        std::forward<Pred>(pred)(func, u0[k], wo, wi, wi_pdf);

        // Integrand.
        if (wi_pdf > 0) {
            Float fk = pr::fabs(wi[2]);
            fk /= wi_pdf;
            fk /= n.prod();
            if (pr::isinf(fk)) {
                continue;
            }
            f += fk;
        }
    }

    std::cout << "Result: " << Float(f) << " (should be close to 6.28)\n";
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
    std::cout << "\n";
    pcg = pr::pcg32(seed);

    // Generate Henyey-Greenstein parameters.
    pr::multi<Float, 4> g = pr::generate_canonical<Float, 4>(pcg);
    pr::multi<Float, 4> w = pr::generate_canonical<Float, 4>(pcg);
    g = Float(1.999) * g - Float(0.999);
    w = w / w.sum();
    std::cout << "Henyey-Greenstein parameters:\n";
    std::cout << "g = " << g << "\n";
    std::cout << "w = " << w << "\n";
    std::cout << "\n";
    std::cout.flush();

    // Generate Rayleigh parameter.
    Float rho = Float(1.999) * generateCanonical() - Float(0.999);
    std::cout << "Rayleigh parameters:\n";
    std::cout << "rho = " << rho << "\n";
    std::cout << "\n";
    std::cout.flush();

    // Generate SGGX parameters.
    Quatf q =
    Quatf::rotate(
            0 * 
            pr::numeric_constants<Float>::M_pi() *
            generateCanonical(),
            Vec3f::uniform_sphere_pdf_sample(generateCanonical2()));
    Vec3f s = 2 * pr::generate_canonical<Float, 3>(pcg) - Float(0.0001);
    s = {0.05 * 0.05, 0.05 * 0.05, 1};
    std::cout << "SGGX parameters:\n";
    std::cout << "q = " << q << "\n";
    std::cout << "s = " << s << "\n";
    std::cout << "\n\n";
    std::cout.flush();

    {
        // Test phase function normalization.
        auto default_pred =
        [=](const auto& func,
            const Vec3f& wo,
            const Vec3f& wi) {
            return func.ps(wo, wi);
        };
        auto diffuse_pred =
        [=](const auto& func,
            const Vec3f& wo,
            const Vec3f& wi) {
            return func.ps(generateCanonical2(), wo, wi);
        };
        testPhase(
            "HgPhase",
             HgPhase(g[0]),
             default_pred);
        testPhase(
            "HgPhaseStack[4]",
             HgPhaseStack(g, w),
             default_pred);
        testPhase(
            "RayleighPhase",
             RayleighPhase(rho),
             default_pred);
        testPhase(
            "MicrovolumeSggxSpecularPhase",
             MicrovolumeSggxSpecularPhase(Mat3f(q), s),
             default_pred);
        testPhase(
            "MicrovolumeSggxDiffusePhase",
             MicrovolumeSggxDiffusePhase(Mat3f(q), s),
             diffuse_pred);
    }
    {
        // Test phase function sampling.
        auto default_pred =
        [=](const auto& func,
            const Vec2f& u0,
            const Vec3f& wo,
            Vec3f& wi,
            Float& wi_pdf) {
            wi = func.ps_sample(u0, wo), wi_pdf = func.ps(wo, wi);
        };
        auto diffuse_pred =
        [=](const auto& func,
            const Vec2f& u0,
            const Vec3f& wo,
            Vec3f& wi,
            Float& wi_pdf) {
            Vec2f u1 = generateCanonical2();
            wi = func.ps_sample(u1, u0, wo, &wi_pdf);
        };
        testPhaseSampling(
            "HgPhase",
             HgPhase(g[0]),
             default_pred);
        testPhaseSampling(
            "HgPhaseStack[4]",
             HgPhaseStack(g, w),
             default_pred);
        testPhaseSampling(
            "RayleighPhase",
             RayleighPhase(rho),
             default_pred);
        testPhaseSampling(
            "MicrovolumeSggxSpecularPhase",
             MicrovolumeSggxSpecularPhase(Mat3f(q), s),
             default_pred);
        testPhaseSampling(
            "MicrovolumeSggxDiffusePhase",
             MicrovolumeSggxDiffusePhase(Mat3f(q), s),
             diffuse_pred);
    }
    return 0;
}
