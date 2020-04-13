#define PREFORM_USE_FENV 1
#include <iostream>
#include <iomanip>
#include <random>
#include <preform/random.hpp>
#include <preform/option_parser.hpp>
#include <preform/float_interval.hpp>

// High-precision float type.
typedef long double Highp;

// Float type.
typedef float Float;

// Float interval type.
typedef pre::float_interval<Float> FloatInterval;

// Permuted-congruential generator.
pre::pcg32 pcg;

void testSequential()
{
    std::cout << "Testing sequential operations:\n";
    std::cout << "This test performs 16384 random floating point operations\n";
    std::cout << "to a FloatInterval and a Highp reference sequentially to\n";
    std::cout << "verify the correctness of the FloatInterval bounds.\n";
    std::cout.flush();

    FloatInterval fi = {};
    Highp fh = 0;
    for (int k = 0; k < 16384; k++) {
        Float f = pre::generate_canonical<Float>(pcg) * 64 - 32;
        switch (pcg(5)) {
            // Add.
            case 0:
                fi += f;
                fh += f;
                break;
            // Subtract.
            case 1:
                fi -= f;
                fh -= f;
                break;
            // Multiply.
            case 2:
                fi *= f;
                fh *= f;
                break;
            // Divide.
            case 3:
                if (f != 0) {
                    fi /= f;
                    fh /= f;
                }
                break;
            // Square-root.
            case 4:
                fi = pre::sqrt(pre::fabs(fi));
                fh = pre::sqrt(pre::fabs(fh));
                break;
            default:
                break;
        }
    }

    std::cout << std::boolalpha;
    std::cout << std::setprecision(20);
    std::cout << "fi.value() = " << fi.value() << "\n";
    std::cout << "fi.lower_bound() = " << fi.lower_bound() << "\n";
    std::cout << "fi.upper_bound() = " << fi.upper_bound() << "\n";
    std::cout << "fi.abs_error() = " << fi.abs_error() << "\n";
    std::cout << "fi.rel_error() = " << fi.rel_error() << "\n";
    std::cout << "fi.contains(fh) = " << fi.contains<true, true>(fh) << "\n\n";
    std::cout.flush();
}

void testPairwise()
{
    std::cout << "Testing pairwise operations:\n";
    std::cout << "This test performs 16384 random floating point operations\n";
    std::cout << "between random independent FloatInterval pairs and random\n";
    std::cout << "corresponding Highp pairs, checking that the result\n";
    std::cout << "FloatInterval contains the result Highp to verify the\n";
    std::cout << "correctness of the FloatInterval bounds.\n";
    std::cout.flush();

    pre::normal_distribution<Float> distr_val(0, 100);
    pre::exponential_distribution<Float> distr_err(1);
    for (int k = 0; k < 16384; k++) {
        // Generate random values.
        Float f0 = distr_val(pcg);
        Float f1 = distr_val(pcg);

        // Generate random lower and upper bounds.
        FloatInterval fi0 = {
            f0,
            f0 - distr_err(pcg),
            f0 + distr_err(pcg)
        };
        FloatInterval fi1 = {
            f1,
            f1 - distr_err(pcg),
            f1 + distr_err(pcg)
        };

        // Generate random high-precision values in intervals.
        Highp fh0 =
            pre::lerp(
            pre::generate_canonical<Highp>(pcg),
            Highp(fi0.lower_bound()),
            Highp(fi0.upper_bound()));
        Highp fh1 =
            pre::lerp(
            pre::generate_canonical<Highp>(pcg),
            Highp(fi1.lower_bound()),
            Highp(fi1.upper_bound()));

        // Random operation.
        FloatInterval fi;
        Highp fh = 0;
        switch (pcg(4)) {
            case 0:
                fi = fi0 + fi1;
                fh = fh0 + fh1;
                break;
            case 1:
                fi = fi0 - fi1;
                fh = fh0 - fh1;
                break;
            case 2:
                fi = fi0 * fi1;
                fh = fh0 * fh1;
                break;
            case 3:
                fi = fi0 / fi1;
                fh = fh0 / fh1;
                break;
            default:
                break;
        }

        if (!pre::isnan(fi.value()) &&
            !fi.contains<true, true>(fh)) {
            std::cerr << "Failure!\n";
            std::cerr << "fi0.value() = " << fi0.value() << "\n";
            std::cerr << "fi0.lower_bound() = " << fi0.lower_bound() << "\n";
            std::cerr << "fi0.upper_bound() = " << fi0.upper_bound() << "\n";
            std::cerr << "fi1.value() = " << fi1.value() << "\n";
            std::cerr << "fi1.lower_bound() = " << fi1.lower_bound() << "\n";
            std::cerr << "fi1.upper_bound() = " << fi1.upper_bound() << "\n";
            std::cerr << "fi.value() = " << fi.value() << "\n";
            std::cerr << "fi.lower_bound() = " << fi.lower_bound() << "\n";
            std::cerr << "fi.upper_bound() = " << fi.upper_bound() << "\n";
            std::cerr << "fh = " << fh << "\n\n";
            std::exit(EXIT_FAILURE);
        }
    }

    std::cout << "Success (16384 tests).\n\n";
    std::cout.flush();
}

int main(int argc, char** argv)
{
    int seed = 0;

    // Option parser.
    pre::option_parser opt_parser("[OPTIONS]");

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
    std::cout << "seed = " << seed << std::endl;
    std::cout << std::endl;
    pcg = pre::pcg32(seed);

    // Test sequential operations.
    testSequential();

    // Test pairwise operations.
    testPairwise();

    return EXIT_SUCCESS;
}
