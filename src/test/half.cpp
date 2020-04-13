#include <iostream>
#include <iomanip>
#include <random>
#include <preform/random.hpp>
#include <preform/option_parser.hpp>
#include <preform/half.hpp>

// Permuted congruential generator.
pre::pcg32 pcg;

// Test half accuracy.
void testHalfAccuracy()
{
    std::cout << "Testing half accuracy:\n";
    std::cout << "This test converts random single precision floats\n";
    std::cout << "between -65504 and +65504 to half precision and back, and\n";
    std::cout << "verifies that the relative error never exceeds 0x1p-11.\n";
    std::cout.flush();

    for (int k = 0; k < 8192; k++) {
        float x0 = pre::copysign(65504.0f *
                   pre::generate_canonical<float>(pcg),
                            pcg(2) == 0 ? +1.0f : -1.0f);

        float xh = pre::half(x0);
        if (!(pre::fabs(xh - x0) / pre::fabs(x0) <= 0x1p-11f)) {
            std::cerr << "Failure!\n";
            std::cerr << "x0 = " << x0 << "\n";
            std::cerr << "xh = " << xh << "\n\n";
            std::exit(EXIT_FAILURE);
        }
    }

    std::cout << "Success (8192 tests).\n\n";
    std::cout.flush();
}

// Test half special cases.
void testHalfSpecialCase(float f)
{
    pre::half h = f;
    std::cout << "Testing half " << f << ":\n";
    std::cout << "half(" << f << ").isinf(): " << h.isinf() << "\n";
    std::cout << "half(" << f << ").isnan(): " << h.isnan() << "\n";
    std::cout << "half(" << f << ").signbit(): " << h.signbit() << "\n";
    std::cout << "float(half(" << h << ")): " << float(h) << "\n\n";
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
    std::cout << "seed = " << seed << "\n\n";
    std::cout.flush();
    pcg = pre::pcg32(seed);

    // Test half accuracy.
    testHalfAccuracy();

    // Test half special cases.
    std::cout << std::showpos;
    std::cout << std::boolalpha;
    testHalfSpecialCase(+0.0f);
    testHalfSpecialCase(-0.0f);
    testHalfSpecialCase(+pre::numeric_limits<float>::infinity());
    testHalfSpecialCase(-pre::numeric_limits<float>::infinity());
    testHalfSpecialCase(+pre::numeric_limits<float>::quiet_NaN());
    testHalfSpecialCase(-pre::numeric_limits<float>::quiet_NaN());

    return EXIT_SUCCESS;
}

