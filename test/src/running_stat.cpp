#include <iostream>
#include <random>
#include <preform/random.hpp>
#include <preform/running_stat.hpp>
#include <preform/option_parser.hpp>

// Float type.
typedef double Float;

// Running statistic.
typedef pr::running_stat<Float> RunningStat;

// Normal distribution.
typedef pr::normal_distribution<Float> NormalDistribution;

// Log-normal distribution.
typedef pr::lognormal_distribution<Float> LognormalDistribution;

// Logistic distribution.
typedef pr::logistic_distribution<Float> LogisticDistribution;

// Permuted congruential generator.
pr::pcg32 pcg;

// Test distribution.
template <typename Distribution>
void testDistribution(
        const char* name,
        const Distribution& distr,
        Float kurtosis)
{
    std::cout << "Testing stat for " << name << ":\n";
    std::cout.flush();

    // Run.
    RunningStat stat;
    Float term;
    for (int k = 0; k < 8388608; k++) {
        if (pr::isfinite((term = distr(pcg)))) {
            stat += term;
        }
    }

    // Result mean.
    std::cout << "stat.mean(): ";
    std::cout <<  stat.mean();
    std::cout << " (this should be close to " << distr.mean() << ")\n";

    // Result variance.
    std::cout << "stat.variance(): ";
    std::cout <<  stat.variance();
    std::cout << " (this should be close to " << distr.variance() << ")\n";

    // Result skewness.
    std::cout << "stat.skewness(): ";
    std::cout <<  stat.skewness();
    std::cout << " (this should be close to " << distr.skewness() << ")\n";

    // Result kurtosis.
    std::cout << "stat.kurtosis(): ";
    std::cout <<  stat.kurtosis();
    std::cout << " (this should be close to " << kurtosis << ")\n\n";
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
    std::cout << "seed = " << seed << std::endl;
    std::cout << std::endl;
    pcg = pr::pcg32(seed);

    // Normal distribution.
    testDistribution(
        "NormalDistribution",
        NormalDistribution(
            pr::generate_canonical<Float>(pcg) * 50 - 25,
            pr::generate_canonical<Float>(pcg) * 10 + 1),
        Float(0));

    {
        // Log-normal distribution.
        Float mu = pr::generate_canonical<Float>(pcg);
        Float sigma = pr::generate_canonical<Float>(pcg);
        Float kurtosis = 
            pr::exp(4 * sigma * sigma) +
            2 * pr::exp(3 * sigma * sigma) + 
            3 * pr::exp(2 * sigma * sigma) - 6;
        testDistribution(
            "LognormalDistribution",
            LognormalDistribution(mu, sigma),
            kurtosis);
    }

    // Logistic distribution.
    testDistribution(
        "LogisticDistribution",
        LogisticDistribution(
            pr::generate_canonical<Float>(pcg) * 50 - 25,
            pr::generate_canonical<Float>(pcg) * 10 + 1),
        Float(6) / Float(5));

    return 0;
}
