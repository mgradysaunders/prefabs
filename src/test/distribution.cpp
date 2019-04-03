#include <iostream>
#include <random>
#include <preform/random.hpp>
#include <preform/sampling.hpp>
#include <preform/neumaier_sum.hpp>
#include <preform/option_parser.hpp>
#include <preform/timer.hpp>

// Float type.
typedef double Float;

// Uniform int distribution.
typedef pr::uniform_int_distribution<Float> UniformIntDistribution;

// Uniform real distribution.
typedef pr::uniform_real_distribution<Float> UniformRealDistribution;

// Poisson distribution.
typedef pr::poisson_distribution<Float> PoissonDistribution;

// Exponential distribution.
typedef pr::exponential_distribution<Float> ExponentialDistribution;

// Bernoulli distribution.
typedef pr::bernoulli_distribution<Float> BernoulliDistribution;

// Binomial distribution.
typedef pr::binomial_distribution<Float> BinomialDistribution;

// Normal distribution.
typedef pr::normal_distribution<Float> NormalDistribution;

// Lognormal distribution.
typedef pr::lognormal_distribution<Float> LognormalDistribution;

// Logistic distribution.
typedef pr::logistic_distribution<Float> LogisticDistribution;

// Tanh distribution.
typedef pr::tanh_distribution<Float> TanhDistribution;

// Weibull distribution.
typedef pr::weibull_distribution<Float> WeibullDistribution;

// Timer.
typedef pr::steady_timer Timer;

// Neumaier sum.
typedef pr::neumaier_sum<Float> NeumaierSum;

// Permuted congruential generator.
pr::pcg32 pcg;

// Test distribution statistics.
template <typename Distribution>
void testIntDistribution(
                const char* name, 
                const Distribution& distribution)
{
    const int n = 262144;
    std::cout << "Testing statistics for ";
    std::cout << name << ":\n";
    std::cout <<
        "This test compares sample statistics of 262144 stratified\n"
        "random samples (by inverse transform) to analytical\n"
        "distribution statistics.\n";
    std::cout.flush();

    // Initialize samples.
    int* x = new int[n];
    Float* u = new Float[n];
    pr::stratify(pcg, n, u);

    // Inverse transform.
    std::cout << "Sampling by inverse transform... ";
    std::cout.flush();
    Timer timer;
    for (int k = 0; k < n; k++) {
        x[k] = int(distribution.cdfinv(u[k]));
    }
    Float us = timer.read<std::nano>() * 1e-3 / n;
    std::cout << "done (~" << us << "us per sample).\n";
    std::cout.flush();

    // Destroy.
    delete[] u;
    u = nullptr;

    // Compute sample moment.
    auto sampleMoment = [&x](Float c, int p) -> Float {
        NeumaierSum s = 0;
        for (int k = 0; k < n; k++) {
            s += pr::nthpow(x[k] - c, p);
        }
        return Float(s) / 
               Float((p & 1) ? n : n - 1);
    };

    // Compute sample moments.
    Float m1 = sampleMoment(0, 1);
    Float mu2 = sampleMoment(m1, 2);
    Float mu3 = sampleMoment(m1, 3);
    Float gamma1 = mu3 / (mu2 * pr::sqrt(mu2));
    Float h = 0;

    {
        // Compute sample entropy.
        NeumaierSum s = 0;
        for (int k = 0; k < n; k++) {
            Float fk = distribution.pmf(x[k]);
            Float hk = pr::log(fk); // No fk due to sampling strategy.
            if (pr::isinf(hk) || 
                pr::isnan(hk)) {
                continue;
            }
            s -= hk;
        }
        h = Float(s) / n;
    }

    // Print sample statistics.
    std::cout << "Sample mean: " << m1 << "\n";
    std::cout << "Sample variance: " << mu2 << "\n";
    std::cout << "Sample skewness: " << gamma1 << "\n";
    std::cout << "Sample entropy: " << h << "\n";

    // Print distribution statistics.
    std::cout << "Distribution mean: " << distribution.mean() << "\n";
    std::cout << "Distribution variance: " << distribution.variance() << "\n";
    std::cout << "Distribution skewness: " << distribution.skewness() << "\n";
    std::cout << "Distribution entropy: " << distribution.entropy() << "\n\n";
    std::cout.flush();

    // Destroy.
    delete[] x;
    x = nullptr;
}

// Test distribution statistics.
template <typename Distribution>
void testRealDistribution(
                const char* name, 
                const Distribution& distribution)
{
    const int n = 262144;
    std::cout << "Testing statistics for ";
    std::cout << name << ":\n";
    std::cout <<
        "This test compares sample statistics of 262144 stratified\n"
        "random samples (by inverse transform) to analytical\n"
        "distribution statistics.\n";
    std::cout.flush();

    // Initialize samples.
    Float* x = new Float[n];
    Float* u = new Float[n];
    pr::stratify(pcg, n, u);

    // Inverse transform.
    std::cout << "Sampling by inverse transform... ";
    std::cout.flush();
    Timer timer;
    for (int k = 0; k < n; k++) {
        x[k] = distribution.cdfinv(u[k]);
    }
    Float us = timer.read<std::nano>() * 1e-3 / n;
    std::cout << "done (~" << us << "us per sample).\n";
    std::cout.flush();

    // Destroy.
    delete[] u;
    u = nullptr;

    // Compute sample moment.
    auto sampleMoment = [&x](Float c, int p) -> Float {
        NeumaierSum s = 0;
        for (int k = 0; k < n; k++) {
            s += pr::nthpow(x[k] - c, p);
        }
        return Float(s) / 
               Float((p & 1) ? n : n - 1);
    };

    // Compute sample moments.
    Float m1 = sampleMoment(0, 1);
    Float mu2 = sampleMoment(m1, 2);
    Float mu3 = sampleMoment(m1, 3);
    Float gamma1 = mu3 / (mu2 * pr::sqrt(mu2));
    Float h = 0;

    {
        // Compute sample entropy.
        NeumaierSum s = 0;
        for (int k = 0; k < n; k++) {
            Float fk = distribution.pdf(x[k]);
            Float hk = pr::log(fk); // No fk due to sampling strategy.
            if (pr::isinf(hk) || 
                pr::isnan(hk)) {
                continue;
            }
            s -= hk;
        }
        h = Float(s) / n;
    }

    // Print sample statistics.
    std::cout << "Sample mean: " << m1 << "\n";
    std::cout << "Sample variance: " << mu2 << "\n";
    std::cout << "Sample skewness: " << gamma1 << "\n";
    std::cout << "Sample entropy: " << h << "\n";

    // Print distribution statistics.
    std::cout << "Distribution mean: " << distribution.mean() << "\n";
    std::cout << "Distribution variance: " << distribution.variance() << "\n";
    std::cout << "Distribution skewness: " << distribution.skewness() << "\n";
    std::cout << "Distribution entropy: " << distribution.entropy() << "\n\n";
    std::cout.flush();

    // Destroy.
    delete[] x;
    x = nullptr;
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
    std::cout << "seed = " << seed << "\n\n";
    std::cout.flush();
    pcg = pr::pcg32(seed);
    
    // Test uniform int distribution.
    testIntDistribution(
        "UniformIntDistribution(-4, 15)",
         UniformIntDistribution(-4, 15));

    // Test uniform real distribution.
    testRealDistribution(
        "UniformRealDistribution(17, 29)",
         UniformRealDistribution(17, 29));

    // Test poisson distribution.
    testIntDistribution(
        "PoissonDistribution(7)",
         PoissonDistribution(7));

    // Test exponentialdistribution.
    testRealDistribution(
        "ExponentialDistribution(5.3)",
         ExponentialDistribution(5.3));

    // Test Bernoulli distribution.
    testIntDistribution(
        "BernoulliDistribution(0.37)",
         BernoulliDistribution(0.37));

    // Test binomial distribution.
    testIntDistribution(
        "BinomialDistribution(1, 0.37)",
         BinomialDistribution(1, 0.37));

    // Test binomial distribution.
    testIntDistribution(
        "BinomialDistribution(51, 0.88)",
         BinomialDistribution(51, 0.88));

    // Test binomial distribution.
    testIntDistribution(
        "BinomialDistribution(1000, 0.25)",
         BinomialDistribution(1000, 0.25));

    // Test normal distribution.
    testRealDistribution(
        "NormalDistribution(4, 3)",
         NormalDistribution(4, 3));

    // Test lognormal distribution.
    testRealDistribution(
        "LognormalDistribution(2, 1)",
         LognormalDistribution(2, 1));

    // Test logistic distribution.
    testRealDistribution(
        "LogisticDistribution(2.7, 3.3)",
         LogisticDistribution(2.7, 3.3));

    // Test tanh distribution.
    testRealDistribution(
        "TanhDistribution(7, 4)",
         TanhDistribution(7, 4));

    // Test Weibull distribution.
    testRealDistribution(
        "WeibullDistribution(0.9, 0.7)",
         WeibullDistribution(0.9, 0.7));

    return EXIT_SUCCESS;
}
