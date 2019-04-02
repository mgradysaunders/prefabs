#include <iostream>
#include <random>
#include <preform/random.hpp>
#include <preform/sampling.hpp>
#include <preform/neumaier_sum.hpp>
#include <preform/option_parser.hpp>
#include <preform/timer.hpp>

// Float type.
typedef double Float;

// TODO UniformIntDistribution

// TODO UniformRealDistribution

// Poisson distribution.
typedef pr::poisson_distribution<Float> PoissonDistribution;

// Exponential distribution.
typedef pr::exponential_distribution<Float> ExponentialDistribution;

// TODO BernoulliDistribution

// Binomial distribution.
typedef pr::binomial_distribution<Float> BinomialDistribution;

// Normal distribution.
typedef pr::normal_distribution<Float> NormalDistribution;

// Lognormal distribution.
typedef pr::lognormal_distribution<Float> LognormalDistribution;

// Logistic distribution.
typedef pr::logistic_distribution<Float> LogisticDistribution;

// TODO TanhDistribution

// TODO WeibullDistribution

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
    std::cout << "Testing distribution statistics for ";
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
    std::cout << "Testing distribution statistics for ";
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
    // TODO options
    
    // TODO Test uniform int distribution.

    // TODO Test uniform real distribution.

    // Test poisson distribution.
    testIntDistribution(
        "PoissonDistribution(7)",
         PoissonDistribution(7));

    // Test exponential distribution.
    testRealDistribution(
        "ExponentialDistribution(5.3)",
         ExponentialDistribution(5.3));

    // TODO Test Bernoulli distribution.

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

    // TODO Test tanh distribution.

    // TODO Test Weibull distribution.

    return 0;
}
