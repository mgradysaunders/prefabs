#include <iostream>
#include <preform/float_atomic.hpp>
#include <preform/thread_pool.hpp>
#include <preform/option_parser.hpp>

// Float type.
typedef float Float;

// Float bits type.
typedef std::uint32_t FloatBits;

// Float atomic.
typedef pre::float_atomic<Float, FloatBits> FloatAtomic;

// Thread pool.
typedef pre::thread_pool ThreadPool;

int main(int argc, char** argv)
{
    int nthreads = 8;

    // Option parser.
    pre::option_parser opt_parser("[OPTIONS]");

    // Display help.
    opt_parser.on_option(
    "-n", "--nthreads", 1,
    [&](char** argv) {
        try {
            nthreads = std::stoi(argv[0]);
            if (!(nthreads >= 1 &&
                  nthreads <= 8)) {
                throw std::exception();
            }
        }
        catch (const std::exception&) {
            throw
                std::runtime_error(
                std::string("-n/--nthreads expects 1 integer in [1,8] ")
                    .append("(can't parse ").append(argv[0])
                    .append(")"));
        }
    })
    << "Specify number of threads. By default, 8.\n";

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

    // Print test description.
    std::cout << "Testing float atomic:\n";
    std::cout << "This test adds 1/4 4096 times in 32 separate jobs\n";
    std::cout << "which may execute on separate threads. This should sum to\n";
    std::cout << "32768.\n";
    std::cout.flush();

    // Thread pool.
    ThreadPool thread_pool(nthreads);

    // Result.
    FloatAtomic result(Float(0));

    // Result futures.
    std::future<void> results[32];

    // Execute parallel sums.
    for (int j = 0; j < 32; j++) {
        results[j] =
        thread_pool.submit([&]() {
            for (int k = 0; k < 4096; k++) {
                result.fetch_addf(Float(0.25));
            }
        });
    }
    for (int j = 0; j < 32; j++) {
        results[j].wait();
    }

    // Shutdown thread pool.
    thread_pool.shutdown();

    // Print test result.
    std::cout << "Result: " << result.loadf() << "\n\n";
    std::cout.flush();

    return EXIT_SUCCESS;
}
