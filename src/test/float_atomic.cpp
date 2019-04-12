#include <iostream>
#include <preform/float_atomic.hpp>
#include <preform/thread_pool.hpp>
#include <preform/option_parser.hpp>

// Float type.
typedef float Float;

// Float bits type.
typedef std::uint32_t FloatBits;

// Float atomic.
typedef pr::float_atomic<Float, FloatBits> FloatAtomic;

int main(int argc, char** argv)
{
    // Option parser.
    pr::option_parser opt_parser("[OPTIONS]");

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

    // TODO

    return EXIT_SUCCESS;
}
