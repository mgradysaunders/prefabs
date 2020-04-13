#include <iostream>
#include <iomanip>
#include <fstream>
#include <preform/byte_order.hpp>
#include <preform/option_parser.hpp>

int main(int argc, char** argv)
{
    pre::byte_order write_order = pre::byte_order::little;
    pre::byte_order read_order = pre::byte_order::little;
    std::string filename = "byte_order.bin";

    // Option parser.
    pre::option_parser opt_parser("[OPTIONS]");

    // Specify write byte order.
    opt_parser.on_option(
    "-w", "--write-order", 1,
    [&](char** argv) {
        try {
            if (!std::strcmp(argv[0], "little")) {
                write_order = pre::byte_order::little;
            }
            else if (!std::strcmp(argv[0], "big")) {
                write_order = pre::byte_order::big;
            }
            else {
                throw std::exception();
            }
        }
        catch (const std::exception&) {
            throw
                std::runtime_error(
                std::string("-w/--write-order expects 'little' or 'big' ")
                    .append("(can't parse ").append(argv[0])
                    .append(")"));
        }
    })
    << "Specify write byte order. By default, little.\n";

    // Specify read byte order.
    opt_parser.on_option(
    "-r", "--read-order", 1,
    [&](char** argv) {
        try {
            if (!std::strcmp(argv[0], "little")) {
                read_order = pre::byte_order::little;
            }
            else if (!std::strcmp(argv[0], "big")) {
                read_order = pre::byte_order::big;
            }
            else {
                throw std::exception();
            }
        }
        catch (const std::exception&) {
            throw
                std::runtime_error(
                std::string("-r/--read-order expects 'little' or 'big' ")
                    .append("(can't parse ").append(argv[0])
                    .append(")"));
        }
    })
    << "Specify read endianness. By default, little.\n";

    // Specify filename.
    opt_parser.on_option(
    "-f", "--filename", 1,
    [&](char** argv) {
        filename = argv[0];
    })
    << "Specify filename. By default, byte_order.bin.\n";

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

    // Print host byte order.
    std::cout << "Host byte order " << pre::to_string(pre::host_byte_order());
    std::cout << "\n\n";
    std::cout.flush();

    std::cout << std::hex;

    {
        // Variables.
        std::uint32_t u32 = 0x01020304UL;
        std::uint64_t u64 = 0xDEADBEEF01020304ULL;
        float sf = 1.234f;
        double df = 1.234;
        long double ldf = 1.234L;
        std::string str = "Hello world!";

        std::cout << "Writing " << filename;
        std::cout << " with byte order " << pre::to_string(write_order);
        std::cout << '\n';
        std::cout.flush();

        // Write input.
        std::cout << "std::uint32_t: "  << u32 << '\n';
        std::cout << "std::uint64_t: "  << u64 << '\n';
        std::cout << "float: "          << sf  << '\n';
        std::cout << "double: "         << df  << '\n';
        std::cout << "long double: "    << ldf << '\n';
        std::cout << "std::string: "    << str << '\n';
        std::cout << '\n';
        std::cout.flush();

        // Write.
        std::ofstream ofs(filename,
            std::ios_base::out |
            std::ios_base::binary);
        pre::byte_stream(ofs, write_order) << u32;
        pre::byte_stream(ofs, write_order) << u64;
        pre::byte_stream(ofs, write_order) << sf;
        pre::byte_stream(ofs, write_order) << df;
        pre::byte_stream(ofs, write_order) << ldf;
        pre::byte_stream(ofs, write_order) << str;
        ofs.close();
    }

    {
        // Variables.
        std::uint32_t u32;
        std::uint64_t u64;
        float sf;
        double df;
        long double ldf;
        std::string str;

        std::cout << "Reading " << filename;
        std::cout << " with byte order " << pre::to_string(read_order);
        std::cout << '\n';
        std::cout.flush();

        // Read.
        std::ifstream ifs(
                filename,
                std::ios_base::in |
                std::ios_base::binary);
        pre::byte_stream(ifs, read_order) >> u32;
        pre::byte_stream(ifs, read_order) >> u64;
        pre::byte_stream(ifs, read_order) >> sf;
        pre::byte_stream(ifs, read_order) >> df;
        pre::byte_stream(ifs, read_order) >> ldf;
        pre::byte_stream(ifs, read_order) >> str;
        ifs.close();

        // Read output.
        std::cout << "std::uint32_t: "  << u32 << '\n';
        std::cout << "std::uint64_t: "  << u64 << '\n';
        std::cout << "float: "          << sf  << '\n';
        std::cout << "double: "         << df  << '\n';
        std::cout << "long double: "    << ldf << '\n';
        std::cout << "std::string: "    << str << '\n';
        std::cout << '\n';
        std::cout.flush();
    }

    return EXIT_SUCCESS;
}

