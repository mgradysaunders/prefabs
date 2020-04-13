#include <iostream>
#include <fstream>
#include <string>
#include <preform/color.hpp>
#include <preform/image2.hpp>
#include <preform/image_filters.hpp>
#include <preform/misc_int.hpp>
#include <preform/worley_noise2.hpp>
#include <preform/option_parser.hpp>

// Float type.
typedef float Float;

// 1-dimensional vector type.
typedef pre::vec1<Float> Vec1f;

// 2-dimensional vector type.
typedef pre::vec2<Float> Vec2f;

// 2-dimensional vector type.
typedef pre::vec2<int> Vec2i;

// Image.
typedef pre::image2<Float, Float, 1> Image2x1;

// Image Mitchell filter.
typedef pre::mitchell_filter2<Float> MitchellFilter2;

// Worley noise.
typedef pre::worley_noise2<Float> WorleyNoise2;

int main(int argc, char** argv)
{
    // Configurable options.
    int seed = 0;
    Vec2i image_dim = {512, 512};
    Vec2f noise_scale = {8, 8};
    std::string ofs_prefix = "image2_mipmap";

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
    << "Specify seed. By default, 0.\n";

    // Specify image dimensions.
    opt_parser.on_option(
    nullptr, "--image-dim", 2,
    [&](char** argv) {
        try {
            image_dim[0] = std::stoi(argv[0]);
            image_dim[1] = std::stoi(argv[1]);
            if (!(image_dim > 0).all()) {
                throw std::exception();
            }
        }
        catch (const std::exception&) {
            throw
                std::runtime_error(
                std::string("--image-dim expects 2 positive integers ")
                    .append("(can't parse ")
                    .append(argv[0]).append(" ")
                    .append(argv[1]).append(")"));
        }
    })
    << "Specify source image dimensions. By default, 512x512.\n";

    // Specify noise scale.
    opt_parser.on_option(
    nullptr, "--noise-scale", 2,
    [&](char** argv) {
        try {
            noise_scale[0] = std::stof(argv[0]);
            noise_scale[1] = std::stof(argv[1]);
        }
        catch (const std::exception&) {
            throw
                std::runtime_error(
                std::string("--noise-scale expects 2 floats ")
                    .append("(can't parse ")
                    .append(argv[0]).append(" ")
                    .append(argv[1]).append(")"));
        }
    })
    << "Specify noise scale. By default, 8x8.\n";

    // Specify output filename prefix.
    opt_parser.on_option(
    nullptr, "--output-prefix", 1,
    [&](char** argv) {
        ofs_prefix = argv[0];
    })
    << "Specify output filename prefix.\n"
    << "By default, image2_mipmap.\n";

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

    // Noise.
    WorleyNoise2 noise(seed);

    // Image.
    image_dim[0] = pre::roundpow2(image_dim[0]);
    image_dim[1] = pre::roundpow2(image_dim[1]);
    Image2x1 image;
    image.resize(image_dim);
    for (int i = 0; i < image_dim[0]; i++)
    for (int j = 0; j < image_dim[1]; j++) {
        // Supersample.
        for (int k = 0; k < 3; k++)
        for (int l = 0; l < 3; l++) {
            Vec2f image_loc = {
                Float(i) + Float(k) / 3,
                Float(j) + Float(l) / 3
            };
            Vec2f noise_loc =  noise_scale * (image_loc / image_dim);
            Vec1f noise_val = {noise.evaluate(noise_loc)};
            Vec2f image_filter_rad = {1, 1};
            MitchellFilter2 image_filter;
            image.reconstruct(
                    noise_val / 9,
                    image_loc,
                    image_filter_rad,
                    image_filter);
        }
    }

    // Contrast stretch.
    Float tmpmin = +pre::numeric_limits<Float>::infinity();
    Float tmpmax = -pre::numeric_limits<Float>::infinity();
    for (int i = 0; i < image_dim[0]; i++)
    for (int j = 0; j < image_dim[1]; j++) {
        tmpmin = pre::fmin(tmpmin, image(i, j)[0]);
        tmpmax = pre::fmax(tmpmax, image(i, j)[0]);
    }
    for (int i = 0; i < image_dim[0]; i++)
    for (int j = 0; j < image_dim[1]; j++) {
        image(i, j) -= tmpmin;
        image(i, j) /= tmpmax - tmpmin;
    }

    try {
        int level = 0;
        while (1) {
            // Write image.
            std::ofstream ofs(
                    ofs_prefix +
                    std::to_string(level) + ".pgm");
            ofs << "P2\n";
            ofs << image.user_size()[0] << ' ';
            ofs << image.user_size()[1] << '\n';
            ofs << "255\n";
            for (int j = 0; j < int(image.user_size()[1]); j++)
            for (int i = 0; i < int(image.user_size()[0]); i++) {
                ofs << int(pre::pack_uint8(
                           pre::srgbenc(image(i, j)[0]))) << ' ';
            }
            ofs.close();
            if ((image.user_size() == 1U).any()) {
                break;
            }

            image.mip_downsample();
            level++;
        }
    }
    catch (const std::exception& exception) {
        std::cerr << "Unhandled exception!\n";
        std::cerr << "exception.what(): " << exception.what() << "\n";
        std::exit(EXIT_FAILURE);
    }

    return EXIT_SUCCESS;
}
