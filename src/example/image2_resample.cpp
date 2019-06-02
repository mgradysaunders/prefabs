#include <iostream>
#include <fstream>
#include <string>
#include <preform/color.hpp>
#include <preform/image2.hpp>
#include <preform/image_filters.hpp>
#include <preform/worley_noise2.hpp>
#include <preform/option_parser.hpp>

// Float type.
typedef float Float;

// 1-dimensional vector type.
typedef pr::vec1<Float> Vec1f;

// 2-dimensional vector type.
typedef pr::vec2<Float> Vec2f;

// 2-dimensional vector type.
typedef pr::vec2<int> Vec2i;

// Image.
typedef pr::image2<Float, Float, 1> Image2x1;

// Image Mitchell filter.
typedef pr::mitchell_filter2<Float> MitchellFilter2;

// Worley noise.
typedef pr::worley_noise2<Float> WorleyNoise2;

int main(int argc, char** argv)
{
    // Configurable options.
    int seed = 0;
    int up_sampling = 1;
    Vec2i source_image_dim = {128, 128};
    Vec2i target_image_dim = {512, 512};
    Vec2f noise_scale = {8, 8};
    std::string source_ofs_name = "image2_resample_source.pgm";
    std::string target_ofs_name = "image2_resample_target.pgm";

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
    << "Specify seed. By default, 0.\n";

    // Specify up-sampling method.
    opt_parser.on_option(
    "-u", "--up-sampling", 1,
    [&](char** argv) {
        try {
            up_sampling = std::stoi(argv[0]);
            if (up_sampling != 0 &&
                up_sampling != 1 &&
                up_sampling != 3) {
                throw std::exception();
            }
        }
        catch (const std::exception&) {
            throw
                std::runtime_error(
                std::string("-u/--up-sampling expects 0, 1, or 3 ")
                    .append("(can't parse ").append(argv[0])
                    .append(")"));
        }
    })
    << "Specify up-sampling method. By default, 1.\n";

    // Specify image dimensions.
    opt_parser.on_option(
    nullptr, "--source-image-dim", 2,
    [&](char** argv) {
        try {
            source_image_dim[0] = std::stoi(argv[0]);
            source_image_dim[1] = std::stoi(argv[1]);
            if (!(source_image_dim > 0).all()) {
                throw std::exception();
            }
        }
        catch (const std::exception&) {
            throw
                std::runtime_error(
                std::string("--source-image-dim expects 2 positive integers ")
                    .append("(can't parse ")
                    .append(argv[0]).append(" ")
                    .append(argv[1]).append(")"));
        }
    })
    << "Specify source image dimensions. By default, 128x128.\n";

    // Specify image dimensions.
    opt_parser.on_option(
    nullptr, "--target-image-dim", 2,
    [&](char** argv) {
        try {
            target_image_dim[0] = std::stoi(argv[0]);
            target_image_dim[1] = std::stoi(argv[1]);
            if (!(target_image_dim > 0).all()) {
                throw std::exception();
            }
        }
        catch (const std::exception&) {
            throw
                std::runtime_error(
                std::string("--target-image-dim expects 2 positive integers ")
                    .append("(can't parse ")
                    .append(argv[0]).append(" ")
                    .append(argv[1]).append(")"));
        }
    })
    << "Specify target image dimensions. By default, 512x512.\n";

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

    // Specify source output filename.
    opt_parser.on_option(
    nullptr, "--source-output", 1,
    [&](char** argv) {
        source_ofs_name = argv[0];
    })
    << "Specify source output filename.\n" 
    << "By default, image2_resample_source.pgm.\n";

    // Specify target output filename.
    opt_parser.on_option(
    nullptr, "--target-output", 1,
    [&](char** argv) {
        target_ofs_name = argv[0];
    })
    << "Specify target output filename.\n"
    << "By default, image2_resample_target.pgm.\n";

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

    // Source image.
    Image2x1 source_image;
    source_image.resize(source_image_dim);
    for (int i = 0; i < source_image_dim[0]; i++)
    for (int j = 0; j < source_image_dim[1]; j++) {
        // Supersample.
        for (int k = 0; k < 3; k++)
        for (int l = 0; l < 3; l++) {
            Vec2f image_loc = {
                Float(i) + Float(k) / 3,
                Float(j) + Float(l) / 3
            };
            Vec2f noise_loc =  noise_scale * (image_loc / source_image_dim);
            Vec1f noise_val = {noise.evaluate(noise_loc)};
            Vec2f image_filter_rad = {1, 1};
            MitchellFilter2 image_filter;
            source_image.reconstruct(
                    noise_val / 9,
                    image_loc,
                    image_filter_rad,
                    image_filter);
        }
    }

    // Contrast stretch.
    Float tmpmin = +pr::numeric_limits<Float>::infinity();
    Float tmpmax = -pr::numeric_limits<Float>::infinity();
    for (int i = 0; i < source_image_dim[0]; i++)
    for (int j = 0; j < source_image_dim[1]; j++) {
        tmpmin = pr::fmin(tmpmin, source_image(i, j)[0]);
        tmpmax = pr::fmax(tmpmax, source_image(i, j)[0]);
    }
    for (int i = 0; i < source_image_dim[0]; i++)
    for (int j = 0; j < source_image_dim[1]; j++) {
        source_image(i, j) -= tmpmin;
        source_image(i, j) /= tmpmax - tmpmin;
    }

    // Target image.
    Image2x1 target_image;
    target_image = source_image;
    target_image.resample(up_sampling, target_image_dim);

    try {
        // Write source image.
        std::ofstream source_ofs(source_ofs_name);
        source_ofs << "P2\n";
        source_ofs << source_image_dim[0] << ' ';
        source_ofs << source_image_dim[1] << '\n';
        source_ofs << "255\n";
        for (int j = 0; j < source_image_dim[1]; j++)
        for (int i = 0; i < source_image_dim[0]; i++) {
            source_ofs << int(pr::pack_uint8(
                              pr::srgbenc(source_image(i, j)[0]))) << ' ';
        }
        source_ofs.close();

        // Write target image.
        std::ofstream target_ofs(target_ofs_name);
        target_ofs << "P2\n";
        target_ofs << target_image_dim[0] << ' ';
        target_ofs << target_image_dim[1] << '\n';
        target_ofs << "255\n";
        for (int j = 0; j < target_image_dim[1]; j++)
        for (int i = 0; i < target_image_dim[0]; i++) {
            target_ofs << int(pr::pack_uint8(
                              pr::srgbenc(target_image(i, j)[0]))) << ' ';
        }
        target_ofs.close();
    }
    catch (const std::exception& exception) {
        std::cerr << "Unhandled exception!\n";
        std::cerr << "exception.what(): " << exception.what() << "\n";
        std::exit(EXIT_FAILURE);
    }
    return EXIT_SUCCESS;
}
