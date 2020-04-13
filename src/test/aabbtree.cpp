#include <iostream>
#include <random>
#include <preform/random.hpp>
#include <preform/multi_random.hpp>
#include <preform/memory_arena.hpp>
#include <preform/memory_arena_allocator.hpp>
#include <preform/timer.hpp>
#include <preform/option_parser.hpp>
#include <preform/aabbtree.hpp>

// Float type.
typedef float Float;

// 3-dimensional vector type.
typedef pre::vec3<Float> Vec3f;

// 3-dimensional axis-aligned bounding box type.
typedef pre::aabb3<Float> AABB3f;

// Axis-aligned bounding box tree.
typedef pre::aabbtree3<Float,
        pre::aabbtree_split_surface_area<16>,
        pre::memory_arena_allocator<char>> AABBTree3;

// Linear axis-aligned bounding box tree.
typedef pre::linear_aabbtree3<float> LinearAABBTree3;

// Timer.
typedef pre::steady_timer Timer;

// Permuted congruential generator.
pre::pcg32 pcg;

// Generate canonical random number.
Float generateCanonical()
{
    return pre::generate_canonical<Float>(pcg);
}

// Generate canonical random 3-dimensional vector.
Vec3f generateCanonical3()
{
    return pre::generate_canonical<Float, 3>(pcg);
}

int depth_histogram[64] = {};

int count_histogram[16] = {};

void process(const LinearAABBTree3::node_type* node, int depth = 0)
{
    if (node->is_branch()) {
        process(node->left_child(), depth + 1);
        process(node->right_child(), depth + 1);
    }
    else {
        if (depth >= 64 ||
            node->count >= 16) {
            throw std::runtime_error("histogram element out of range");
        }
        ++depth_histogram[depth];
        ++count_histogram[node->count];
    }
}

int main(int argc, char** argv)
{
    int seed = 0;
    int nboxes = 1048576;

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

    // Specify number of boxes.
    opt_parser.on_option(
    "-n", "--nboxes", 1,
    [&](char** argv) {
        try {
            nboxes = std::stoi(argv[0]);
            if (!(nboxes > 0)) {
                throw std::exception(); // Trigger catch block.
            }
        }
        catch (const std::exception&) {
            throw
                std::runtime_error(
                std::string("-n/--nboxes expects 1 positive integer ")
                    .append("(can't parse ").append(argv[0])
                    .append(")"));
        }
    })
    << "Specify number of random boxes. By default, 1048576.\n";

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

    // Generate boxes.
    std::cout << "Generating " << nboxes << " random boxes... ";
    std::cout.flush();
    Timer timer = Timer();
    AABB3f* boxes = new AABB3f[nboxes];
    for (int k = 0; k < nboxes; k++) {
        Vec3f point = generateCanonical3() * 500 - 250;
        Vec3f half_extent = generateCanonical3() * 20 + 1;
        boxes[k] = {
            point - half_extent,
            point + half_extent
        };
    }
    std::cout << "done (" << timer.read<std::micro>() / 1e6 << " sec).\n\n";
    std::cout.flush();

    // Initialize axis-aligned bounding box tree.
    std::cout << "Initializing axis-aligned bounding box tree... ";
    std::cout.flush();
    timer = Timer();
    AABBTree3* tree = new AABBTree3();
    tree->init(
        &boxes[0],
        &boxes[0] + nboxes,
        [](const AABB3f& box) -> AABB3f { return box; });
    std::cout << "done (" << timer.read<std::micro>() / 1e6 << " sec).\n\n";
    std::cout.flush();

    // Don't need this anymore.
    delete[] boxes;
    boxes = nullptr;

    // Initialize linear axis-aligned bounding box tree.
    std::cout << "Initializing linear axis-aligned bounding box tree... ";
    std::cout.flush();
    timer = Timer();
    LinearAABBTree3* linear_tree = new LinearAABBTree3(*tree);
    std::cout << "done (" << timer.read<std::micro>() / 1e6 << " sec).\n\n";
    std::cout.flush();

    // Don't need this anymore.
    delete tree;
    tree = nullptr;

    try {
        // Process.
        process(linear_tree->begin());
    }
    catch (const std::runtime_error& exception) {
        std::cerr << "Unhandled exception!\n";
        std::cerr << "exception.what(): " << exception.what() << "\n";
        std::exit(EXIT_FAILURE);
    }

    // Depth histogram maximum.
    double depth_max =
        *std::max_element(
        &depth_histogram[0],
        &depth_histogram[0] + 64);

    // Histogram of leaf depths.
    std::cout << "Histogram of leaf depths:\n";
    for (int depth = 0; depth < 64; depth++) {
        if (!depth_histogram[depth]) {
            continue;
        }
        std::cout << depth << ": ";
        for (int nstars =
                    depth_histogram[depth] /
                    depth_max * 70; nstars >= 0; nstars--) {
            std::cout << "*";
        }
        std::cout << "\n";
    }
    std::cout << "\n";
    std::cout.flush();

    // Count histogram maximum.
    double count_max =
        *std::max_element(
        &count_histogram[0],
        &count_histogram[0] + 16);

    // Histogram of leaf object counts.
    std::cout << "Histogram of leaf object counts:\n";
    for (int count = 0; count < 16; count++) {
        if (!count_histogram[count]) {
            continue;
        }
        std::cout << count << ": ";
        for (int nstars =
                    count_histogram[count] /
                    count_max * 70; nstars >= 0; nstars--) {
            std::cout << "*";
        }
        std::cout << "\n";
    }
    std::cout << "\n";
    std::cout.flush();

    // Clean up.
    delete linear_tree;

    // Done.
    return EXIT_SUCCESS;
}
