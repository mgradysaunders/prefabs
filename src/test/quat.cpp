#include <iostream>
#include <iomanip>
#include <random>
#include <preform/random.hpp>
#include <preform/option_parser.hpp>
#include <preform/multi.hpp>
#include <preform/multi_math.hpp>
#include <preform/quat.hpp>

// Floating point type.
typedef float Float;

// 3-dimensional vector type.
typedef pre::vec3<Float> Vec3f;

// 4x4-dimensional matrix type.
typedef pre::mat4<Float> Mat4f;

// Quaternion type.
typedef pre::quat<Float> Quat;

// Dual-quaternion type.
typedef pre::dualquat<Float> Dualquat;

// Permuted congruential generator.
pre::pcg32 pcg;

void testTransform()
{
    // Print description.
    std::cout << "Testing transform:\n";
    std::cout << "This test generates a random rigid-body transform\n";
    std::cout << "using both Dualquat and Mat4 operations independently,\n";
    std::cout << "then casts the Dualquat as Mat4, and the Mat4 as\n";
    std::cout << "Dualquat, to verify the correctness of the transform\n";
    std::cout << "and casting implementations. Note that the Dualquat\n";
    std::cout << "results may vary by an overall sign.\n";
    std::cout.flush();

    // Random angle.
    Float theta =
        pre::generate_canonical<Float>(pcg) *
        pre::numeric_constants<Float>::M_pi() * 2;

    // Random axis.
    Vec3f hatv =
    Vec3f::uniform_sphere_pdf_sample({
        pre::generate_canonical<Float>(pcg),
        pre::generate_canonical<Float>(pcg)
    });

    // Random translation.
    Vec3f w = {
        pre::generate_canonical<Float>(pcg) * 16 - 8,
        pre::generate_canonical<Float>(pcg) * 16 - 8,
        pre::generate_canonical<Float>(pcg) * 16 - 8
    };

    // Print transform.
    std::cout << "rotation angle = " << theta << "\n";
    std::cout << "rotation axis = " << hatv << "\n";
    std::cout << "translation = " << w << "\n";
    std::cout.flush();

    // Dualquat construction.
    Dualquat x0 =
        Dualquat::translate(w) *
        Dualquat::rotate(theta, hatv);
    std::cout << "dual quaternion = " << x0 << "\n";
    std::cout << "dual quaternion as matrix = " << Mat4f(x0) << "\n";
    std::cout << "recovered rotation angle = " << x0.rotation_angle() << "\n";
    std::cout << "recovered rotation axis = " << x0.rotation_axis() << "\n";
    std::cout << "recovered translation = " << x0.translation() << "\n";
    std::cout.flush();

    // Mat4 construction.
    Mat4f x1 =
        pre::dot(
        Mat4f::translate(w),
        Mat4f::rotate(theta, hatv));
    std::cout << "matrix = " << x1 << "\n";
    std::cout << "matrix as dual quaternion = " << Dualquat(x1) << "\n\n";
    std::cout.flush();
}

void testTransformSlerp()
{
    // Print description.
    std::cout << "Testing transform slerp:\n";
    std::cout << "This test generates two random rigid-body transforms\n";
    std::cout << "to use as Dualquat slerp endpoints, then verifies that\n";
    std::cout << "slerp(0) and slerp(1) return the endpoints, then compares\n";
    std::cout << "the returned slerp derivative to a numerical estimate of\n";
    std::cout << "the slerp derivative.\n";
    std::cout.flush();

    // Random angle.
    Float theta0 =
        pre::generate_canonical<Float>(pcg) *
        pre::numeric_constants<Float>::M_pi() * 2;
    Float theta1 =
        pre::generate_canonical<Float>(pcg) *
        pre::numeric_constants<Float>::M_pi() * 2;

    // Random axis.
    Vec3f hatv0 =
    Vec3f::uniform_sphere_pdf_sample({
        pre::generate_canonical<Float>(pcg),
        pre::generate_canonical<Float>(pcg)
    });
    Vec3f hatv1 =
    Vec3f::uniform_sphere_pdf_sample({
        pre::generate_canonical<Float>(pcg),
        pre::generate_canonical<Float>(pcg)
    });

    // Random translation.
    Vec3f w0 = {
        pre::generate_canonical<Float>(pcg) * 16 - 8,
        pre::generate_canonical<Float>(pcg) * 16 - 8,
        pre::generate_canonical<Float>(pcg) * 16 - 8
    };
    Vec3f w1 = {
        pre::generate_canonical<Float>(pcg) * 16 - 8,
        pre::generate_canonical<Float>(pcg) * 16 - 8,
        pre::generate_canonical<Float>(pcg) * 16 - 8
    };

    // Dual quaternion endpoints.
    Dualquat x0 =
        Dualquat::translate(w0) *
        Dualquat::rotate(theta0, hatv0);
    Dualquat x1 =
        Dualquat::translate(w1) *
        Dualquat::rotate(theta1, hatv1);
    std::cout << "dual quaternion 0 = " << x0 << "\n";
    std::cout << "dual quaternion 1 = " << x1 << "\n";
    std::cout << "slerp(0) = " << Dualquat::slerp(0, x0, x1) << "\n";
    std::cout << "slerp(1) = " << Dualquat::slerp(1, x0, x1) << "\n";
    std::cout.flush();

    // Dual quaternion derivative comparison.
    Float h = 0.001;
    Float mu = pre::generate_canonical<Float>(pcg);
    Dualquat dy_dmu;
    Dualquat y0 = Dualquat::slerp(mu, x0, x1, &dy_dmu);
    Dualquat y1 = Dualquat::slerp(mu + h, x0, x1);
    Dualquat dy_dmu_est = (y1 - y0) / h;
    std::cout << "slerp(" << mu << ") deriv = " << dy_dmu << "\n";
    std::cout << "slerp(" << mu << ") deriv estimate = "
        << dy_dmu_est << "\n\n";
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

    // Test transform.
    testTransform();

    // Test transform slerp.
    testTransformSlerp();

    return EXIT_SUCCESS;
}

