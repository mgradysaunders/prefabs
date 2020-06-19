---
layout: page
title: preform
---

# What is preform?

**Preform** is a collection of C++11/14/17 header-only utilities. It
contains general-use, boilerplate interfaces as well as more specific 
(yet type-generic) data structures and algorithms for use in CPU-based 
graphics programming and image processing. Preform depends only on the 
C++ standard library, and thus imposes no link dependencies.

Refer to the [doxygen][1]-generated [API documentation][2] for 
more information.

[1]: http://doxygen.nl
[2]: https://mgradysaunders.github.io/preform/doxygen/html

### Installing with CMake

Preform provides a CMake package in order to make it easier to 
install and incorporate into other projects. Run the following shell
commands to install Preform (assuming a UNIX system where `sudo` grants
root access):
```
$ git clone https://github.com/mgradysaunders/preform
$ cd preform
$ mkdir build && cd build
$ cmake ..
$ cmake --build .
$ sudo make install
```
This installs the `include/preform/` directory into the install tree
specified by `CMAKE_INSTALL_PREFIX`. This also installs relevant CMake
configuration files to find Preform in a CMake script with the
`find_package()` macro. So, to use Preform in another CMake-managed
project, find the `Preform` package, then link the project target(s) 
to `Preform::Preform`. For example:
```
# Find Preform package.
find_package(Preform REQUIRED)
# Link my-target to Preform::Preform.
target_link_libraries(my-target Preform::Preform)
```
