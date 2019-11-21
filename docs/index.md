---
layout: page
title: preform
subtitle: C++ header-only utilities
---

# What is preform?
Preform is a collection of C++11/14/17 header-only utilities, 
containing both general-use, boilerplate interfaces as well as more
specific (yet type-generic) data structures and algorithms for use
in CPU-based graphics programming and image processing. Preform depends 
only on the C++ standard library. Thus, being header only, preform 
imposes no link dependencies.

Among other general-use utilities, preform features an option parser for 
processing command line arguments, a timer for concisely interacting with
`std::chrono`, and statically-sized type-generic queue and stack data 
structures for dodging dynamic allocation. Regarding more specific utilities,
preform features statically-sized type-generic multi-dimensional arrays
supporting broadcast operations, which serve as the basis for axis-aligned 
bounding boxes, axis-aligned bounding box trees, image objects, 
and noise function objects, to name a few. 

Refer to the [doxygen][1]-generated [API documentation][2] for 
more information.

[1]: http://doxygen.nl
[2]: https://mgradysaunders.github.io/preform/doxygen/html

### Quickstart

Although preform is a header-only library, the repository includes some
example and test programs 1) to demonstrate how things work and 2) to verify
that things are working correctly. So, to get started with preform, use `git` 
to clone the repository and `cmake` to build the examples and tests.
```
$ git clone https://github.com/mgradysaunders/preform
$ cd preform
$ mkdir bin && cd bin
$ cmake ..
$ cmake --build .
```
This builds examples to `preform/bin/example/` and tests to 
`preform/bin/test/`. Note that most of these programs use 
`pr::option_parser` to parse command-line arguments, and print usage
information and option descriptions if run with `-h` or `--help`.
