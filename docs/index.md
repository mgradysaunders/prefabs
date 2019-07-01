---
layout: page
title: preform
subtitle: Header-only utilities
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

To get started with preform, clone the 
repository and build and run the tests and examples. To do so successfully, 
you'll need `git`, `rake`, and `clang++` version 5 or newer to be available
on the command line.

```
$ git clone https://github.com/mgradysaunders/preform
$ cd preform
$ rake build:test
$ rake build:example
```

The `Rakefile` uses `clang++` to compile by default because clang
typically follows the C++ standard more strictly, and is thus useful for
catching bugs and/or questionable metaprogramming techniques. You may edit 
the `Rakefile` to replace `clang++` with `g++` if you cannot (or would rather 
not) install a sufficiently recent version of `clang++`.
