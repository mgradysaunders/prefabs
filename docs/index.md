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
