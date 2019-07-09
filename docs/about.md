---
layout: page
title: preform
subtitle: C++ header-only utilities
---

# About
Preform began as a folder of code I felt as if I had 
written and rewritten a thousand times. As such, preform is less of a 
structured, strictly developed library designed to serve a particular purpose,
and more of a collection of personal programming utilities. The API is thus
subject to change and expand, especially as I continue to use preform
as a codebase for future projects. I may eventually consider versioning stable 
releases, though that seems far from necessary at the moment. 

At any snapshot in time, preform ought to contain type-generic
implementations of particular data structures and algorithms which are
reasonably well-documented, reasonably well-tested, and reasonably efficient.
The APIs for interacting with these implementations, however, are subject
to change. Preform therefore ought to be suitable for use in rapid-prototyping
and projects with smaller development timelines&mdash;and perhaps unsuitable 
for use in indefinitely running projects interested in maintaining
consistently backwards-compatible APIs.

# Coding conventions
It may be worth reviewing the coding conventions used in preform.
In this context, _coding conventions_ implies conventions related to 
naming and formatting, as well as conventions related to documentation 
and file organization. 

Preform follows the C++ standard naming convention. So, by and large, 
everything is lowercase and separated by underscores. Furthermore, type 
definitions inside structs and classes end with `_type`, excluding standard 
container type definitions such as `reference`. Private member variables of 
structs and classes feature a trailing underscore&mdash;for example, 
`private_member_`. Names reserved by the standard are unused&mdash;for 
example, no name features a trailing `_t`. Template parameter declarations 
prefer the `typename` keyword instead of the `class` keyword.

Now, to make a few pedantic notes about formatting. Tabs are expanded 
as 4 spaces, such that 4 spaces equals 1 level of indentation. It is not 
necessary to indent the body of an outermost namespace. When declaring the 
body of a struct, class, or function definition, the opening brace appears 
on the next line. No line should exceed 80 characters, being standard 
terminal width. 

Preform uses [doxygen][1] to build the [API documentation][2]. Thus,
C-style asterisk comment blocks containing doxygen-syntax appear throughout 
the implementation. The `Doxyfile` predefines `DOXYGEN`, which is used to elide 
unimportant structures and preprocessing macros from the documentation.
Doxygen modules are the preferred structure for organizing documentation. 
Each header file contains at least 1 module. The description of each module 
designates the associated header file as well as the necessary C++ version. 
The `DoxygenFix.js` javascript file is a script which adjusts 
formatting and substitutes concise alternatives for
lengthy type expressions.

[1]: http://doxygen.nl
[2]: https://mgradysaunders.github.io/preform/doxygen/html

### Development environment

Preform is developed and tested on Linux, and is thus
written to be compiled with either `g++` or `clang++`. In particular, 
preform occassionally makes use of `__attribute__` syntax and the
`__PRETTY_FUNCTION__` macro for clearer function names in error messages. It
should be possible to compile preform with another sufficiently recent
C++ compiler, though it may be necessary to define

```
#define __attribute__(x)
#define __PRETTY_FUNCTION__ __func__
```

before including preform headers. On that note, `preform/quadmath.hpp` is 
only suitable for use with `g++`, as it depends on the availability 
`libquadmath` and the associated GNU extensions for quadruple precision 
floating point literals. That is, to successfully compile code using
`preform/quadmath.hpp`, you must compile with `-std=gnu++14` 
(or `-std=gnu++17`, or any newer standard) and link with `-lquadmath`.
