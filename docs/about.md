---
layout: page
title: preform
subtitle: Header-only utilities
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

TODO documentation

TODO file organization
