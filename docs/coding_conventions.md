---
layout: page
title: preform
subtitle: Header-only utilities
---

# Coding conventions
It is worth reviewing the coding conventions used in preform.
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
on the next line. No line should exceed 80 characters in width, standard 
terminal width. 
