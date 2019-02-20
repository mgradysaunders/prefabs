/* Copyright (c) 2018-19 M. Grady Saunders
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 
 *   1. Redistributions of source code must retain the above
 *      copyright notice, this list of conditions and the following
 *      disclaimer.
 * 
 *   2. Redistributions in binary form must reproduce the above
 *      copyright notice, this list of conditions and the following
 *      disclaimer in the documentation and/or other materials
 *      provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 * OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/*+-+*/
// A ruby script generates this file, DO NOT EDIT

namespace pr {

/**
 * @addtogroup multi_vec_types
 */
/**@{*/

/**
 * @brief Vector alias (1-dimensional).
 */
template <typename T>
using vec1 = multi<T, 1>;

/**
 * @brief Vector alias (2-dimensional).
 */
template <typename T>
using vec2 = multi<T, 2>;

/**
 * @brief Vector alias (3-dimensional).
 */
template <typename T>
using vec3 = multi<T, 3>;

/**
 * @brief Vector alias (4-dimensional).
 */
template <typename T>
using vec4 = multi<T, 4>;

/**
 * @brief Matrix alias (1-by-1-dimensional).
 */
template <typename T>
using mat1x1 = multi<T, 1, 1>;

/**
 * @brief Matrix alias (1-by-2-dimensional).
 */
template <typename T>
using mat1x2 = multi<T, 1, 2>;

/**
 * @brief Matrix alias (1-by-3-dimensional).
 */
template <typename T>
using mat1x3 = multi<T, 1, 3>;

/**
 * @brief Matrix alias (1-by-4-dimensional).
 */
template <typename T>
using mat1x4 = multi<T, 1, 4>;

/**
 * @brief Matrix alias (2-by-1-dimensional).
 */
template <typename T>
using mat2x1 = multi<T, 2, 1>;

/**
 * @brief Matrix alias (2-by-2-dimensional).
 */
template <typename T>
using mat2x2 = multi<T, 2, 2>;

/**
 * @brief Matrix alias (2-by-3-dimensional).
 */
template <typename T>
using mat2x3 = multi<T, 2, 3>;

/**
 * @brief Matrix alias (2-by-4-dimensional).
 */
template <typename T>
using mat2x4 = multi<T, 2, 4>;

/**
 * @brief Matrix alias (3-by-1-dimensional).
 */
template <typename T>
using mat3x1 = multi<T, 3, 1>;

/**
 * @brief Matrix alias (3-by-2-dimensional).
 */
template <typename T>
using mat3x2 = multi<T, 3, 2>;

/**
 * @brief Matrix alias (3-by-3-dimensional).
 */
template <typename T>
using mat3x3 = multi<T, 3, 3>;

/**
 * @brief Matrix alias (3-by-4-dimensional).
 */
template <typename T>
using mat3x4 = multi<T, 3, 4>;

/**
 * @brief Matrix alias (4-by-1-dimensional).
 */
template <typename T>
using mat4x1 = multi<T, 4, 1>;

/**
 * @brief Matrix alias (4-by-2-dimensional).
 */
template <typename T>
using mat4x2 = multi<T, 4, 2>;

/**
 * @brief Matrix alias (4-by-3-dimensional).
 */
template <typename T>
using mat4x3 = multi<T, 4, 3>;

/**
 * @brief Matrix alias (4-by-4-dimensional).
 */
template <typename T>
using mat4x4 = multi<T, 4, 4>;

/**
 * @brief Matrix alias (1-by-1-dimensional).
 */
template <typename T>
using mat1 = multi<T, 1, 1>;

/**
 * @brief Matrix alias (2-by-2-dimensional).
 */
template <typename T>
using mat2 = multi<T, 2, 2>;

/**
 * @brief Matrix alias (3-by-3-dimensional).
 */
template <typename T>
using mat3 = multi<T, 3, 3>;

/**
 * @brief Matrix alias (4-by-4-dimensional).
 */
template <typename T>
using mat4 = multi<T, 4, 4>;

/**@}*/

} // namespace pr
