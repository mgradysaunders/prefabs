/* Copyright (c) 2018-20 M. Grady Saunders
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
#if !DOXYGEN
#if !(__cplusplus >= 201703L)
#error "preform/aabb.hpp requires >=C++17"
#endif // #if !(__cplusplus >= 201703L)
#endif // #if !DOXYGEN
#pragma once
#ifndef PREFORM_AABB_HPP
#define PREFORM_AABB_HPP

// for std::basic_istream
#include <istream>

// for std::basic_ostream
#include <ostream>

// for pre::multi
#include <preform/multi.hpp>

// for pre::multi wrappers
#include <preform/multi_math.hpp>

namespace pre {

/**
 * @defgroup aabb Axis-aligned bounding box
 *
 * `<preform/aabb.hpp>`
 *
 * __C++ version__: >=C++17
 */
/**@{*/

/**
 * @brief Axis-aligned bounding box.
 */
template <typename T, std::size_t N>
class aabb
{
public:

    // Sanity check.
    static_assert(
        std::is_arithmetic<T>::value,
        "T must be arithmetic");

    /**
     * @brief Value type.
     */
    typedef multi<T, N> value_type;

    /**
     * @brief Entry type.
     */
    typedef T entry_type;

    /**
     * @brief Float type.
     */
    typedef std::conditional_t<
            std::is_floating_point<T>::value, T, double> float_type;

public:

    /**
     * @name Constructors
     */
    /**@{*/

    /**
     * @brief Default constructor.
     */
    constexpr aabb() = default;

    /**
     * @brief Constructor.
     */
    constexpr aabb(const multi<T, N>& arr) :
            arr_{arr, arr}
    {
    }

    /**
     * @brief Constructor.
     */
    constexpr aabb(const multi<T, N>& arr0, const multi<T, N>& arr1) :
            arr_{arr0, arr1}
    {
    }

    /**@}*/

public:

    /**
     * @name Accessors
     */
    /**@{*/

    /**
     * @brief Index accessor.
     */
    constexpr multi<T, N>& operator[](std::size_t pos)
    {
        return arr_[pos];
    }

    /**
     * @brief Index accessor, const variant.
     */
    constexpr const multi<T, N>& operator[](std::size_t pos) const
    {
        return arr_[pos];
    }

    /**@}*/

public:

    /**
     * @name Misc
     */
    /**@{*/

    /**
     * @brief Diagonal.
     */
    constexpr multi<T, N> diag() const
    {
        return arr_[1] - arr_[0];
    }

    /**
     * @brief Hyper volume.
     *
     * @note
     * Hyper volume in @f$ N @f$ dimensions measures the entire
     * @f$ N @f$ dimensional subspace. That is,
     * @f[
     *      V = \prod_{k \in L} (P_{\max[k]} - P_{\min[k]})
     * @f]
     * for index set @f$ L = [0, N) @f$.
     * - In @f$ N = 2 @f$ dimensions, @f$ V @f$ measures signed area.
     * - In @f$ N = 3 @f$ dimensions, @f$ V @f$ measures signed volume.
     */
    constexpr T volume() const
    {
        multi<T, N> d = diag();
        return d.prod();
    }

    /**
     * @brief Hyper surface area.
     *
     * @note
     * Hyper surface area in @f$ N @f$ dimensions measures the
     * @f$ N - 1 @f$ dimensional subspace on the border of the @f$ N @f$
     * dimensional volume. That is,
     * @f[
     *      S = \sum_{k \in L} \prod_{j \in L_k} 2 (P_{\max[j]} - P_{\min[j]})
     * @f]
     * for index sets @f$ L = [0, N) @f$ and @f$ L_k = L \setminus k @f$.
     * - In @f$ N = 2 @f$ dimensions, @f$ S @f$ measures signed perimeter.
     * - In @f$ N = 3 @f$ dimensions, @f$ S @f$ measures signed surface area.
     */
    constexpr T surface_area() const
    {
        if constexpr (N == 1) {
            return 1;
        }
        else {
            T a = T(0);
            multi<T, N> d = diag();
            for (std::size_t k = 0; k < N; k++) {
                T t = T(1);
                for (std::size_t j = 0; j < N; j++) {
                    if (j != k) {
                        t *= d[j];
                    }
                }
                a += t;
            }
            return T(2) * a; // Wrong for N = 1.
        }
    }

    /**
     * @brief Center.
     */
    constexpr multi<float_type, N> center() const
    {
        return float_type(0.5) * (arr_[0] + arr_[1]);
    }

    /**
     * @brief Radius.
     */
    float_type radius() const
    {
        return float_type(0.5) * pre::length(arr_[1] - arr_[0]);
    }

    /**
     * @brief Lerp minimum and maximum coordinates.
     */
    constexpr multi<float_type, N> lerp(const multi<float_type, N>& u) const
    {
        return (1 - u) * arr_[0] + u * arr_[1];
    }

    /**
     * @brief Cast operator.
     */
    template <typename U, std::size_t M>
    constexpr operator aabb<U, M>() const
    {
        return {
            static_cast<multi<U, M>>(arr_[0]),
            static_cast<multi<U, M>>(arr_[1])
        };
    }

    /**@}*/

public:

    /**
     * @name Tests
     */
    /**@{*/

    /**
     * @brief Hyper volume is non-negative?
     */
    constexpr operator bool() const
    {
        return (arr_[0] <= arr_[1]).all();
    }

    /**
     * @brief Overlaps other?
     */
    template <
        bool inclusive0 = true,
        bool inclusive1 = false
        >
    constexpr bool overlaps(const aabb<T, N>& oth) const
    {
        if constexpr (inclusive0 && inclusive1) {
            // Both inclusive.
            return (arr_[0] <= oth.arr_[1] &&
                    arr_[1] >= oth.arr_[0]).all();
        }
        else if constexpr (inclusive0 && !inclusive1) {
            // First inclusive, second exclusive.
            return (arr_[0] <= oth.arr_[1] &&
                    arr_[1] > oth.arr_[0]).all();
        }
        else if constexpr (!inclusive0 && inclusive1) {
            // First exclusive, second inclusive.
            return (arr_[0] < oth.arr_[1] &&
                    arr_[1] >= oth.arr_[0]).all();
        }
        else {
            // Both exclusive.
            return (arr_[0] < oth.arr_[1] &&
                    arr_[1] > oth.arr_[0]).all();
        }
    }

    /**
     * @brief Contains other?
     */
    template <
        bool inclusive0 = true,
        bool inclusive1 = false
        >
    constexpr bool contains(const aabb<T, N>& oth) const
    {
        if constexpr (inclusive0 && inclusive1) {
            // Both inclusive.
            return (arr_[0] <= oth.arr_[0] &&
                    arr_[1] >= oth.arr_[1]).all();
        }
        else if constexpr (inclusive0 && !inclusive1) {
            // First inclusive, second exclusive.
            return (arr_[0] <= oth.arr_[0] &&
                    arr_[1] > oth.arr_[1]).all();
        }
        else if constexpr (!inclusive0 && inclusive1) {
            // First exclusive, second inclusive.
            return (arr_[0] < oth.arr_[0] &&
                    arr_[1] >= oth.arr_[1]).all();
        }
        else {
            // Both exclusive.
            return (arr_[0] < oth.arr_[0] &&
                    arr_[1] > oth.arr_[1]).all();
        }
    }

    /**
     * @brief Overlaps array?
     */
    template <
        bool inclusive0 = true,
        bool inclusive1 = false
        >
    constexpr bool overlaps(const multi<T, N>& arr) const
    {
        return overlaps<inclusive0, inclusive1>(aabb<T, N>(arr));
    }

    /**
     * @brief Contains array?
     */
    template <
        bool inclusive0 = true,
        bool inclusive1 = false
        >
    constexpr bool contains(const multi<T, N>& arr) const
    {
        return contains<inclusive0, inclusive1>(aabb<T, N>(arr));
    }

    /**@}*/

private:

    /**
     * @brief Arrays.
     */
    multi<T, N> arr_[2] = {
        multi<T, N>(+pre::numeric_limits<T>::max()),
        multi<T, N>(-pre::numeric_limits<T>::max())
    };

public:

    /**
     * @name Stream operators
     */
    /**@{*/

    /**
     * @brief Parse from `std::basic_istream`.
     *
     * Format is `(b0,b1)`. Sets `std::ios_base::failbit` on error.
     */
    template <typename C, typename Ctraits>
    friend
    inline std::basic_istream<C, Ctraits>& operator>>(
           std::basic_istream<C, Ctraits>& is, aabb& box)
    {
        C ch;
        if (!(is >> ch) ||
            !Ctraits::eq(ch,
             Ctraits::to_char_type('('))) {
            is.setstate(std::ios_base::failbit);
            return is;
        }
        is >> box[0];
        if (!(is >> ch) ||
            !Ctraits::eq(ch,
             Ctraits::to_char_type(','))) {
            is.setstate(std::ios_base::failbit);
            return is;
        }
        is >> box[1];
        if (!(is >> ch) ||
            !Ctraits::eq(ch,
             Ctraits::to_char_type(')'))) {
            is.setstate(std::ios_base::failbit);
            return is;
        }
        return is;
    }

    /**
     * @brief Write into `std::basic_ostream`.
     *
     * Format is `(b0,b1)`.
     */
    template <typename C, typename Ctraits>
    friend
    inline std::basic_ostream<C, Ctraits>& operator<<(
           std::basic_ostream<C, Ctraits>& os, const aabb& box)
    {
        os << '(';
        os << box[0] << ',';
        os << box[1] << ')';
        return os;
    }

    /**@}*/
};

/**
 * @name Binary operators (aabb/aabb)
 */
/**@{*/

/**
 * @brief Set union.
 */
template <typename T, std::size_t N>
__attribute__((always_inline))
inline aabb<T, N> operator|(const aabb<T, N>& box0, const aabb<T, N>& box1)
{
    if constexpr (std::is_floating_point<T>::value) {
        return {
            pre::fmin(box0[0], box1[0]),
            pre::fmax(box0[1], box1[1])
        };
    }
    else {
        return {
            pre::min(box0[0], box1[0]),
            pre::max(box0[1], box1[1])
        };
    }
}

/**
 * @brief Set intersection.
 */
template <typename T, std::size_t N>
__attribute__((always_inline))
inline aabb<T, N> operator&(const aabb<T, N>& box0, const aabb<T, N>& box1)
{
    if constexpr (std::is_floating_point<T>::value) {
        return {
            pre::fmax(box0[0], box1[0]),
            pre::fmin(box0[1], box1[1])
        };
    }
    else {
        return {
            pre::max(box0[0], box1[0]),
            pre::min(box0[1], box1[1])
        };
    }
}

/**@}*/

/**
 * @name Binary operators (aabb/multi)
 */
/**@{*/

/**
 * @brief Set union.
 */
template <typename T, std::size_t N>
__attribute__((always_inline))
inline aabb<T, N> operator|(const aabb<T, N>& box, const multi<T, N>& arr)
{
    return box | aabb<T, N>(arr);
}

/**
 * @brief Set intersection.
 */
template <typename T, std::size_t N>
__attribute__((always_inline))
inline aabb<T, N> operator&(const aabb<T, N>& box, const multi<T, N>& arr)
{
    return box & aabb<T, N>(arr);
}

/**@}*/

/**
 * @name Binary operators (multi/aabb)
 */
/**@{*/

/**
 * @brief Set union.
 */
template <typename T, std::size_t N>
__attribute__((always_inline))
inline aabb<T, N> operator|(const multi<T, N>& arr, const aabb<T, N>& box)
{
    return aabb<T, N>(arr) | box;
}

/**
 * @brief Set intersection.
 */
template <typename T, std::size_t N>
__attribute__((always_inline))
inline aabb<T, N> operator&(const multi<T, N>& arr, const aabb<T, N>& box)
{
    return aabb<T, N>(arr) & box;
}

/**@}*/

/**
 * @name Binary operators (aabb/any)
 */
/**@{*/

/**
 * @brief Generic `operator|=`.
 */
template <
    typename T, std::size_t N,
    typename U
    >
__attribute__((always_inline))
inline aabb<T, N>& operator|=(aabb<T, N>& box, const U& any)
{
    return box = box | any;
}

/**
 * @brief Generic `operator&=`.
 */
template <
    typename T, std::size_t N,
    typename U
    >
__attribute__((always_inline))
inline aabb<T, N>& operator&=(aabb<T, N>& box, const U& any)
{
    return box = box & any;
}

/**@}*/

/**
 * @brief Template alias for convenience.
 */
template <typename T>
using aabb1 = aabb<T, 1>;

/**
 * @brief Template alias for convenience.
 */
template <typename T>
using aabb2 = aabb<T, 2>;

/**
 * @brief Template alias for convenience.
 */
template <typename T>
using aabb3 = aabb<T, 3>;

/**
 * @brief Template alias for convenience.
 */
template <typename T>
using aabb4 = aabb<T, 4>;

/**@}*/

} // namespace pre

#endif // #ifndef PREFORM_AABB_HPP
