/* Copyright (c) 2018 M. Grady Saunders
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
#error "prefabs/aabb.hpp requires >=C++17"
#endif // #if !(__cplusplus >= 201703L)
#endif // #if !DOXYGEN
#pragma once
#ifndef PREFABS_AABB_HPP
#define PREFABS_AABB_HPP

// for pr::multi
#include <prefabs/multi.hpp>

// for pr::multi math wrappers
#include <prefabs/multi_math.hpp>

namespace pr {

/**
 * @defgroup aabb Axis-aligned bounding box
 *
 * `<prefabs/aabb.hpp>`
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

    /**
     * @brief Value type.
     */
    typedef multi<T, N> value_type;

    /**
     * @brief Entry type.
     */
    typedef T entry_type;

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
     * @brief Diagonal.
     */
    constexpr multi<T, N> diag() const
    {
        return arr_[1] - arr_[0];
    }

    /**
     * @brief Hyper volume.
     */
    constexpr T volume() const
    {
        multi<T, N> d = diag();
        return d.prod();
    }

    /**
     * @brief Hyper surface area.
     */
    constexpr T surface_area() const
    {
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
        return T(2) * a;
    }

    /**
     * @brief Center.
     */
    constexpr multi<T, N> center() const
    {
        return (arr_[0] + arr_[1]) / T(2);
    }

    /**
     * @brief Lerp minimum and maximum coordinates.
     */
    template <typename U>
    constexpr multi<T, N> lerp(const U& u) const
    {
        return (1 - u) * arr_[0] + u * arr_[1];
    }

public:

    /**
     * @name Tests
     */
    /**@{*/

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

    /**
     * @brief Hyper volume is non-negative?
     */
    constexpr operator bool() const
    {
        return (arr_[0] <= arr_[1]).all();
    }

    /**@}*/

private:

    /**
     * @brief Arrays.
     */
    multi<T, N> arr_[2] = {
        multi<T, N>::value(+pr::numeric_limits<T>::max()),
        multi<T, N>::value(-pr::numeric_limits<T>::max())
    };
};

/**
 * @name Binary operators (aabb/aabb)
 */
/**@{*/

/**
 * @brief Set union.
 */
template <typename T, std::size_t N>
constexpr aabb<T, N> operator|(const aabb<T, N>& box0, const aabb<T, N>& box1)
{
    return {pr::min(box0[0], box1[0]),
            pr::max(box0[1], box1[1])};
}

/**
 * @brief Set intersection.
 */
template <typename T, std::size_t N>
constexpr aabb<T, N> operator&(const aabb<T, N>& box0, const aabb<T, N>& box1)
{
    return {pr::max(box0[0], box1[0]),
            pr::min(box0[1], box1[1])};
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
constexpr aabb<T, N> operator|(const aabb<T, N>& box, const multi<T, N>& arr)
{
    return {pr::min(box[0], arr),
            pr::max(box[1], arr)};
}

/**
 * @brief Set intersection.
 */
template <typename T, std::size_t N>
constexpr aabb<T, N> operator&(const aabb<T, N>& box, const multi<T, N>& arr)
{
    return {pr::max(box[0], arr),
            pr::min(box[1], arr)};
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
constexpr aabb<T, N> operator|(const multi<T, N>& arr, const aabb<T, N>& box)
{
    return {pr::min(arr, box[0]),
            pr::max(arr, box[1])};
}

/**
 * @brief Set intersection.
 */
template <typename T, std::size_t N>
constexpr aabb<T, N> operator&(const multi<T, N>& arr, const aabb<T, N>& box)
{
    return {pr::max(arr, box[0]),
            pr::min(arr, box[1])};
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
constexpr aabb<T, N>& operator|=(aabb<T, N>& box, const U& any)
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
constexpr aabb<T, N>& operator&=(aabb<T, N>& box, const U& any)
{
    return box = box & any;
}

/**@}*/

/**@}*/

} // namespace pr

#endif // #ifndef PREFABS_AABB_HPP
