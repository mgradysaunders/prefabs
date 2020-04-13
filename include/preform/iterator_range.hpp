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
#if !(__cplusplus >= 201402L)
#error "preform/iterator_range.hpp requires >=C++14"
#endif // #if !(__cplusplus >= 201402L)
#endif // #if !DOXYGEN
#pragma once
#ifndef PREFORM_ITERATOR_RANGE_HPP
#define PREFORM_ITERATOR_RANGE_HPP

// for std::iterator_traits
#include <iterator>

namespace pre {

/**
 * @defgroup iterator_range Iterator range
 *
 * `<preform/iterator_range.hpp>`
 *
 * __C++ version__: >=C++14
 */
/**@{*/

/**
 * @brief Iterator range.
 */
template <typename T>
class iterator_range
{
public:

    /**
     * @brief Iterator type.
     */
    typedef T iterator;

    /**
     * @brief Value type.
     */
    typedef typename std::iterator_traits<T>::value_type value_type;

    /**
     * @brief Pointer.
     */
    typedef typename std::iterator_traits<T>::pointer pointer;

    /**
     * @brief Reference.
     */
    typedef typename std::iterator_traits<T>::reference reference;

    /**
     * @brief Difference type.
     */
    typedef typename std::iterator_traits<T>::difference_type
            difference_type;

public:

    /**
     * @brief Default constructor.
     */
    constexpr iterator_range() = default;

    /**
     * @brief Constructor.
     */
    constexpr iterator_range(iterator from, iterator to) :
        begin_(from),
        end_(to)
    {
    }

    /**
     * @brief Iterator to begin.
     */
    constexpr iterator begin()
    {
        return begin_;
    }

    /**
     * @brief Iterator to end.
     */
    constexpr iterator end()
    {
        return end_;
    }

    /**
     * @brief Size.
     */
    constexpr difference_type size()
    {
        if (empty()) {
            return 0;
        }
        else {
            return std::distance(begin_, end_);
        }
    }

    /**
     * @brief Empty?
     */
    constexpr bool empty()
    {
        return begin_ == end_;
    }

    /**
     * @brief Front.
     */
    constexpr reference front()
    {
        return operator[](0);
    }

    /**
     * @brief Back.
     */
    constexpr reference back()
    {
        return operator[](size() - 1);
    }

    /**
     * @brief Access operator.
     */
    constexpr reference operator[](difference_type pos)
    {
        return *std::next(begin_, pos);
    }

    /**
     * @brief Cast operator.
     */
    template <typename U>
    constexpr operator iterator_range<U>()
    {
        return {
            U(begin_),
            U(end_)
        };
    }

private:

    /**
     * @brief Iterator to begin.
     */
    iterator begin_ = iterator();

    /**
     * @brief Iterator to end.
     */
    iterator end_ = iterator();
};

/**
 * @brief Make iterator range.
 */
template <typename T>
constexpr iterator_range<T> make_iterator_range(T from, T to)
{
    return {from, to};
}

/**
 * @brief Make iterator range.
 */
template <typename T>
constexpr iterator_range<typename T::iterator>
                    make_iterator_range(T& container)
{
    return {container.begin(), container.end()};
}

/**
 * @brief Make iterator range.
 */
template <typename T>
constexpr iterator_range<typename T::const_iterator>
                    make_iterator_range(const T& container)
{
    return {container.begin(), container.end()};
}

/**@}*/

} // namespace pre

#endif // #ifndef PREFORM_ITERATOR_RANGE_HPP
