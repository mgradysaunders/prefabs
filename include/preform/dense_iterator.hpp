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
#if !DOXYGEN
#if !(__cplusplus >= 201402L)
#error "preform/dense_iterator.hpp requires >=C++14"
#endif // #if !(__cplusplus >= 201402L)
#endif // #if !DOXYGEN
#pragma once
#ifndef PREFORM_DENSE_ITERATOR_HPP
#define PREFORM_DENSE_ITERATOR_HPP

// for std::iterator_traits, std::random_access_iterator_tag
#include <iterator>

// for std::is_base_of
#include <type_traits>

namespace pr {

/**
 * @defgroup dense_iterator Dense iterator
 *
 * `<preform/dense_iterator.hpp>`
 *
 * __C++ version__: >=C++14
 */
/**@{*/

/**
 * @brief Dense iterator.
 */
template <typename T> 
class dense_iterator
{
public:

    // Sanity check.
    static_assert(
        std::is_base_of<
        std::random_access_iterator_tag,
        typename std::iterator_traits<T>::iterator_category>::value,
        "T must be random access");

    /**
     * @brief Difference type.
     */
    typedef typename std::iterator_traits<T>::difference_type difference_type;

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
     * @brief Iterator category.
     */
    typedef std::random_access_iterator_tag iterator_category;

public:

    /**
     * @brief Position.
     */
    T pos_;

    /**
     * @brief Increment stride.
     */
    difference_type stride_;

public:

    /**
     * @brief Pre-increment.
     */
    constexpr dense_iterator& operator++()
    {
        pos_ += stride_; return *this;
    }

    /**
     * @brief Pre-decrement.
     */
    constexpr dense_iterator& operator--()
    {
        pos_ -= stride_; return *this;
    }

    /**
     * @brief Post-increment.
     */
    constexpr dense_iterator operator++(int)
    {
        dense_iterator res = *this; operator++(); return res;
    }

    /**
     * @brief Post-decrement.
     */
    constexpr dense_iterator operator--(int)
    {
        dense_iterator res = *this; operator--(); return res;
    }

    /**
     * @brief Dereference.
     */
    constexpr reference operator*()
    {
        return *pos_;
    }

    /**
     * @brief Pointer access.
     */
    constexpr pointer operator->()
    {
        return pos_;
    }

    /**
     * @brief Arbitrary access.
     */
    constexpr reference operator[](difference_type n)
    {
        return *(pos_ + stride_ * n);
    }

    /**
     * @brief Arbitrary increment.
     */
    constexpr dense_iterator& operator+=(difference_type n)
    {
        pos_ += stride_ * n; return *this;
    }

    /**
     * @brief Arbitrary increment.
     */
    constexpr dense_iterator operator+(difference_type n) const
    {
        return {pos_ + stride_ * n, stride_};
    }

    /**
     * @brief Arbitrary increment.
     */
    friend
    constexpr dense_iterator operator+(difference_type n, 
              dense_iterator itr)
    {
        return itr + n;
    }

    /**
     * @brief Arbitrary decrement.
     */
    constexpr dense_iterator& operator-=(difference_type n)
    {
        pos_ -= stride_ * n; return *this;
    }

    /**
     * @brief Arbitrary decrement.
     */
    constexpr dense_iterator operator-(difference_type n) const
    {
        return {pos_ - stride_ * n, stride_};
    }

    /**
     * @brief Distance.
     */
    constexpr difference_type operator-(dense_iterator other) const
    {
        return (pos_ - other.pos_) / other.stride_;
    }

public:

    /**
     * @brief Compare equal.
     */
    constexpr bool operator==(dense_iterator other) const
    {
        return pos_ == other.pos_;
    }

    /**
     * @brief Compare not equal.
     */
    constexpr bool operator!=(dense_iterator other) const
    {
        return pos_ != other.pos_;
    }

    /**
     * @brief Compare less.
     */
    constexpr bool operator<(dense_iterator other) const
    {
        if (stride_ > 0) {
            return pos_ < other.pos_;
        }
        else {
            return pos_ > other.pos_;
        }
    }

    /**
     * @brief Compare greater.
     */
    constexpr bool operator>(dense_iterator other) const
    {
        if (stride_ > 0) {
            return pos_ > other.pos_;
        }
        else {
            return pos_ < other.pos_;
        }
    }

    /**
     * @brief Compare less equal. 
     */
    constexpr bool operator<=(dense_iterator other) const
    {
        if (stride_ > 0) {
            return pos_ <= other.pos_;
        }
        else {
            return pos_ >= other.pos_;
        }
    }

    /**
     * @brief Compare greater equal. 
     */
    constexpr bool operator>=(dense_iterator other) const
    {
        if (stride_ > 0) {
            return pos_ >= other.pos_;
        }
        else {
            return pos_ <= other.pos_;
        }
    }
};

/**@}*/

} // namespace pr

#endif // #ifndef PREFORM_DENSE_ITERATOR_HPP
