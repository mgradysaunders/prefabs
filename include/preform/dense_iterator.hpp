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
 *
 * @tparam Tbase_iterator
 * Base iterator, must be random access.
 */
template <typename Tbase_iterator>
class dense_iterator
{
public:

    // Sanity check.
    static_assert(
        std::is_base_of<
        std::random_access_iterator_tag,
        typename std::iterator_traits<
                Tbase_iterator>::iterator_category>::value,
        "Tbase_iterator must be random access");

    /**
     * @brief Base iterator.
     */
    typedef Tbase_iterator base_iterator;

    /**
     * @brief Base iterator traits.
     */
    typedef std::iterator_traits<base_iterator> base_iterator_traits;

    /**
     * @brief Difference type.
     */
    typedef typename base_iterator_traits::difference_type difference_type;

    /**
     * @brief Value type.
     */
    typedef typename base_iterator_traits::value_type value_type;

    /**
     * @brief Pointer.
     */
    typedef typename base_iterator_traits::pointer pointer;

    /**
     * @brief Reference.
     */
    typedef typename base_iterator_traits::reference reference;

    /**
     * @brief Iterator category.
     */
    typedef std::random_access_iterator_tag iterator_category;

public:

    /**
     * @brief Default constructor.
     */
    constexpr dense_iterator() = default;

    /**
     * @brief Constructor.
     */
    constexpr dense_iterator(base_iterator itr, difference_type inc = 1) :
            itr_(itr),
            inc_(inc)
    {
    }

public:

    /**
     * @brief Iterator.
     */
    base_iterator itr_ = base_iterator();

    /**
     * @brief Iterator increment.
     */
    difference_type inc_ = 0;

public:

    /**
     * @brief Pre-increment.
     */
    constexpr dense_iterator& operator++()
    {
        itr_ += inc_; return *this;
    }

    /**
     * @brief Pre-decrement.
     */
    constexpr dense_iterator& operator--()
    {
        itr_ -= inc_; return *this;
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
        return *itr_;
    }

    /**
     * @brief Pointer access.
     */
    constexpr pointer operator->()
    {
        return itr_;
    }

    /**
     * @brief Arbitrary access.
     */
    constexpr reference operator[](difference_type n)
    {
        return *(itr_ + inc_ * n);
    }

    /**
     * @brief Arbitrary increment.
     */
    constexpr dense_iterator& operator+=(difference_type n)
    {
        itr_ += inc_ * n; return *this;
    }

    /**
     * @brief Arbitrary increment.
     */
    constexpr dense_iterator operator+(difference_type n) const
    {
        return {itr_ + inc_ * n, inc_};
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
        itr_ -= inc_ * n; return *this;
    }

    /**
     * @brief Arbitrary decrement.
     */
    constexpr dense_iterator operator-(difference_type n) const
    {
        return {itr_ - inc_ * n, inc_};
    }

    /**
     * @brief Distance.
     */
    constexpr difference_type operator-(dense_iterator other) const
    {
        return (itr_ - other.itr_) / other.inc_;
    }

public:

    /**
     * @brief Compare equal.
     */
    constexpr bool operator==(dense_iterator other) const
    {
        return itr_ == other.itr_;
    }

    /**
     * @brief Compare not equal.
     */
    constexpr bool operator!=(dense_iterator other) const
    {
        return itr_ != other.itr_;
    }

    /**
     * @brief Compare less.
     */
    constexpr bool operator<(dense_iterator other) const
    {
        if (inc_ > 0) {
            return itr_ < other.itr_;
        }
        else {
            return itr_ > other.itr_;
        }
    }

    /**
     * @brief Compare greater.
     */
    constexpr bool operator>(dense_iterator other) const
    {
        if (inc_ > 0) {
            return itr_ > other.itr_;
        }
        else {
            return itr_ < other.itr_;
        }
    }

    /**
     * @brief Compare less equal.
     */
    constexpr bool operator<=(dense_iterator other) const
    {
        if (inc_ > 0) {
            return itr_ <= other.itr_;
        }
        else {
            return itr_ >= other.itr_;
        }
    }

    /**
     * @brief Compare greater equal.
     */
    constexpr bool operator>=(dense_iterator other) const
    {
        if (inc_ > 0) {
            return itr_ >= other.itr_;
        }
        else {
            return itr_ <= other.itr_;
        }
    }
};

/**@}*/

} // namespace pr

#endif // #ifndef PREFORM_DENSE_ITERATOR_HPP
