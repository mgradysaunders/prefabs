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
#error "preform/dense_vector_view.hpp requires >=C++14"
#endif // #if !(__cplusplus >= 201402L)
#endif // #if !DOXYGEN
#pragma once
#ifndef PREFORM_DENSE_VECTOR_VIEW_HPP
#define PREFORM_DENSE_VECTOR_VIEW_HPP

// for std::plus, std:multiplies, ...
#include <functional>

// for std::basic_istream
#include <istream>

// for std::basic_ostream
#include <ostream>

// for std::forward
#include <utility>

// for std::invalid_argument
#include <stdexcept>

// for pre::dense_iterator
#include <preform/dense_iterator.hpp>

namespace pre {

/**
 * @defgroup dense_vector_view Dense vector view
 *
 * `<preform/dense_vector_view.hpp>`
 *
 * __C++ version__: >=C++14
 */
/**@{*/

/**
 * @brief Dense vector view.
 *
 * @tparam Tbase_iterator
 * Base iterator, must be random access.
 */
template <typename Tbase_iterator>
struct dense_vector_view
{
    // Sanity check.
    static_assert(
        std::is_base_of<
        std::random_access_iterator_tag,
        typename std::iterator_traits<
                Tbase_iterator>::iterator_category>::value,
        "Tbase_iterator must be random access");

public:

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
    typedef std::remove_const_t<
            typename base_iterator_traits::value_type> value_type;

    /**
     * @brief Pointer.
     */
    typedef typename base_iterator_traits::pointer pointer;

    /**
     * @brief Reference.
     */
    typedef typename base_iterator_traits::reference reference;

    /**
     * @brief Iterator.
     */
    typedef dense_iterator<base_iterator> iterator;

    /**
     * @brief Reverse iterator.
     */
    typedef std::reverse_iterator<iterator> reverse_iterator;

public:

    /**
     * @brief Default constructor.
     */
    constexpr dense_vector_view() = default;

    /**
     * @brief Constructor.
     *
     * @param[in] begin_itr
     * Begin iterator.
     *
     * @param[in] begin_inc
     * Begin iterator increment.
     *
     * @param[in] sz
     * Size.
     */
    constexpr dense_vector_view(
            base_iterator begin_itr,
            difference_type begin_inc,
            difference_type sz) :
                begin_itr_(begin_itr),
                begin_inc_(begin_inc),
                size_(sz)
    {
    }

    /**
     * @brief Constructor.
     *
     * @param[in] from
     * Range from.
     *
     * @param[in] to
     * Range to.
     *
     * @param[in] inc
     * Iterator increment.
     */
    constexpr dense_vector_view(
            base_iterator from,
            base_iterator to,
            difference_type inc = 1) :
                begin_itr_(from),
                begin_inc_(inc),
                size_((to - from) / inc)
    {
    }

public:

    /**
     * @brief Begin iterator.
     */
    base_iterator begin_itr_ = base_iterator();

    /**
     * @brief Begin iterator increment.
     */
    difference_type begin_inc_ = 0;

    /**
     * @brief Size.
     */
    difference_type size_ = 0;

public:

    /**
     * @brief Size.
     */
    constexpr difference_type size() const
    {
        return size_;
    }

    /**
     * @brief Empty?
     */
    constexpr bool empty() const
    {
        return !(size_ > 0);
    }

    /**
     * @brief Begin iterator.
     */
    constexpr iterator begin()
    {
        return {
            begin_itr_,
            begin_inc_
        };
    }

    /**
     * @brief End iterator.
     */
    constexpr iterator end()
    {
        if (begin_itr_) {
            return begin() + size();
        }
        else {
            return begin();
        }
    }

    /**
     * @brief Reverse begin iterator.
     */
    constexpr reverse_iterator rbegin()
    {
        return reverse_iterator(end());
    }

    /**
     * @brief Reverse end iterator.
     */
    constexpr reverse_iterator rend()
    {
        return reverse_iterator(begin());
    }

    /**
     * @brief Access operator.
     */
    constexpr reference operator[](difference_type n)
    {
        return *(begin() + n);
    }

public:

    /**
     * @name Range manipulation
     */
    /**@{*/

    /**
     * @brief Range @f$ [i_0, i_1) @f$.
     *
     * @throw std::invalid_argument
     * If invalid index.
     */
    constexpr dense_vector_view range(
                difference_type i0,
                difference_type i1)
    {
        if (!(i0 >= 0 && i0 <= size_ &&
              i1 >= 0 && i1 <= size_)) {
            throw std::invalid_argument(__PRETTY_FUNCTION__);
        }
        if (!begin_itr_) {
            return dense_vector_view();
        }
        else {
            return {
                begin_itr_ +
                begin_inc_ * (i0 - (i0 > i1)),
                (i0 > i1) ? -begin_inc_ : begin_inc_,
                (i0 > i1) ? (i0 - i1) : (i1 - i0)
            };
        }
    }

    /**
     * @brief Reverse.
     *
     * @note
     * Equivalent to `range(size(), 0)`.
     */
    constexpr dense_vector_view reverse()
    {
        return range(size_, 0);
    }

    /**@}*/

public:

    /**
     * @name Cast operators
     */
    /**@{*/

    /**
     * @brief Cast as bool.
     *
     * @note
     * Equivalent to `!empty()`.
     */
    constexpr operator bool() const
    {
        return !empty();
    }

    /**
     * @brief Cast as different underlying iterator.
     */
    template <typename U>
    constexpr operator dense_vector_view<U>() const
    {
        return {
            begin_itr_,
            begin_inc_,
            size_
        };
    }

    /**@}*/

public:

    /**
     * @name Folds
     */
    /**@{*/

    /**
     * @brief Left-to-right fold.
     *
     * @note
     * - @f$ k \gets 0 @f$
     * - @f$ x \gets v_k @f$
     * - While @f$ k + 1 < M @f$:
     *      + @f$ x \gets f(x, v_{k + 1}) @f$
     *      + @f$ k \gets k + 1 @f$
     * - Return @f$ x @f$
     */
    template <typename F>
    constexpr value_type foldl(F&& func)
    {
        auto itr = begin();
        auto val = value_type(*itr++);
        while (itr < end()) {
            val = std::forward<F>(func)(val, *itr++);
        }
        return val;
    }

    /**
     * @brief Right-to-left fold.
     *
     * @note
     * - @f$ k \gets M - 1 @f$
     * - @f$ x \gets v_k @f$
     * - While @f$ k - 1 \ge 0 @f$:
     *      + @f$ x \gets f(x, v_{k - 1}) @f$
     *      + @f$ k \gets k - 1 @f$
     * - Return @f$ x @f$
     */
    template <typename F>
    constexpr value_type foldr(F&& func)
    {
        auto itr = rbegin();
        auto val = value_type(*itr++);
        while (itr < rend()) {
            val = std::forward<F>(func)(val, *itr++);
        }
        return val;
    }


    /**
     * @brief Left-to-right accumulating fold.
     *
     * @note
     * - @f$ k \gets 0 @f$
     * - While @f$ k + 1 < M @f$:
     *      + @f$ v_{k + 1} \gets f(v_{k + 1}, v_k) @f$
     *      + @f$ k \gets k + 1 @f$
     */
    template <typename F>
    constexpr dense_vector_view& foldl_accum(F&& func)
    {
        auto itr1 = begin();
        auto itr2 = itr1 + 1;
        while (itr2 < end()) {
            *itr2 = std::forward<F>(func)(*itr2, *itr1);
            ++itr1;
            ++itr2;
        }
        return *this;
    }

    /**
     * @brief Right-to-left accumulating fold.
     *
     * @note
     * - @f$ k \gets M - 1 @f$
     * - While @f$ k - 1 \ge 0 @f$:
     *      + @f$ v_{k - 1} \gets f(v_{k - 1}, v_k) @f$
     *      + @f$ k \gets k - 1 @f$
     */
    template <typename F>
    constexpr dense_vector_view& foldr_accum(F&& func)
    {
        auto itr1 = rbegin();
        auto itr2 = itr1 + 1;
        while (itr2 < rend()) {
            *itr2 = std::forward<F>(func)(*itr2, *itr1);
            ++itr1;
            ++itr2;
        }
        return *this;
    }

    /**
     * @brief Left-to-right adjacent fold.
     *
     * @note
     * - @f$ k \gets 0 @f$
     * - While @f$ k + 1 < M @f$:
     *      + @f$ v_k \gets f(v_k, v_{k + 1}) @f$
     *      + @f$ k \gets k + 1 @f$
     */
    template <typename F>
    constexpr dense_vector_view& foldl_adjacent(F&& func)
    {
        auto itr1 = begin();
        auto itr2 = itr1 + 1;
        while (itr2 < end()) {
            *itr1 = std::forward<F>(func)(*itr1, *itr2);
            ++itr1;
            ++itr2;
        }
        return *this;
    }

    /**
     * @brief Right-to-left adjacent fold.
     *
     * @note
     * - @f$ k \gets M - 1 @f$
     * - While @f$ k - 1 \ge 0 @f$:
     *      + @f$ v_k \gets f(v_k, v_{k - 1}) @f$
     *      + @f$ k \gets k - 1 @f$
     */
    template <typename F>
    constexpr dense_vector_view& foldr_adjacent(F&& func)
    {
        auto itr1 = rbegin();
        auto itr2 = itr1 + 1;
        while (itr2 < rend()) {
            *itr1 = std::forward<F>(func)(*itr1, *itr2);
            ++itr1;
            ++itr2;
        }
        return *this;
    }

    /**@}*/

public:

    /**
     * @name Fold utilities
     */
    /**@{*/

    /**
     * @brief Sum.
     *
     * @note
     * Equivalent to `foldl(std::plus<value_type>())`.
     */
    constexpr value_type sum()
    {
        return foldl(std::plus<value_type>());
    }

    /**
     * @brief Product.
     *
     * @note
     * Equivalent to `foldl(std::multiplies<value_type>())`.
     */
    constexpr value_type prod()
    {
        return foldl(std::multiplies<value_type>());
    }

    /**
     * @brief Accumulating sum.
     *
     * @note
     * Equivalent to `foldl_accum(std::plus<value_type>())`.
     */
    constexpr dense_vector_view& accum_sum()
    {
        return foldl_accum(std::plus<value_type>());
    }

    /**
     * @brief Adjacent difference.
     *
     * @note
     * Equivalent to `foldr_adjacent(std::minus<value_type>())`.
     */
    constexpr dense_vector_view& adjacent_difference()
    {
        return foldr_adjacent(std::minus<value_type>());
    }

    /**@}*/

public:

    /**
     * @name Stream operators
     */
    /**@{*/

    /**
     * @brief Parse from `std::basic_istream`.
     *
     * Format is `[v0,v1,...]`.
     * Sets `std::ios_base::failbit` on error.
     */
    template <typename C, typename Ctraits>
    friend
    inline std::basic_istream<C, Ctraits>& operator>>(
           std::basic_istream<C, Ctraits>& is, dense_vector_view arr)
    {
        C ch;
        if (!(is >> ch) ||
            !Ctraits::eq(ch,
             Ctraits::to_char_type('['))) {
            is.setstate(std::ios_base::failbit);
            return is;
        }
        if (arr.size() > 0) {
            auto itr = arr.begin();
            is >> *itr++;
            while (itr < arr.end()) {
                if (!(is >> ch) ||
                    !Ctraits::eq(ch,
                     Ctraits::to_char_type(','))) {
                    is.setstate(std::ios_base::failbit);
                    return is;
                }
                is >> *itr++;
            }
        }
        if (!(is >> ch) ||
            !Ctraits::eq(ch,
             Ctraits::to_char_type(']'))) {
            is.setstate(std::ios_base::failbit);
            return is;
        }
        return is;
    }

    /**
     * @brief Write into `std::basic_ostream`.
     *
     * Format is `[v0,v1,...]`.
     */
    template <typename C, typename Ctraits>
    friend
    inline std::basic_ostream<C, Ctraits>& operator<<(
           std::basic_ostream<C, Ctraits>& os, dense_vector_view arr)
    {
        os << '[';
        if (arr.size() > 0) {
            auto itr = arr.begin();
            os << *itr++;
            while (itr < arr.end()) {
                os << ',';
                os << *itr++;
            }
        }
        os << ']';
        return os;
    }

    /**@}*/
};

/**@}*/

} // namespace pre

#if !DOXYGEN
#include "dense_vector_view.inl"
#endif // #if !DOXYGEN

#endif // #ifndef PREFORM_DENSE_VECTOR_VIEW_HPP
