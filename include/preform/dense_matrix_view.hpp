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
#error "preform/dense_matrix_view.hpp requires >=C++14"
#endif // #if !(__cplusplus >= 201402L)
#endif // #if !DOXYGEN
#pragma once
#ifndef PREFORM_DENSE_MATRIX_VIEW_HPP
#define PREFORM_DENSE_MATRIX_VIEW_HPP

// for pr::dense_vector_view, pr::dense_iterator, ...
#include <preform/dense_vector_view.hpp>

namespace pr {

/**
 * @defgroup dense_matrix_view Dense matrix view
 *
 * `<preform/dense_matrix_view.hpp>`
 *
 * __C++ version__: >=C++14
 */
/**@{*/

/**
 * @brief Dense matrix view.
 *
 * @tparam Tbase_iterator
 * Base iterator, must be random access.
 */
template <typename Tbase_iterator>
struct dense_matrix_view
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
    typedef std::iterator_traits<Tbase_iterator> base_iterator_traits;

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

public:

    /**
     * @brief Begin iterator.
     */
    base_iterator begin_itr_ = base_iterator();

    /**
     * @brief Begin iterator increment in 0th dimension (0th index multiplier).
     */
    difference_type begin_inc0_ = 0;

    /**
     * @brief Begin iterator increment in 1st dimension (1st index multiplier).
     */
    difference_type begin_inc1_ = 0;

    /**
     * @brief Size of 0th dimension.
     */
    difference_type size0_ = 0;

    /**
     * @brief Size of 1st dimension.
     */
    difference_type size1_ = 0;

public:

    /**
     * @brief Size.
     */
    constexpr std::pair<difference_type, difference_type> size() const
    {
        return {
            size0_,
            size1_
        };
    }

    /**
     * @brief Size of 0th dimension.
     */
    constexpr difference_type size0() const
    {
        return size0_;
    }

    /**
     * @brief Size of 1st dimension.
     */
    constexpr difference_type size1() const
    {
        return size1_;
    }

    /**
     * @brief Empty?
     */
    constexpr bool empty() const
    {
        return !(size0_ > 0 && size1_ > 0);
    }

    /**
     * @brief Access operator.
     *
     * @note
     * Equivalent to `row(n)`.
     */
    constexpr dense_vector_view<Tbase_iterator> operator[](difference_type n)
    {
        return row(n);
    }

public:

    /**
     * @name Range manipulation
     */
    /**@{*/

    /**
     * @brief Row.
     *
     * @throw std::invalid_argument
     * If invalid index.
     */
    constexpr dense_vector_view<Tbase_iterator> row(difference_type i)
    {
        if (i < 0 || i >= size0_) {
            throw std::invalid_argument(__PRETTY_FUNCTION__);
        }

        return {
            begin_itr_ +
            begin_inc0_ * i,
            begin_inc1_,
            size1_
        };
    }

    /**
     * @brief Column.
     *
     * @throw std::invalid_argument
     * If invalid index.
     */
    constexpr dense_vector_view<Tbase_iterator> col(difference_type j)
    {
        if (j < 0 || j >= size1_) {
            throw std::invalid_argument(__PRETTY_FUNCTION__);
        }

        return {
            begin_itr_ +
            begin_inc1_ * j,
            begin_inc0_,
            size0_
        };
    }

    /**
     * @brief Diagonal.
     *
     * @throw std::invalid_argument
     * If invalid index.
     */
    constexpr dense_vector_view<Tbase_iterator> diag(difference_type k = 0)
    {
        if (k <= -size0_ || k >= size1_) {
            throw std::invalid_argument(__PRETTY_FUNCTION__);
        }

        if (k >= 0) {
            // Above diagonal.
            return {
                begin_itr_ +
                begin_inc1_ * k,
                begin_inc0_ + begin_inc1_,
                std::min(size0_, size1_ - k)
            };
        }
        else {
            // Below diagonal.
            return {
                begin_itr_ -
                begin_inc0_ * k,
                begin_inc0_ + begin_inc1_,
                std::min(size0_ + k, size1_)
            };
        }
    }

    /**
     * @brief Transpose.
     */
    constexpr dense_matrix_view transpose()
    {
        return {
            begin_itr_,
            begin_inc1_,
            begin_inc0_,
            size1_,
            size0_
        };
    }

    /**
     * @brief Block @f$ [i_0, i_1) \times [j_0, j_1) @f$.
     *
     * @throw std::invalid_argument
     * If invalid index.
     */
    constexpr dense_matrix_view block(
                    difference_type i0, difference_type j0,
                    difference_type i1, difference_type j1)
    {
        if (!(i0 >= 0 && i0 <= size0_ && j0 >= 0 && j0 <= size1_ &&
              i1 >= 0 && i1 <= size0_ && j1 >= 0 && j1 <= size1_)) {
            throw std::invalid_argument(__PRETTY_FUNCTION__);
        }

        if (!begin_itr_) {
            return dense_matrix_view();
        }
        else {
            return {
                begin_itr_ +
                begin_inc0_ * (i0 - (i0 > i1)) +
                begin_inc1_ * (j0 - (j0 > j1)),
                (i0 > i1) ? -begin_inc0_ : begin_inc0_,
                (j0 > j1) ? -begin_inc1_ : begin_inc1_,
                (i0 > i1) ? (i0 - i1) : (i1 - i0),
                (j0 > j1) ? (j0 - j1) : (j1 - j0)
            };
        }
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
    constexpr operator dense_matrix_view<U>() const
    {
        return {
            begin_itr_,
            begin_inc0_,
            begin_inc1_,
            size0_,
            size1_
        };
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
           std::basic_istream<C, Ctraits>& is, dense_matrix_view arr)
    {
        C ch;
        if (!(is >> ch) ||
            !Ctraits::eq(ch,
             Ctraits::to_char_type('['))) {
            is.setstate(std::ios_base::failbit);
            return is;
        }
        if (arr.size0() > 0) {
            is >> arr[0];
            for (difference_type i = 1; i < arr.size0(); i++) {
                if (!(is >> ch) ||
                    !Ctraits::eq(ch,
                     Ctraits::to_char_type(','))) {
                    is.setstate(std::ios_base::failbit);
                    return is;
                }
                is >> arr[i];
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
           std::basic_ostream<C, Ctraits>& os, dense_matrix_view arr)
    {
        os << '[';
        if (arr.size0() > 0) {
            os << arr[0];
            for (difference_type i = 1; i < arr.size0(); i++) {
                os << ',';
                os << arr[i];
            }
        }
        os << ']';
        return os;
    }

    /**@}*/
};

/**@}*/

} // namespace pr

#if !DOXYGEN
#include "dense_matrix_view.inl"
#endif // #if !DOXYGEN

#endif // #ifndef PREFORM_DENSE_MATRIX_VIEW_HPP
