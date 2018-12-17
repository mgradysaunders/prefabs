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
#pragma once
#ifndef PREFABS_MULTI_HPP
#define PREFABS_MULTI_HPP

// use c++17
#if !(__cplusplus >= 201703L) 
#error "prefabs/multi.hpp requires C++17"
#endif // #if !(__cplusplus >= 201703L) 

// for std::max_align_t
#include <cstddef>

// for std::basic_istream
#include <istream>

// for std::basic_ostream
#include <ostream>

// for std::forward
#include <utility>

// for std::reverse_iterator
#include <iterator>

// for std::alignment_of, std::true_type, ...
#include <type_traits>

namespace pr {

/**
 * @defgroup multi Multi-dimensional arrays
 *
 * `<prefabs/multi.hpp>`
 */
/**@{*/

#if !DOXYGEN

template <typename, std::size_t...> 
struct multi;

/**
 * @brief Multi value type helper.
 */
template <typename T, std::size_t... N>
struct multi_value_type
{
    using type = multi<T, N...>;
};

/**
 * @brief Multi value type helper, base case.
 */
template <typename T>
struct multi_value_type<T>
{
    using type = T;
};

/**
 * @brief Multi type trait.
 */
template <typename T>
struct is_multi : std::false_type
{
};

/**
 * @brief Multi type trait, truth case.
 */
template <typename T, std::size_t... N>
struct is_multi<multi<T, N...>> : std::true_type
{
};

#endif // #if !DOXYGEN

/**
 * @brief Multi-dimensional array.
 */
template <typename T, std::size_t M, std::size_t... N>
struct multi<T, M, N...>
{
public:

    static_assert(M > 0, "invalid dimension");

    /**
     * @name Container typedefs
     */
    /**@{*/

    /**
     * @brief Size type.
     */
    typedef std::size_t size_type;

    /**
     * @brief Difference type.
     */
    typedef std::ptrdiff_t difference_type;

    /**
     * @brief Value type.
     */
    typedef typename multi_value_type<T, N...>::type value_type;

    /**
     * @brief Reference.
     */
    typedef value_type& reference;

    /**
     * @brief Pointer.
     */
    typedef value_type* pointer;

    /**
     * @brief Const reference.
     */
    typedef const value_type& const_reference;

    /**
     * @brief Const pointer.
     */
    typedef const value_type* const_pointer;

    /**
     * @brief Forward iterator.
     */
    typedef pointer iterator;

    /**
     * @brief Forward const iterator.
     */
    typedef const_pointer const_iterator;

    /**
     * @brief Reverse iterator.
     */
    typedef std::reverse_iterator<iterator> reverse_iterator;

    /**
     * @brief Reverse const iterator.
     */
    typedef std::reverse_iterator<const_iterator> const_reverse_iterator;

    /**
     * @brief Bottom-level entry type.
     */
    typedef T entry_type;

    /**
     * @brief Rebind entry type.
     */
    template <typename U>
    struct rebind 
    {
        using type = multi<U, M, N...>;
    };

    /**@}*/

public:

    /**
     * @brief Array alignment, to promote vectorization by compiler.
     */
    static constexpr std::size_t alignment() noexcept
    {
        std::size_t align = sizeof(value_type[M]);
        if (align & (align - 1)) {
            return std::alignment_of<value_type>::value;
        }
        if (align > std::alignment_of<std::max_align_t>::value) {
            align = std::alignment_of<std::max_align_t>::value;
        }
        return align;
    }

    /**
     * @brief Array.
     *
     * This should be treated as a private member variable. It is public
     * only to ensure the implicit array constructor is available.
     */
    alignas(alignment()) value_type v_[M] = {};

public:


    /**
     * @name Container interface
     */
    /**@{*/

    /**
     * @brief Size.
     */
    static constexpr size_type size() noexcept
    {
        return M;
    }

    /**
     * @brief Total size.
     */
    static constexpr size_type total_size() noexcept
    {
        return (M * ... * N);
    }

    /**
     * @brief Forward begin iterator.
     */
    constexpr iterator begin() noexcept
    {
        return &v_[0];
    }

    /**
     * @brief Forward begin iterator, const variant.
     */
    constexpr const_iterator begin() const noexcept
    {
        return &v_[0];
    }

    /**
     * @brief Forward begin iterator, force const variant.
     */
    constexpr const_iterator cbegin() const noexcept
    {
        return begin(); // Force const
    }

    /**
     * @brief Forward end iterator.
     */
    constexpr iterator end() noexcept
    {
        return begin() + size();
    }

    /**
     * @brief Forward end iterator, const variant.
     */
    constexpr const_iterator end() const noexcept
    {
        return begin() + size();
    }

    /**
     * @brief Forward end iterator, force const variant.
     */
    constexpr const_iterator cend() const noexcept
    {
        return end(); // Force const
    }

    /**
     * @brief Reverse begin iterator.
     */
    constexpr reverse_iterator rbegin() noexcept
    {
        return reverse_iterator(end());
    }

    /**
     * @brief Reverse begin iterator, const variant.
     */
    constexpr const_reverse_iterator rbegin() const noexcept
    {
        return const_reverse_iterator(end());
    }

    /**
     * @brief Reverse begin iterator, force const variant.
     */
    constexpr const_reverse_iterator crbegin() const noexcept
    {
        return rbegin(); // Force const
    }

    /**
     * @brief Reverse end iterator.
     */
    constexpr reverse_iterator rend() noexcept
    {
        return reverse_iterator(begin());
    }

    /**
     * @brief Reverse end iterator, const variant.
     */
    constexpr const_reverse_iterator rend() const noexcept
    {
        return const_reverse_iterator(begin());
    }

    /**
     * @brief Reverse end iterator, force const variant.
     */
    constexpr const_reverse_iterator crend() const noexcept
    {
        return rend(); // Force const
    }

    /**
     * @brief Front element.
     */
    constexpr reference front() noexcept
    {
        return *begin();
    }

    /**
     * @brief Front element, const variant.
     */
    constexpr const_reference front() const noexcept
    {
        return *begin();
    }

    /**
     * @brief Back element.
     */
    constexpr reference back() noexcept
    {
        return *rbegin();
    }

    /**
     * @brief Back element, const variant.
     */
    constexpr const_reference back() const noexcept
    {
        return *rbegin();
    }

    /**@}*/

public:

    /**
     * @name Access operators
     */
    /**@{*/

    /**
     * @brief Index accessor.
     */
    template <typename P>
    constexpr decltype(auto) operator[](P p) noexcept
    {
        return (v_[p]);
    }

    /**
     * @brief Index accessor, const variant.
     */
    template <typename P>
    constexpr decltype(auto) operator[](P p) const noexcept
    {
        return (v_[p]);
    }

    /**
     * @brief Multi-index accessor.
     */
    template <typename P, typename... Q>
    constexpr decltype(auto) operator()(P p, Q&&... q) noexcept
    {
        if constexpr (sizeof...(Q) == 0) {
            return operator[](p);
        }
        else {
            return operator[](p).operator()(std::forward<Q>(q)...);
        }
    }

    /**
     * @brief Multi-index accessor, const variant.
     */
    template <typename P, typename... Q>
    constexpr decltype(auto) operator()(P p, Q&&... q) const noexcept
    {
        if constexpr (sizeof...(Q) == 0) {
            return operator[](p);
        }
        else {
            return operator[](p).operator()(std::forward<Q>(q)...);
        }
    }

    /**@}*/

public:

    /**
     * @name Increment operators
     */
    /**@{*/

    /**
     * @brief Entrywise pre-increment.
     */
    constexpr multi& operator++() 
    {
        for (value_type& val : *this) { ++val; } return *this;
    }

    /**
     * @brief Entrywise pre-decrement.
     */
    constexpr multi& operator--() 
    {
        for (value_type& val : *this) { --val; } return *this;
    }

    /**
     * @brief Entrywise post-increment.
     */
    constexpr multi operator++(int) 
    {
        multi tmp = *this; operator++(); return tmp;
    }

    /**
     * @brief Entrywise post-decrement.
     */
    constexpr multi operator--(int) 
    {
        multi tmp = *this; operator--(); return tmp;
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
    constexpr value_type foldl(F&& f) const
    {
        auto itr = begin();
        auto val = value_type(*itr++);
        while (itr < end()) {
            val = std::forward<F>(f)(val, *itr++);
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
    constexpr value_type foldr(F&& f) const
    {
        auto itr = rbegin();
        auto val = value_type(*itr++);
        while (itr < rend()) {
            val = std::forward<F>(f)(val, *itr++);
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
    constexpr multi& foldl_accum(F&& f)
    {
        auto itr1 = begin();
        auto itr2 = itr1 + 1;
        while (itr2 < end()) {
            *itr2 = std::forward<F>(f)(*itr2, *itr1);
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
    constexpr multi& foldr_accum(F&& f)
    {
        auto itr1 = rbegin();
        auto itr2 = itr1 + 1;
        while (itr2 < rend()) {
            *itr2 = std::forward<F>(f)(*itr2, *itr1);
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
    constexpr multi& foldl_adjacent(F&& f)
    {
        auto itr1 = begin();
        auto itr2 = itr1 + 1;
        while (itr2 < end()) {
            *itr1 = std::forward<F>(f)(*itr1, *itr2);
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
    constexpr multi& foldr_adjacent(F&& f)
    {
        auto itr1 = rbegin();
        auto itr2 = itr1 + 1;
        while (itr2 < rend()) {
            *itr1 = std::forward<F>(f)(*itr1, *itr2);
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
    constexpr value_type sum() const
    {
        return foldl(std::plus<value_type>());
    }

    /**
     * @brief Product.
     *
     * @note
     * Equivalent to `foldl(std::multiplies<value_type>())`.
     */
    constexpr value_type prod() const
    {
        return foldl(std::multiplies<value_type>());
    }

    /**
     * @brief Accumulating sum.
     *
     * @note
     * Equivalent to `foldl_accum(std::plus<value_type>())`.
     */
    constexpr multi& accum_sum()
    {
        return foldl_accum(std::plus<value_type>());
    }

    /**
     * @brief Adjacent difference.
     *
     * @note
     * Equivalent to `foldr_adjacent(std::minus<value_type>())`.
     */
    constexpr multi& adjacent_difference()
    {
        return foldr_adjacent(std::minus<value_type>());
    }

    /**@}*/

public:

    /**
     * @name Reshaping
     */
    /**@{*/

    /**
     * @brief Fill from entry.
     */
    constexpr multi& fill(const entry_type& ent)
    {
        for (value_type& val : *this) {
            if constexpr (std::is_same<
                            entry_type, 
                            value_type>::value) {
                val = ent;
            }
            else {
                val.fill(ent);
            }
        }
        return *this;
    }

    /**
     * @brief Fill from entry array.
     */
    constexpr multi& fill(const entry_type* pos)
    {
        for (value_type& val : *this) {
            if constexpr (std::is_same<
                            entry_type, 
                            value_type>::value) {
                val = *pos++;
            }
            else {
                val.fill(pos);
                pos += value_type::total_size();
            }
        }
        return *this;
    }

    /**
     * @brief Entry initializer.
     */
    static constexpr multi value(const entry_type& val)
    {
        return multi().fill(val);
    }

    /**
     * @brief Entry array initializer.
     */
    static constexpr multi array(const entry_type* pos)
    {
        return multi().fill(pos);
    }

    /**
     * @brief Flatten.
     */
    constexpr multi<T, total_size()> flatten() const
    {
        if constexpr (
                    sizeof(multi<T, M, N...>) == sizeof(T) * total_size() &&
                    sizeof(multi<T, M, N...>) ==
                    sizeof(multi<T, total_size()>)) {
            // Memory layouts must be identical.
            return reinterpret_cast<const multi<T, total_size()>&>(*this);
        }
        else {
            multi<T, total_size()> res;
            do_flatten(&res[0]);
            return res;
        }
    }

#if !DOXYGEN

    // Helper for flatten.
    constexpr void do_flatten(entry_type* pos) const
    {
        for (const value_type& val : *this) {
            if constexpr (std::is_same<
                            entry_type, 
                            value_type>::value) {
                *pos++ = val;
            }
            else {
                val.do_flatten(pos);
                pos += value_type::total_size();
            }
        }
    }

#endif // #if !DOXYGEN

    /**
     * @brief Reshape.
     */
    template <std::size_t... K>
    constexpr multi<T, K...> reshape() const
    {
        if constexpr (
                    sizeof(multi<T, M, N...>) == sizeof(T) * total_size() &&
                    sizeof(multi<T, M, N...>) ==
                    sizeof(multi<T, K...>)) {
            // Memory layouts must be identical.
            return reinterpret_cast<const multi<T, K...>&>(*this);
        }
        else {
            multi<T, K...> res;
            multi<T, total_size()> tmp = flatten();
            res.fill(&tmp[0]);
            return res;

            // Sanity check.
            static_assert(total_size() ==
                            multi<T, K...>::total_size());
        }
    }

    /**
     * @brief Cast operator.
     */
    template <typename U, std::size_t... K>
    constexpr operator multi<U, K...>() const
    {
        multi<U, K...> res;
        auto itr1 = begin();
        auto itr2 = res.begin();
        while (itr1 < end() &&
               itr2 < res.end()) {
            *itr2 = typename multi<U, K...>::value_type(*itr1);
            ++itr1;
            ++itr2;
        }
        return res;
    }

    /**@}*/

public:

    // TODO any

    // TODO all

    // TODO operator==

    // TODO operator!=

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
    template <typename Char, typename Traits>
    friend
    inline std::basic_istream<Char, Traits>& operator>>(
           std::basic_istream<Char, Traits>& is, multi& arr)
    {
        Char ch;
        if (!(is >> ch) ||
            !Traits::eq(ch,
             Traits::to_char_type('['))) {
            is.setstate(std::ios_base::failbit);
            return is;
        }
        auto itr = arr.begin();
        is >> *itr++;
        while (itr < arr.end()) {
            if (!(is >> ch) ||
                !Traits::eq(ch,
                 Traits::to_char_type(','))) {
                is.setstate(std::ios_base::failbit);
                return is;
            }
            is >> *itr++;
        }
        if (!(is >> ch) ||
            !Traits::eq(ch,
             Traits::to_char_type(']'))) {
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
    template <typename Char, typename Traits>
    friend
    inline std::basic_ostream<Char, Traits>& operator<<(
           std::basic_ostream<Char, Traits>& os, const multi& arr)
    {
        auto itr = arr.begin();
        os << '[';
        os << *itr++;
        while (itr < arr.end()) {
            os << ',';
            os << *itr++;
        }
        os << ']'; 
        return os;
    }

    /**@}*/
};

/**@}*/

#include "multi.inl"

} // namespace pr

#endif // #ifndef PREFABS_MULTI_HPP
