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
#if !(__cplusplus >= 201703L)
#error "preform/multi.hpp requires >=C++17"
#endif // #if !(__cplusplus >= 201703L)
#endif // #if !DOXYGEN
#pragma once
#ifndef PREFORM_MULTI_HPP
#define PREFORM_MULTI_HPP

// for std::max_element, std::min_element
#include <algorithm>

// for std::max_align_t
#include <cstddef>

// for std::basic_istream
#include <istream>

// for std::basic_ostream
#include <ostream>

// for std::initializer_list
#include <initializer_list>

// for std::forward
#include <utility>

// for std::reverse_iterator
#include <iterator>

// for std::alignment_of, std::true_type, ...
#include <type_traits>

namespace pr {

/**
 * @defgroup multi Multi-dimensional array
 *
 * `<preform/multi.hpp>`
 *
 * __C++ version__: >=C++17
 */
/**@{*/

#if !DOXYGEN

template <typename, std::size_t...> 
struct multi;

template <typename T, std::size_t... N>
struct multi_value_type
{
    using type = multi<T, N...>;
};

template <typename T>
struct multi_value_type<T>
{
    using type = T;
};

template <typename T, std::size_t... N>
struct multi_initializer_list;

template <typename T, std::size_t M, std::size_t... N>
struct multi_initializer_list<T, M, N...>
{
    using type = 
        std::initializer_list<
        typename multi_initializer_list<T, N...>::type>;
};

template <typename T>
struct multi_initializer_list<T>
{
    using type = T;
};

template <typename T>
struct is_multi : std::false_type
{
};

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

    static_assert(M > 0, "M must be positive");

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
     * @brief Iterator.
     */
    typedef pointer iterator;

    /**
     * @brief Const iterator.
     */
    typedef const_pointer const_iterator;

    /**
     * @brief Reverse iterator.
     */
    typedef std::reverse_iterator<iterator> reverse_iterator;

    /**
     * @brief Const reverse iterator.
     */
    typedef std::reverse_iterator<const_iterator> const_reverse_iterator;

    /**
     * @brief Bottom-level entry type.
     */
    typedef T entry_type;

    /**
     * @brief Flatten type.
     */
    typedef multi<T, (M * ... * N)> flatten_type;

    /**
     * @brief Rebind entry type.
     */
    template <typename U>
    struct rebind 
    {
        typedef multi<U, M, N...> other;
    };

    /**
     * @brief Initializer list.
     */
    typedef typename multi_initializer_list<T, M, N...>::type initializer_list;

    /**@}*/

private:

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
     */
    alignas(alignment()) value_type v_[M] = {};

public:

    /**
     * @brief Default constructor.
     */
    constexpr multi() = default;

    /**
     * @brief Constructor.
     */
    __attribute__((always_inline))
    constexpr multi(initializer_list list)
    {
        auto itr = begin();
        auto itrlist = list.begin();
        for (; itr < end() &&
               itrlist < list.end(); 
               ++itr, 
               ++itrlist) {
            *itr = *itrlist;
        }
    }

    /**
     * @brief Constructor.
     */
    __attribute__((always_inline))
    constexpr explicit multi(const entry_type& ent)
    {
        fill(ent);
    }

    /**
     * @brief Constructor.
     */
    __attribute__((always_inline))
    constexpr explicit multi(const entry_type* ptr) __attribute__((nonnull))
    {
        fill(ptr);
    }

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
     * @brief Begin iterator.
     */
    constexpr iterator begin() noexcept
    {
        return &v_[0];
    }

    /**
     * @brief Begin iterator, const variant.
     */
    constexpr const_iterator begin() const noexcept
    {
        return &v_[0];
    }

    /**
     * @brief Begin iterator, force const variant.
     */
    constexpr const_iterator cbegin() const noexcept
    {
        return begin(); // Force const
    }

    /**
     * @brief End iterator.
     */
    constexpr iterator end() noexcept
    {
        return begin() + size();
    }

    /**
     * @brief End iterator, const variant.
     */
    constexpr const_iterator end() const noexcept
    {
        return begin() + size();
    }

    /**
     * @brief End iterator, force const variant.
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
    __attribute__((always_inline))
    constexpr decltype(auto) operator[](P p) noexcept
    {
        return (v_[p]);
    }

    /**
     * @brief Index accessor, const variant.
     */
    template <typename P>
    __attribute__((always_inline))
    constexpr decltype(auto) operator[](P p) const noexcept
    {
        return (v_[p]);
    }

    /**
     * @brief Multiple-index accessor.
     */
    template <typename P, typename... Q>
    __attribute__((always_inline))
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
     * @brief Multiple-index accessor, const variant.
     */
    template <typename P, typename... Q>
    __attribute__((always_inline))
    constexpr decltype(auto) operator()(P p, Q&&... q) const noexcept
    {
        if constexpr (sizeof...(Q) == 0) {
            return operator[](p);
        }
        else {
            return operator[](p).operator()(std::forward<Q>(q)...);
        }
    }

    /**
     * @brief Multi-index accessor.
     */
    template <typename P, std::size_t K>
    __attribute__((always_inline))
    constexpr decltype(auto) operator[](const multi<P, K>& p) noexcept
    {
        if constexpr (K == 1) {
            return operator[](*p.begin());
        }
        else {
            multi<P, K - 1> q;
            std::copy(
                p.begin() + 1, p.end(),
                q.begin());
            return operator[](*p.begin()).operator[](q);
        }
    }

    /**
     * @brief Multi-index accessor, const variant.
     */
    template <typename P, std::size_t K>
    __attribute__((always_inline))
    constexpr decltype(auto) operator[](const multi<P, K>& p) const noexcept
    {
        if constexpr (K == 1) {
            return operator[](*p.begin());
        }
        else {
            multi<P, K - 1> q;
            std::copy(
                p.begin() + 1, p.end(),
                q.begin());
            return operator[](*p.begin()).operator[](q);
        }
    }

    /**
     * @brief Next index.
     */
    template <typename P>
    static constexpr void next_index(multi<P, 1 + sizeof...(N)>& pos)
    {
        multi<P, 1 + sizeof...(N)> dim = {{P(M), P(N)...}};
        auto itrdim = dim.rbegin();
        auto itrpos = pos.rbegin();
        for (; itrpos < pos.rend(); ++itrdim, ++itrpos) {
            (*itrpos)++;
            if (*itrpos >= *itrdim) {
                *itrpos = 0;
            }
            else {
                return;
            }
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
    constexpr value_type foldl(F&& func) const
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
    constexpr value_type foldr(F&& func) const
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
    constexpr multi& foldl_accum(F&& func)
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
    constexpr multi& foldr_accum(F&& func)
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
    constexpr multi& foldl_adjacent(F&& func)
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
    constexpr multi& foldr_adjacent(F&& func)
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
    __attribute__((always_inline))
    constexpr multi& fill(const T& ent)
    {
        for (value_type& val : *this) {
            if constexpr (std::is_same<T, value_type>::value) {
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
    __attribute__((always_inline))
    constexpr multi& fill(const T* ptr) __attribute__((nonnull))
    {
        for (value_type& val : *this) {
            if constexpr (std::is_same<T, value_type>::value) {
                val = *ptr++;
            }
            else {
                val.fill(ptr);
                ptr += value_type::total_size();
            }
        }
        return *this;
    }

    /**
     * @brief Flatten.
     */
    constexpr flatten_type flatten() const
    {
        flatten_type res; 
        flatten_into(&res[0]); 
        return res;
    }

#if !DOXYGEN

    // Flatten into array.
    constexpr void flatten_into(T* ptr) const
    {
        for (const value_type& val : *this) {
            if constexpr (std::is_same<T, value_type>::value) {
                *ptr++ = val;
            }
            else {
                val.flatten_into(ptr);
                ptr += value_type::total_size();
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
        if constexpr (sizeof...(K) == 1) {
            return flatten();
        }
        else {
            multi<T, K...> res;
            multi<T, total_size()> tmp = flatten();
            res.fill(&tmp[0]);
            return res;
        }

        // Sanity check.
        static_assert(total_size() ==
                        multi<T, K...>::total_size());
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

    /**
     * @brief Swizzle.
     */
    template <typename P, typename... Q>
    __attribute__((always_inline))
    constexpr multi<T, 1 + sizeof...(Q), N...> swizzle(P p, Q&&... q) const
    {
        return {{
            operator[](p),
            operator[](std::forward<Q>(q))...
        }};
    }

    template <typename P, std::size_t K>
    constexpr multi<T, K, N...> swizzle(const multi<P, K>& k)  const
    {
        multi<T, K, N...> res;
        auto itrres = res.begin();
        auto itrk = k.begin();
        for (; itrk < k.end(); ++itrres, ++itrk) {
            *itrres = operator[](*itrk);
        }
        return res;
    }

    /**@}*/

public:

    /**
     * @name Tests
     */
    /**@{*/

    /**
     * @brief Any entries true?
     */
    constexpr bool any() const
    {
        for (const value_type& val : *this) {
            if constexpr (std::is_same<T, value_type>::value) {
                if (val) {
                    return true;
                }
            }
            else {
                if (val.any()) {
                    return true;
                }
            }
        }
        return false;
    }

    /**
     * @brief All entries true?
     */
    constexpr bool all() const
    {
        for (const value_type& val : *this) {
            if constexpr (std::is_same<T, value_type>::value) {
                if (!val) {
                    return false;
                }
            }
            else {
                if (!val.all()) {
                    return false;
                }
            }
        }
        return true;
    }

    /**
     * @brief Index of minimum element.
     */
    constexpr size_type argmin() const
    {
        static_assert(sizeof...(N) == 0, "argmin() for vectors only");
        return std::min_element(begin(), end()) - begin();
    }

    /**
     * @brief Index of maximum element.
     */
    constexpr size_type argmax() const
    {
        static_assert(sizeof...(N) == 0, "argmax() for vectors only");
        return std::max_element(begin(), end()) - begin();
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
           std::basic_istream<C, Ctraits>& is, multi& arr)
    {
        C ch;
        if (!(is >> ch) ||
            !Ctraits::eq(ch,
             Ctraits::to_char_type('['))) {
            is.setstate(std::ios_base::failbit);
            return is;
        }
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
           std::basic_ostream<C, Ctraits>& os, const multi& arr)
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

/**
 * @name Bool array conversion
 */
/**@{*/

/**
 * @brief Bool array to binary number.
 */
template <std::size_t N>
constexpr std::size_t multitoindex(const multi<bool, N>& arr)
{
    std::size_t num = 0;
    std::size_t bit = 0;
    for (auto itrarr = arr.begin(); 
              itrarr < arr.end(); ++itrarr, ++bit) {
        num |= (std::size_t(*itrarr) << bit);
    }
    return num;
}

/**@}*/

/**@}*/

} // namespace pr

#if !DOXYGEN
#include "multi.inl"
#endif // #if !DOXYGEN

#endif // #ifndef PREFORM_MULTI_HPP
