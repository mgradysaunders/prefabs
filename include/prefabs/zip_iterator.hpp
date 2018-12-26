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
#error "prefabs/zip_iterator.hpp requires C++17"
#endif // #if !(__cplusplus >= 201703L)
#endif // #if !DOXYGEN
#pragma once
#ifndef PREFABS_ZIP_ITERATOR_HPP
#define PREFABS_ZIP_ITERATOR_HPP

// for std::iterator_traits, std::input_iterator_tag, ...
#include <iterator>

// for std::forward, std::integer_sequence, ...
#include <utility>

// for std::tuple
#include <tuple>

// for std::enable_if_t, std::common_type_t, std::is_base_of, ...
#include <type_traits>

namespace pr {

/**
 * @defgroup zip_iterator Zip iterator
 *
 * `<prefabs/zip_iterator.hpp>`
 *
 * __C++ version__: >=C++17
 */
/**@{*/

#if !DOXYGEN
template <
    typename T, 
    typename Tsequence = 
        std::make_integer_sequence<
        std::size_t, std::tuple_size<T>::value>
    >
class zip_iterator;
#endif // #if !DOXYGEN

/**
 * @brief Zip iterator.
 */
template <typename... Ts, std::size_t... N>
class zip_iterator<
        std::tuple<Ts...>,
        std::integer_sequence<std::size_t, N...>>
{
public:

    /**
     * @brief Value type.
     */
    typedef std::tuple<typename
            std::iterator_traits<Ts>::value_type...>
                value_type;

    /**
     * @brief Pointer.
     */
    typedef std::tuple<typename
            std::iterator_traits<Ts>::pointer...>
                pointer;

    /**
     * @brief Reference.
     */
    typedef std::tuple<typename
            std::iterator_traits<Ts>::reference...>
                reference;

    /**
     * @brief Difference type.
     */
    typedef std::common_type_t<typename
            std::iterator_traits<Ts>::difference_type...>
                difference_type;

    /**
     * @brief Iterator category.
     */
    typedef std::common_type_t<typename
            std::iterator_traits<Ts>::iterator_category...>
                iterator_category;

public:

    /**
     * @brief Pre-increment.
     */
    constexpr zip_iterator& operator++()
    {
        pos_ = {(++std::get<N>(pos_))...};
        return *this;
    }

    /**
     * @brief Pre-decrement.
     */
    template <
        bool B = 
            std::is_base_of<
            std::bidirectional_iterator_tag, iterator_category>::value
        >
    constexpr std::enable_if_t<B, zip_iterator&> operator--()
    {
        pos_ = {(--std::get<N>(pos_))...};
        return *this;
    }

    /**
     * @brief Post-increment.
     */
    constexpr zip_iterator operator++(int)
    {
        zip_iterator res = *this;
        pos_ = {(++std::get<N>(pos_))...};
        return res;
    }

    /**
     * @brief Post-decrement.
     */
    template <
        bool B = 
            std::is_base_of<
            std::bidirectional_iterator_tag, iterator_category>::value
        >
    constexpr std::enable_if_t<B, zip_iterator> operator--(int)
    {
        zip_iterator res = *this;
        pos_ = {(--std::get<N>(pos_))...};
        return res;
    }

    /**
     * @brief Dereference.
     */
    constexpr reference operator*()
    {
        return {(*std::get<N>(pos_))...};
    }

    /**
     * @brief Arbitrary access.
     */
    template <
        bool B =
            std::is_base_of<
            std::random_access_iterator_tag, iterator_category>::value
        >
    constexpr std::enable_if_t<B, reference> operator[](difference_type n)
    {
        return {(std::get<N>(pos_)[n])...};
    }


    /**
     * @brief Arbitrary increment.
     */
    template <
        bool B =
            std::is_base_of<
            std::random_access_iterator_tag, iterator_category>::value
        >
    constexpr std::enable_if_t<B, zip_iterator&> operator+=(difference_type n)
    {
        pos_ = {(std::get<N>(pos_) += n)...};
        return *this;
    }

    /**
     * @brief Arbitrary increment.
     */
    template <
        bool B =
            std::is_base_of<
            std::random_access_iterator_tag, iterator_category>::value
        >
    constexpr std::enable_if_t<B, zip_iterator> 
                        operator+(difference_type n) const
    {
        return {(std::get<N>(pos_) + n)...};
    }

    /**
     * @brief Arbitrary increment.
     */
    template <
        bool B =
            std::is_base_of<
            std::random_access_iterator_tag, iterator_category>::value
        >
    friend
    constexpr std::enable_if_t<B, zip_iterator> 
                        operator+(difference_type n, zip_iterator itr)
    {
        return itr + n;
    }

    /**
     * @brief Arbitrary decrement.
     */
    template <
        bool B =
            std::is_base_of<
            std::random_access_iterator_tag, iterator_category>::value
        >
    constexpr std::enable_if_t<B, zip_iterator&> operator-=(difference_type n)
    {
        pos_ = {(std::get<N>(pos_) -= n)...};
        return *this;
    }

    /**
     * @brief Arbitrary decrement.
     */
    template <
        bool B =
            std::is_base_of<
            std::random_access_iterator_tag, iterator_category>::value
        >
    constexpr std::enable_if_t<B, zip_iterator> 
                        operator-(difference_type n) const
    {
        return {(std::get<N>(pos_) - n)...};
    }

    /**
     * @brief Distance.
     */
    template <
        bool B =
            std::is_base_of<
            std::random_access_iterator_tag, iterator_category>::value
        >
    constexpr std::enable_if_t<B, difference_type>
                        operator-(zip_iterator other) const
    {
        return std::get<0>(pos_) - std::get<0>(other.pos_);
    }

public:

    /**
     * @brief Compare equal.
     */
    constexpr bool operator==(const zip_iterator& other) const
    {
        return pos_ == other.pos_;
    }

    /**
     * @brief Compare not equal.
     */
    constexpr bool operator!=(const zip_iterator& other) const
    {
        return pos_ != other.pos_;
    }

    /**
     * @brief Compare less.
     */
    template <
        bool B =
            std::is_base_of<
            std::random_access_iterator_tag, iterator_category>::value
        >
    constexpr std::enable_if_t<B, bool> operator<(zip_iterator other) const
    {
        return std::get<0>(pos_) < std::get<0>(other.pos_);
    }

    /**
     * @brief Compare greater.
     */
    template <
        bool B =
            std::is_base_of<
            std::random_access_iterator_tag, iterator_category>::value
        >
    constexpr std::enable_if_t<B, bool> operator>(zip_iterator other) const
    {
        return std::get<0>(pos_) > std::get<0>(other.pos_);
    }

    /**
     * @brief Compare less equal.
     */
    template <
        bool B =
            std::is_base_of<
            std::random_access_iterator_tag, iterator_category>::value
        >
    constexpr std::enable_if_t<B, bool> operator<=(zip_iterator other) const
    {
        return std::get<0>(pos_) <= std::get<0>(other.pos_);
    }

    /**
     * @brief Compare greater equal.
     */
    template <
        bool B =
            std::is_base_of<
            std::random_access_iterator_tag, iterator_category>::value
        >
    constexpr std::enable_if_t<B, bool> operator>=(zip_iterator other) const
    {
        return std::get<0>(pos_) >= std::get<0>(other.pos_);
    }

public:

    /**
     * @brief Iterator positions.
     */
    std::tuple<Ts...> pos_;
};

/**
 * @brief Zip iterators.
 */
template <
    typename... Ts
    >
constexpr zip_iterator<std::tuple<Ts...>> zip(Ts&&... ts)
{
    return {{std::forward<Ts>(ts)...}};
}

/**@}*/

} // namespace pr

#endif // #ifndef PREFABS_ZIP_ITERATOR_HPP
