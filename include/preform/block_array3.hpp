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
#error "preform/block_array3.hpp requires >=C++17"
#endif // #if !(__cplusplus >= 201703L)
#endif // #if !DOXYGEN
#pragma once
#ifndef PREFORM_BLOCK_ARRAY3_HPP
#define PREFORM_BLOCK_ARRAY3_HPP

// for std::vector
#include <vector>

// for pr::multi
#include <preform/multi.hpp>

// for pr::first1, pr::roundpow2, ...
#include <preform/misc_int.hpp>

#if !DOXYGEN
#ifndef L1_LINE
#define L1_LINE 64
#endif // #ifndef L1_LINE
#endif // #if !DOXYGEN

namespace pr {

/**
 * @defgroup block_array3 Block array (3-dimensional)
 *
 * `<preform/block_array3.hpp>`
 *
 * __C++ version__: >= C++17
 */
/**@{*/

/**
 * @brief Block array (3-dimensional).
 *
 * Block array with block size @f$ B = 2^k @f$ for an integer
 * @f$ k > 0 @f$, which is chosen to promote cache performance. Specifically,
 * @f$ k @f$ is chosen such that each @f$ B^2 @f$ block fits on an L1 cache
 * line which, by default, the implementation assumes to be 64 bytes. To
 * change this assumption, define `L1_LINE` to the appropriate
 * size before including the data structure.
 *
 * @note The implementation blocks _only_ the 0th and 1st dimensions. As a
 * result, the addresses of adjacent locations in the 2nd dimension are usually
 * far apart.
 */
template <
    typename T,
    typename Talloc = std::allocator<T>
    >
class block_array3
{
public:

    /**
     * @name Container typedefs
     */
    /**@{*/

    /**
     * @brief Internal container.
     */
    typedef std::vector<T, Talloc> container;

    /**
     * @brief Size type.
     */
    typedef typename container::size_type size_type;

    /**
     * @brief Difference type.
     */
    typedef typename container::difference_type difference_type;

    /**
     * @brief Value type.
     */
    typedef typename container::value_type value_type;

    /**
     * @brief Reference.
     */
    typedef typename container::reference reference;

    /**
     * @brief Pointer.
     */
    typedef typename container::pointer pointer;

    /**
     * @brief Const reference.
     */
    typedef typename container::const_reference const_reference;

    /**
     * @brief Const pointer.
     */
    typedef typename container::const_pointer const_pointer;

    /**
     * @brief Forward iterator.
     */
    typedef typename container::iterator iterator;

    /**
     * @brief Forward const iterator.
     */
    typedef typename container::const_iterator const_iterator;

    /**
     * @brief Reverse iterator.
     */
    typedef typename container::reverse_iterator reverse_iterator;

    /**
     * @brief Reverse const iterator.
     */
    typedef typename container::const_reverse_iterator const_reverse_iterator;

    /**@}*/

public:

    /**
     * @brief Binary log of the block size @f$ \log_{2}{B} @f$.
     */
    static constexpr
    size_type block_size_log2 = first1(roundpow2(L1_LINE / sizeof(T))) / 2;

    /**
     * @brief Block size @f$ B @f$.
     */
    static constexpr
    size_type block_size = 1 << block_size_log2;


    /**
     * @brief Binary log of the block area @f$ \log_{2}{B^{2}} @f$.
     */
    static constexpr
    size_type block_area_log2 = block_size_log2 * 2;

    /**
     * @brief Block area @f$ B^2 @f$.
     */
    static constexpr
    size_type block_area = 1 << block_area_log2;

    /**
     * @brief Round `num` up to multiple of `block_size`.
     */
    static constexpr
    size_type round_size(size_type num) noexcept
    {
        return (num + (block_size - 1)) & ~(block_size - 1);
    }

    /**
     * @brief Round `num` up to multiple of `block_size`.
     */
    static constexpr
    multi<size_type, 3> round_size(multi<size_type, 3> num) noexcept
    {
        return {
            round_size(num[0]),
            round_size(num[1]),
            num[2]
        };
    }

public:

    /**
     * @name Constructors
     */
    /**@{*/

    /**
     * @brief Default constructor.
     */
    block_array3() noexcept(noexcept(Talloc())) = default;

    /**
     * @brief Default constructor with allocator.
     */
    explicit
    block_array3(const Talloc& alloc) noexcept : data_(alloc)
    {
    }

    /**
     * @brief Copy constructor.
     */
    block_array3(const block_array3& other) :
        user_size_(other.user_size_),
        data_size_(other.data_size_),
        data_(other.data_)
    {
    }

    /**
     * @brief Copy constructor with allocator.
     */
    block_array3(const block_array3& other, const Talloc& alloc) :
        user_size_(other.user_size_),
        data_size_(other.data_size_),
        data_(other.data_, alloc)
    {
    }

    /**
     * @brief Move constructor.
     */
    block_array3(block_array3&& other) noexcept :
        user_size_(std::move(other.user_size_)),
        data_size_(std::move(other.data_size_)),
        data_(std::move(other.data_))
    {
    }

    /**
     * @brief Move constructor with allocator.
     */
    block_array3(block_array3&& other, const Talloc& alloc) :
        user_size_(std::move(other.user_size_)),
        data_size_(std::move(other.data_size_)),
        data_(std::move(other.data_), alloc)
    {
    }

    /**
     * @brief Construct from `count` copies of `value`.
     */
    block_array3(
            multi<size_type, 3> count,
            const T& value = T(), const Talloc& alloc = Talloc()) :
          user_size_{count},
          data_size_{round_size(count)},
          data_(data_size_.prod(), value, alloc)
    {
    }

    /**@}*/

public:

    /**
     * @name Assignment operators
     */
    /**@{*/

    /**
     * @brief Default copy assignment operator.
     */
    block_array3& operator=(const block_array3&) = default;

    /**
     * @brief Default move assignment operator.
     */
    block_array3& operator=(block_array3&&) = default;

    /**@}*/

public:

    /**
     * @name Container interface
     */
    /**@{*/

    /**
     * @brief Resize array.
     */
    void resize(multi<size_type, 3> count)
    {
        user_size_ = count;
        data_size_ = round_size(count);
        data_.resize(data_size_.prod());
    }

    /**
     * @brief Clear contents.
     */
    void clear()
    {
        data_.clear();
        data_size_ = {0, 0, 0};
        user_size_ = {0, 0, 0};
    }

    /**
     * @brief Shrink allocation to fit contents.
     */
    void shrink_to_fit()
    {
        data_.shrink_to_fit();
    }

    /**
     * @brief Size.
     */
    size_type size() noexcept
    {
        return data_.size();
    }

    /**
     * @brief User size.
     */
    const multi<size_type, 3>& user_size() const noexcept
    {
        return user_size_;
    }

    /**
     * @brief User size after rounding.
     */
    const multi<size_type, 3>& data_size() const noexcept
    {
        return data_size_;
    }

    /**
     * @brief Empty?
     */
    bool empty() const noexcept
    {
        return data_.empty();
    }

    /**
     * @brief Forward begin iterator.
     */
    iterator begin() noexcept
    {
        return data_.begin();
    }

    /**
     * @brief Forward begin iterator, const variant.
     */
    const_iterator begin() const noexcept
    {
        return data_.begin();
    }

    /**
     * @brief Forward begin iterator, force const variant.
     */
    const_iterator cbegin() const noexcept
    {
        return data_.cbegin();
    }

    /**
     * @brief Forward end iterator.
     */
    iterator end() noexcept
    {
        return data_.end();
    }

    /**
     * @brief Forward end iterator, const variant.
     */
    const_iterator end() const noexcept
    {
        return data_.end();
    }

    /**
     * @brief Forward end iterator, force const variant.
     */
    const_iterator cend() const noexcept
    {
        return data_.cend();
    }

    /**
     * @brief Reverse begin iterator.
     */
    reverse_iterator rbegin() noexcept
    {
        return data_.rbegin();
    }

    /**
     * @brief Reverse begin iterator, const variant.
     */
    const_reverse_iterator rbegin() const noexcept
    {
        return data_.rbegin();
    }

    /**
     * @brief Reverse begin iterator, force const variant.
     */
    const_reverse_iterator crbegin() const noexcept
    {
        return data_.crbegin();
    }

    /**
     * @brief Reverse end iterator.
     */
    reverse_iterator rend() noexcept
    {
        return data_.rend();
    }

    /**
     * @brief Reverse end iterator, const variant.
     */
    const_reverse_iterator rend() const noexcept
    {
        return data_.rend();
    }

    /**
     * @brief Reverse end iterator, force const variant.
     */
    const_reverse_iterator crend() const noexcept
    {
        return data_.crend();
    }

    /**
     * @brief Front element.
     */
    reference front() noexcept
    {
        return data_.front();
    }

    /**
     * @brief Front element, const variant.
     */
    const_reference front() const noexcept
    {
        return data_.front();
    }

    /**
     * @brief Back element.
     */
    reference back() noexcept
    {
        return data_.back();
    }

    /**
     * @brief Back element, const variant.
     */
    const_reference back() const noexcept
    {
        return data_.back();
    }

    /**@}*/

public:

    /**
     * @name Indexing
     */
    /**@{*/

    /**
     * @brief 3-dimensional index to 1-dimensional index.
     *
     * 3-dimensional index @f$ (l_0, l_1, l_2) @f$ to
     * 1-dimensional index @f$ j @f$. In particular,
     * @f[
     *      j = B^2 (q_0 + (n_0 / B) q_1) + (r_0 + B r_1) + n_0 n_1 l_2
     * @f]
     * where the @f$ q_k, r_k @f$ denote the quotients and
     * remainders of the @f$ l_k / B @f$ respectively.
     */
    size_type convert(multi<size_type, 3> loc) const noexcept
    {
        // Quotient and remainder with respect to block size.
        multi<size_type, 2> loc_quo = loc >> block_size_log2;
        multi<size_type, 2> loc_rem = loc & (block_size - 1);

        // Multipliers.
        loc_quo[0] <<= block_area_log2;
        loc_quo[1] <<= block_size_log2;
        loc_rem[1] <<= block_size_log2;
        return loc_quo[0] + loc_quo[1] * data_size_[0] +
               loc_rem[0] + loc_rem[1] +
                   // Sheet offset.
                   loc[2] *
                   data_size_[0] *
                   data_size_[1];
    }

    /**
     * @brief 1-dimensional index to 3-dimensional index.
     *
     * 1-dimensional index @f$ j @f$ to
     * 3-dimensional index @f$ (l_0, l_1, l_2) @f$. In particular,
     * @f[
     * \begin{aligned}
     *      l_0 &= B q_0 + r_0
     * \\   l_1 &= B q_1 + r_1
     * \\   l_2 &= s
     * \end{aligned}
     * @f]
     * where
     * - @f$ q_0 @f$ and @f$ q_1 @f$ denote the _remainder and quotient_ of
     *   @f$ q / (n_0 / B) @f$ respectively, and
     * - @f$ r_0 @f$ and @f$ r_1 @f$ denote the _remainder and quotient_ of
     *   @f$ r / B @f$ respectively,
     *
     * where, in turn,
     * - @f$ q @f$ and @f$ r @f$ denote the _quotient and remainder_ of
     *   @f$ t / B^2 @f$ respectively, and
     * - @f$ s @f$ and @f$ t @f$ denote the _quotient and remainder_ of
     *   @f$ j / (n_0 n_1) @f$ respectively.
     */
    multi<size_type, 3> convert(size_type pos) const noexcept
    {
        // Quotient and remainder with respect to sheet area.
        size_type pos_quo = pos / (data_size_[0] * data_size_[1]);
        size_type pos_rem = pos % (data_size_[0] * data_size_[1]);

        // Quotient and remainder with respect to block area.
        size_type tmp_quo = pos_rem >> block_area_log2;
        size_type tmp_rem = pos_rem & (block_area - 1);

        // Number of blocks in dimension 0.
        size_type num = data_size_[0] >> block_size_log2;

        // Recover index.
        return {
            (tmp_quo % num) << block_size_log2 | (tmp_rem & (block_size - 1)),
            (tmp_quo / num) << block_size_log2 | (tmp_rem >> block_size_log2),
             pos_quo
        };
    }

    /**@}*/

public:

    /**
     * @name Access operators
     */
    /**@{*/

    /**
     * @brief 1-dimensional accessor.
     */
    reference operator[](size_type pos)
    {
        return data_[pos];
    }

    /**
     * @brief 1-dimensional accessor, const variant.
     */
    const_reference operator[](size_type pos) const
    {
        return data_[pos];
    }

    /**
     * @brief 1-dimensional accessor with range check.
     *
     * @throw std::out_of_range
     * If `pos` is out of range.
     */
    reference at(size_type pos)
    {
        return data_.at(pos);
    }

    /**
     * @brief 1-dimensional accessor with range check, const variant.
     *
     * @throw std::out_of_range
     * If `pos` is out of range.
     */
    const_reference at(size_type pos) const
    {
        return data_.at(pos);
    }

    /**
     * @brief 3-dimensional accessor.
     */
    reference operator[](multi<size_type, 3> loc)
    {
        return data_[convert(loc)];
    }

    /**
     * @brief 3-dimensional accessor, const variant.
     */
    const_reference operator[](multi<size_type, 3> loc) const
    {
        return data_[convert(loc)];
    }

    /**
     * @brief 3-dimensional accessor.
     */
    reference operator()(size_type k0, size_type k1, size_type k2)
    {
        return data_[convert({k0, k1, k2})];
    }

    /**
     * @brief 3-dimensional accessor, const variant.
     */
    const_reference operator()(size_type k0, size_type k1, size_type k2) const
    {
        return data_[convert({k0, k1, k2})];
    }

    /**
     * @brief 3-dimensional accessor with range check.
     *
     * @throw std::out_of_range
     * If `loc` is out of range.
     */
    reference at(multi<size_type, 3> loc)
    {
        if (loc[0] >= data_size_[0] ||
            loc[1] >= data_size_[1] ||
            loc[2] >= data_size_[2])
            throw std::out_of_range(__func__);

        return data_[convert(loc)];
    }

    /**
     * @brief 3-dimensional accessor with range check, const variant.
     *
     * @throw std::out_of_range
     * If `loc` is out of range.
     */
    const_reference at(multi<size_type, 3> loc) const
    {
        if (loc[0] >= data_size_[0] ||
            loc[1] >= data_size_[1] ||
            loc[2] >= data_size_[2])
            throw std::out_of_range(__func__);

        return data_[convert(loc)];
    }

    /**@}*/

public:

    /**
     * @name Flattening
     */
    /**@{*/

    /**
     * @brief Flatten to linear array.
     */
    template <typename Tforward_itr>
    void flatten(Tforward_itr pos) const
    {
        for (size_type k = 0; k < user_size_[2]; k++) // Sheets.
        for (size_type i = 0; i < user_size_[0]; i++)
        for (size_type j = 0; j < user_size_[1]; j++) {
            *pos++ = (*this)(i, j, k);
        }
    }

    /**
     * @brief Flatten to linear array.
     */
    std::vector<T, Talloc> flatten() const
    {
        std::vector<T, Talloc> res;
        res.resize(user_size_.prod());
        flatten(res.begin());
        return res;
    }

    /**@}*/

protected:

    /**
     * @brief User size.
     */
    multi<size_type, 3> user_size_;

    /**
     * @brief User size after rounding.
     */
    multi<size_type, 3> data_size_;

    /**
     * @brief Data array.
     */
    std::vector<T, Talloc> data_;
};

/**@}*/

} // namespace pr

#endif // #ifndef PREFORM_BLOCK_ARRAY3_HPP
