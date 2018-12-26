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
#ifndef PREFABS_TIMER_HPP
#define PREFABS_TIMER_HPP

// for std::int64_t
#include <cstdint>

// for std::chrono
#include <chrono>

namespace pr {

/**
 * @defgroup timer Timer
 *
 * `<prefabs/timer.hpp>`
 */
/**@{*/

/**
 * @brief Timer.
 */
template <typename T>
class timer
{
public:

    /**
     * @brief Constructor.
     */
    timer() : start_(T::now())
    {
    }

    /**
     * @brief Read.
     */
    template <typename R>
    std::int64_t read() const
    {
        return 
            std::chrono::duration_cast<
            std::chrono::duration<std::int64_t, R>>(
                                    T::now() - start_).count();
    }

private:

    /**
     * @brief Time point at construction.
     */
    std::chrono::time_point<T> start_;
};

/**
 * @brief Timer for `std::chrono::system_clock`.
 */
using system_timer = timer<std::chrono::system_clock>;

/**
 * @brief Timer for `std::chrono::steady_clock`.
 */
using steady_timer = timer<std::chrono::steady_clock>;

/**
 * @brief Timer for `std::chrono::high_resolution_clock`.
 */
using high_resolution_timer = timer<std::chrono::high_resolution_clock>;


/**@}*/

} // namespace pr

#endif // #ifndef PREFABS_TIMER_HPP
