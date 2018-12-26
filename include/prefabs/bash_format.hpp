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
#ifndef PREFABS_BASH_FORMAT_HPP
#define PREFABS_BASH_FORMAT_HPP

// for std::basic_ostream
#include <ostream>

namespace pr {

/**
 * @defgroup bash_format Bash formatting
 *
 * `<prefabs/bash_format.hpp>`.
 */
/**@{*/

/**
 * @brief Bash format enum.
 */
enum class bash_format : int
{
    /**
     * @brief Reset all.
     */
    reset = 0,

    /**
     * @brief Bold.
     */
    bold = 1,

    /**
     * @brief Dim.
     */
    dim = 2,

    /**
     * @brief Underline.
     */
    underline = 4,

    /**
     * @brief Blinking.
     */
    blink = 5,

    /**
     * @brief Reverse colors.
     */
    reverse = 7,

    /**
     * @brief Hidden.
     */
    hidden = 8,

    /**
     * @brief Reset bold.
     */
    reset_bold = 21,

    /**
     * @brief Reset dim.
     */
    reset_dim = 22,

    /**
     * @brief Reset underline.
     */
    reset_underline = 24,

    /**
     * @brief Reset blinking.
     */
    reset_blink = 25,

    /**
     * @brief Reset reverse colors.
     */
    reset_reverse = 27,

    /**
     * @brief Reset hidden.
     */
    reset_hidden = 28,

    /**
     * @brief Foreground default.
     */
    fg_default = 39,

    /**
     * @brief Foreground black.
     */
    fg_black = 30,

    /**
     * @brief Foreground red.
     */
    fg_red = 31,

    /**
     * @brief Foreground green.
     */
    fg_green = 32,

    /**
     * @brief Foreground yellow.
     */
    fg_yellow = 33,

    /**
     * @brief Foreground blue.
     */
    fg_blue = 34,

    /**
     * @brief Foreground magenta.
     */
    fg_magenta = 35,

    /**
     * @brief Foreground cyan.
     */
    fg_cyan = 36,

    /**
     * @brief Foreground light gray.
     */
    fg_light_gray = 37,

    /**
     * @brief Foreground dark gray.
     */
    fg_dark_gray = 90,

    /**
     * @brief Foreground light red.
     */
    fg_light_red = 91,

    /**
     * @brief Foreground light green.
     */
    fg_light_green = 92,

    /**
     * @brief Foreground light yellow.
     */
    fg_light_yellow = 93,

    /**
     * @brief Foreground light blue.
     */
    fg_light_blue = 94,

    /**
     * @brief Foreground light magenta.
     */
    fg_light_magenta = 95,

    /**
     * @brief Foreground light cyan.
     */
    fg_light_cyan = 96,

    /**
     * @brief Foreground white.
     */
    fg_white = 97,

    /**
     * @brief Background default.
     */
    bg_default = 49,

    /**
     * @brief Background black.
     */
    bg_black = 40,

    /**
     * @brief Background red.
     */
    bg_red = 41,

    /**
     * @brief Background green.
     */
    bg_green = 42,

    /**
     * @brief Background yellow.
     */
    bg_yellow = 43,

    /**
     * @brief Background blue.
     */
    bg_blue = 44,

    /**
     * @brief Background magenta.
     */
    bg_magenta = 45,

    /**
     * @brief Background cyan.
     */
    bg_cyan = 46,

    /**
     * @brief Background light gray.
     */
    bg_light_gray = 47,

    /**
     * @brief Background dark gray.
     */
    bg_dark_gray = 100,

    /**
     * @brief Background light red.
     */
    bg_light_red = 101,

    /**
     * @brief Background light green.
     */
    bg_light_green = 102,

    /**
     * @brief Background light yellow.
     */
    bg_light_yellow = 103,

    /**
     * @brief Background light blue.
     */
    bg_light_blue = 104,

    /**
     * @brief Background light magenta.
     */
    bg_light_magenta = 105,

    /**
     * @brief Background light cyan.
     */
    bg_light_cyan = 106,

    /**
     * @brief Background white.
     */
    bg_white = 107
};


/**
 * @brief Write into `std::basic_ostream`.
 */
template <typename C, typename Ctraits>
inline std::basic_ostream<C, Ctraits>& operator<<(
       std::basic_ostream<C, Ctraits>& os, bash_format fmt)
{
#if __linux__
    return os << "\e[" << int(fmt) << "m";
#else
    return os;
#endif // #if __linux__
}

/**@}*/

} // namespace pr

#endif // #ifndef PREFABS_BASH_FORMAT_HPP
