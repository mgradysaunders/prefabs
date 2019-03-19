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
#if !(__cplusplus >= 201103L)
#error "preform/bash_format.hpp requires >=C++11"
#endif // #if !(__cplusplus >= 201103L)
#endif // #if !DOXYGEN
#pragma once
#ifndef PREFORM_BASH_FORMAT_HPP
#define PREFORM_BASH_FORMAT_HPP

#if __linux__
extern "C" {
// for ioctl
#include <sys/ioctl.h>

// for STDOUT_FILENO
#include <unistd.h>
}
#endif // #if __linux__

// for std::reverse, std::max, std::min
#include <algorithm>

// for std::basic_ostream
#include <ostream>

namespace pr {

/**
 * @defgroup bash_format Bash formatting
 *
 * `<preform/bash_format.hpp>`.
 *
 * __C++ version__: >=C++11
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
__attribute__((always_inline))
inline std::basic_ostream<C, Ctraits>& operator<<(
       std::basic_ostream<C, Ctraits>& os, bash_format fmt)
{
#if __linux__
    return os << "\e[" << int(fmt) << "m";
#else
    return os;
#endif // #if __linux__
}

/**
 * @brief Terminal dimensions.
 */
struct terminal_dims
{
    /**
     * @brief Rows.
     */
    int rows;

    /**
     * @brief Columns.
     */
    int cols;

    /**
     * @brief Get current terminal dimensions.
     *
     * @note
     * On Linux, uses `ioctl` to fetch terminal size. Otherwise,
     * defaults to 24 rows by 80 columns.
     */
    static terminal_dims get()
    {
    #if __linux__
        struct winsize winsz;
        ioctl(STDOUT_FILENO, TIOCGWINSZ, &winsz);
        return {
            int(winsz.ws_row),
            int(winsz.ws_col)
        };
    #else
        return {
            24,
            80
        };
    #endif // #if __linux__
    }
};

/**
 * @brief Terminal progress bar.
 */
struct terminal_progress_bar
{
public:

    /**
     * @brief Amount complete, between 0 and 1.
     */
    double amount = 0.0;

    /**
     * @brief Write into `std::basic_ostream`.
     */
    template <typename C, typename Ctraits>
    friend
    inline std::basic_ostream<C, Ctraits>& operator<<(
           std::basic_ostream<C, Ctraits>& os, terminal_progress_bar bar)
    {
        // Print bracket.
        os << '[';

        // Terminal dimensions.
        terminal_dims dims =
        terminal_dims::get();
        if (dims.cols < 8) {
            // Error?
            dims = {
                24,
                80
            };
        }

        // Amount.
        double amount =
            std::max(0.0,
            std::min(1.0, bar.amount));

        // Columns total.
        int columns = dims.cols - 2;

        // Message.
        char message[8];
        message[0] = '%';
        message[2] = '.';
        char* messageitr = &message[1];
        int tmp = amount * 1000;
        while (tmp >= 10) {

            // Reduce.
            int quo = tmp / 10;
            int rem = tmp % 10;
            tmp = quo;

            // Digit.
            *messageitr++ = rem + '0';
            if (messageitr == &message[2]) {
                messageitr++; // Skip decimal point.
            }
        }
        if (tmp > 0) {
            // Digit.
            *messageitr++ = tmp + '0';
        }
        *messageitr = '\0';

        // Reverse.
        std::reverse(
            message,
            messageitr);

        // Print message.
        os << message;

        // Print progress.
        int column0 = messageitr - message;
        int column1 =
            std::max(column0,
                 int(columns * amount));
        for (; column0 < column1; column0++) { os << '='; }
        for (; column1 < columns; column1++) { os << '.'; }

        // Print bracket.
        os << ']';

        return os;
    }
};

/**@}*/

} // namespace pr

#endif // #ifndef PREFORM_BASH_FORMAT_HPP
