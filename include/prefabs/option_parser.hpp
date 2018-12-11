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
#ifndef PREFABS_OPTION_PARSER_HPP
#define PREFABS_OPTION_PARSER_HPP

// for assert
#include <cassert>

// for std::strcmp, std::strchr, ...
#include <cstring>

// for std::isalpha, std::isdigit, ...
#include <cctype>

// for std::function
#include <functional>

// for std::list
#include <list>

// for std::string
#include <string>

// for std::invalid_argument, std::runtime_error, ...
#include <stdexcept>

namespace pr {

/**
 * @defgroup option_parser Option parser
 */
/**@{*/

/**
 * @brief Is option string?
 *
 * An option string is `/^--?[a-zA-Z](?:-?[a-zA-Z0-9]+)*$/`.
 */
inline bool isoptstr(const char* s)
{
    // [-]+
    if (!s || 
        *s != '-') {
        return false;
    }
    ++s;
    if (*s == '-') {
        ++s;
    }

    // [a-zA-Z]
    if (!std::isalpha(static_cast<unsigned char>(*s))) {
        return false;
    }
    ++s;

    // (?:-?[a-zA-Z0-9]+)
    for (const char* t = s; true; 
                     s = t) {
        // -?
        if (*t == '-') {
            ++t;
        }
        // [a-zA-Z0-9]
        if (!std::isalpha(static_cast<unsigned char>(*t)) &&
            !std::isdigit(static_cast<unsigned char>(*t))) {
            break;
        }
        // [a-zA-Z0-9]*
        while (std::isalpha(static_cast<unsigned char>(*t)) ||
               std::isdigit(static_cast<unsigned char>(*t))) {
            ++t;
        }
    }

    return *s == '\0';
}

/**
 * @brief Option descriptor.
 */
class option
{
public:

    /**
     * @brief Name abbreviation.
     */
    const char* name_abbrv;

    /**
     * @brief Name. 
     */
    const char* name;

    /**
     * @brief Argument count.
     */
    int argc;

    /**
     * @brief Callback.
     */
    std::function<void(char**)> callback;

public:

    operator std::string() const
    {
        std::string desc;
        if (name_abbrv && 
            name) {
            desc.append(name_abbrv).append("/")
                .append(name);
        }
        else {
            desc.append(name_abbrv ? name_abbrv : name);
        }
        return desc;
    }
};

/**
 * @brief Option parser.
 */
class option_parser
{
public:

    /**
     * @brief Default constructor.
     */
    option_parser() = default;

    /**
     * @brief Add on-option callback.
     *
     * @param[in] name_abbrv
     * Name abbreviation.
     *
     * @param[in] name
     * Name.
     *
     * @param[in] argc
     * Argument count.
     *
     * @param[in] callback
     * Callback.
     */
    void on_option(
        const char* name_abbrv,
        const char* name,
        int argc,
        const std::function<void(char**)>& callback)
    {
        assert(name_abbrv || name);
        assert(!name_abbrv || isoptstr(name_abbrv));
        assert(!name || isoptstr(name));
        assert(!(argc < 0));
        assert(callback);

        // emplace
        opts_.emplace_back(option{
            name_abbrv,
            name,
            argc,
            callback
        });
    }

    /**
     * @brief Set on-positional callback.
     *
     * @param[in] callback
     * Callback.
     */
    void on_positional(
            const std::function<void(char*)>& callback)
    {
        pos_callback_ = callback;
    }

    /**
     * @brief Parse.
     *
     * @param[in] argc
     * Argument count.
     *
     * @param[in] argv
     * Argument pointer.
     *
     * @throw std::runtime_error
     * If unknown option or improper use of option.
     */
    void parse(int argc, char** argv)
    {
        assert(argc > 0);
        assert(argv);
        --argc;
        ++argv;

        while (argc > 0) {

            // look for '=', if present, truncate *argv
            char* eq = nullptr;
            if ((eq = std::strchr(*argv, '='))) {
                *eq = '\0';
            }

            // is option string?
            if (isoptstr(*argv)) {

                // process
                bool opt_okay = false;
                for (option& opt : opts_) {
                    const char* name_abbrv = opt.name_abbrv;
                    const char* name = opt.name;
                    if ((name_abbrv && !std::strcmp(name_abbrv, *argv)) ||
                        (name && !std::strcmp(name, *argv))) {

                        if (eq) {
                            // shift
                            *argv = eq + 1;
                        }
                        else {
                            // consume
                            --argc;
                            ++argv;
                        }

                        // not enough args?
                        if (argc < opt.argc ||
                                 (!opt.argc && eq)) { // or no args and eq?
                            throw 
                                std::runtime_error(
                                std::string(opt)
                                    .append(" expects ")
                                    .append(std::to_string(opt.argc))
                                    .append(" argument(s)"));
                        }

                        // delegate
                        opt.callback(argv);
                        opt_okay = true;

                        // consume
                        argc -= opt.argc;
                        argv += opt.argc;
                        break;
                    }
                }

                // unknown option?
                if (!opt_okay) {
                    throw 
                        std::runtime_error(
                        std::string("unknown option ")
                            .append(*argv));
                }
            }
            else {
                // undo truncate
                if (eq) {
                    *eq = '=';
                }

                // positional
                if (pos_callback_) {
                    pos_callback_(*argv);
                }

                // consume
                --argc;
                ++argv;
            }
        }
    }

private:

    /**
     * @brief Options.
     */
    std::list<option> opts_;

    /**
     * @brief Positional callback.
     */
    std::function<void(char*)> pos_callback_;
};

/**@}*/

} // namespace pr

#endif // #ifndef PREFABS_OPTION_PARSER_HPP
