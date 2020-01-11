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
#error "preform/option_parser.hpp requires >=C++14"
#endif // #if !(__cplusplus >= 201402L)
#endif // #if !DOXYGEN
#pragma once
#ifndef PREFORM_OPTION_PARSER_HPP
#define PREFORM_OPTION_PARSER_HPP

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

// for std::stringstream
#include <sstream>

// for std::basic_ostream
#include <ostream>

// for std::quoted
#include <iomanip>

// for std::invalid_argument, std::runtime_error, ...
#include <stdexcept>

namespace pr {

/**
 * @defgroup option_parser Option parser
 *
 * `<preform/option_parser.hpp>`
 *
 * __C++ version__: >=C++14
 */
/**@{*/

/**
 * @brief Option parser.
 */
class option_parser
{
public:

    /**
     * @brief Constructor.
     */
    option_parser(const char* prog_usage) : prog_usage_(prog_usage)
    {
        opt_groups_.emplace_back(option_group{
            nullptr,
            std::list<option>(),
            std::function<void()>(nullptr),
            std::function<void()>(nullptr),
            std::function<void(char*)>(nullptr),
            std::stringstream()
        });
    }

    /**
     * @brief Set in-group for subsequent calls.
     *
     * @param[in] name
     * Name.
     */
    std::ostream& in_group(const char* name)
    {
        for (auto itr = opt_groups_.begin();
                  itr != opt_groups_.end(); ++itr) {
            if ((!itr->name && !name) ||
                 (itr->name && !std::strcmp(itr->name, name))) {
                opt_groups_.splice(
                opt_groups_.end(), opt_groups_, itr);
                return opt_groups_.back().help;
            }
        }
        opt_groups_.emplace_back(option_group{
            name,
            std::list<option>(),
            std::function<void()>(nullptr),
            std::function<void()>(nullptr),
            std::function<void(char*)>(nullptr),
            std::stringstream()
        });

        return opt_groups_.back().help;
    }

    /**
     * @brief Set on-begin callback.
     *
     * @param[in] callback
     * Callback.
     */
    void on_begin(const std::function<void()>& callback)
    {
        opt_groups_.back().on_begin = callback;
    }

    /**
     * @brief Set on-end callback.
     *
     * @param[in] callback
     * Callback.
     */
    void on_end(const std::function<void()>& callback)
    {
        opt_groups_.back().on_end = callback;
    }

    /**
     * @brief Set on-positional callback.
     *
     * @param[in] callback
     * Callback.
     */
    void on_positional(const std::function<void(char*)>& callback)
    {
        opt_groups_.back().on_positional = callback;
    }

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
     *
     * @returns Help stream.
     */
    std::ostream& on_option(
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
        opt_groups_.back().opts.emplace_back(option{
            name_abbrv,
            name,
            argc,
            callback,
            std::stringstream()
        });

        return opt_groups_.back().opts.back().help;
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
        prog_name_ = *argv;
        --argc;
        ++argv;

        // Group.
        std::list<option_group>::iterator itropt_group = opt_groups_.begin();

        // On begin.
        if (itropt_group->on_begin) {
            itropt_group->on_begin();
        }

        while (argc > 0) {

            // Look for '=', if present, truncate *argv.
            char* eq = nullptr;
            if ((eq = std::strchr(*argv, '='))) {
                *eq = '\0';
            }

            // Is option string?
            if (isoptstr(*argv)) {

                // Process.
                bool opt_okay = false;
                for (option& opt : itropt_group->opts) {
                    const char* name_abbrv = opt.name_abbrv;
                    const char* name = opt.name;
                    if ((name_abbrv && !std::strcmp(name_abbrv, *argv)) ||
                        (name && !std::strcmp(name, *argv))) {

                        if (eq) {
                            // Shift.
                            *argv = eq + 1;
                        }
                        else {
                            // Consume.
                            --argc;
                            ++argv;
                        }

                        // Not enough args?
                        if (argc < opt.argc ||
                                 (!opt.argc && eq)) { // or no args and eq?
                            std::stringstream ss;
                            if (itropt_group->name) {
                                ss << std::string(*itropt_group);
                                ss << ' ';
                            }
                            ss << std::string(opt);
                            ss << " expects " << opt.argc << " argument(s)";
                            throw std::runtime_error(ss.str());
                        }

                        // Delegate.
                        opt.on_option(argv);
                        opt_okay = true;

                        // Consume.
                        argc -= opt.argc;
                        argv += opt.argc;
                        break;
                    }
                }

                // Unknown option?
                if (!opt_okay) {
                    std::stringstream ss;
                    if (itropt_group->name) {
                        ss << std::string(*itropt_group);
                        ss << ' ';
                    }
                    ss << "Unknown option " << *argv;
                    throw std::runtime_error(ss.str());
                }
            }
            else {
                // Undo truncate.
                if (eq) {
                    *eq = '=';
                }

                // Find group?
                bool itrfound = false;
                for (auto itr = opt_groups_.begin();
                          itr != opt_groups_.end(); ++itr) {
                    if (itr->name && !std::strcmp(itr->name, *argv)) {
                        // End.
                        if (itropt_group->on_end) {
                            itropt_group->on_end();
                        }

                        itrfound = true;
                        itropt_group = itr;

                        // Begin.
                        if (itropt_group->on_begin) {
                            itropt_group->on_begin();
                        }
                        break;
                    }
                }

                if (!itrfound) {

                    // Positional.
                    if (itropt_group->on_positional) {
                        itropt_group->on_positional(*argv);
                    }
                    else {
                        std::stringstream ss;
                        if (itropt_group->name) {
                            ss << std::string(*itropt_group);
                            ss << ' ';
                        }
                        ss << "Unexpected positional argument ";
                        ss << std::quoted(*argv);
                        throw std::runtime_error(ss.str());
                    }
                }

                // Consume.
                --argc;
                ++argv;
            }
        }

        // End.
        if (itropt_group->on_end) {
            itropt_group->on_end();
        }
    }

    /**
     * @brief Write help to `std::basic_ostream`.
     */
    template <typename C, typename Ctraits>
    friend
    inline std::basic_ostream<C, Ctraits>& operator<<(
           std::basic_ostream<C, Ctraits>& os, option_parser& opt_parse)
    {
        os << "Usage: ";
        os << opt_parse.prog_name_ << ' ';
        os << opt_parse.prog_usage_ << '\n';
        os << '\n';
        for (option_group& opt_group : opt_parse.opt_groups_) {
            if (opt_group.name) {
                os << std::string(opt_group).c_str() << '\n';
                os << opt_group.help.str().c_str();
                os << '\n';
            }
            for (option& opt : opt_group.opts) {
                os << std::string(opt).c_str();
                os << '{' << opt.argc << '}';
                os << '\n';
                os << '\t';
                std::string str = opt.help.str();
                for (char ch : str) {
                    os << ch;
                    if (ch == '\n') {
                        os << '\t';
                    }
                }
                os << '\n';
            }
        }
        return os;
    }

private:

    /**
     * @brief Is option string?
     *
     * An option string is `/^--?[a-zA-Z](?:-?[a-zA-Z0-9]+)*$/`.
     */
    static bool isoptstr(const char* s)
    {
        // --?
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

        // (?:-?[a-zA-Z0-9]+)*
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
     * @brief Option.
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
        std::function<void(char**)> on_option;

        /**
         * @brief Help stringstream.
         */
        std::stringstream help;

    public:

        /**
         * @brief Form string from `name_abbrv` and `name`.
         */
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
     * @brief Option group.
     */
    class option_group
    {
    public:

        /**
         * @brief Name.
         */
        const char* name;

        /**
         * @brief Options.
         */
        std::list<option> opts;

        /**
         * @brief On-begin callback.
         */
        std::function<void()> on_begin;

        /**
         * @brief On-end callback.
         */
        std::function<void()> on_end;

        /**
         * @brief On-positional callback.
         */
        std::function<void(char*)> on_positional;

        /**
         * @brief Help stringstream.
         */
        std::stringstream help;

    public:

        /**
         * @brief Form string from `name`.
         */
        operator std::string() const
        {
            if (!name) {
                return "";
            }
            else {
                return std::string("<").append(name).append(">");
            }
        }
    };

    /**
     * @brief Program name (from `argv[0]`).
     */
    const char* prog_name_;

    /**
     * @brief Program usage.
     */
    const char* prog_usage_;

    /**
     * @brief Option groups.
     */
    std::list<option_group> opt_groups_;
};

/**@}*/

} // namespace pr

#endif // #ifndef PREFORM_OPTION_PARSER_HPP
