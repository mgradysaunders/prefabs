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
 * `<prefabs/option_parser.hpp>`
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

// TODO relocate under option_parser
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

// TODO relocate under option_parser
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
     * @brief Option descriptors.
     */
    std::list<option> opts;

    /**
     * @brief Begin callback.
     */
    std::function<void(void)> begin_callback;

    /**
     * @brief End callback.
     */
    std::function<void(void)> end_callback;

    /**
     * @brief Positional callback.
     */
    std::function<void(char*)> pos_callback;

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
        groups_.emplace_back(option_group{
            nullptr,
            std::list<option>(),
            std::function<void(void)>(nullptr),
            std::function<void(void)>(nullptr),
            std::function<void(char*)>(nullptr)
        });
    }

    /**
     * @brief Set in-group for subsequent calls.
     *
     * @param[in] name
     * Name.
     */
    void in_group(const char* name)
    {
        for (auto it = groups_.begin(); 
                  it != groups_.end(); ++it) {
            if ((!it->name && !name) ||
                 (it->name && !std::strcmp(it->name, name))) {
                groups_.splice(groups_.end(), groups_, it);
                return;
            }
        }
        groups_.emplace_back(option_group{
            name,
            std::list<option>(),
            std::function<void(void)>(nullptr),
            std::function<void(void)>(nullptr),
            std::function<void(char*)>(nullptr)
        });
    }

    /**
     * @brief Set on-begin callback.
     *
     * @param[in] callback
     * Callback.
     */
    void on_begin(
            const std::function<void(void)>& callback)
    {
        groups_.back().begin_callback = callback;
    }

    /**
     * @brief Set on-end callback.
     *
     * @param[in] callback
     * Callback.
     */
    void on_end(
            const std::function<void(void)>& callback)
    {
        groups_.back().end_callback = callback;
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
        groups_.back().pos_callback = callback;
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
        groups_.back().opts.emplace_back(option{
            name_abbrv,
            name,
            argc,
            callback,
            std::stringstream()
        });

        return groups_.back().opts.back().help;
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

        // group
        std::list<option_group>::iterator itgroup = groups_.begin();

        // begin
        if (itgroup->begin_callback) {
            itgroup->begin_callback();
        }

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
                for (option& opt : itgroup->opts) {
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
                            std::stringstream ss;
                            if (itgroup->name) {
                                ss << std::string(*itgroup);
                                ss << ' ';
                            }
                            ss << std::string(opt);
                            ss << " expects " << opt.argc << " argument(s)";
                            throw std::runtime_error(ss.str());
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
                    std::stringstream ss;
                    if (itgroup->name) {
                        ss << std::string(*itgroup);
                        ss << ' ';
                    }
                    ss << "Unknown option " << *argv;
                    throw std::runtime_error(ss.str());
                }
            }
            else {
                // undo truncate
                if (eq) {
                    *eq = '=';
                }

                // find group?
                bool itfound = false;
                for (auto it = groups_.begin(); 
                          it != groups_.end(); it++) {
                    if (it->name && !std::strcmp(it->name, *argv)) {
                        // end
                        if (itgroup->end_callback) {
                            itgroup->end_callback();
                        }

                        itfound = true;
                        itgroup = it;

                        // begin
                        if (itgroup->begin_callback) {
                            itgroup->begin_callback();
                        }
                        break;
                    }
                }

                if (!itfound) {

                    // positional
                    if (itgroup->pos_callback) {
                        itgroup->pos_callback(*argv);
                    }
                    else {
                        std::stringstream ss;
                        if (itgroup->name) {
                            ss << std::string(*itgroup);
                            ss << ' ';
                        }
                        ss << "Unexpected positional argument ";
                        ss << std::quoted(*argv);
                        throw std::runtime_error(ss.str());
                    }
                }

                // consume
                --argc;
                ++argv;
            }
        }

        // end
        if (itgroup->end_callback) {
            itgroup->end_callback();
        }
    }

    /**
     * @brief Write help to `std::basic_ostream`.
     */
    template <typename Char, typename Traits>
    friend
    inline std::basic_ostream<Char, Traits>& operator<<(
           std::basic_ostream<Char, Traits>& os, option_parser& opt_parse)
    {
        os << "Usage: ";
        os << opt_parse.prog_name_ << ' ';
        os << opt_parse.prog_usage_ << '\n';
        os << '\n';
        for (option_group& group : opt_parse.groups_) {
            if (group.name) {
                os << std::string(group).c_str();
                os << '\n';
                os << '\n';
            }
            for (option& opt : group.opts) {
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
    std::list<option_group> groups_;
};

/**@}*/

} // namespace pr

#endif // #ifndef PREFABS_OPTION_PARSER_HPP
