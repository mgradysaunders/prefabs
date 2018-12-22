funcs = [
['finc',        [['T','x']]],
['fdec',        [['T','x']]],
['fclamp',      [['T','x'],['T','a'],['T','b']]],
['frepeat',     [['T','x'],['T','a'],['T','b']]],
['fmirror',     [['T','x'],['T','a'],['T','b']]],
['fcycle',      [['T','x'],['T','a'],['T','b'],['cycle_mode','mode']]]
]

puts <<STR
/**
 * @addtogroup multi_float_helpers
 */
/**@{*/

STR

puts <<STR
/**
 * @name Float helpers (multi)
 */
/**@{*/

STR

for func in funcs
    funcname = func[0]
    args1 = []
    args2 = []
    args3 = []
    incrs = []
    decls = []
    for arg in func[1]
        if arg[0][-1] != '*' 
            args1 << "const multi<#{arg[0]}, N...>& #{arg[1]}"
            args3 << "*itr#{arg[1]}"
            decls << "auto itr#{arg[1]} = #{arg[1]}.begin();"
        else
            args1 << "multi<#{arg[0].slice(0..-2)}, N...>* #{arg[1]}"
            args3 << "&(*itr#{arg[1]})"
            decls << "auto itr#{arg[1]} = #{arg[1]}->begin();"
        end
        if arg[0] == 'T'
            args2 << 'T()'
        else
            args2 << "std::declval<#{arg[0]}>()"
        end
        incrs << "++itr#{arg[1]}"
    end
    args1 = args1.join ", "
    args2 = args2.join ", "
    args3 = args3.join ", "
    incrs << "++itrres"
    incrs = incrs.join ", "
    decls = decls.join "\n    "
    restype = "multi<std::decay_t<decltype(pr::#{funcname}(#{args2}))>, N...>"
    puts <<STR
/**
 * @brief Wrap `pr::#{funcname}()`.
 */
template <typename T, std::size_t... N>
__attribute__((always_inline))
inline #{restype} #{funcname}(#{args1})
{
    #{restype} res;
    #{decls}
    auto itrres = res.begin();
    for (; itrres < res.end(); #{incrs}) {
        *itrres = pr::#{funcname}(#{args3});
    }
    return res;
}

STR
end

puts <<STR
/**
 * @brief Wrap `pr::fstretch()`.
 */
template <typename U, typename T, std::size_t... N>
__attribute__((always_inline))
inline multi<std::decay_t<decltype(pr::fstretch<U>(T()))>, N...> fstretch(const multi<T, N...>& x)
{
    multi<std::decay_t<decltype(pr::fstretch<U>(T()))>, N...> res;
    auto itrx = x.begin();
    auto itrres = res.begin();
    for (; itrres < res.end(); ++itrx, ++itrres) {
        *itrres = pr::fstretch<U>(*itrx);
    }
    return res;
}

STR

puts <<STR
/**@}*/

STR

puts <<STR
/**@}*/

STR
