funcs = [
['abs',         [['T','x']]],
['arg',         [['T','x']]],
['min',         [['T','x'],['T','y']]],
['max',         [['T','x'],['T','y']]],
['fabs',        [['T','x']]],
['fma',         [['T','x'], ['T','y'], ['T','z']]],
['fmin',        [['T','x'], ['T','y']]],
['fmax',        [['T','x'], ['T','y']]],
['fdim',        [['T','x'], ['T','y']]],
['fmod',        [['T','x'], ['T','y']]],
['remquo',      [['T','x'], ['T','y'], ['int*','q']]],
['remainder',   [['T','x'], ['T','y']]],
['nearbyint',   [['T','x']]],
['floor',       [['T','x']]],
['ceil',        [['T','x']]],
['trunc',       [['T','x']]],
['round',       [['T','x']]],
['rint',        [['T','x']]],
['lrint',       [['T','x']]],
['llrint',      [['T','x']]],
['lround',      [['T','x']]],
['llround',     [['T','x']]],
['frexp',       [['T','x'], ['int*','p']]],
['ldexp',       [['T','x'], ['int','p']]],
['logb',        [['T','x']]],
['ilogb',       [['T','x']]],
['scalbn',      [['T','x'], ['int','p']]],
['scalbln',     [['T','x'], ['long','p']]],
['modf',        [['T','x'], ['T*','p']]],
['nextafter',   [['T','x'], ['T','y']]],
['nexttoward',  [['T','x'], ['long double','y']]],
['copysign',    [['T','x'], ['T','y']]],
['signbit',     [['T','x']]],
['isnan',       [['T','x']]],
['isinf',       [['T','x']]],
['isfinite',    [['T','x']]],
['isnormal',    [['T','x']]],
['exp',         [['T','x']]],
['log',         [['T','x']]],
['exp2',        [['T','x']]],
['log2',        [['T','x']]],
['log10',       [['T','x']]],
['expm1',       [['T','x']]],
['log1p',       [['T','x']]],
['pow',         [['T','x'], ['T','y']]],
['sqrt',        [['T','x']]],
['cbrt',        [['T','x']]],
['hypot',       [['T','x'], ['T','y']]],
['erf',         [['T','x']]],
['erfc',        [['T','x']]],
['lgamma',      [['T','x']]],
['tgamma',      [['T','x']]],
['sin',         [['T','x']]],
['cos',         [['T','x']]],
['tan',         [['T','x']]],
['asin',        [['T','x']]],
['acos',        [['T','x']]],
['atan',        [['T','x']]],
['atan2',       [['T','y'], ['T','x']]],
['sinh',        [['T','x']]],
['cosh',        [['T','x']]],
['tanh',        [['T','x']]],
['asinh',       [['T','x']]],
['acosh',       [['T','x']]],
['atanh',       [['T','x']]],
['csc',         [['T','x']]],
['sec',         [['T','x']]],
['cot',         [['T','x']]],
['csch',        [['T','x']]],
['sech',        [['T','x']]],
['coth',        [['T','x']]],
['acsc',        [['T','x']]],
['asec',        [['T','x']]],
['acot',        [['T','x']]],
['acsch',       [['T','x']]],
['asech',       [['T','x']]],
['acoth',       [['T','x']]]
]

puts <<STR
/**
 * @addtogroup multi_math
 */
/**@{*/

STR

puts <<STR
/**
 * @name Math wrappers (multi)
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
            decls << "auto itr#{arg[1]} = #{arg[1]}.begin();"
            args3 << "*itr#{arg[1]}"
        else
            args1 << "multi<#{arg[0].chop}, N...>* #{arg[1]}"
            decls << "auto itr#{arg[1]} = #{arg[1]}->begin();"
            args3 << "&(*itr#{arg[1]})"
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
/**@}*/

STR

puts <<STR
/**@}*/

STR
