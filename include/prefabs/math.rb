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
['atanh',       [['T','x']]]
]

puts <<STR
/**
 * @addtogroup math
 */
/**@{*/

STR

puts <<STR
/**
 * @name Math wrappers
 */
/**@{*/

STR

for func in funcs
    funcname = func[0]
    args1 = []
    args2 = []
    for arg in func[1]
        args1 << "#{arg[0]} #{arg[1]}"
        args2 << "#{arg[1]}"
    end
    args1 = args1.join ", "
    args2 = args2.join ", "
    puts <<STR
/**
 * @brief Wrap `std::#{funcname}()`.
 */
template <typename T>
__attribute__((always_inline)) 
inline auto #{funcname}(#{args1}) -> decltype(std::#{funcname}(#{args2}))
{
    return std::#{funcname}(#{args2});
}

STR
end

puts <<STR
/**@}*/

STR

puts <<STR
/**@}*/

STR
