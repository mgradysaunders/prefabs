funcs = [
['abs',       :c128,      '__float128', [['T','x']]],
['arg',       :c128,      '__float128', [['T','x']]],
['real',      :c128,      '__float128', [['T','x']]],
['imag',      :c128,      '__float128', [['T','x']]],
['conj',      :c128,      'T',          [['T','x']]],
['proj',      :c128,      'T',          [['T','x']]],
['fabs',      :f128,      'T',          [['T','x']]],
['fma',       :f128,      'T',          [['T','x'], ['T','y'], ['T','z']]],
['fmin',      :f128,      'T',          [['T','x'], ['T','y']]],
['fmax',      :f128,      'T',          [['T','x'], ['T','y']]],
['fdim',      :f128,      'T',          [['T','x'], ['T','y']]],
['fmod',      :f128,      'T',          [['T','x'], ['T','y']]],
['remquo',    :f128,      'T',          [['T','x'], ['T','y'], ['int*','q']]],
['remainder', :f128,      'T',          [['T','x'], ['T','y']]],
['nearbyint', :f128,      'T',          [['T','x']]],
['floor',     :f128,      'T',          [['T','x']]],
['ceil',      :f128,      'T',          [['T','x']]],
['trunc',     :f128,      'T',          [['T','x']]],
['round',     :f128,      'T',          [['T','x']]],
['rint',      :f128,      'T',          [['T','x']]],
['lrint',     :f128,      'long',       [['T','x']]],
['llrint',    :f128,      'long long',  [['T','x']]],
['lround',    :f128,      'long',       [['T','x']]],
['llround',   :f128,      'long long',  [['T','x']]],
['frexp',     :f128,      'T',          [['T','x'], ['int*','p']]],
['ldexp',     :f128,      'T',          [['T','x'], ['int','p']]],
['logb',      :f128,      'T',          [['T','x']]],
['ilogb',     :f128,      'int',        [['T','x']]],
['scalbn',    :f128,      'T',          [['T','x'], ['int','p']]],
['scalbln',   :f128,      'T',          [['T','x'], ['long','p']]],
['modf',      :f128,      'T',          [['T','x'], ['T*','p']]],
['nextafter', :f128,      'T',          [['T','x'], ['T','y']]],
['copysign',  :f128,      'T',          [['T','x'], ['T','y']]],
['signbit',   :f128,      'bool',       [['T','x']]],
['isnan',     :f128,      'bool',       [['T','x']]],
['isinf',     :f128,      'bool',       [['T','x']]],
['isfinite',  :f128,      'bool',       [['T','x']]],
['exp',       :f128_c128, 'T',          [['T','x']]],
['log',       :f128_c128, 'T',          [['T','x']]],
#['exp2',      :f128,      'T',          [['T','x']]],
['log2',      :f128,      'T',          [['T','x']]],
['log10',     :f128_c128, 'T',          [['T','x']]],
['expm1',     :f128,      'T',          [['T','x']]],
['log1p',     :f128,      'T',          [['T','x']]],
['pow',       :f128_c128, 'T',          [['T','x'], ['T','y']]],
['sqrt',      :f128_c128, 'T',          [['T','x']]],
['cbrt',      :f128,      'T',          [['T','x']]],
['hypot',     :f128,      'T',          [['T','x'], ['T','y']]],
['erf',       :f128,      'T',          [['T','x']]],
['erfc',      :f128,      'T',          [['T','x']]],
['lgamma',    :f128,      'T',          [['T','x']]],
['tgamma',    :f128,      'T',          [['T','x']]],
['sin',       :f128_c128, 'T',          [['T','x']]],
['cos',       :f128_c128, 'T',          [['T','x']]],
['tan',       :f128_c128, 'T',          [['T','x']]],
['asin',      :f128_c128, 'T',          [['T','x']]],
['acos',      :f128_c128, 'T',          [['T','x']]],
['atan',      :f128_c128, 'T',          [['T','x']]],
['atan2',     :f128,      'T',          [['T','y'], ['T','x']]],
['sinh',      :f128_c128, 'T',          [['T','x']]],
['cosh',      :f128_c128, 'T',          [['T','x']]],
['tanh',      :f128_c128, 'T',          [['T','x']]],
['asinh',     :f128_c128, 'T',          [['T','x']]],
['acosh',     :f128_c128, 'T',          [['T','x']]],
['atanh',     :f128_c128, 'T',          [['T','x']]]
]

def get_quadname funcname, category
    case category
    when :f128
        case funcname
        when 'isfinite' then 'finiteq'
        else "#{funcname}q"
        end
    when :c128
        case funcname
        when 'conj' then 'conjq'
        else "c#{funcname}q"
        end
    end
end

puts <<STR
namespace pre {

/**
 * @addtogroup quadmath
 */
/**@{*/

STR

puts <<STR
/**
 * @name Quadmath wrappers (__float128)
 */
/**@{*/

STR

for func in funcs
    if func[1] == :f128 or
       func[1] == :f128_c128
        funcname = func[0]
        quadname = get_quadname funcname, :f128
        result = "std::enable_if_t<std::is_same<T, __float128>::value, #{func[2]}>"
        args1 = []
        args2 = []
        for arg in func[3]
            args1 << "#{arg[0]} #{arg[1]}"
            args2 << "#{arg[1]}"
        end
        args1 = args1.join ", "
        args2 = args2.join ", "
        puts <<STR
/**
 * @brief Wrap `#{quadname}()`.
 */
template <typename T>
__attribute__((always_inline))
inline #{result} #{funcname}(#{args1})
{
    return ::#{quadname}(#{args2});
}

STR
    end
end

puts <<STR
/**@}*/

STR

puts <<STR
/**
 * @name Quadmath wrappers (__complex128)
 */
/**@{*/

STR

for func in funcs
    if func[1] == :c128 or
       func[1] == :f128_c128
        funcname = func[0]
        quadname = get_quadname funcname, :c128
        if func[2] == 'T'
            result = "std::enable_if_t<std::is_same<U, __complex128>::value, U>"
        else
            result = "std::enable_if_t<std::is_same<U, __complex128>::value, #{func[2]}>"
        end
        args1 = []
        args2 = []
        for arg in func[3]
            arg0 = arg[0].clone
            arg1 = arg[1].clone
            if arg0 == 'T'
                arg0 = 'U'
            end
            args1 << "#{arg0} #{arg1}"
            args2 << "#{arg1}"
        end
        args1 = args1.join ", "
        args2 = args2.join ", "
        puts <<STR
/**
 * @brief Wrap `#{quadname}()`.
 */
template <typename U>
__attribute__((always_inline))
inline #{result} #{funcname}(#{args1})
{
    return ::#{quadname}(#{args2});
}

STR
    end
end

puts <<STR
/**@}*/

STR


funcs_trig_rcp = [
['csc', 'sin'],
['sec', 'cos'],
['cot', 'tan'],
['csch', 'sinh'],
['sech', 'cosh'],
['coth', 'tanh']
]

funcs_trig_rcp_inv = [
['acsc', 'asin'],
['asec', 'acos'],
['acot', 'atan'],
['acsch', 'asinh'],
['asech', 'acosh'],
['acoth', 'atanh']
]

puts <<STR
/**
 * @name Reciprocal trigonometric functions (__float128/__complex128)
 */
/**@{*/

STR

for func in funcs_trig_rcp
    puts <<STR
/**
 * @brief Reciprocal of `pre::#{func[1]}()`.
 */
template <typename T>
__attribute__((always_inline))
inline std::enable_if_t<
                std::is_same<T, __float128>::value ||
                std::is_same<T, __complex128>::value, T> #{func[0]}(T x)
{
    return T(1) / pre::#{func[1]}(x);
}

STR
end

for func in funcs_trig_rcp_inv
    puts <<STR
/**
 * @brief Inverse of `pre::#{func[0][1..-1]}()`.
 */
template <typename T>
__attribute__((always_inline))
inline std::enable_if_t<
                std::is_same<T, __float128>::value ||
                std::is_same<T, __complex128>::value, T> #{func[0]}(T x)
{
    return pre::#{func[1]}(T(1) / x);
}

STR
end

puts <<STR
/**@}*/

STR

puts <<STR
/**@}*/

} // namespace pre

STR
