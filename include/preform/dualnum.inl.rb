funcs = [
['exp', [:numer, 'pre::exp($$)']],
['log', [:denom, '$$']],
['exp2', [:numer, 'pre::numeric_constants<T>::M_ln2() * pre::exp2($$)']],
['log2', [:denom, '(pre::numeric_constants<T>::M_ln2() * $$)']],
['log10', [:denom, '(pre::numeric_constants<T>::M_ln10() * $$)']],
['expm1', [:numer, 'pre::exp($$)']],
['log1p', [:denom, '(T(1) + $$)']],
['sqrt', [:denom, '(T(2) * pre::sqrt($$))']],
['cbrt', [:denom, '(T(3) * pre::nthpow(pre::cbrt($$), 2))']],
['erf', [:numer, 'pre::numeric_constants<T>::M_2_sqrtpi() * pre::exp(-pre::nthpow($$, 2))']],
['erfc', [:numer, '-pre::numeric_constants<T>::M_2_sqrtpi() * pre::exp(-pre::nthpow($$, 2))']],
['sin', [:numer, 'pre::cos($$)']],
['cos', [:numer, '-pre::sin($$)']],
['tan', [:denom, 'pre::nthpow(pre::cos($$), 2)']],
['asin', [:denom, 'pre::sqrt(T(1) - pre::nthpow($$, 2))']],
['acos', [:denom, '-pre::sqrt(T(1) - pre::nthpow($$, 2))']],
['atan', [:denom, '(T(1) + pre::nthpow($$, 2))']],
['sinh', [:numer, 'pre::cosh($$)']],
['cosh', [:numer, 'pre::sinh($$)']],
['tanh', [:denom, 'pre::nthpow(pre::cosh($$), 2)']],
['asinh', [:denom, 'pre::sqrt(pre::nthpow($$, 2) + T(1))']],
['acosh', [:denom, 'pre::sqrt(pre::nthpow($$, 2) - T(1))']],
['atanh', [:denom, '(T(1) - pre::nthpow($$, 2))']]
]

puts <<STR
namespace pre {

/**
 * @addtogroup dualnum
 */
/**@{*/

STR

puts <<STR
/**
 * @name Math (dualnum)
 *
 * @f[
 *      f(a + \\varepsilon b) = 
 *      f(a) + \\varepsilon f'(a) b
 * @f]
 */
/**@{*/

STR

for func in funcs
    name = func[0]
    expr = func[1][1].gsub(/\$\$/, 'x.real()')
    case func[1][0]
    when :numer then expr = "#{expr} * x.dual()"
    when :denom then expr = "x.dual() / #{expr}"
    end
    puts <<STR
/**
 * @brief Dual number implementation of `pre::#{name}()`.
 */
template <typename T>
__attribute__((always_inline))
inline dualnum<T> #{name}(const dualnum<T>& x)
{
    return {
        pre::#{name}(x.real()),
        #{expr}
    };
}

STR
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
 * @name Reciprocal trigonometric functions (dualnum)
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
inline dualnum<T> #{func[0]}(const dualnum<T>& x)
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
inline dualnum<T> #{func[0]}(const dualnum<T>& x)
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
