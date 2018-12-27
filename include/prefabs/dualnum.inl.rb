funcs = [
['exp', [:numer, 'pr::exp($$)']],
['log', [:denom, '$$']],
['exp2', [:numer, 'pr::numeric_constants<T>::M_ln2() * pr::exp2($$)']],
['log2', [:denom, '(pr::numeric_constants<T>::M_ln2() * $$)']],
['log10', [:denom, '(pr::numeric_constants<T>::M_ln10() * $$)']],
['expm1', [:numer, 'pr::exp($$)']],
['log1p', [:denom, '(T(1) + $$)']],
['sqrt', [:denom, '(T(2) * pr::sqrt($$))']],
['cbrt', [:denom, '(T(3) * pr::nthpow(pr::cbrt($$), 2))']],
['erf', [:numer, 'pr::numeric_constants<T>::M_2_sqrtpi() * pr::exp(-pr::nthpow($$, 2))']],
['erfc', [:numer, '-pr::numeric_constants<T>::M_2_sqrtpi() * pr::exp(-pr::nthpow($$, 2))']],
['sin', [:numer, 'pr::cos($$)']],
['cos', [:numer, '-pr::sin($$)']],
['tan', [:denom, 'pr::nthpow(pr::cos($$), 2)']],
['asin', [:denom, 'pr::sqrt(T(1) - pr::nthpow($$, 2))']],
['acos', [:denom, '-pr::sqrt(T(1) - pr::nthpow($$, 2))']],
['atan', [:denom, '(T(1) + pr::nthpow($$, 2))']],
['sinh', [:numer, 'pr::cosh($$)']],
['cosh', [:numer, 'pr::sinh($$)']],
['tanh', [:denom, 'pr::nthpow(pr::cosh($$), 2)']],
['asinh', [:denom, 'pr::sqrt(pr::nthpow($$, 2) + T(1))']],
['acosh', [:denom, 'pr::sqrt(pr::nthpow($$, 2) - T(1))']],
['atanh', [:denom, '(T(1) - pr::nthpow($$, 2))']]
]

puts <<STR
namespace pr {

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
 *      f(\\real(x) + \\varepsilon \\dual(x)) = 
 *      f(\\real(x)) + \\varepsilon f'(\\real(x)) \\dual(x)
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
 * @brief Dual number implementation of `pr::#{name}()`.
 */
template <typename T>
__attribute__((always_inline))
inline dualnum<T> #{name}(const dualnum<T>& x)
{
    return {
        pr::#{name}(x.real()),
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
 * @brief Reciprocal of `pr::#{func[1]}()`.
 */
template <typename T>
__attribute__((always_inline))
inline dualnum<T> #{func[0]}(const dualnum<T>& x)
{
    return T(1) / pr::#{func[1]}(x);
}

STR
end

for func in funcs_trig_rcp_inv
    puts <<STR
/**
 * @brief Inverse of `pr::#{func[0][1..-1]}()`.
 */
template <typename T>
__attribute__((always_inline))
inline dualnum<T> #{func[0]}(const dualnum<T>& x)
{
    return pr::#{func[1]}(T(1) / x);
}

STR
end

puts <<STR
/**@}*/

STR

puts <<STR
/**@}*/

} // namespace pr

STR
