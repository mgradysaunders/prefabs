/**
 * @addtogroup dualnum
 */
/**@{*/

/**
 * @name Math (dualnum)
 *
 * @f[
 *      f(\real(x) + \varepsilon \dual(x)) = 
 *      f(\real(x)) + \varepsilon f'(\real(x)) \dual(x)
 * @f]
 */
/**@{*/

/**
 * @brief Dual number implementation of `pr::exp()`.
 */
template <typename T>
__attribute__((always_inline))
inline dualnum<T> exp(const dualnum<T>& x)
{
    return {
        pr::exp(x.real()),
        pr::exp(x.real()) * x.dual()
    };
}

/**
 * @brief Dual number implementation of `pr::log()`.
 */
template <typename T>
__attribute__((always_inline))
inline dualnum<T> log(const dualnum<T>& x)
{
    return {
        pr::log(x.real()),
        x.dual() / x.real()
    };
}

/**
 * @brief Dual number implementation of `pr::exp2()`.
 */
template <typename T>
__attribute__((always_inline))
inline dualnum<T> exp2(const dualnum<T>& x)
{
    return {
        pr::exp2(x.real()),
        pr::numeric_constants<T>::M_ln2() * pr::exp2(x.real()) * x.dual()
    };
}

/**
 * @brief Dual number implementation of `pr::log2()`.
 */
template <typename T>
__attribute__((always_inline))
inline dualnum<T> log2(const dualnum<T>& x)
{
    return {
        pr::log2(x.real()),
        x.dual() / (pr::numeric_constants<T>::M_ln2() * x.real())
    };
}

/**
 * @brief Dual number implementation of `pr::log10()`.
 */
template <typename T>
__attribute__((always_inline))
inline dualnum<T> log10(const dualnum<T>& x)
{
    return {
        pr::log10(x.real()),
        x.dual() / (pr::numeric_constants<T>::M_ln10() * x.real())
    };
}

/**
 * @brief Dual number implementation of `pr::expm1()`.
 */
template <typename T>
__attribute__((always_inline))
inline dualnum<T> expm1(const dualnum<T>& x)
{
    return {
        pr::expm1(x.real()),
        pr::exp(x.real()) * x.dual()
    };
}

/**
 * @brief Dual number implementation of `pr::log1p()`.
 */
template <typename T>
__attribute__((always_inline))
inline dualnum<T> log1p(const dualnum<T>& x)
{
    return {
        pr::log1p(x.real()),
        x.dual() / (T(1) + x.real())
    };
}

/**
 * @brief Dual number implementation of `pr::sqrt()`.
 */
template <typename T>
__attribute__((always_inline))
inline dualnum<T> sqrt(const dualnum<T>& x)
{
    return {
        pr::sqrt(x.real()),
        x.dual() / (T(2) * pr::sqrt(x.real()))
    };
}

/**
 * @brief Dual number implementation of `pr::cbrt()`.
 */
template <typename T>
__attribute__((always_inline))
inline dualnum<T> cbrt(const dualnum<T>& x)
{
    return {
        pr::cbrt(x.real()),
        x.dual() / (T(3) * pr::nthpow(pr::cbrt(x.real()), 2))
    };
}

/**
 * @brief Dual number implementation of `pr::erf()`.
 */
template <typename T>
__attribute__((always_inline))
inline dualnum<T> erf(const dualnum<T>& x)
{
    return {
        pr::erf(x.real()),
        pr::numeric_constants<T>::M_2_sqrtpi() * pr::exp(-pr::nthpow(x.real(), 2)) * x.dual()
    };
}

/**
 * @brief Dual number implementation of `pr::erfc()`.
 */
template <typename T>
__attribute__((always_inline))
inline dualnum<T> erfc(const dualnum<T>& x)
{
    return {
        pr::erfc(x.real()),
        -pr::numeric_constants<T>::M_2_sqrtpi() * pr::exp(-pr::nthpow(x.real(), 2)) * x.dual()
    };
}

/**
 * @brief Dual number implementation of `pr::sin()`.
 */
template <typename T>
__attribute__((always_inline))
inline dualnum<T> sin(const dualnum<T>& x)
{
    return {
        pr::sin(x.real()),
        pr::cos(x.real()) * x.dual()
    };
}

/**
 * @brief Dual number implementation of `pr::cos()`.
 */
template <typename T>
__attribute__((always_inline))
inline dualnum<T> cos(const dualnum<T>& x)
{
    return {
        pr::cos(x.real()),
        -pr::sin(x.real()) * x.dual()
    };
}

/**
 * @brief Dual number implementation of `pr::tan()`.
 */
template <typename T>
__attribute__((always_inline))
inline dualnum<T> tan(const dualnum<T>& x)
{
    return {
        pr::tan(x.real()),
        x.dual() / pr::nthpow(pr::cos(x.real()), 2)
    };
}

/**
 * @brief Dual number implementation of `pr::asin()`.
 */
template <typename T>
__attribute__((always_inline))
inline dualnum<T> asin(const dualnum<T>& x)
{
    return {
        pr::asin(x.real()),
        x.dual() / pr::sqrt(T(1) - pr::nthpow(x.real(), 2))
    };
}

/**
 * @brief Dual number implementation of `pr::acos()`.
 */
template <typename T>
__attribute__((always_inline))
inline dualnum<T> acos(const dualnum<T>& x)
{
    return {
        pr::acos(x.real()),
        x.dual() / -pr::sqrt(T(1) - pr::nthpow(x.real(), 2))
    };
}

/**
 * @brief Dual number implementation of `pr::atan()`.
 */
template <typename T>
__attribute__((always_inline))
inline dualnum<T> atan(const dualnum<T>& x)
{
    return {
        pr::atan(x.real()),
        x.dual() / (T(1) + pr::nthpow(x.real(), 2))
    };
}

/**
 * @brief Dual number implementation of `pr::sinh()`.
 */
template <typename T>
__attribute__((always_inline))
inline dualnum<T> sinh(const dualnum<T>& x)
{
    return {
        pr::sinh(x.real()),
        pr::cosh(x.real()) * x.dual()
    };
}

/**
 * @brief Dual number implementation of `pr::cosh()`.
 */
template <typename T>
__attribute__((always_inline))
inline dualnum<T> cosh(const dualnum<T>& x)
{
    return {
        pr::cosh(x.real()),
        pr::sinh(x.real()) * x.dual()
    };
}

/**
 * @brief Dual number implementation of `pr::tanh()`.
 */
template <typename T>
__attribute__((always_inline))
inline dualnum<T> tanh(const dualnum<T>& x)
{
    return {
        pr::tanh(x.real()),
        x.dual() / pr::nthpow(pr::cosh(x.real()), 2)
    };
}

/**
 * @brief Dual number implementation of `pr::asinh()`.
 */
template <typename T>
__attribute__((always_inline))
inline dualnum<T> asinh(const dualnum<T>& x)
{
    return {
        pr::asinh(x.real()),
        x.dual() / pr::sqrt(pr::nthpow(x.real(), 2) + T(1))
    };
}

/**
 * @brief Dual number implementation of `pr::acosh()`.
 */
template <typename T>
__attribute__((always_inline))
inline dualnum<T> acosh(const dualnum<T>& x)
{
    return {
        pr::acosh(x.real()),
        x.dual() / pr::sqrt(pr::nthpow(x.real(), 2) - T(1))
    };
}

/**
 * @brief Dual number implementation of `pr::atanh()`.
 */
template <typename T>
__attribute__((always_inline))
inline dualnum<T> atanh(const dualnum<T>& x)
{
    return {
        pr::atanh(x.real()),
        x.dual() / (T(1) - pr::nthpow(x.real(), 2))
    };
}

/**@}*/

/**
 * @name Reciprocal trigonometric functions (dualnum)
 */
/**@{*/

/**
 * @brief Reciprocal of `pr::sin()`.
 */
template <typename T>
__attribute__((always_inline))
inline dualnum<T> csc(const dualnum<T>& x)
{
    return T(1) / pr::sin(x);
}

/**
 * @brief Reciprocal of `pr::cos()`.
 */
template <typename T>
__attribute__((always_inline))
inline dualnum<T> sec(const dualnum<T>& x)
{
    return T(1) / pr::cos(x);
}

/**
 * @brief Reciprocal of `pr::tan()`.
 */
template <typename T>
__attribute__((always_inline))
inline dualnum<T> cot(const dualnum<T>& x)
{
    return T(1) / pr::tan(x);
}

/**
 * @brief Reciprocal of `pr::sinh()`.
 */
template <typename T>
__attribute__((always_inline))
inline dualnum<T> csch(const dualnum<T>& x)
{
    return T(1) / pr::sinh(x);
}

/**
 * @brief Reciprocal of `pr::cosh()`.
 */
template <typename T>
__attribute__((always_inline))
inline dualnum<T> sech(const dualnum<T>& x)
{
    return T(1) / pr::cosh(x);
}

/**
 * @brief Reciprocal of `pr::tanh()`.
 */
template <typename T>
__attribute__((always_inline))
inline dualnum<T> coth(const dualnum<T>& x)
{
    return T(1) / pr::tanh(x);
}

/**
 * @brief Inverse of `pr::csc()`.
 */
template <typename T>
__attribute__((always_inline))
inline dualnum<T> acsc(const dualnum<T>& x)
{
    return pr::asin(T(1) / x);
}

/**
 * @brief Inverse of `pr::sec()`.
 */
template <typename T>
__attribute__((always_inline))
inline dualnum<T> asec(const dualnum<T>& x)
{
    return pr::acos(T(1) / x);
}

/**
 * @brief Inverse of `pr::cot()`.
 */
template <typename T>
__attribute__((always_inline))
inline dualnum<T> acot(const dualnum<T>& x)
{
    return pr::atan(T(1) / x);
}

/**
 * @brief Inverse of `pr::csch()`.
 */
template <typename T>
__attribute__((always_inline))
inline dualnum<T> acsch(const dualnum<T>& x)
{
    return pr::asinh(T(1) / x);
}

/**
 * @brief Inverse of `pr::sech()`.
 */
template <typename T>
__attribute__((always_inline))
inline dualnum<T> asech(const dualnum<T>& x)
{
    return pr::acosh(T(1) / x);
}

/**
 * @brief Inverse of `pr::coth()`.
 */
template <typename T>
__attribute__((always_inline))
inline dualnum<T> acoth(const dualnum<T>& x)
{
    return pr::atanh(T(1) / x);
}

/**@}*/

/**@}*/

