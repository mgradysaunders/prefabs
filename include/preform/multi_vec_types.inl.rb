puts <<STR
namespace pr {

/**
 * @addtogroup multi_vec_types
 */
/**@{*/

STR

for m in [1, 2, 3, 4]
    puts <<STR
/**
 * @brief Vector alias (#{m}-dimensional).
 */
template <typename T>
using vec#{m} = multi<T, #{m}>;

STR
end

for m in [1, 2, 3, 4]
for n in [1, 2, 3, 4]
    puts <<STR
/**
 * @brief Matrix alias (#{m}-by-#{n}-dimensional).
 */
template <typename T>
using mat#{m}x#{n} = multi<T, #{m}, #{n}>;

STR
end
end

for m in [1, 2, 3, 4]
    puts <<STR
/**
 * @brief Matrix alias (#{m}-by-#{m}-dimensional).
 */
template <typename T>
using mat#{m} = multi<T, #{m}, #{m}>;

STR
end

puts <<STR
/**@}*/

} // namespace pr
STR
