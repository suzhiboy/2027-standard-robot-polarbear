#ifndef STRUCT_TYPEDEF_H
#define STRUCT_TYPEDEF_H

#include <stdint.h>

/* 数学常量加保护：math.h 或其他头文件若已定义同名宏则沿用，避免重定义冲突 */
#ifndef M_E
#define M_E 2.7182818284590452354f
#endif
#ifndef M_LOG2E
#define M_LOG2E 1.4426950408889634074f
#endif
#ifndef M_LOG10E
#define M_LOG10E 0.43429448190325182765f
#endif
#ifndef M_LN2
#define M_LN2 0.69314718055994530942f
#endif
#ifndef M_LN10
#define M_LN10 2.30258509299404568402f
#endif
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923f
#endif
#ifndef M_PI_4
#define M_PI_4 0.78539816339744830962f
#endif
#ifndef M_1_PI
#define M_1_PI 0.31830988618379067154f
#endif
#ifndef M_2_PI
#define M_2_PI 0.63661977236758134308f
#endif
#ifndef M_2_SQRTPI
#define M_2_SQRTPI 1.12837916709551257390f
#endif
#ifndef M_SQRT2
#define M_SQRT2 1.41421356237309504880f
#endif
#ifndef M_SQRT1_2
#define M_SQRT1_2 0.70710678118654752440f
#endif

#define GRAVITY 9.8f

/* int8_t/uint8_t 等精确宽度整数类型统一由 <stdint.h> 提供，不再重复定义 */
typedef unsigned char bool_t;
typedef float fp32;
typedef double fp64;

#endif
