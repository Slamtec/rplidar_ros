/*
 *  RoboPeak Project
 *  Copyright 2009 - 2013
 *  
 *  RPOS - Byte Operations
 *
 */

#pragma once

// byte swapping operations for compiling time

#define __static_byteswap_16(x)  ((_u16)(				\
	(((_u16)(x) & (_u16)0x00FFU) << 8) |			\
	(((_u16)(x) & (_u16)0xFF00U) >> 8)))

#define __static_byteswap_32(x) ((_u32)(				\
	(((_u32)(x) & (_u32)0x000000FFUL) << 24) |		\
	(((_u32)(x) & (_u32)0x0000FF00UL) <<  8) |		\
	(((_u32)(x) & (_u32)0x00FF0000UL) >>  8) |		\
	(((_u32)(x) & (_u32)0xFF000000UL) >> 24)))

#define __static_byteswap_64(x) ((_u64)(				\
	(((_u64)(x) & (_u64)0x00000000000000ffULL) << 56) |	\
	(((_u64)(x) & (_u64)0x000000000000ff00ULL) << 40) |	\
	(((_u64)(x) & (_u64)0x0000000000ff0000ULL) << 24) |	\
	(((_u64)(x) & (_u64)0x00000000ff000000ULL) <<  8) |	\
	(((_u64)(x) & (_u64)0x000000ff00000000ULL) >>  8) |	\
	(((_u64)(x) & (_u64)0x0000ff0000000000ULL) >> 24) |	\
	(((_u64)(x) & (_u64)0x00ff000000000000ULL) >> 40) |	\
	(((_u64)(x) & (_u64)0xff00000000000000ULL) >> 56)))


#define __fast_swap(a, b) do { (a) ^= (b); (b) ^= (a); (a) ^= (b); } while(0)


static inline _u16 __byteswap_16(_u16 x)
{
#ifdef __arch_byteswap_16
    return __arch_byteswap_16(x);
#else
    return __static_byteswap_16(x);
#endif
}

static inline _u32 __byteswap_32(_u32 x)
{
#ifdef __arch_byteswap_32
    return __arch_byteswap_32(x);
#else
    return __static_byteswap_32(x);
#endif
}

static inline _u64 __byteswap_64(_u64 x)
{
#ifdef __arch_byteswap_64
    return __arch_byteswap_64(x);
#else
    return __static_byteswap_64(x);
#endif
}


#ifdef float
static inline float __byteswap_float(float x)
{
#ifdef __arch_byteswap_float
    return __arch_byteswap_float(x);
#else
    _u8 * raw = (_u8 *)&x;
    __fast_swap(raw[0], raw[3]);
    __fast_swap(raw[1], raw[2]);
    return x;
#endif
}
#endif


#ifdef double
static inline double __byteswap_double(double x)
{
#ifdef __arch_byteswap_double
    return __arch_byteswap_double(x);
#else
    _u8 * raw = (_u8 *)&x;
    __fast_swap(raw[0], raw[7]);
    __fast_swap(raw[1], raw[6]);
    __fast_swap(raw[2], raw[5]);
    __fast_swap(raw[3], raw[4]);
    return x;
#endif
}
#endif
