/*
 *  RoboPeak Project
 *  Copyright 2009 - 2013
 *  
 *  RPOS - Endianness Helper
 *
 */

#pragma once


#if !defined(_CPU_ENDIAN_BIG) && !defined(_CPU_ENDIAN_SMALL)
// CPU Endianness is not specified, assume little endian.
#define _CPU_ENDIAN_SMALL
#endif

#if defined(_CPU_ENDIAN_BIG) && defined(_CPU_ENDIAN_SMALL)
#error "_CPU_ENDIAN_BIG and _CPU_ENDIAN_SMALL cannot be defined at the same time."
#endif

#include "hal/byteops.h"

#if defined(_CPU_ENDIAN_SMALL)


// we don't want to conflict with the Linux kernel...
#ifndef __KERNEL__ 
#define constant_cpu_to_le64(x) ((_u64)(x))
#define constant_le64_to_cpu(x) ((_u64)(x))
#define constant_cpu_to_le32(x) ((_u32)(x))
#define constant_le32_to_cpu(x) ((_u32)(x))
#define constant_cpu_to_le16(x) ((_u16)(x))
#define constant_le16_to_cpu(x) ((_u16)(x))
#define constant_cpu_to_be64(x) (__static_byteswap_64((x)))
#define constant_be64_to_cpu(x) __static_byteswap_64((_u64)(x))
#define constant_cpu_to_be32(x) (__static_byteswap_32((x)))
#define constant_be32_to_cpu(x) __static_byteswap_32((_u32)(x))
#define constant_cpu_to_be16(x) (__static_byteswap_16((x)))
#define constant_be16_to_cpu(x) __static_byteswap_16((_u16)(x))

#define cpu_to_le64(x) ((_u64)(x))
#define le64_to_cpu(x) ((_u64)(x))
#define cpu_to_le32(x) ((_u32)(x))
#define le32_to_cpu(x) ((_u32)(x))
#define cpu_to_le16(x) ((_u16)(x))
#define le16_to_cpu(x) ((_u16)(x))
#define cpu_to_be64(x) (__byteswap_64((x)))
#define be64_to_cpu(x) __byteswap_64((_u64)(x))
#define cpu_to_be32(x) (__byteswap_32((x)))
#define be32_to_cpu(x) __byteswap_32((_u32)(x))
#define cpu_to_be16(x) (__byteswap_16((x)))
#define be16_to_cpu(x) __byteswap_16((_u16)(x))
#endif

#define cpu_to_float_le(x) ((float)x)
#define float_le_to_cpu(x) ((float)x)

#define cpu_to_float_be(x) __byteswap_float(x)
#define float_be_to_cpu(x) __byteswap_float(x)

#define cpu_to_double_le(x) ((double)x)
#define double_le_to_cpu(x) ((double)x)

#define cpu_to_double_be(x) __byteswap_double(x)
#define double_be_to_cpu(x) __byteswap_double(x)

#else

// we don't want to conflict with the Linux kernel...
#ifndef __KERNEL__ 
#define constant_cpu_to_le64(x) (__static_byteswap_64((x)))
#define constant_le64_to_cpu(x) __static_byteswap_64((_u64)(x))
#define constant_cpu_to_le32(x) (__static_byteswap_32((x)))
#define constant_le32_to_cpu(x) __static_byteswap_32((_u32)(x))
#define constant_cpu_to_le16(x) (__static_byteswap_16((x)))
#define constant_le16_to_cpu(x) __static_byteswap_16((_u16)(x))
#define constant_cpu_to_be64(x) ((_u64)(x))
#define constant_be64_to_cpu(x) ((_u64)(x))
#define constant_cpu_to_be32(x) ((_u32)(x))
#define constant_be32_to_cpu(x) ((_u32)(x))
#define constant_cpu_to_be16(x) ((_u16)(x))
#define constant_be16_to_cpu(x) ((_u16)(x))

#define cpu_to_le64(x) (__byteswap_64((x)))
#define le64_to_cpu(x) __byteswap_64((_u64)(x))
#define cpu_to_le32(x) (__byteswap_32((x)))
#define le32_to_cpu(x) __byteswap_32((_u32)(x))
#define cpu_to_le16(x) (__byteswap_16((x)))
#define le16_to_cpu(x) __byteswap_16((_u16)(x))
#define cpu_to_be64(x) ((_u64)(x))
#define be64_to_cpu(x) ((_u64)(x))
#define cpu_to_be32(x) ((_u32)(x))
#define be32_to_cpu(x) ((_u32)(x))
#define cpu_to_be16(x) ((_u16)(x))
#define be16_to_cpu(x) ((_u16)(x))
#endif


#define cpu_to_float_le(x) __byteswap_float(x)
#define float_le_to_cpu(x) __byteswap_float(x)

#define cpu_to_float_be(x) ((float)x)
#define float_be_to_cpu(x) ((float)x)


#define cpu_to_double_le(x) __byteswap_double(x)
#define double_le_to_cpu(x) __byteswap_double(x)

#define cpu_to_double_be(x) ((double)x)
#define double_be_to_cpu(x) ((double)x)

#endif
