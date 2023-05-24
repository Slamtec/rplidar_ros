/*
 *      Common Data Types for RP
 */

#ifndef _INFRA_HAL_TYPES_H_
#define _INFRA_HAL_TYPES_H_

//Basic types
//
#ifdef WIN32

//fake stdint.h for VC only

typedef signed   char     int8_t;
typedef unsigned char     uint8_t;

typedef __int16           int16_t;
typedef unsigned __int16  uint16_t;

typedef __int32           int32_t;
typedef unsigned __int32  uint32_t;

typedef __int64           int64_t;
typedef unsigned __int64  uint64_t;


#define RPMODULE_EXPORT  __declspec(dllexport)
#define RPMODULE_IMPORT  __declspec(dllimport)

#else

#include <stdint.h>

#define RPMODULE_EXPORT
#define RPMODULE_IMPORT

#endif


//based on stdint.h
typedef int8_t         _s8;
typedef uint8_t        _u8;

typedef int16_t        _s16;
typedef uint16_t       _u16;

typedef int32_t        _s32;
typedef uint32_t       _u32;

typedef int64_t        _s64;
typedef uint64_t       _u64;

#define __small_endian

#ifndef __GNUC__
#define __attribute__(x)
#endif


// The _word_size_t uses actual data bus width of the current CPU
#ifdef _AVR_
typedef _u8            _word_size_t;
#define THREAD_PROC    
#elif defined (WIN64)
typedef _u64           _word_size_t;
#define THREAD_PROC    __stdcall
#elif defined (WIN32)
typedef _u32           _word_size_t;
#define THREAD_PROC    __stdcall
#elif defined (__GNUC__)
typedef unsigned long  _word_size_t;
#define THREAD_PROC   
#elif defined (__ICCARM__)
typedef _u32            _word_size_t;
#define THREAD_PROC  
#endif

#define _multi_thread
#define _single_thread

typedef uint32_t u_result;

#define RESULT_OK                       0
#define RESULT_FAIL_BIT                 0x80000000
#define RESULT_ALREADY_DONE             0x20
#define RESULT_INVALID_DATA             (0x8000 | RESULT_FAIL_BIT)
#define RESULT_OPERATION_FAIL           (0x8001 | RESULT_FAIL_BIT)
#define RESULT_OPERATION_TIMEOUT        (0x8002 | RESULT_FAIL_BIT)
#define RESULT_OPERATION_STOP           (0x8003 | RESULT_FAIL_BIT)
#define RESULT_OPERATION_NOT_SUPPORT    (0x8004 | RESULT_FAIL_BIT)
#define RESULT_FORMAT_NOT_SUPPORT       (0x8005 | RESULT_FAIL_BIT)
#define RESULT_INSUFFICIENT_MEMORY      (0x8006 | RESULT_FAIL_BIT)
#define RESULT_OPERATION_ABORTED        (0x8007 | RESULT_FAIL_BIT)
#define RESULT_NOT_FOUND                (0x8008 | RESULT_FAIL_BIT)
#define RESULT_RECONNECTING             (0x8009 | RESULT_FAIL_BIT)

#define IS_OK(x)    ( ((x) & RESULT_FAIL_BIT) == 0 )
#define IS_FAIL(x)  ( ((x) & RESULT_FAIL_BIT) )


typedef _word_size_t (THREAD_PROC * thread_proc_t ) ( void * );


#if defined (_BUILD_AS_DLL)
#if defined (_BUILD_DLL_EXPORT)
#define RPMODULE_IMPEXP  RPMODULE_EXPORT
#else
#define RPMODULE_IMPEXP  RPMODULE_IMPORT
#endif
#else
#define RPMODULE_IMPEXP 
#endif

#endif
