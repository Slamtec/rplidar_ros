#ifndef _INFRA_HAL_ASSERT_H
#define _INFRA_HAL_ASSERT_H

#ifdef WIN32
#include <crtdbg.h>
#ifndef assert
#define assert(x) _ASSERT(x)
#endif
#elif defined(_AVR_)
#define assert(x) 
#elif defined(__GNUC__)
#ifndef assert
#define assert(x) 
#endif
#else
#error assert.h cannot identify your platform
#endif 
#endif
