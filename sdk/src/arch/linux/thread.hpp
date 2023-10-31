/*
 *  RPLIDAR SDK
 *
 *  Copyright (c) 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 *  Copyright (c) 2014 - 2018 Shanghai Slamtec Co., Ltd.
 *  http://www.slamtec.com
 *
 */
/*
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "arch/linux/arch_linux.h"

#include <sched.h>
#include <sys/types.h>
#include <sys/syscall.h>
#include <sys/time.h>
#include <sys/resource.h>

namespace rp{ namespace hal{

Thread Thread::create(thread_proc_t proc, void * data)
{
    Thread newborn(proc, data);
    
    // tricky code, we assume pthread_t is not a structure but a word size value
    assert( sizeof(newborn._handle) >= sizeof(pthread_t));

    pthread_create((pthread_t *)&newborn._handle, NULL, (void * (*)(void *))proc, data);

    return newborn;
}

u_result Thread::terminate()
{
    if (!this->_handle) return RESULT_OK;
    
    return pthread_cancel((pthread_t)this->_handle)==0?RESULT_OK:RESULT_OPERATION_FAIL;
}

u_result Thread::SetSelfPriority( priority_val_t p)
{

    pid_t selfTid = syscall(SYS_gettid);

        // check whether current schedule policy supports priority levels
    int current_policy = SCHED_OTHER;
    struct sched_param current_param;
    int nice = 0;
    int ans;

    if (sched_getparam(selfTid, &current_param))
    {
        // cannot retreieve values
        return RESULT_OPERATION_FAIL;
    }   

    int pthread_priority_min;

#if 1
    pthread_priority_min = sched_get_priority_min(SCHED_RR);
#else
    pthread_priority_min = 1;
#endif
	int pthread_priority = 0 ;

	switch(p)
	{
	case PRIORITY_REALTIME:
		//pthread_priority = pthread_priority_max;
        current_policy = SCHED_RR;
        pthread_priority = pthread_priority_min + 1;
        nice = 0;
		break;
	case PRIORITY_HIGH:
		//pthread_priority = (pthread_priority_max + pthread_priority_min)/2;
        current_policy = SCHED_RR;
        pthread_priority = pthread_priority_min;
        nice = 0;
		break;
	case PRIORITY_NORMAL:
        pthread_priority = 0;
        current_policy = SCHED_OTHER;
        nice = 0;
        break;
	case PRIORITY_LOW:
        pthread_priority = 0;
        current_policy = SCHED_OTHER;
        nice = 10;
        break;
	case PRIORITY_IDLE:
		pthread_priority = 0;
        current_policy = SCHED_IDLE;
        nice = 0;
		break;
	}
    // change the inhertiable behavior
    current_policy |= SCHED_RESET_ON_FORK;

    current_param.__sched_priority = pthread_priority;

  

    
    // do not use pthread version as it will make the priority be inherited by a thread child
	if ( (ans = sched_setscheduler(selfTid, current_policy , &current_param)) )
	{
        if (ans == EPERM)
        {
            //DBG_PRINT("warning, current process hasn't the right permission to set threads priority\n");
        }
		return RESULT_OPERATION_FAIL;
	}


    if ((current_policy == SCHED_OTHER) || (current_policy == SCHED_BATCH))
    {
        if (setpriority(PRIO_PROCESS, selfTid, nice)) {
            return RESULT_OPERATION_FAIL;
        }
    }
    

	return  RESULT_OK;
}

Thread::priority_val_t Thread::getPriority()
{
	if (!this->_handle) return PRIORITY_NORMAL;

    int current_policy;
    struct sched_param current_param;
    if (pthread_getschedparam( (pthread_t) this->_handle, &current_policy, &current_param))
    {
        // cannot retreieve values
        return PRIORITY_NORMAL;
    }   

    int pthread_priority_max = sched_get_priority_max(SCHED_RR);
    int pthread_priority_min = sched_get_priority_min(SCHED_RR);

    if (current_param.__sched_priority ==(pthread_priority_max ))
	{
		return PRIORITY_REALTIME;
	}
	if (current_param.__sched_priority >=(pthread_priority_max + pthread_priority_min)/2)
	{
		return PRIORITY_HIGH;
	}
	return PRIORITY_NORMAL;
}

u_result Thread::join(unsigned long timeout)
{
    if (!this->_handle) return RESULT_OK;
    
    pthread_join((pthread_t)(this->_handle), NULL);
    this->_handle = 0;
    return RESULT_OK;
}

}}
