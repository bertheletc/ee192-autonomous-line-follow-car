/*----------------------------------------------------------------------------
 *      CMSIS-RTOS  -  RTX
 *----------------------------------------------------------------------------
 *      Name:    RTX_Conf_CM.C
 *      Purpose: Configuration of CMSIS RTX Kernel for Cortex-M
 *      Rev.:    V4.70.1
 *----------------------------------------------------------------------------
 *
 * Copyright (c) 1999-2009 KEIL, 2009-2015 ARM Germany GmbH
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  - Neither the name of ARM  nor the names of its contributors may be used
 *    to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *---------------------------------------------------------------------------*/

#include "cmsis_os.h"


/*----------------------------------------------------------------------------
 *      RTX User configuration part BEGIN
 *---------------------------------------------------------------------------*/

//-------- <<< Use Configuration Wizard in Context Menu >>> -----------------
//
// <h>Thread Configuration
// =======================
//
//   <o>Number of concurrent running user threads <1-250>
//   <i> Defines max. number of user threads that will run at the same time.
//   <i> Default: 6
#ifndef OS_TASKCNT
#  if   defined(TARGET_LPC1768) || defined(TARGET_LPC2368)   || defined(TARGET_LPC4088) || defined(TARGET_LPC4088_DM) || defined(TARGET_LPC4330) || defined(TARGET_LPC4337) || defined(TARGET_LPC1347) || defined(TARGET_K64F) || defined(TARGET_STM32F401RE)\
   || defined(TARGET_STM32F410RB) || defined(TARGET_KL46Z) || defined(TARGET_KL43Z)  || defined(TARGET_STM32F407) || defined(TARGET_F407VG)  || defined(TARGET_STM32F303VC) || defined(TARGET_LPC1549) || defined(TARGET_LPC11U68) \
   || defined(TARGET_STM32F411RE) || defined(TARGET_STM32F405RG) || defined(TARGET_K22F) || defined(TARGET_STM32F429ZI) || defined(TARGET_STM32F401VC) || defined(TARGET_MAX32610) || defined(TARGET_MAX32600) || defined(TARGET_TEENSY3_1) \
   || defined(TARGET_STM32L152RE) || defined(TARGET_STM32F446RE) || defined(TARGET_STM32F446VE) || defined(TARGET_STM32L476VG) || defined(TARGET_STM32L476RG) || defined(TARGET_STM32F469NI) || defined(TARGET_STM32F746NG) || defined(TARGET_STM32F746ZG) || defined(TARGET_STM32L152RC)
#    define OS_TASKCNT         14
#  elif defined(TARGET_LPC11U24) || defined(TARGET_STM32F303RE) || defined(TARGET_STM32F303K8) || defined(TARGET_LPC11U35_401)  || defined(TARGET_LPC11U35_501) || defined(TARGET_LPCCAPPUCCINO) || defined(TARGET_LPC1114) \
   || defined(TARGET_LPC812)   || defined(TARGET_KL25Z)         || defined(TARGET_KL26Z)       || defined(TARGET_KL27Z)         || defined(TARGET_KL05Z)        || defined(TARGET_STM32F100RB)  || defined(TARGET_STM32F051R8) \
   || defined(TARGET_STM32F103RB) || defined(TARGET_LPC824) || defined(TARGET_STM32F302R8) || defined(TARGET_STM32F334R8) || defined(TARGET_STM32F334C8) \
   || defined(TARGET_STM32L031K6) || defined(TARGET_STM32L053R8) || defined(TARGET_STM32L053C8) || defined(TARGET_STM32L073RZ) || defined(TARGET_STM32F072RB) || defined(TARGET_STM32F091RC) || defined(TARGET_NZ32_SC151) \
   || defined(TARGET_SSCI824)  || defined(TARGET_STM32F030R8) || defined(TARGET_STM32F070RB)
#    define OS_TASKCNT         6
#  else
#    error "no target defined"
#  endif
#endif

#ifdef __MBED_CMSIS_RTOS_CM
//   <o>Idle stack size [bytes] <64-4096:8><#/4>
//   <i> Defines default stack size for the Idle thread.
#ifndef OS_IDLESTKSIZE
 #define OS_IDLESTKSIZE 128
#endif
#else // __MBED_CMSIS_RTOS_CM
//   <o>Default Thread stack size [bytes] <64-4096:8><#/4>
//   <i> Defines default stack size for threads with osThreadDef stacksz = 0
//   <i> Default: 200
#ifndef OS_STKSIZE
 #define OS_STKSIZE     200
#endif
#endif // __MBED_CMSIS_RTOS_CM

//   <o>Main Thread stack size [bytes] <64-32768:8><#/4>
#ifndef OS_MAINSTKSIZE
#  if   defined(TARGET_LPC1768) || defined(TARGET_LPC2368)   || defined(TARGET_LPC4088) || defined(TARGET_LPC4088_DM) || defined(TARGET_LPC4330) || defined(TARGET_LPC4337) || defined(TARGET_LPC1347)  || defined(TARGET_K64F) || defined(TARGET_STM32F401RE)\
   || defined(TARGET_STM32F410RB) || defined(TARGET_KL46Z) || defined(TARGET_KL43Z) || defined(TARGET_STM32F407) || defined(TARGET_F407VG)  || defined(TARGET_STM32F303VC) || defined(TARGET_LPC1549) || defined(TARGET_LPC11U68) \
   || defined(TARGET_STM32F411RE) || defined(TARGET_STM32F405RG) || defined(TARGET_K22F) || defined(TARGET_STM32F429ZI) || defined(TARGET_STM32F401VC) || defined(TARGET_MAX32610) || defined(TARGET_MAX32600) || defined(TARGET_TEENSY3_1) \
   || defined(TARGET_STM32L152RE) || defined(TARGET_STM32F446RE) || defined(TARGET_STM32F446VE) || defined(TARGET_STM32L476VG) || defined(TARGET_STM32L476RG) || defined(TARGET_STM32F469NI) || defined(TARGET_STM32F746NG) || defined(TARGET_STM32F746ZG) || defined(TARGET_STM32L152RC)
#      define OS_MAINSTKSIZE    256
#  elif defined(TARGET_LPC11U24) || defined(TARGET_LPC11U35_401)  || defined(TARGET_LPC11U35_501) || defined(TARGET_LPCCAPPUCCINO)  || defined(TARGET_LPC1114) \
   || defined(TARGET_LPC812)   || defined(TARGET_KL25Z)         || defined(TARGET_KL26Z)        || defined(TARGET_KL27Z)        || defined(TARGET_KL05Z)        || defined(TARGET_STM32F100RB)  || defined(TARGET_STM32F051R8) \
   || defined(TARGET_STM32F103RB) || defined(TARGET_LPC824) || defined(TARGET_STM32F302R8) || defined(TARGET_STM32F072RB) || defined(TARGET_STM32F091RC) || defined(TARGET_NZ32_SC151) \
   || defined(TARGET_SSCI824) || defined(TARGET_STM32F030R8) || defined(TARGET_STM32F070RB)
#      define OS_MAINSTKSIZE    128
#  elif defined(TARGET_STM32F334R8) || defined(TARGET_STM32F303RE) ||  defined(TARGET_STM32F303K8) ||  defined(TARGET_STM32F334C8) || defined(TARGET_STM32L031K6) || defined(TARGET_STM32L053R8) || defined(TARGET_STM32L053C8) || defined(TARGET_STM32L073RZ)
#      define OS_MAINSTKSIZE    112
#  else
#    error "no target defined"
#  endif
#endif

#ifndef __MBED_CMSIS_RTOS_CM
//   <o>Number of threads with user-provided stack size <0-250>
//   <i> Defines the number of threads with user-provided stack size.
//   <i> Default: 0
#ifndef OS_PRIVCNT
 #define OS_PRIVCNT     0
#endif
 
//   <o>Total stack size [bytes] for threads with user-provided stack size <0-1048576:8><#/4>
//   <i> Defines the combined stack size for threads with user-provided stack size.
//   <i> Default: 0
#ifndef OS_PRIVSTKSIZE
 #define OS_PRIVSTKSIZE 0       // this stack size value is in words
#endif
#endif // __MBED_CMSIS_RTOS_CM

//   <q>Stack overflow checking
//   <i> Enable stack overflow checks at thread switch.
//   <i> Enabling this option increases slightly the execution time of a thread switch.
#ifndef OS_STKCHECK
 #define OS_STKCHECK    1
#endif
 
//   <q>Stack usage watermark
//   <i> Initialize thread stack with watermark pattern for analyzing stack usage (current/maximum) in System and Thread Viewer.
//   <i> Enabling this option increases significantly the execution time of osThreadCreate.
#ifndef OS_STKINIT
#define OS_STKINIT      0
#endif
 
//   <o>Processor mode for thread execution 
//     <0=> Unprivileged mode 
//     <1=> Privileged mode
//   <i> Default: Privileged mode
#ifndef OS_RUNPRIV
 #define OS_RUNPRIV     1
#endif

// </h>
 
// <h>RTX Kernel Timer Tick Configuration
// ======================================
//   <q> Use Cortex-M SysTick timer as RTX Kernel Timer
//   <i> Cortex-M processors provide in most cases a SysTick timer that can be used as 
//   <i> as time-base for RTX.
#ifndef OS_SYSTICK
 #define OS_SYSTICK     1
#endif
//
//   <o>RTOS Kernel Timer input clock frequency [Hz] <1-1000000000>
//   <i> Defines the input frequency of the RTOS Kernel Timer.  
//   <i> When the Cortex-M SysTick timer is used, the input clock 
//   <i> is on most systems identical with the core clock.
#ifndef OS_CLOCK
#  if defined(TARGET_LPC1768) || defined(TARGET_LPC2368) || defined(TARGET_TEENSY3_1)
#    define OS_CLOCK       96000000

#  elif defined(TARGET_LPC1347) || defined(TARGET_STM32F303VC) || defined(TARGET_LPC1549) || defined(TARGET_STM32F334R8) || defined(TARGET_STM32F334C8) || defined(TARGET_STM32F303RE)
#    define OS_CLOCK       72000000

#  elif defined(TARGET_STM32F303K8)
#    define OS_CLOCK       64000000

#  elif defined(TARGET_LPC11U24) || defined(TARGET_LPC11U35_401)  || defined(TARGET_LPC11U35_501) || defined(TARGET_LPCCAPPUCCINO)  || defined(TARGET_LPC1114) || defined(TARGET_KL25Z) \
     || defined(TARGET_KL26Z) || defined(TARGET_KL27Z) || defined(TARGET_KL05Z) || defined(TARGET_KL46Z) || defined(TARGET_KL43Z) || defined(TARGET_STM32F051R8) || defined(TARGET_LPC11U68) || defined(TARGET_STM32F072RB) || defined(TARGET_STM32F091RC)
#    define OS_CLOCK       48000000

#  elif defined(TARGET_LPC812)
#    define OS_CLOCK       36000000

#  elif defined(TARGET_LPC824) || defined(TARGET_SSCI824)
#    define OS_CLOCK       30000000

#  elif  defined(TARGET_STM32F100RB)
#    define OS_CLOCK       24000000

#  elif defined(TARGET_LPC4088) || defined(TARGET_LPC4088_DM) || defined(TARGET_K64F) || defined(TARGET_K22F)
#    define OS_CLOCK       120000000

#  elif defined(TARGET_LPC4330)
#    define OS_CLOCK       204000000

#  elif defined(TARGET_LPC4337)
#    define OS_CLOCK       204000000

#  elif defined(TARGET_STM32F407) || defined(TARGET_F407VG)
#    define OS_CLOCK       168000000

#  elif defined(TARGET_STM32F401RE)
#    define OS_CLOCK       84000000

#  elif defined(TARGET_STM32F411RE)
#     define OS_CLOCK      100000000

#  elif defined(TARGET_STM32F410RB)
#     define OS_CLOCK      100000000

#elif defined(TARGET_STM32F103RB)
#    define OS_CLOCK       72000000

#elif defined(TARGET_STM32F429ZI)
#    define OS_CLOCK       168000000

#elif defined(TARGET_STM32F302R8)
#    define OS_CLOCK       72000000

#elif defined(TARGET_STM32L031K6) || defined(TARGET_STM32L053R8) || defined(TARGET_STM32L053C8) || defined(TARGET_STM32L073RZ)
#    define OS_CLOCK       32000000

#elif defined(TARGET_STM32F401VC)
#    define OS_CLOCK       84000000

#  elif defined(TARGET_STM32F746NG) || defined(TARGET_STM32F746ZG)
#     define OS_CLOCK      216000000

#elif defined(TARGET_MAX32610) || defined(TARGET_MAX32600)
#    define OS_CLOCK       24000000

#elif defined(TARGET_NZ32_SC151)
#    define OS_CLOCK       32000000

#elif defined(TARGET_STM32L152RE)
#    define OS_CLOCK       24000000

#elif (defined(TARGET_STM32F446RE) || defined(TARGET_STM32F446VE))
#    define OS_CLOCK       180000000

#elif defined(TARGET_STM32F030R8)
#    define OS_CLOCK       48000000

#elif defined(TARGET_STM32F070RB)
#    define OS_CLOCK       48000000

#elif defined(TARGET_STM32L476VG) || defined(TARGET_STM32L476RG)
#    define OS_CLOCK       80000000

#elif defined(TARGET_STM32F469NI)
#    define OS_CLOCK       168000000

#elif defined(TARGET_STM32L152RC)
#    define OS_CLOCK       24000000

#  else
#    error "no target defined"
#  endif
#endif
 
//   <o>RTX Timer tick interval value [us] <1-1000000>
//   <i> The RTX Timer tick interval value is used to calculate timeout values.
//   <i> When the Cortex-M SysTick timer is enabled, the value also configures the SysTick timer.
//   <i> Default: 1000  (1ms)
#ifndef OS_TICK
 #define OS_TICK        1000
#endif

// </h>

// <h>System Configuration
// =======================
//
// <e>Round-Robin Thread switching
// ===============================
//
// <i> Enables Round-Robin Thread switching.
#ifndef OS_ROBIN
 #define OS_ROBIN       1
#endif

//   <o>Round-Robin Timeout [ticks] <1-1000>
//   <i> Defines how long a thread will execute before a thread switch.
//   <i> Default: 5
#ifndef OS_ROBINTOUT
 #define OS_ROBINTOUT   5
#endif

// </e>

// <e>User Timers
// ==============
//   <i> Enables user Timers
#ifndef OS_TIMERS
 #define OS_TIMERS      1
#endif

//   <o>Timer Thread Priority
//                        <1=> Low
//     <2=> Below Normal  <3=> Normal  <4=> Above Normal
//                        <5=> High
//                        <6=> Realtime (highest)
//   <i> Defines priority for Timer Thread
//   <i> Default: High
#ifndef OS_TIMERPRIO
 #define OS_TIMERPRIO   5
#endif
 
//   <o>Timer Thread stack size [bytes] <64-4096:8><#/4>
//   <i> Defines stack size for Timer thread.
//   <i> Default: 200
#ifndef OS_TIMERSTKSZ
 #define OS_TIMERSTKSZ  200
#endif
 
//   <o>Timer Callback Queue size <1-32>
//   <i> Number of concurrent active timer callback functions.
//   <i> Default: 4
#ifndef OS_TIMERCBQS
 #define OS_TIMERCBQS   4
#endif

// </e>

//   <o>ISR FIFO Queue size<4=>   4 entries  <8=>   8 entries
//                         <12=> 12 entries  <16=> 16 entries
//                         <24=> 24 entries  <32=> 32 entries
//                         <48=> 48 entries  <64=> 64 entries
//                         <96=> 96 entries
//   <i> ISR functions store requests to this buffer,
//   <i> when they are called from the interrupt handler.
//   <i> Default: 16 entries
#ifndef OS_FIFOSZ
 #define OS_FIFOSZ      16
#endif

// </h>

//------------- <<< end of configuration section >>> -----------------------

// Standard library system mutexes
// ===============================
//  Define max. number system mutexes that are used to protect
//  the arm standard runtime library. For microlib they are not used.
#ifndef OS_MUTEXCNT
 #define OS_MUTEXCNT    12
#endif

/*----------------------------------------------------------------------------
 *      RTX User configuration part END
 *---------------------------------------------------------------------------*/

#define OS_TRV          ((uint32_t)(((double)OS_CLOCK*(double)OS_TICK)/1E6)-1)


/*----------------------------------------------------------------------------
 *      OS Idle daemon
 *---------------------------------------------------------------------------*/
extern void rtos_idle_loop(void);

void os_idle_demon (void) {
    /* The idle demon is a system thread, running when no other thread is      */
    /* ready to run.                                                           */
    rtos_idle_loop();
}

/*----------------------------------------------------------------------------
 *      RTX Errors
 *---------------------------------------------------------------------------*/
extern void error(const char* format, ...);
extern osThreadId svcThreadGetId (void);

void os_error (uint32_t err_code) {
    /* This function is called when a runtime error is detected. Parameter     */
    /* 'err_code' holds the runtime error code (defined in RTX_Config.h).      */
    osThreadId err_task = svcThreadGetId();
    error("RTX error code: 0x%08X, task ID: 0x%08X\n", err_code, err_task);
}

void sysThreadError(osStatus status) {
    if (status != osOK) {
        osThreadId err_task = svcThreadGetId();
        error("CMSIS-RTOS error status: 0x%08X, task ID: 0x%08X\n", status, err_task);
    }
}

/*----------------------------------------------------------------------------
 *      RTX Configuration Functions
 *---------------------------------------------------------------------------*/

#include "RTX_CM_lib.h"

/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/
