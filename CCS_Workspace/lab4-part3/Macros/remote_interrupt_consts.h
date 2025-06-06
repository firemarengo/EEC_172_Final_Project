//*****************************************************************************
//
//  remote_interrupt_consts.h
//
//  Defines the Macros for all of the remote IR signal analysis (Lab 3).
//
//*****************************************************************************
#ifndef __REMOTE_INTERRUPT_CONSTS_H__
#define __REMOTE_INTERRUPT_CONSTS_H__

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif


//*****************************************************************************
//                      MACRO DEFINITIONS
//*****************************************************************************
#define FOREVER                     1
#define TIMER_PERIOD                50      // in microseconds.
#define SYSCLKFREQ                  80000000ULL
#define BUFF_SIZE                   1000    // for string typing message buffer.

// macro to convert ticks to microseconds
#define TICKS_TO_US(ticks) \
    ((((ticks) / SYSCLKFREQ) * 1000000ULL) + \
    ((((ticks) % SYSCLKFREQ) * 1000000ULL) / SYSCLKFREQ))\

// macro to convert microseconds to ticks
#define US_TO_TICKS(us) ((SYSCLKFREQ / 1000000ULL) * (us))

// systick reload value set to 10ms period
// (PERIOD_SEC) * (SYSCLKFREQ) = PERIOD_TICKS
//
// (Needs to be calculated manually if changed.)
#define SYSTICK_RELOAD_VAL 800000UL

// Tolerance for multiple button presses, in microseconds.
#define TOL_US 500000UL

// Tolerance for multiple button presses, in "timer interrupt counts."
// (We should be using these macros in main().)
#define TOL    TOL_US / TIMER_PERIOD

/** Macros for pulse width definitions. **/
// Max/min pulse widths for "start", "0", and "1", in microseconds.
#define START_MAX_PW_US    4500
#define START_MIN_PW_US    4250
#define ZERO_MAX_PW_US     1650
#define ZERO_MIN_PW_US     1600
#define ONE_MAX_PW_US      600
#define ONE_MIN_PW_US      500

// Max/min pulse widths for "start", "0", and "1", in "timer interrupt counts."
// (We should be using these macros in main().)
#define START_MAX_PW       START_MAX_PW_US / TIMER_PERIOD
#define START_MIN_PW       START_MIN_PW_US / TIMER_PERIOD
#define ZERO_MAX_PW        ZERO_MAX_PW_US  / TIMER_PERIOD
#define ZERO_MIN_PW        ZERO_MIN_PW_US  / TIMER_PERIOD
#define ONE_MAX_PW         ONE_MAX_PW_US   / TIMER_PERIOD
#define ONE_MIN_PW         ONE_MIN_PW_US   / TIMER_PERIOD


//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif


#endif // __REMOTE_INTERRUPT_CONSTS_H__
