//*****************************************************************************
//
//  remote_hex_codes.h
//
//  Defines the Macros for all of the remote IR signal hex codes (to each button).
//  (Lab 3).
//
//*****************************************************************************
#ifndef __REMOTE_HEX_CODES_H__
#define __REMOTE_HEX_CODES_H__

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
#define BUTTON_0        0xDF20F708
#define BUTTON_1        0xDF207788
#define BUTTON_2        0xDF20B748
#define BUTTON_3        0xDF2037C8
#define BUTTON_4        0xDF20D728
#define BUTTON_5        0xDF2057A8
#define BUTTON_6        0xDF209768
#define BUTTON_7        0xDF2017E8
#define BUTTON_8        0xDF20E718
#define BUTTON_9        0xDF206798
#define BUTTON_LAST     0xDF20A758
#define BUTTON_MUTE     0xDF20AF50


//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif


#endif // __REMOTE_HEX_CODES_H__
