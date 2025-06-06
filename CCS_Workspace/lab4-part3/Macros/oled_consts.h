//*****************************************************************************
//
//  oled_consts.h
//
//  Defines the Macros for all of the OLED constants (Lab 2).
//
//*****************************************************************************
#ifndef __OLED_CONSTS_H__
#define __OLED_CONSTS_H__

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
#define TX_X_ORIGIN      0
#define TX_Y_ORIGIN      0
#define RX_X_ORIGIN      0
#define RX_Y_ORIGIN      HEIGHT / 2;

#define TEXT_COLOR       BLACK
#define BG_COLOR         WHITE
#define CHAR_WIDTH       6
#define CHAR_HEIGHT      8
#define TEXT_SIZE        2

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif


#endif // __OLED_CONSTS_H__
