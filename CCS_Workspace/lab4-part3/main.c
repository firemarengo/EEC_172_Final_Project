//*****************************************************************************
//
// Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/ 
// 
// 
//  Redistribution and use in source and binary forms, with or without 
//  modification, are permitted provided that the following conditions 
//  are met:
//
//    Redistributions of source code must retain the above copyright 
//    notice, this list of conditions and the following disclaimer.
//
//    Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the 
//    documentation and/or other materials provided with the   
//    distribution.
//
//    Neither the name of Texas Instruments Incorporated nor the names of
//    its contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
//  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
//  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
//  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
//  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//*****************************************************************************

//*****************************************************************************
//
// Application Name     - SPI Demo
// Application Overview - The demo application focuses on showing the required 
//                        initialization sequence to enable the CC3200 SPI 
//                        module in full duplex 4-wire master and slave mode(s).
//
//*****************************************************************************


//*****************************************************************************
//
//! \addtogroup SPI_Demo
//! @{
//
//*****************************************************************************

// Standard includes
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <time.h>

// Simplelink includes
#include "simplelink.h"

// Driverlib includes
#include "hw_apps_rcm.h"
#include "hw_common_reg.h"
#include "hw_ints.h"
#include "hw_memmap.h"
#include "hw_nvic.h"
#include "hw_types.h"
#include "interrupt.h"
#include "prcm.h"
#include "rom.h"
#include "rom_map.h"
#include "spi.h"
#include "systick.h"
#include "timer.h"
#include "uart.h"
#include "utils.h"

// OLED interface includes
#include "oled/Adafruit_GFX.h"
#include "oled/Adafruit_SSD1351.h"
#include "oled/glcdfont.h"
#include "oled/oled_test.h"

// Common interface includes
#include "common.h"
#include "gpio.h"
#include "gpio_if.h"
#include "pin_mux_config.h"
#include "uart_if.h"
#include "timer_if.h"

// Custom includes
#include "network_utils.h"

// Macro includes
#include "Macros/remote_interrupt_consts.h"
#include "Macros/remote_hex_codes.h"
#include "Macros/spi_consts.h"
#include "Macros/oled_consts.h"

//*****************************************************************************
//                      MACRO DEFINITIONS
//*****************************************************************************
/** Macros for AWS HTTP **/
#define APPLICATION_NAME      "SSL"
#define APPLICATION_VERSION   "SQ24"
#define SERVER_NAME           "a1v0g26wa10u93-ats.iot.us-east-2.amazonaws.com" // CHANGE ME
#define GOOGLE_DST_PORT       8443

#define GETHEADER "GET /things/Haaris_CC3200/shadow HTTP/1.1\r\n"
#define POSTHEADER "POST /things/Haaris_CC3200/shadow HTTP/1.1\r\n"             // CHANGE ME
#define HOSTHEADER "Host: a1v0g26wa10u93-ats.iot.us-east-2.amazonaws.com\r\n"   // CHANGE ME
#define CHEADER "Connection: Keep-Alive\r\n"
#define CTHEADER "Content-Type: application/json; charset=utf-8\r\n"
#define CLHEADER1 "Content-Length: "
#define CLHEADER2 "\r\n\r\n"

#define DATA1 "{" \
            "\"state\": {\r\n"                                              \
                "\"desired\" : {\r\n"                                       \
                    "\"default\" :\""                                       \
                        "Hello phone, "                                     \
                        "message from CC3200 via AWS IoT!"                  \
                        "\"\r\n"                                            \
                "}"                                                         \
            "}"                                                             \
        "}\r\n\r\n"

//*****************************************************************************
//
//                 GLOBAL VARIABLES -- Start
//
//*****************************************************************************
//*****************************************************************************
//                      Global Variables for Vector Table
//*****************************************************************************
#if defined(ccs)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif

//*****************************************************************************
//               Global Variables used by the interrupt handlers.
//*****************************************************************************
/** Interrupt counters. **/
static volatile unsigned long g_ulSysTickValue;
static volatile unsigned long g_ulBase;
static volatile unsigned long g_ulBase2;
static volatile unsigned long g_ulTimerInts;
static volatile unsigned long g_ulTimerInts2;
static volatile unsigned long prevTimerCountForMultPresses;
static volatile unsigned long currTimerCountForMultPresses;

/** Globals for finding input word (from remote) using interrupts. **/
static volatile unsigned long input_word;
static volatile unsigned long prev_input_word;
static volatile unsigned char buff[BUFF_SIZE];
static volatile unsigned int buff_index;
static volatile tBoolean cleanBuffFlag = false;

static volatile unsigned int int_buff_index;

/** Multi-tap state variables **/
static volatile char curr_button_pressed = '\0';
static volatile char prev_button_pressed = '\0';
static volatile char curr_input_char     = '\0';
static volatile int input_index          = 0;

/** Symbol-timeout counter & flag (1 s = TOL ticks) **/
static volatile unsigned int symbolTimeoutCount  = 0;
static volatile unsigned int input_count = 0;

static volatile bool symbolTimeoutFlag = false;
static volatile bool timer2flag = false;
static volatile unsigned long symbolTimerCount = 0;  // counts timer2 interrupts

// track systick counter periods elapsed
// if it is not 0, we know the transmission ended
volatile int systick_cnt = 0;

//*****************************************************************************
//     Global Variables used as cursor position variables for OLED typing
//*****************************************************************************
unsigned int cursor_x_tx = 0;
unsigned int cursor_y_tx = HEIGHT / 2;
unsigned int cursor_x_rx = 0;
unsigned int cursor_y_rx = 0;
//*****************************************************************************
//
//                 GLOBAL VARIABLES -- End
//
//*****************************************************************************



//****************************************************************************
//
//                    LOCAL FUNCTION PROTOTYPES -- Start
//
//****************************************************************************
static void BoardInit(void);

/** Functions for drawing on OLED. **/
static char hexToButton(unsigned long);
static void appendCharTX(char);
static void appendCharRx(char);
static void deleteCharTX(void);
static void deleteCharRX(void);

/** Interrupt Handlers. **/
static void SysTickReset(void);
static void SysTickHandler(void);
static void TimerBaseIntHandler(void);
static void Timer2BaseIntHandler(void);
static unsigned int time_between_button_presses(void);
static void GPIOIntHandler(void);
static void UART1IntHandler(void);

/** Remote button analysis. **/
static char getChar(int, unsigned long);

/** Functions for REST API.**/
static int set_time();
static int http_post_message(int, char *, int);



//*****************************************************************************
//
//! Board Initialization & Configuration
//!
//! \param  None
//!
//! \return None
//
//*****************************************************************************
static void
BoardInit(void)
{
/* In case of TI-RTOS vector table is initialize by OS itself */
#ifndef USE_TIRTOS
  //
  // Set vector table base
  //
#if defined(ccs)
    MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
#endif
#if defined(ewarm)
    MAP_IntVTableBaseSet((unsigned long)&__vector_table);
#endif
#endif
    //
    // Enable Processor
    //
    MAP_IntMasterEnable();
    MAP_IntEnable(FAULT_SYSTICK);

    PRCMCC3200MCUInit();
}

//*****************************************************************************
//
//! Translates hex code from remote to char respresenting
//! the respective TV remote button.
//!
//! \param  input_word: Unsigned long.  The hex code recieved from the
//!                     TV remote via the IR receiver.
//!
//! \return button: char. A char representing the TV remote button pushed.
//
//*****************************************************************************
static char hexToButton(unsigned long x) {
    switch(x) {
        case(BUTTON_0)    : return '0';
        case(BUTTON_1)    : return '1';
        case(BUTTON_2)    : return '2';
        case(BUTTON_3)    : return '3';
        case(BUTTON_4)    : return '4';
        case(BUTTON_5)    : return '5';
        case(BUTTON_6)    : return '6';
        case(BUTTON_7)    : return '7';
        case(BUTTON_8)    : return '8';
        case(BUTTON_9)    : return '9';
        case(BUTTON_LAST) : return 'l';
        case(BUTTON_MUTE) : return 'm';
        default           : return '\0';
    }
}


//*****************************************************************************
//
//! Appends TX string on OLED screen based on input character.
//!
//! Works by manipulating the display of the individual character on the
//! OLED screen, as opposed to rewriting the entire input string.
//!
//! \param  input:  char. A character to be appended to the string on the
//!                 OLED.
//!
//! \return none
//
//*****************************************************************************
static void appendCharTX(char input) {
    // Add character to OLED.
    drawChar(cursor_x_tx, cursor_y_tx, input, TEXT_COLOR, BG_COLOR, TEXT_SIZE);

    // Update tx cursor coordinates (y-coord isn't updated until wrap around).
    cursor_x_tx += (CHAR_WIDTH * TEXT_SIZE);

    // Wrap cursor around, if too far to the right or too far down.
    if (cursor_x_tx > (WIDTH - CHAR_WIDTH * TEXT_SIZE)) {
        cursor_x_tx = 0;
        cursor_y_tx += (CHAR_HEIGHT * TEXT_SIZE);
    }
    if (cursor_y_tx >= HEIGHT) {
        cursor_x_tx = 0;
        cursor_y_tx = HEIGHT / 2;
    }

}

//*****************************************************************************
//
//! Appends RX string on OLED screen based on input character.
//!
//! Works by manipulating the display of the individual character on the
//! OLED screen, as opposed to rewriting the entire input string.
//!
//! (For the RX string, a different function will use this function to print
//!  the entire string at once.)
//!
//! \param  input:  char. A character to be appended to the string on the
//!                 OLED.
//!
//! \return none
//
//*****************************************************************************
static void appendCharRx(char input) {
    // Add character to OLED.
    drawChar(cursor_x_rx, cursor_y_rx, input, TEXT_COLOR, BG_COLOR, TEXT_SIZE);

    // Update rx cursor coordinates (y-coord isn't updated until wrap around).
    cursor_x_rx += (CHAR_WIDTH * TEXT_SIZE);

    // Wrap cursor around, if too far to the right or too far down.
    if (cursor_x_rx > (WIDTH - CHAR_WIDTH * TEXT_SIZE)) {
        cursor_x_rx = 0;
        cursor_y_rx += (CHAR_HEIGHT * TEXT_SIZE);
    }
    if (cursor_y_rx >= HEIGHT / 2) {
        cursor_x_rx = 0;
        cursor_y_rx = 0;
    }
}

//*****************************************************************************
//
//! Deletes last character from TX string on OLED screen.
//!
//! Works by manipulating the display of the individual character on the
//! OLED screen, as opposed to rewriting the entire input string.
//!
//! \param  none
//!
//! \return none
//
//*****************************************************************************
static void deleteCharTX() {
    // Return cursor to previous position.  Wrap around, if needed.
    if (!cursor_x_tx) {
        if (cursor_y_tx == HEIGHT / 2) return;

        cursor_x_tx = (WIDTH - CHAR_WIDTH * TEXT_SIZE);
        cursor_y_tx -= (CHAR_HEIGHT * TEXT_SIZE);
    }
    cursor_x_tx -= (CHAR_WIDTH * TEXT_SIZE);

    // Replace previous char with space on screen.  This will make the
    // previous char look blank, which is what we want.
    //
    // Note that we leave the cursor where it is, after replacing.  This
    // is so the next character to be written can overwrite the space.
    drawChar(cursor_x_tx, cursor_y_tx, ' ', TEXT_COLOR, BG_COLOR, TEXT_SIZE);
}

//*****************************************************************************
//
//! Deletes last character from RX string on OLED screen.
//!
//! Works by manipulating the display of the individual character on the
//! OLED screen, as opposed to rewriting the entire input string.
//!
//! \param  none
//!
//! \return none
//
//*****************************************************************************
static void deleteCharRX() {
    // Return cursor to previous position.  Wrap around, if needed.
    if (!cursor_x_rx) {
        if (!cursor_y_rx) return;

        cursor_x_rx = (WIDTH - CHAR_WIDTH * TEXT_SIZE);
        cursor_y_rx -= (CHAR_HEIGHT * TEXT_SIZE);
    }
    cursor_x_rx -= (CHAR_WIDTH * TEXT_SIZE);

    // Replace previous char with space on screen.  This will make the
    // previous char look blank, which is what we want.
    //
    // Note that we leave the cursor where it is, after replacing.  This
    // is so the next character to be written can overwrite the space.
    drawChar(cursor_x_rx, cursor_y_rx, ' ', TEXT_COLOR, BG_COLOR, TEXT_SIZE);
}

//*****************************************************************************
//
//! Resets SysTick Counter.
//!
//! \param  None
//!
//! \return none
//
//*****************************************************************************
static inline void SysTickReset(void) {
    // any write to the ST_CURRENT register clears it
    // after clearing it automatically gets reset without
    // triggering exception logic
    // see reference manual section 3.2.1
    HWREG(NVIC_ST_CURRENT) = 1;

    // clear the global count variable
    systick_cnt = 0;
}

//*****************************************************************************
//
//! SysTick Interrupt Handler.
//!
//! Keeps track of whether or not systick counter incremented and/or wrapped.
//!
//! \param  None
//!
//! \return none
//
//*****************************************************************************
static void SysTickHandler(void) {
    // increment every time the systick handler fires
    systick_cnt++;
}

//*****************************************************************************
//
//! The interrupt handler for the first timer interrupt.
//! (This timer interrupt is used for parsing the individual bits
//!  of each waveform.)
//!
//! \param  None
//!
//! \return none
//
//*****************************************************************************
static void TimerBaseIntHandler(void)
{
    // Clear the timer interrupt.
    Timer_IF_InterruptClear(g_ulBase);

    g_ulTimerInts++;
}

//*****************************************************************************
//
//! The interrupt handler for the second timer interrupt.
//! (This timer interrupt is used as a "time" to determining whether or
//!  not a button has been press has been repeated within a threshold.)
//!
//! \param  None
//!
//! \return none
//
//*****************************************************************************
static void Timer2BaseIntHandler(void) {
    // Clear the timer interrupt.
    Timer_IF_InterruptClear(g_ulBase2);

    g_ulTimerInts2++;

    if (timer2flag) {
        currTimerCountForMultPresses = g_ulTimerInts2;
        prevTimerCountForMultPresses = currTimerCountForMultPresses;
        g_ulTimerInts2          = 0;
        symbolTimeoutCount      = 0;
        symbolTimeoutFlag       = false;
        timer2flag              = false;
    }

    if (++symbolTimeoutCount >= TOL) {
        symbolTimeoutFlag   = true;
        symbolTimeoutCount  = 0;
    }
}

//*****************************************************************************
//
//! Calculates and returns the number of Timer 2 interrupts between
//! the current button press and the last button press.  Then, sets
//! previous timer counter to current one.
//!
//! \param  None
//!
//! \return result: unsigned int.  Difference between the current Timer 2
//!                 interrupt count and the previous one.
//
//*****************************************************************************
static unsigned int time_between_button_presses(void) {
    unsigned int result = currTimerCountForMultPresses - prevTimerCountForMultPresses;
    prevTimerCountForMultPresses = currTimerCountForMultPresses;

    return result;
}

//*****************************************************************************
//
//! The interrupt handler for the GPIO interrupt.
//!
//! \param  None
//!
//! \return none
//
//*****************************************************************************
static void GPIOIntHandler(void) {
    static bool prev_state = true;
    unsigned int diff;
    unsigned int input_val;

    // Get and clear status.
    //
    // (Interrupts for GPIO on the LaunchPad are triggered for an entire
    //  set of 8 pins at the same time, not for each individual pin.
    //  We get the status so that we can verify that the interrupt
    //  occurred for the exact pin we want.)
    unsigned long ulStatus = GPIOIntStatus(GPIOA0_BASE, true);
    GPIOIntClear(GPIOA0_BASE, ulStatus);

    // If the interrupt is for pin 3.
    if (ulStatus & GPIO_PIN_3) {
        // A falling edge has occurred.
        if (prev_state) {
            prev_state = false;

            if (!systick_cnt) {
                diff = g_ulTimerInts;

                // Calculate which bit the pulse width represented.
                if      (diff >= ZERO_MIN_PW && diff <= ZERO_MAX_PW) input_val = 0x0;
                else if (diff >= ONE_MIN_PW  && diff <= ONE_MAX_PW)  input_val = 0x1;
                else return;

                // If "0" or "1" detected, add it to input_word.
                input_word = (input_word << 1) + input_val;
                input_count++;
            }
            else {
                input_count = 0;
                timer2flag = true;
            }
        }

        // A rising edge has occurred.
        else {
            prev_state = true;

            g_ulTimerInts = 0;
            SysTickReset();
        }
    }
}

//*****************************************************************************
//
//! The interrupt handler for the UART1 interrupt.
//!
//! \param  None
//!
//! \return none
//
//*****************************************************************************
static void UART1IntHandler(void) {
    static tBoolean isRxStart = true;

    // Get and clear status.
    unsigned long ulStatus = UARTIntStatus(UARTA1_BASE, true);
    UARTIntClear(UARTA1_BASE, ulStatus);

    char input = 0x11;

    while (UARTCharsAvail(UARTA1_BASE) || UARTBusy(UARTA1_BASE)) {
        if (UARTCharsAvail(UARTA1_BASE)) {
            if (isRxStart) {
                while (cursor_x_rx != 0 || cursor_y_rx != 0) deleteCharRX();
                isRxStart = false;
            }
            input = UARTCharGetNonBlocking(UARTA1_BASE);
            if (input != '!') appendCharRx(input);
            if (input == '\0') isRxStart = true;
        }

        if (UARTBusy(UARTA1_BASE)) {
            if (int_buff_index <= buff_index) {
                UARTCharPut(UARTA1_BASE, buff[int_buff_index++]);
            }
        }
    }

    int_buff_index = 0;
}

//*****************************************************************************
//
//! Returns the appropriate alphanumerical symbol, given the remote button pressed,
//! and the index of the corresponding symbol in the list of all the alphanumerical
//! characters associated with that button.
//!
//! \param  input_index: int. Represents the index number of the list of all the
//!                      alphanumerical characters associated with button_pressed.
//!         button_pressed: unsigned long. Represents the remote button pressed.
//!
//! \return char. The corresponding alphanumerical character.
//
//*****************************************************************************
static char getChar(int input_index, unsigned long button_pressed) {

    char x[5];
    switch(button_pressed) {
        case BUTTON_0:
            strcpy(x, " 0");
            break;
        case BUTTON_1:
            strcpy(x, "");
            break;
        case BUTTON_2:
            strcpy(x, "ABC2");
            break;
        case BUTTON_3:
            strcpy(x, "DEF3");
            break;
        case BUTTON_4:
            strcpy(x, "GHI4");
            break;
        case BUTTON_5:
            strcpy(x, "JKL5");
            break;
        case BUTTON_6:
            strcpy(x, "MNO6");
            break;
        case BUTTON_7:
            strcpy(x, "PQRS7");
            break;
        case BUTTON_8:
            strcpy(x, "TUV8");
            break;
        case BUTTON_9:
            strcpy(x, "WXYZ9");
            break;
        default:
            x[0] = '\0';
            break;
    }

    int len = 0;
    while (x[len] != '\0') len++;

    // Wrap around using modulo
    return x[input_index % len];
}

//*****************************************************************************
//
//! This function updates the date and time of CC3200.
//!
//! \param None
//!
//! \return
//!     0 for success, negative otherwise
//!
//*****************************************************************************

static int set_time() {
    long retVal;

    time_t rawtime;
    struct tm * timeinfo;

    time(&rawtime);
    timeinfo = localtime(&rawtime);

    g_time.tm_day  = timeinfo->tm_mday;
    g_time.tm_mon  = timeinfo->tm_mon + 1;
    g_time.tm_year = timeinfo->tm_year + 1900;
    g_time.tm_sec  = timeinfo->tm_sec;
    g_time.tm_hour = timeinfo->tm_hour - 1;
    g_time.tm_min  = timeinfo->tm_min;

    retVal = sl_DevSet(SL_DEVICE_GENERAL_CONFIGURATION,
                          SL_DEVICE_GENERAL_CONFIGURATION_DATE_TIME,
                          sizeof(SlDateTime),(unsigned char *)(&g_time));

    ASSERT_ON_ERROR(retVal);
    return SUCCESS;
}

//*****************************************************************************
//
//! Sends POST request to AWS using input string and length.  The POST request
//! uses macros defined earlier, in the following format (message divided into
//! multiple lines for clarity of reading):
//!
//! POSTHEADER
//! HOSTHEADER
//! CHEADER
//! \r\n\r\n
//! CTHEADER
//! CLHEADER1 <length of DATA--defined in function>
//! CLHEADER2
//! <DATA--defined in function>
//!
//! \param  iTLSSockID: int. TLS socket ID of access port connection for message.
//!         str: char *. Message to be posted in HTTP request.
//!         strLength: int. Length of str.
//!
//! \return 0 on success. Negative number on error.
//!
//*****************************************************************************
static int http_post_message(int iTLSSockID, char * str, int strLength){
    char acSendBuff[512];
    char acRecvbuff[1460];
    char cCLLength[200];
    char* pcBufHeaders;
    int lRetVal = 0;

    // First part of DATA portion of POST request.
    char * DATA_PART_1 = "{"            \
            "\"state\": {\r\n"          \
                "\"desired\": {\r\n"    \
                    "\"default\" :\"";

    // Second part of DATA portion of POST request.
    char data2Buff[strLength];
    char * ptrData2Buff = data2Buff;
    char * DATA_PART_2 = ptrData2Buff;
    strcpy(ptrData2Buff, str);
    ptrData2Buff += strLength;

    // Third part of DATA portion of POST request.
    char * DATA_PART_3 = "\"\r\n"   \
                    "}"             \
                "}"                 \
            "}\r\n\r\n";

    int dataLength1 = strlen(DATA_PART_1);
    int dataLength2 = strLength;
    int dataLength3 = strlen(DATA_PART_3);

    /* Add the following message into pcBufHeaders as a single string (message
     * divided into multiple lines for clarity of reading):
     *
     * POST /things/Haaris_CC3200/shadow HTTP/1.1\r\n
     * Host: a1v0g26wa10u93-ats.iot.us-east-2.amazonaws.com\r\n
     * Connection: Keep-Alive\r\n\r\n\r\n
     * Content-Type: application/json; charset=utf-8\r\n
     * Content-Length: <calculated in this http-post()>\r\n\r\n
     * {
     *  \"state\": {\r\n
     *      \"desired\": {\r\n
     *          \"var\" : \"
     *              "\"<str>\"\r\n"
     *         }
     *     }
     * }\r\n\r\n
     */
    pcBufHeaders = acSendBuff;
    strcpy(pcBufHeaders, POSTHEADER);
    pcBufHeaders += strlen(POSTHEADER);
    strcpy(pcBufHeaders, HOSTHEADER);
    pcBufHeaders += strlen(HOSTHEADER);
    strcpy(pcBufHeaders, CHEADER);
    pcBufHeaders += strlen(CHEADER);
    strcpy(pcBufHeaders, "\r\n\r\n");

    strcpy(pcBufHeaders, CTHEADER);
    pcBufHeaders += strlen(CTHEADER);
    strcpy(pcBufHeaders, CLHEADER1);

    pcBufHeaders += strlen(CLHEADER1);
    sprintf(cCLLength, "%d", dataLength1 + dataLength2 + dataLength3);

    strcpy(pcBufHeaders, cCLLength);
    pcBufHeaders += strlen(cCLLength);
    strcpy(pcBufHeaders, CLHEADER2);
    pcBufHeaders += strlen(CLHEADER2);

    strcpy(pcBufHeaders, DATA_PART_1);
    pcBufHeaders += strlen(DATA_PART_1);
    strcpy(pcBufHeaders, DATA_PART_2);
    pcBufHeaders += strlen(DATA_PART_2);
    strcpy(pcBufHeaders, DATA_PART_3);
    pcBufHeaders += strlen(DATA_PART_3);

    int testDataLength = strlen(pcBufHeaders);

    // Print message to be sent to terminal.
    UART_PRINT(acSendBuff);


    //
    // Send the packet to the server */
    //
    lRetVal = sl_Send(iTLSSockID, acSendBuff, strlen(acSendBuff), 0);
    if(lRetVal < 0) {
        UART_PRINT("POST failed. Error Number: %i\n\r",lRetVal);
        sl_Close(iTLSSockID);
        GPIO_IF_LedOn(MCU_RED_LED_GPIO);
        return lRetVal;
    }
    lRetVal = sl_Recv(iTLSSockID, &acRecvbuff[0], sizeof(acRecvbuff), 0);
    if(lRetVal < 0) {
        UART_PRINT("Received failed. Error Number: %i\n\r",lRetVal);
        //sl_Close(iSSLSockID);
        GPIO_IF_LedOn(MCU_RED_LED_GPIO);
           return lRetVal;
    }
    else {
        acRecvbuff[lRetVal+1] = '\0';
        UART_PRINT(acRecvbuff);
        UART_PRINT("\n\r\n\r");
    }

    return 0;
}

//*****************************************************************************
//
//! Main function for spi demo application
//!
//! \param none
//!
//! \return None.
//
//*****************************************************************************
void main()
{
    long lRetVal = -1;

        // Initialize board configurations
        BoardInit();

        // Pinmuxing for IR receiver setup.
        PinMuxConfig();

        // Initializing the Terminal for UART0.
        InitTerm();

        // Clearing the Terminal for UART0.
        ClearTerm();


        /*** UART1 SETUP ***/
        MAP_UARTConfigSetExpClk(UARTA1_BASE, MAP_PRCMPeripheralClockGet(PRCM_UARTA1),
                                UART_BAUD_RATE, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                                UART_CONFIG_PAR_NONE));
        MAP_UARTIntRegister(UARTA1_BASE, UART1IntHandler);
        MAP_UARTIntEnable(UARTA1_BASE, (UART_INT_TX | UART_INT_RX));
        MAP_UARTFIFOLevelSet(UARTA1_BASE, UART_FIFO_TX1_8, UART_FIFO_RX1_8);
        MAP_UARTFIFOEnable(UARTA1_BASE);


        /*** TIMER INTERRUPT SETTUP ***/
        /** Timer 1 **/
        // Base address for timer used to create a clock.
        //
        // (We will not be using ticks directly to measure time.
        //  We will instead use "interrupt counts" on a timer that
        //  interrupts every TIMER_PERIOD microseconds).
        g_ulBase = TIMERA0_BASE;

        // Configuring the timer.
        Timer_IF_Init(PRCM_TIMERA0, g_ulBase, TIMER_CFG_PERIODIC, TIMER_A, 0);

        // Setup the interrupt for the timer timeouts.
        Timer_IF_IntSetup(g_ulBase, TIMER_A, TimerBaseIntHandler);

        // Turn on the timers feeding values in uSec
        MAP_TimerLoadSet(g_ulBase, TIMER_A, US_TO_TICKS(TIMER_PERIOD));
        MAP_TimerEnable(g_ulBase, TIMER_A);

        /** Timer 2 **/
        g_ulBase2 = TIMERA1_BASE;
        Timer_IF_Init(PRCM_TIMERA1, g_ulBase2, TIMER_CFG_PERIODIC, TIMER_A, 0);
        Timer_IF_IntSetup(g_ulBase2, TIMER_A, Timer2BaseIntHandler);
        MAP_TimerLoadSet(g_ulBase2, TIMER_A, US_TO_TICKS(TIMER_PERIOD));
        MAP_TimerEnable(g_ulBase2, TIMER_A);


        /*** GPIO INTERRUPT SETUP ***/
        GPIOIntEnable(GPIOA0_BASE, GPIO_PIN_3);
        GPIOIntTypeSet(GPIOA0_BASE, GPIO_PIN_3, GPIO_BOTH_EDGES);
        GPIOIntRegister(GPIOA0_BASE, GPIOIntHandler);


        /*** SYSTICK INTERRUPT SETUP ***/
        MAP_SysTickPeriodSet(SYSTICK_RELOAD_VAL);
        MAP_SysTickIntRegister(SysTickHandler);
        MAP_SysTickIntEnable();
        MAP_SysTickEnable();


        /*** SPI SETUP ***/
        MAP_PRCMPeripheralClkEnable(PRCM_GSPI,PRCM_RUN_MODE_CLK);
        MAP_PRCMPeripheralReset(PRCM_GSPI);
        MAP_SPIReset(GSPI_BASE);
        MAP_SPIConfigSetExpClk(GSPI_BASE,MAP_PRCMPeripheralClockGet(PRCM_GSPI),
                         SPI_IF_BIT_RATE,SPI_MODE_MASTER,SPI_SUB_MODE_0,
                         (SPI_SW_CTRL_CS |
                         SPI_4PIN_MODE |
                         SPI_TURBO_OFF |
                         SPI_CS_ACTIVEHIGH |
                         SPI_WL_8));
        MAP_SPIEnable(GSPI_BASE);


        // Initialize Adafruit OLED
        Adafruit_Init();

        // Clear display.  Create all white background.
        fillScreen(BG_COLOR);

        /** REST API for AWS SETUP **/
        // Test serial terminal
        UART_PRINT("My terminal works!\n\r");

        // Initialize global default app configuration
        g_app_config.host = SERVER_NAME;
        g_app_config.port = GOOGLE_DST_PORT;

        //Connect the CC3200 to the local access point
        lRetVal = connectToAccessPoint();

        //Set time so that encryption can be used
        lRetVal = set_time();
        if(lRetVal < 0) {
            UART_PRINT("Unable to set time in the device");
            LOOP_FOREVER();
        }

        //Connect to the website with TLS encryption
        lRetVal = tls_connect();
        if(lRetVal < 0) {
            ERR_PRINT(lRetVal);
        }

        //
        // Loop forever while the timers run.
        //
        while (FOREVER) {
            if (input_count == 32) {
                curr_button_pressed = hexToButton(input_word);
                if (curr_button_pressed == '\0' || curr_button_pressed == '1') continue;


                unsigned int interval = time_between_button_presses();

                tBoolean isMultiPress = (interval <= TOL) && (curr_button_pressed == prev_button_pressed);
                input_index = (isMultiPress) ? input_index + 1 : 0;

                curr_input_char = getChar(input_index, input_word);


                // Handle special keys:
                //  ‘l’ = “last” (ignore), ‘m’ = newline, otherwise literal char
                //
                // We appropriately change buff_index within each condition, for simplicity.
                //
                // (Before conditional, buff_index points at first '\0' after string.
                //  We must ensure that this is true after the conditional as well!)
                if (input_word == BUTTON_LAST) {
                    deleteCharTX();

                    if (buff_index > 0) --buff_index;
                    buff[buff_index] = '\0';
                }
                else if (input_word == BUTTON_MUTE) {
                    UARTCharPut(UARTA1_BASE, '!');
                    UARTCharPut(UARTA1_BASE, '!');
                    UARTCharPut(UARTA1_BASE, '!');
                    UARTCharPut(UARTA1_BASE, '!');

                    while (cursor_x_tx != 0 || cursor_y_tx != HEIGHT / 2) deleteCharTX();

                    http_post_message(lRetVal, buff, buff_index);

                    while (buff_index) buff[--buff_index] = '\0';
                    input_index = 0;
                    int_buff_index = 0;

                }
                else {
                    if (isMultiPress) deleteCharTX();
                    appendCharTX(curr_input_char);

                    if (isMultiPress) buff[--buff_index] = '\0';
                    buff[buff_index++] = curr_input_char;
                }

                // Reset for the next IR frame
                input_count          = 0;
                timer2flag           = false;
                prev_button_pressed  = curr_button_pressed;
            }

            if (symbolTimeoutFlag) {
                // Clear multi-tap state so next press starts a new letter
                input_index         = 0;
                prev_button_pressed = '\0';
                symbolTimeoutFlag   = false;
            }
        }

}

