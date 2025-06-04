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
// Application Name     -   SSL Demo
// Application Overview -   This is a sample application demonstrating the
//                          use of secure sockets on a CC3200 device.The
//                          application connects to an AP and
//                          tries to establish a secure connection to the
//                          Google server.
// Application Details  -
// docs\examples\CC32xx_SSL_Demo_Application.pdf
// or
// http://processors.wiki.ti.com/index.php/CC32xx_SSL_Demo_Application
//
//*****************************************************************************


//*****************************************************************************
//
//! \addtogroup ssl
//! @{
//
//*****************************************************************************

#include <stdio.h>
#include <time.h>

// Simplelink includes
#include "simplelink.h"

//Driverlib includes
#include "hw_types.h"
#include "hw_ints.h"
#include "rom.h"
#include "rom_map.h"
#include "interrupt.h"
#include "prcm.h"
#include "utils.h"
//#include "uart.h"

//Common interface includes
#include "pinmux.h"
#include "gpio_if.h"
#include "common.h"
#include "uart_if.h"

// Custom includes
#include "utils/network_utils.h"
#include "sevenAxisData.h"

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
                    "\"default\" :\""                                           \
                        "Hello phone. "                                     \
                        "Goodbye!"                  \
                        "\"\r\n"                                            \
                "}"                                                         \
            "}"                                                             \
        "}\r\n\r\n"

#define BUFF_SIZE       512
//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************

#if defined(ccs) || defined(gcc)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif

//*****************************************************************************
//                 GLOBAL VARIABLES -- End: df
//*****************************************************************************


//****************************************************************************
//                      LOCAL FUNCTION PROTOTYPES
//****************************************************************************
static int set_time();
static void BoardInit(void);
static int http_post(int);
static int http_get(int);
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
static void BoardInit(void) {
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
//! Main 
//!
//! \param  none
//!
//! \return None
//!
//*****************************************************************************
void main() {
    long lRetVal = -1;
    //
    // Initialize board configuration
    //
    BoardInit();

    PinMuxConfig();

    InitTerm();
    ClearTerm();
    UART_PRINT("My terminal works!\n\r");

    // initialize global default app configuration
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

    // Test for implementing values. //
    sevenAxisData data;
    data.dist = 3.14;
    data.acc_x = 4.5555;
    data.acc_y = 19.456;
    data.acc_z = 987.7894;
    data.gyro_x = 24.5;
    data.gyro_y = 55.6;
    data.gyro_z = 66.3;

    char buffer[BUFF_SIZE];
    parseData(data, buffer, BUFF_SIZE);

    int msgLength = strlen(buffer);
    http_post_message(lRetVal, buffer, msgLength);

    sl_Stop(SL_STOP_TIMEOUT);
    LOOP_FOREVER();
}
//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! Sends POST request to AWS.  The POST request uses macros defined earlier,
//! in the following format (message divided into multiple lines for clarity
//! of reading):
//!
//! POSTHEADER
//! HOSTHEADER
//! CHEADER
//! \r\n\r\n
//! CTHEADER
//! CLHEADER1 <DATA1 length>
//! CLHEADER2
//! DATA1
//!
//! \param  iTLSSockID: int. TLS socket ID of access port connection for message.
//!
//! \return 0 on success. Negative number on error.
//!
//*****************************************************************************
static int http_post(int iTLSSockID){
    char acSendBuff[512];
    char acRecvbuff[1460];
    char cCLLength[200];
    char* pcBufHeaders;
    int lRetVal = 0;

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
     *          \"default\" : \"
     *              "Hello phone, message from CC3200 via AWS IoT!\r\n"
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

    int dataLength = strlen(DATA1);

    strcpy(pcBufHeaders, CTHEADER);
    pcBufHeaders += strlen(CTHEADER);
    strcpy(pcBufHeaders, CLHEADER1);

    pcBufHeaders += strlen(CLHEADER1);
    sprintf(cCLLength, "%d", dataLength);

    strcpy(pcBufHeaders, cCLLength);
    pcBufHeaders += strlen(cCLLength);
    strcpy(pcBufHeaders, CLHEADER2);
    pcBufHeaders += strlen(CLHEADER2);

    strcpy(pcBufHeaders, DATA1);
    pcBufHeaders += strlen(DATA1);

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
//! Sends GET request to AWS.  The GET request uses macros defined earlier,
//! in the following format (message divided into multiple lines for clarity
//! of reading):
//!
//! GETHEADER
//! HOSTHEADER
//! CHEADER
//! \r\n\r\n
//!
//! \param  iTLSSockID: int. TLS socket ID of access port connection for message.
//!
//! \return 0 on success. Negative number on error.
//!
//*****************************************************************************
static int http_get(int iTLSSockID) {
    char acSendBuff[512];
    char acRecvbuff[1460];
    char* pcBufHeaders;
    int lRetVal = 0;


    /* Add the following message into pcBufHeaders as a single string (message
     * divided into multiple lines for clarity of reading):
     *
     * GET /things/Haaris_CC3200/shadow HTTP/1.1\r\n
     * Host: a1v0g26wa10u93-ats.iot.us-east-2.amazonaws.com\r\n
     * Connection: Keep-Alive\r\n\r\n\r\n
     */
    pcBufHeaders = acSendBuff;
    strcpy(pcBufHeaders, GETHEADER);
    pcBufHeaders += strlen(GETHEADER);
    strcpy(pcBufHeaders, HOSTHEADER);
    pcBufHeaders += strlen(HOSTHEADER);
    strcpy(pcBufHeaders, CHEADER);
    pcBufHeaders += strlen(CHEADER);
    strcpy(pcBufHeaders, "\r\n\r\n");

    int testDataLength = strlen(pcBufHeaders);


    // Print message to be sent to terminal.
    UART_PRINT(acSendBuff);


    //
    // Send the packet to the server */
    //
    lRetVal = sl_Send(iTLSSockID, acSendBuff, strlen(acSendBuff), 0);
    if(lRetVal < 0) {
        UART_PRINT("GET failed. Error Number: %i\n\r",lRetVal);
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
                    "\"body\" : ";

    // Second part of DATA portion of POST request.
    char data2Buff[strLength];
    char * ptrData2Buff = data2Buff;
    char * DATA_PART_2 = ptrData2Buff;
    strcpy(ptrData2Buff, str);
    ptrData2Buff += strLength;

    // Third part of DATA portion of POST request.
    char * DATA_PART_3 =    "}"             \
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
