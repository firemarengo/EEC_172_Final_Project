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
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <time.h>

// Simplelink includes
#include "simplelink.h"

//Driverlib includes
#include "hw_types.h"
#include "hw_ints.h"
#include "hw_apps_rcm.h"
#include "hw_common_reg.h"
#include "gpio.h"
#include "rom.h"
#include "rom_map.h"
#include "hw_memmap.h"
#include "timer.h"
#include "interrupt.h"
#include "prcm.h"
#include "utils.h"
#include "uart.h"

//Common interface includes
#include "pinmux.h"
#include "gpio_if.h"
#include "common.h"
#include "uart_if.h"
#include "i2c_if.h"
#include "timer_if.h"

// Custom includes
#include "utils/network_utils.h"

// OLED includes
#include "spi.h"
#include "oled/Adafruit_GFX.h"
#include "oled/Adafruit_SSD1351.h"
#include "oled/glcdfont.h"
#include "oled/oled_test.h"

// Custom includes
#include "utils/network_utils.h"
#include "sevenAxisData.h"


//*****************************************************************************
//                      MACRO DEFINITIONS
//*****************************************************************************
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

#define BUFF_SIZE       512
#define FOREVER                    1

#define SPI_IF_BIT_RATE  100000
#define TR_BUFF_SIZE     100


//
// HC-SR04 trigger pin: PIN_15 --> GPIOA2, GPIO_PIN_6
//
#define HC_SR04_TRIG_BASE    GPIOA2_BASE
#define HC_SR04_TRIG_PIN     GPIO_PIN_6

// IMU-specific defines
#define IMU_ADDR        0x6A    // SA0 = 0 0x6A
#define OUTX_L_G        0x22
#define OUTX_L_A        0x28
#define CTRL1_XL        0x10
#define CTRL9_XL        0x18
#define CTRL2_G         0x11



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
//
// Globals used by the timer interrupt handler.
//
//*****************************************************************************
static volatile unsigned long g_ulSysTickValue;
static volatile unsigned long g_ulBase;
static volatile unsigned long g_ulRefBase;
static volatile unsigned long g_ulRefTimerInts = 0;
static volatile unsigned long g_ulIntClearVector;
unsigned long g_ulTimerInts;
int startFlag = 1;
int lRetVal_g = 0;

//*****************************************************************************
//                 GLOBAL VARIABLES -- End: df
//*****************************************************************************


//****************************************************************************
//                      LOCAL FUNCTION PROTOTYPES
//****************************************************************************
void readGyroXYZ(int16_t *, int16_t *, int16_t *);
void readAccelXYZ(int16_t *, int16_t *, int16_t *);
void TriggerPulse(void);
void TimerBaseIntHandler(void);
void TimerRefIntHandler(void);
void EchoIntHandler(void);
void IMUinit(void);

static int set_time();
static void BoardInit(void);
static int http_post_message(int, char *, int);

//****************************************************************************
//                      LOCAL FUNCTION DEFINITIONS
//****************************************************************************
void readGyroXYZ(int16_t *gx, int16_t *gy, int16_t *gz)
{
    unsigned char buf[6];
    unsigned char reg = OUTX_L_G;
    // Point to OUTX_L_G
    I2C_IF_Write(IMU_ADDR, &reg, 1, 0);
    // Read 6 bytes: X_L, X_H, Y_L, Y_H, Z_L, Z_H
    I2C_IF_Read(IMU_ADDR, buf, 6);
    *gx = (int16_t)((buf[1] << 8) | buf[0]);
    *gy = (int16_t)((buf[3] << 8) | buf[2]);
    *gz = (int16_t)((buf[5] << 8) | buf[4]);
}

void readAccelXYZ(int16_t *ax, int16_t *ay, int16_t *az)
{
    unsigned char buf[6];
    unsigned char reg = OUTX_L_A;
    // Point to OUTX_L_A
    I2C_IF_Write(IMU_ADDR, &reg, 1, 0);
    // Read 6 bytes: X_L, X_H, Y_L, Y_H, Z_L, Z_H
    I2C_IF_Read(IMU_ADDR, buf, 6);
    *ax = (int16_t)((buf[1] << 8) | buf[0]);
    *ay = (int16_t)((buf[3] << 8) | buf[2]);
    *az = (int16_t)((buf[5] << 8) | buf[4]);
}


//****************************************************************
// TriggerPulse
//
// Generates a ~10 us HIGH on HC_SR04_TRIG_PIN.
//*************************************************************
void TriggerPulse(void)
{
    // Drive trigger HIGH
    MAP_GPIOPinWrite(HC_SR04_TRIG_BASE, HC_SR04_TRIG_PIN, HC_SR04_TRIG_PIN);
    //
    // Delay ~10 us: Assuming an 80 MHz clock, each loop of UtilsDelay(~8) is ~1 us.
    // You may need to tune this constant based on actual clock speed.
    //
    //GPIO_IF_LedOn(MCU_GREEN_LED_GPIO);
    //Report("On\r\n");
    MAP_UtilsDelay(100);
    //GPIO_IF_LedOff(MCU_GREEN_LED_GPIO);
    //Report("Off\r\n");
    //
    // Drive trigger LOW
    //
    MAP_GPIOPinWrite(HC_SR04_TRIG_BASE, HC_SR04_TRIG_PIN, 0);
}


//*****************************************************************************
// The interrupt handler for the first timer interrupt.
//*****************************************************************************
void TimerBaseIntHandler(void)
{

    if(startFlag) {
            MAP_TimerIntClear(TIMERA0_BASE, TIMER_TIMA_TIMEOUT);
            return;
        }

        // Clear the timer interrupt.
        //Timer_IF_InterruptClear(g_ulBase);
        MAP_TimerIntClear(TIMERA0_BASE, TIMER_TIMA_TIMEOUT);


        g_ulTimerInts ++;
        //GPIO_IF_LedToggle(MCU_GREEN_LED_GPIO);

        TriggerPulse();
        //Report("Pulse %lu \r\n", g_ulTimerInts);
}


//*****************************************************************************
// The interrupt handler for the second timer interrupt.
//*****************************************************************************
void TimerRefIntHandler(void)
{

    // Clear timer interrupt.
    Timer_IF_InterruptClear(g_ulRefBase);

    g_ulRefTimerInts ++;
    GPIO_IF_LedToggle(MCU_RED_LED_GPIO);
}

//****************************************************************************
// Board Initialization & Configuration
//*****************************************************************************
static void BoardInit(void)
{
#ifndef USE_TIRTOS
#if defined(ccs)
    MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
#endif
#if defined(ewarm)
    MAP_IntVTableBaseSet((unsigned long)&__vector_table);
#endif
#endif
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

//*************************************************************************
//    main function uses timer to generate periodic interrupts.
//**************************************************************************

//********* **********************************************
//Echo pin
//*********************************

// Calibration offsets (raw LSB units, 16bit int)
static int16_t gx_cal, gy_cal, gz_cal;
static int16_t ax_cal, ay_cal, az_cal;
// Last filtered values (physical units -> float)
static float gx_last, gy_last, gz_last;
static float ax_last, ay_last, az_last;

// Buffers and raw readings
uint8_t  buf[6];
int16_t  raw_ax, raw_ay, raw_az;
int16_t  raw_gx, raw_gy, raw_gz;

static volatile unsigned long ulEchoStart = 0;
static volatile unsigned long ulEchoEnd   = 0;
float vel_x = 0, pos_x = 0;
float vel_y = 0, pos_y = 0;
float dt = 0.05;

void EchoIntHandler(void)
{
    // Read & clear the interrupt flag
    if(startFlag) {
        MAP_GPIOIntClear(GPIOA1_BASE, 0x1);
        return;
    }
    unsigned long ulStatus = MAP_GPIOIntStatus(GPIOA1_BASE, true);
    MAP_GPIOIntClear(GPIOA1_BASE,0x1);
    //Report("Literally anything.\r\n");
    if (ulStatus & 0x1) {
        // Read current pin level to distinguish rising vs. falling
        if (MAP_GPIOPinRead(GPIOA1_BASE, 0x1)) {
            // Rising edge --> record start time
            ulEchoStart = MAP_TimerValueGet(TIMERA1_BASE, TIMER_A);
            //Report("Rising\r\n");
        } else {
            // Falling edge --> record end time

            ulEchoEnd = MAP_TimerValueGet(TIMERA1_BASE, TIMER_A);
            //Report("Falling\r\n");
            // Compute pulse width in microseconds
            unsigned long ulDelta;
            if (ulEchoStart >= ulEchoEnd) {
                ulDelta = ulEchoStart - ulEchoEnd;
            } else {
                // Timer wrapped around
                ulDelta = ulEchoStart + (0xFFFFFFFF - ulEchoEnd) + 1;
            }


            // Convert to distance in inches:
            // distance_in = (ulDelta * 0.00675)
            //  since 0.00675 in/us = (34300 cm/s / 2.54 cm/in) / 2 / 1e6 s/us

            //============================================================================
            //IMPORTANT CALIBRATION VALUE: scalar calibration based on measured vs true values.
            float distanceCal = 39.0f / 36.0f; // true/displayed, for an inaccurate reading
            //=============================================================================

            float ulDistIn = distanceCal * ((float)ulDelta * 0.00675f);
            //Report("Distance: %u in\r\n", ulDistIn);

            //Code for IMU will go here:
            readGyroXYZ(&raw_gx, &raw_gy, &raw_gz);
            readAccelXYZ(&raw_ax, &raw_ay, &raw_az);
            //Report("raw delGyro (X,Y,Z) = %d, %d, %d LSB\r\n", raw_gx, raw_gy, raw_gz);

            // Convert to physical units (subtract offsets, apply sensitivity)
            float gx_f = ((float)(raw_gx - gx_cal)) * 0.0175f;   // +-125 dps -> 8.75 mdps/LSB
            float gy_f = ((float)(raw_gy - gy_cal)) * 0.0175f;
            float gz_f = ((float)(raw_gz - gz_cal)) * 0.0175f;
//            float gx_f = ((float)(raw_gx - gx_cal)) * 0.00875f;   // +-250 dps -> 8.75 mdps/LSB
//            float gy_f = ((float)(raw_gy - gy_cal)) * 0.00875f;
//            float gz_f = ((float)(raw_gz - gz_cal)) * 0.00875f;

            float ax_f = ((float)(raw_ax - ax_cal)) * 0.000061f;  // +-2 g -> 0.061 mg/LSB
            float ay_f = ((float)(raw_ay - ay_cal)) * 0.000061f;
            float az_f = ((float)(raw_az - az_cal)) * 0.000061f;

            sevenAxisData data;
            data.dist = ulDistIn;
            data.gyro_x = gx_f;
            data.gyro_y = gy_f;
            data.gyro_z = gz_f;
            data.acc_x  = ax_f;
            data.acc_y  = ay_f;
            data.acc_z  = az_f;

            char buffer[BUFF_SIZE];
            parseData(data, buffer, BUFF_SIZE, false);

            int msgLength = strlen(buffer);
            http_post_message(lRetVal_g, buffer, msgLength);

            //  low-pass: y[n] = 0.7*x[n] + 0.3*y[n-1]
//            gx_f = 0.7f * gx_f + 0.3f * gx_last;
//            gy_f = 0.7f * gy_f + 0.3f * gy_last;
//            gz_f = 0.7f * gz_f + 0.3f * gz_last;
//
//            ax_f = 0.7f * ax_f + 0.3f * ax_last;
//            ay_f = 0.7f * ay_f + 0.3f * ay_last;
//            az_f = 0.7f * az_f + 0.3f * az_last;

            // Save filtered values for next iteration
//            gx_last = gx_f;   gy_last = gy_f;   gz_last = gz_f;
//            ax_last = ax_f;   ay_last = ay_f;   az_last = az_f;

            // Print to UART (in dps and g)
//            Report("%u,%7.f,%7.f,%7.f,%7.f,%7.f,%7.f\r\n", ulDistIn,
                   //gx_f, gy_f, gz_f,ax_f, ay_f, az_f);
            Report("%7.1f,%7.2f,%7.2f,%7.2f,%7.3f,%7.3f,%7.3f\r\n", ulDistIn,
                   gx_f, gy_f, gz_f,ax_f, ay_f, az_f);
            // state variables (initialized to zero before you start)



            //pos_y = (int)(pos_y * 100.0f) / 100.0f;
            //Report("x=%.3f, y=%.3f\r\n", pos_x, pos_y);

//            Report("Gyro (X, Y, Z) = %7.2f, %7.2f, %7.2f dps\r\n",
//                   gx_f, gy_f, gz_f);
//            Report("Accel(X, Y, Z) = %7.3f, %7.3f, %7.3f g\r\n",
//                   ax_f, ay_f, az_f);
//            Report("----------------------------------------\r\n");
        }
    }
}

void IMUinit(void) {
    //unsigned char cfg;
    uint8_t cfg;

    // Disable I3C (CTRL9_XL = 0x18, bit0 = 1)
    cfg = 0x01;
    I2C_IF_Write(IMU_ADDR, (unsigned char[]){ CTRL9_XL, cfg }, 2, 1);

    // Configure accel --> 104 Hz, +/-2 g (CTRL1_XL = 0x10, 0x40)
    cfg = 0x40;
    I2C_IF_Write(IMU_ADDR, (unsigned char[]){ CTRL1_XL, cfg }, 2, 1);

    // Configure gyro  --> 104 Hz, +/-250 dps (CTRL2_G  = 0x11, 0x44)
    cfg = 0x44;
    I2C_IF_Write(IMU_ADDR, (unsigned char[]){ CTRL2_G,  cfg }, 2, 1);

    // Allow time for settings to take effect
    MAP_UtilsDelay(4000000);

    // Then, do what we used to do with int first; once at initalization,
    // effectively zero the IMU. Save computation during running.

    readGyroXYZ(&raw_gx, &raw_gy, &raw_gz);
    readAccelXYZ(&raw_ax, &raw_ay, &raw_az);
    gx_cal = raw_gx;
    gy_cal = raw_gy;
    gz_cal = raw_gz;
    ax_cal = raw_ax;
    ay_cal = raw_ay;
    az_cal = raw_az;

    sevenAxisData data;
    data.dist = 0;
    data.gyro_x = gx_cal;
    data.gyro_y = gy_cal;
    data.gyro_z = gz_cal;
    data.acc_x  = ax_cal;
    data.acc_y  = ay_cal;
    data.acc_z  = az_cal;

    char buffer[BUFF_SIZE];
    parseData(data, buffer, BUFF_SIZE, true);

    int msgLength = strlen(buffer);
    http_post_message(lRetVal_g, buffer, msgLength);
//    gx_last = gy_last = gz_last = 0.0f;
//    ax_last = ay_last = az_last = 0.0f;

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
    unsigned long g_ulBase = TIMERA0_BASE;
    unsigned long g_ulEchoTimerBase = TIMERA1_BASE;
    long lRetVal = -1;

    //
    // Initialize board configuration
    //
    BoardInit();
    PinMuxConfig();
    InitTerm();
    ClearTerm();
    I2C_IF_Open(I2C_MASTER_MODE_FST);
    Report("d,gx,gy,gz,ax,ay,az\r\n");

    // Enable the SPI module clock
    MAP_PRCMPeripheralClkEnable(PRCM_GSPI,PRCM_RUN_MODE_CLK);

    // Reset the peripheral
    MAP_PRCMPeripheralReset(PRCM_GSPI);

    // Reset SPI interface settings.
    MAP_SPIReset(GSPI_BASE);

    // Configure SPI interface
    MAP_SPIConfigSetExpClk(GSPI_BASE,MAP_PRCMPeripheralClockGet(PRCM_GSPI),
                     SPI_IF_BIT_RATE,SPI_MODE_MASTER,SPI_SUB_MODE_0,
                     (SPI_SW_CTRL_CS |
                     SPI_4PIN_MODE |
                     SPI_TURBO_OFF |
                     SPI_CS_ACTIVEHIGH |
                     SPI_WL_8));

    // Enable SPI for communication
    MAP_SPIEnable(GSPI_BASE);

    // Initialize Adafruit OLED
    Adafruit_Init();
    fillScreen(WHITE);
    IMUinit();

    // MAP_UtilsDelay(4000000);

    //Echo-pin interrupt
    MAP_GPIOIntRegister(GPIOA1_BASE, EchoIntHandler);
    MAP_GPIOIntTypeSet(GPIOA1_BASE, 0x1, GPIO_BOTH_EDGES);
    MAP_GPIOIntClear(GPIOA1_BASE, 0x1);
    MAP_GPIOIntEnable(GPIOA1_BASE, 0x1);
    MAP_IntEnable(INT_GPIOA1);

    //TimerA1 as 1us running counter
    MAP_PRCMPeripheralClkEnable(PRCM_TIMERA1, PRCM_RUN_MODE_CLK);
    MAP_TimerConfigure(g_ulEchoTimerBase, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PERIODIC);
    MAP_TimerPrescaleSet(g_ulEchoTimerBase, TIMER_A, 79);
    MAP_TimerLoadSet(g_ulEchoTimerBase, TIMER_A, 0xFFFFFFFF);
    MAP_TimerEnable(g_ulEchoTimerBase, TIMER_A);


    //TimerA0 for periodic trigger pulses (every 500 ms here), 2Hz
                 //TESTING WITHOUT THESE INTERRUPTS
    g_ulBase = TIMERA0_BASE;
    MAP_PRCMPeripheralClkEnable(PRCM_TIMERA0, PRCM_RUN_MODE_CLK);
    Timer_IF_Init(PRCM_TIMERA0, g_ulBase, TIMER_CFG_PERIODIC, TIMER_A, 0);
    Timer_IF_IntSetup(g_ulBase, TIMER_A, TimerBaseIntHandler);
    Timer_IF_Start(g_ulBase, TIMER_A, dt*1000);

    startFlag = 0;

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

    sl_Stop(SL_STOP_TIMEOUT);
    LOOP_FOREVER();
}
//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
