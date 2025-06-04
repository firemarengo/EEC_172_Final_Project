/*
 * sevenAxisData.c
 *
 *  Created on: Jun 3, 2025
 *      Author: haaris
 */

#include <stdio.h>
#include "sevenAxisData.h"

const char * PARSEFORMAT = "{\r\n" \
            "\"dist\": %.3f,\r\n"                                           \
            "\"acc_x\": %.3f,\r\n"                                          \
            "\"acc_y\": %.3f,\r\n"                                          \
            "\"acc_z\": %.3f,\r\n"                                          \
            "\"gyro_x\": %.3f,\r\n"                                         \
            "\"gyro_y\": %.3f,\r\n"                                         \
            "\"gyro_z\": %.3f\r\n"                                          \
        "}";

void parseData(sevenAxisData data, char * buff, unsigned int buffLength) {
    snprintf(buff, buffLength, PARSEFORMAT, data.dist, data.acc_x,
             data.acc_y, data.acc_z, data.gyro_x, data.gyro_y, data.gyro_z);
}
