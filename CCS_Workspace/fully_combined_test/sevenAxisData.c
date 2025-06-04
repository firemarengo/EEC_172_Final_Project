/*
 * sevenAxisData.c
 *
 *  Created on: Jun 3, 2025
 *      Author: haaris
 */

#include <stdio.h>
#include <string.h>
#include "sevenAxisData.h"

const char * PARSEFORMAT = "{\r\n" \
            "\"d\": %.3f,\r\n"                                           \
            "\"ax\": %.3f,\r\n"                                          \
            "\"ay\": %.3f,\r\n"                                          \
            "\"az\": %.3f,\r\n"                                          \
            "\"gx\": %.3f,\r\n"                                         \
            "\"gy\": %.3f,\r\n"                                         \
            "\"gz\": %.3f\r\n"                                          \
        "},\r\n"                                                            \
        "\"isReset\": %s\r\n";

void parseData(sevenAxisData data, char * buff, unsigned int buffLength, bool isReset) {
    char truth[4] = "true";
    char falsehood[5] = "false";
    char buf[10];

    if (isReset) {
        strncpy(buf, truth, 4);
        buf[4] = '\0';
    }
    else {
        strncpy(buf, falsehood, 5);
        buf[5] = '\0';
    }

    snprintf(buff, buffLength, PARSEFORMAT, data.dist, data.acc_x,
             data.acc_y, data.acc_z, data.gyro_x, data.gyro_y, data.gyro_z, buf);
}
