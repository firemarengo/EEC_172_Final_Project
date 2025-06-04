/*
 * sevenAxisData.h
 *
 *  Created on: Jun 3, 2025
 *      Author: haaris
 */

#ifndef SEVENAXISDATA_H_
#define SEVENAXISDATA_H_

#include <stdio.h>
#include <stdbool.h>

typedef struct {
    float dist;
    float acc_x;
    float acc_y;
    float acc_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
} sevenAxisData;

// Parse a sevenAxisData object into a char * that's
// inputed to the function as a parameter.
//
// The char * object will be formatted such that it can
// be directly placed into the http_post_values function
// in main.c.
void parseData(sevenAxisData data, char * buff, unsigned int buffLength, bool isReset);

#endif /* SEVENAXISDATA_H_ */
