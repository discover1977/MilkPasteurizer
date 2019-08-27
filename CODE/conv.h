/*
 * conv.h
 *
 *  Created on: 27 рту. 2019 у.
 *      Author: gavrilov.iv
 */

#ifndef CONV_H_
#define CONV_H_

#include <avr/io.h>
#include <stdlib.h>
#include <math.h>

#define STR_LEN		10

char* float_to_str(float val, uint8_t precision, uint8_t width);

long map(long x, long in_min, long in_max, long out_min, long out_max);
float mapf(float x, float in_min, float in_max, float out_min, float out_max);
long constrain(long val, long min, long max);
float constrainF(float val, float min, float max);

#endif /* CONV_H_ */
