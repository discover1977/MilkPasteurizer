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
#include <string.h>
#include <math.h>

#define BUF_SIZE		10
#define POINT	'.'

void float_to_str(char *buf, float val, uint8_t precision);
void float_to_srt_set_point(char p);

long map(long x, long in_min, long in_max, long out_min, long out_max);
float mapf(float x, float in_min, float in_max, float out_min, float out_max);
long constrain(long val, long min, long max);
float constrainF(float val, float min, float max);

#endif /* CONV_H_ */
