/*
 * conv.c
 *
 *  Created on: 27 рту. 2019 у.
 *      Author: gavrilov.iv
 */

#include "conv.h"

static char pointChar = POINT;

void float_to_strP(char *buf, float val, uint8_t precision, char p) {
	pointChar = p;
	float_to_str(buf, val, precision);
	pointChar = POINT;
}

void float_to_str(char *buf, float val, uint8_t precision) {
	char tmpStr[BUF_SIZE];
	uint16_t mult = 1;
	int16_t i = 0;
	uint8_t j = 0;
	int16_t tmp;

	for(i = 0; i < BUF_SIZE; i++) tmpStr[i] = buf[i] = 0x00;

	if(val != 0.0) {
		for(i = 0; i < precision; i++) mult *= 10;
		tmp = fabs(val) * mult;

		for(i = 0; i < BUF_SIZE; i++) {
			tmpStr[i] = ((tmp % 10) + '0');
			tmp /= 10;
			if(tmp <= 0) break;
		}

		if(fabs(val) < 1.0) {
			i++;
			tmpStr[i] = '0';
		}

		if(val < 0.0) {
			i++;
			tmpStr[i] = '-';
		}

		tmp = i;
		for(j = 0; j < (BUF_SIZE + 1); j++) {
			if(j == tmp - (precision - 1)) {
				buf[j] = pointChar;
			}
			else {
				buf[j] = tmpStr[i];
				if(--i < 0) break;
			}
		}
	}
	else {
		if(precision == 0) {
			buf[0] = '0';
		}
		else {
			for(i = 0; i < (precision + 2); i++) {
				if(i == 1) {
					buf[i] = pointChar;
				}
				else buf[i] = '0';
			}
		}
	}
}

long map(long x, long in_min, long in_max, long out_min, long out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

long constrain(long val, long min, long max) {
	if(val < min) return min;
	else if(val > max) return max;
	else return val;
}

float constrainF(float val, float min, float max) {
	if(val < min) return min;
	else if(val > max) return max;
	else return val;
}
