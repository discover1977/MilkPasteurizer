/*
 * conv.c
 *
 *  Created on: 27 рту. 2019 у.
 *      Author: gavrilov.iv
 */

#include "conv.h"

char* float_to_str(float val, uint8_t precision, uint8_t width) {
	static char str[STR_LEN];
	char tmpStr[STR_LEN];
	uint16_t mult = 1;
	int16_t i = 0;
	uint8_t j = 0;
	int16_t tmp;


	for(i = 0; i < STR_LEN; i++) tmpStr[i] = str[i] = 0x00;

	if(val != 0.0) {
		for(i = 0; i < precision; i++) mult *= 10;
		tmp = fabs(val) * mult;

		for(i = 0; i < STR_LEN; i++) {
			tmpStr[i] = ((tmp % 10) + '0'/*0x30*/);
			tmp /= 10;
			if(tmp <= 0) break;
		}

		if(fabs(val) < 1.0) {
			i++;
			tmpStr[i] = '0'/*0x30*/;
		}

		if(val < 0.0) {
			i++;
			tmpStr[i] = '-';
		}

		if(width > 0) {
			tmp = i;
			width -= 2;
			for(uint8_t c = 0; c < width - tmp; c++) {
				i++;
				tmpStr[i] = ' ';
			}
		}

		tmp = i;
		for(j = 0; j < (STR_LEN + 1); j++) {
			if(j == tmp - (precision - 1)) {
				str[j] = '.';
			}
			else {
				str[j] = tmpStr[i];
				if(--i < 0) break;
			}
		}

		return str;
	}
	else {
		if(precision == 0) {
			str[0] = '0';
		}
		else {
			for(i = 0; i < (precision + 2); i++) {
				if(i == 1) {
					str[i] = '.';
				}
				else str[i] = '0';
			}
		}
		return str;
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
