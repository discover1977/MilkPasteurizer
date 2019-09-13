/*
 * filter.h
 *
 *  Created on: 11 сент. 2019 г.
 *      Author: gavrilov.iv
 */

#ifndef FILTER_H_
#define FILTER_H_

#include <avr/io.h>

#define ACC_BITS			5
#define AccSizeMacro(val)	(1 << val)
#define FW_CHANN_NUMBER	2
int16_t fw_filter(int16_t Value, uint8_t channel);
void fw_filter_init();

#endif /* FILTER_H_ */
