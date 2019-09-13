/*
 * filter.c
 *
 *  Created on: 11 сент. 2019 г.
 *      Author: gavrilov.iv
 */

#include "filter.h"

static int16_t Buffer[FW_CHANN_NUMBER][AccSizeMacro(ACC_BITS)];
static uint16_t Index[FW_CHANN_NUMBER];
static int64_t Summ[FW_CHANN_NUMBER];

int16_t fw_filter(int16_t Value, uint8_t channel) {

	if(channel > (FW_CHANN_NUMBER - 1)) return -1;

	Summ[channel] += Value;
	Summ[channel] -= Buffer[channel][Index[channel]];

	Buffer[channel][Index[channel]++] = Value;

	if (Index[channel] == AccSizeMacro(ACC_BITS)) {
		Index[channel] = 0;
	}

	return (Summ[channel] >> ACC_BITS);
}

void fw_filter_init() {
	for(int ch = 0; ch < FW_CHANN_NUMBER; ch++) {
		Summ[ch] = 0;
		Index[ch] = 0;
		for(int i = 0; i < AccSizeMacro(ACC_BITS); i++) {
			Buffer[ch][i] = 0;
		}
	}
}
