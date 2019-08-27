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

#endif /* CONV_H_ */
