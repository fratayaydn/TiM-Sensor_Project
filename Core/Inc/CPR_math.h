/*
 * CPR_math.h
 *
 *  Created on: Sep 20, 2020
 *      Author: Twissell
 */

#ifndef INC_CPR_MATH_H_
#define INC_CPR_MATH_H_

#include "arm_const_structs.h"
#include "arm_math.h"

#define PI 3.14159265358979f
#define NUMBER_HARMONICS 3

float32_t CPR_CompressionDepth( float32_t *aInput, uint16_t fftSize, uint8_t ifftFlag,
		uint8_t bitReverseFlag);

#endif /* INC_CPR_MATH_H_ */
