/*
 * cTranscendental.c
 *
 *  Created on: Sep 14, 2023
 *      Author: Theo
 */

#include "main.h"
#define ARM_MATH_CM4
#include "arm_math.h"

#define THRESH 0.000001

void cTranscendental(float32_t x, float32_t omega, float32_t phi, float32_t *output){
	float32_t x0 = x;
	float32_t x1;

	while(1){
		x1 = x0 - (x0 * x0 - arm_cos_f32(omega * x0 + phi)) / (2 * x0 + omega * arm_sin_f32(omega * x0 + phi));

		if(x1 - x0 < THRESH && x1 - x0 < THRESH)
			break;

		x0 = x1;
	}

	*output = x1;
}
