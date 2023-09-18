/*
 * cTranscendental.c
 *
 *  Created on: Sep 14, 2023
 *      Author: Theo & Philippe
 */

#include "main.h"
#define ARM_MATH_CM4
#include "arm_math.h"

#define THRESH 0.000001

void cTranscendental(float32_t x, float32_t omega, float32_t phi, float32_t *output){
	float32_t x0 = x;
	float32_t x1;

	while(1){
		// Compute the new guess
		x1 = x0 - (x0 * x0 - arm_cos_f32(omega * x0 + phi)) / (2 * x0 + omega * arm_sin_f32(omega * x0 + phi));

		// Check if new and previous results are close enough and break
		if(x1 - x0 < THRESH && x1 - x0 < THRESH)
			break;

		// Set the new guess as previous guess and repeat
		x0 = x1;
	}

	// Store the result in output pointer
	*output = x1;
}
