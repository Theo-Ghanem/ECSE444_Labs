/*
 * cSqrt.c
 *
 *  Created on: Sep 14, 2023
 *      Author: Theo & Philippe
 */

#include "main.h"
#define ARM_MATH_CM4
#include "arm_math.h"

#define THRESH 0.000001

void cSqrt(float32_t input, float32_t *output){

	if (input <0){
		*output=-1;
		return;
	}
	if (input == 0){
		*output=0;
		return;
	}

	float32_t x = input;
	float32_t root;

	while(1){
		// Compute new guess
		root = 0.5 * (x + (input / x));

		// Check if new and previous results are close enough and break
		if(root - x < THRESH && x - root < THRESH)
			break;

		// Set the new guess as previous guess and repeat
		x = root;
	}

	// Store the result in output pointer
	*output=root;
}

