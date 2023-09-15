/*
 * cSqrt.c
 *
 *  Created on: Sep 14, 2023
 *      Author: Theo
 */

#include "main.h"
#define ARM_MATH_CM4
#include "arm_math.h"

#define THRESH 0.000001

void cSqrt(float32_t input, float32_t *output){
	float32_t x = input;
	float32_t root;

	while(1){
		root = 0.5 * (x + (input / x));

		if(root - x < THRESH && x - root < THRESH)
			break;

		x = root;
	}

	*output=root;
}

