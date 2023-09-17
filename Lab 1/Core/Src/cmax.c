/*
 * cmax.c
 *
 *  Created on: Sep 11, 2023
 *      Author: Theo
 */
#include "main.h"

void cMax(float *array, uint32_t size, float *max, uint32_t *maxIndex){
	(*max) = array[0];
	(*maxIndex) = 0;

	// Loop through all elements in array
	for (uint32_t i = 1; i < size; i++) {
			// If a new value is greater than max, set it as max
			if (array[i] > (*max)) {
				(*max) = array[i];
				(*maxIndex) = i;
			} // if
		} // for
} //cmax
