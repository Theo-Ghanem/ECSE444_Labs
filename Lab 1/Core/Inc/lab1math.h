/*
 * lab1math.h
 *
 *  Created on: Sep 11, 2023
 *      Author: Theo & Philippe
 */

#ifndef INC_LAB1MATH_H_
#define INC_LAB1MATH_H_


void cMax(float *array, uint32_t size, float *max, uint32_t *maxIndex);
extern void asmMax(float *array, uint32_t size, float *max, uint32_t *maxIndex);
extern void cSqrt(float32_t input,float32_t* output);
extern void asmSqrt(float32_t input,float32_t* output);
extern void cTranscendental(float32_t x, float32_t omega, float32_t phi, float32_t *output);
extern void asmTranscendental(float32_t x, float32_t omega, float32_t phi, float32_t *output);

#endif /* INC_LAB1MATH_H_ */
