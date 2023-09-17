/*
 * asmSqrt.s
 *
 *  Created on: Sep 12, 2023
 *      Author: Theo
 */

 // unified indicates that we're using a mix of different ARM instructions,
// e.g., 16-bit Thumb and 32-bit ARM instructions may be present (and are)
.syntax unified

// .global exports the label asmMax, which is expected by lab1math.h
.global asmSqrt

// .section marks a new section in assembly. .text identifies it as source code;
// .rodata marks it as read-only, setting it to go in FLASH, not SRAM
.section .text.rodata

/**
 * void asmMax(float32_t input, float32_t *output);
 *
 * S0 = input
 * R0 = pointer to output
 */



asmSqrt:
  VSQRT.F32 S1, S0          // Calculate the square root and store it in S1
  VSTR    S1, [R0]          // Store the result in the memory location specified by R1
  BX LR

