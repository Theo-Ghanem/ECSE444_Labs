/*
 * asmSqrt.s
 *
 *  Created on: Sep 12, 2023
 *      Author: Theo & Philippe
 */

 // unified indicates that we're using a mix of different ARM instructions,
// e.g., 16-bit Thumb and 32-bit ARM instructions may be present (and are)
.syntax unified

// .global exports the label asmMax, which is expected by lab1math.h
.global asmTranscendental

.section .data
THRESH: .float 0.000001
float_nan: .float NaN
// .section marks a new section in assembly. .text identifies it as source code;
// .rodata marks it as read-only, setting it to go in FLASH, not SRAM
.section .text.rodata

/**
 * extern void asmTranscendental(float32_t x, float32_t omega, float32_t phi, float32_t *output);
 *
 * S0 = x
 * S1 = omega
 * S2 = phi
 * R0 = pointer to output
 */
//void cTranscendental(float32_t x, float32_t omega, float32_t phi, float32_t *output){
//	float32_t x0 = x;
//	float32_t x1;
//
//	while(1){
//		x1 = x0 - (x0 * x0 - arm_cos_f32(omega * x0 + phi)) / (2 * x0 + omega * arm_sin_f32(omega * x0 + phi));
//
//		if(x1 - x0 < THRESH && x1 - x0 < THRESH)
//			break;
//
//		x0 = x1;
//	}
//
//	*output = x1;
//}

 asmTranscendental:
	PUSH {R4, R5, LR}
	VMOV S4, S0 // X
	VMOV S5, S1 // omega
	VMOV S6, S2 //phi
	MOV R4, R0  //output address

	MOV R5, #0 //counter for loop (to make sure we don't get stuck)

convergence_loop:
	CMP R5, #2000			//if we reach 2000 exit the loop (no solution)
	BEQ divergence			// exit and return NaN

	VMOV S7, S6 			// copy over phi
	VMLA.F32 S7, S4, S5 	// multiply and add : (omega * x0 + phi)
	VMOV S0, S7 			// Move (omega * x0 + phi) to S0
	BL arm_cos_f32			// cos(omega * x0 + phi)
	VMOV S8, S0				// move result of cos(omega * x0 + phi) to S8

	VMOV S0, S7				// Move (omega * x0 + phi) to S0
	BL arm_sin_f32			// sin(omega * x0 + phi)
	VMOV S7, S0 			// Move result of sin(omega * x0 + phi) to S7

	VMUL.F32 S9, S4, S4 	// x^2
	VSUB.F32 S9, S9, S8 	// x^2 - cos

	vmov S12, #2.0          // Load the constant 2.0 into S12
	vmul.f32 S10, S4, S12   // 2 * x


	VMUL.F32 S7, S7, S5		// omega * sin
	VADD.F32 S10, S10, S7	// 2 * x + omega * sin
	VCMP.f32 S10, #0        // Ensure denominator is not 0
	VMRS APSR_nzcv, FPSCR 	// Done to get the condition flag from VCMP
	BEQ divergence			// If denominator is 0, exit and return NaN

	VDIV.F32 S9, S9, S10	// division
	VSUB.F32 S10, S4, S9	// x0 - division

	VSUB.F32 S11, S10, S4	// x1 - x0
	VABS.F32 S11, S11		// abs(x1 - x0) to compare to our threshold

	ldr r6, =THRESH  		// Load the location constant 0.00001 into R6
	vldr.f32 S12, [r6]		// Load the constant 0.00001 into S12
	VCMP.f32 S11, S12       // Compare S11(result) to S12(threshold)
	VMRS APSR_nzcv, FPSCR 	// Done to get the condition flag from VCMP

	BLT end_convergence_loop // if result is less than thresh break out of loop

	VMOV S4, S10			// Store new computation as input
	ADD R5, R5, #1			// Increment loop counter
	B convergence_loop		// Go back to beginning of loop

divergence:					// If no solution or denominator is 0
	LDR R5, =float_nan		// Return NaN
	VLDR S10, [R5]

end_convergence_loop:

	VSTR S10, [R4]			// Store result to output pointer

	POP {R4, R5, LR}
 	BX LR

