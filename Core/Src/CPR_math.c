/*
 * CPR_math.c
 *
 *  Created on: Sep 20, 2020
 *      Author: Twissell
 */

#include "CPR_math.h"

/**
 * @brief Calculate the phase of
 *
 * @param FFT of signal array
 * @param Fundamental frequency of signal
 * @param[out] Phase angle array
 * @param Length of complex signal array
 *
 * @return Calculated phase angle of input signal
 **/
void CPR_Calc_Phase(float32_t *aInput, uint32_t *fcc, float32_t *phaseAngle,
		uint16_t Length) {
	uint16_t i;

	/* Find phase angles between by arc tangent of complex and real values at first 3 harmonic angles*/
	for (i = 0; i < (NUMBER_HARMONICS); i++) {
		*(phaseAngle + i) = PI / 2 - atan2(*(aInput + (2 * (*fcc * (i + 1)))), *(aInput + (2 * (*fcc * (i + 1))) + 1));
	}

}

/**
 * @brief Do FFT and calculate the fundamental frequency, amplitute of harmonics
 *
 * @param Input acceleration signal array for FFT
 * @param Size of FFT
 * @param ifftFlag       flag that selects forward (ifftFlag=0) or inverse (ifftFlag=1) transform
 * @param bitReverseFlag flag that enables (bitReverseFlag=1) or disables (bitReverseFlag=0) bit reversal of output
 * @param[out] Magnitude of harmonics array
 * @param[out] Fundamental frequency of signal
 *
 * @return none
 **/
void CPR_Calc_Harmonics(float32_t *aInput, uint16_t fftSize, uint8_t ifftFlag,
		uint8_t bitReverseFlag, float32_t *A, uint32_t *fcc) {

	float32_t aOutput[fftSize];

	arm_cfft_instance_f32 CFFT_Instance;

	switch (fftSize) { /* Choose CFFT Instance according to N-point FFT */
	case 256:
		CFFT_Instance = arm_cfft_sR_f32_len256;
		break;
	case 512:
		CFFT_Instance = arm_cfft_sR_f32_len512;
		break;
	case 1024:
		CFFT_Instance = arm_cfft_sR_f32_len1024;
		break;
	}

	arm_cfft_f32(&CFFT_Instance, aInput, ifftFlag, bitReverseFlag); /* Complex FFT */

	arm_cmplx_mag_f32(aInput, aOutput, fftSize); /* Calculate amplitute by using complex values of FFT */

	arm_max_f32(aOutput, fftSize, A, fcc); /* Take max value and its index(Fundamental frequency) */

	*A = *A / fftSize; /* Calculate 1st harmonic's Amplitute and write to array's address */
	*(A + 1) = *(aOutput + (*fcc * 2)) / fftSize; /* Calculate 2nd harmonic's Amplitute and write to array's address */
	*(A + 2) = *(aOutput + (*fcc * 3)) / fftSize; /* Calculate 3rd harmonic's Amplitute and write to array's address */

}
/**
 * @brief Calculate Sk values
 *
 * @param[out] Sk values array for reconstructing S(t) signal
 * @param Magnitude of harmonics array
 * @param Fundamental frequency of signal
 *
 * @return none
 **/
void CRP_Calc_Sk(float32_t *Sk, float32_t *A, uint32_t *fcc) {
	uint8_t i;
	/* Calculate Sk values according to function in the paper */
	for (i = 0; i < (NUMBER_HARMONICS - 1); i++) {
		*(Sk + i) = (1000.0 * (*(A + i))) / ((2.0 * PI * (i + 1) * (*fcc)) * (2.0 * PI * (i + 1) * (*fcc)));
	}
}
/**
 * @brief Reconstruct Compression Depth signal(S(k))
 *
 * @param[in,out] Input acceleration signal array for FFT
 * @param Size of FFT
 * @param ifftFlag       flag that selects forward (ifftFlag=0) or inverse (ifftFlag=1) transform
 * @param bitReverseFlag flag that enables (bitReverseFlag=1) or disables (bitReverseFlag=0) bit reversal of output
 *
 * @return none
 **/
void CPR_Reconstruct_st(float32_t *aInput, uint16_t fftSize, uint8_t ifftFlag,
		uint8_t bitReverseFlag) {

	float32_t A[NUMBER_HARMONICS];
	float32_t Sk[NUMBER_HARMONICS];
	float32_t phaseAngle[NUMBER_HARMONICS];
	uint32_t fcc;
	uint16_t i;
	uint8_t j;

	CPR_Calc_Harmonics(aInput, fftSize, ifftFlag, bitReverseFlag, A, &fcc);
	CPR_Calc_Phase(aInput, &fcc, phaseAngle, fftSize * 2);
	CRP_Calc_Sk(Sk, A, &fcc);

	/* Reconstruct cosine function with acquired values */
	for (i = 0; i < fftSize; i++) {
		float32_t value = 0;
		for (j = 0; j < (NUMBER_HARMONICS - 1); j++) {
			value += Sk[j] * cos(fcc * (j + 1.0f) * 2.0f * PI * (float32_t) i / fftSize + phaseAngle[j]);
		}

		/* Using same array to use less memory */
		*(aInput + (2 * i)) = value;
		*(aInput + (2 * i + 1)) = 0;
	}
}

/**
 * @brief Calculate compression depth(millimeters) by reconstructing Compression Depth signal(S(k)) and its peak to peak value
 *
 * @param Input acceleration signal array for FFT
 * @param Size of FFT
 * @param ifftFlag       flag that selects forward (ifftFlag=0) or inverse (ifftFlag=1) transform
 * @param bitReverseFlag flag that enables (bitReverseFlag=1) or disables (bitReverseFlag=0) bit reversal of output
 *
 * @return float Compression Depth in millimeters
 **/
float32_t CPR_CompressionDepth(float32_t *aInput, uint16_t fftSize,
		uint8_t ifftFlag, uint8_t bitReverseFlag) {

	float32_t maxValue, minValue;
	uint32_t maxIndex, minIndex;
	CPR_Reconstruct_st(aInput, fftSize, ifftFlag, bitReverseFlag);

	/* Take max and min amplitude values */
	arm_max_f32(aInput, fftSize * 2, &maxValue, &maxIndex);
	arm_min_f32(aInput, fftSize * 2, &minValue, &minIndex);

	return maxValue - minValue; /* Subtraction of max and min amplitute values gives compression depth */

}
