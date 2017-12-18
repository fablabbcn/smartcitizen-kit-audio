#ifndef _AUDIO_ANALYZER_H_INCLUDED
#define _AUDIO_ANALYZER_H_INCLUDED

#include <Arduino.h>

#define ARM_MATH_CM0PLUS
#include <arm_math.h>
#include <stddef.h>

#include "../AudioInput/AudioInI2S.h"
#include "ConstantsSound.h"

enum WeightingType{
	A_WEIGHTING,
   	C_WEIGHTING,
   	Z_WEIGHTING
};

//CLASS
class AudioAnalyser
{
public:

	void scaling(void *vector, int vectorSize, double factor, bool multDiv); // TO ERASE
	void window(void *vector, int vectorSize); // TO ERASE
	void scalingandwindow(void *vector, int vectorSize);
  	void equalising(void *inputBuffer, int inputSize);	
};

#endif