#ifndef _FFT_ANALYSER_H_INCLUDED
#define _FFT_ANALYSER_H_INCLUDED

#include <Arduino.h>

#define ARM_MATH_CM0PLUS
#include <arm_math.h>

#include "../AudioInput/AudioInI2S.h"

#include "AudioAnalyser.h"
#include "ConstantsSound.h"

//CLASS
class FFTAnalyser : public AudioAnalyser
{
public:

  FFTAnalyser(int bufferSize, int fftSize, WeightingType weighting_type); //
  ~FFTAnalyser(); //

  double sensorRead(int spectrum[]);
  double sensorRead();
  bool configure(AudioInI2S& input);

private:

  void weighting(void *inputBuffer, int inputSize);
  void convert2DB(void *inputVector, void *outputVector, int vectorSize, int factor);
  double rms(void *inputBuffer, int inputSize, int typeRMS, int factor);
  void fft(void *inputBuffer, void* outputBuffer, int fftBufferSize);  
  
  //BUFFER Sizes
  int _fftSize;
  int _bufferSize; //Already usable bufferSize
  WeightingType _weighting_type;
  //RMS Results
  double _rmsSpecB;
  //BUFFERS
  void* _sampleBuffer;
  void* _fftBuffer;
  void* _spectrumBuffer;
  void* _spectrumBufferDB;
  //FFT
  arm_rfft_instance_q31 _S31;
};

#endif