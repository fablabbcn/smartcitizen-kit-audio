#ifndef _AFSK_ANALYSER_H_INCLUDED
#define _AFSK_ANALYSER_H_INCLUDED

#include <Arduino.h>

#define ARM_MATH_CM0PLUS
#include <arm_math.h>

#include "../AudioInput/AudioInI2S.h"

#include "AudioAnalyser.h"
#include "ConstantsSound.h"

//CLASS
class AFSKAnalyser : public AudioAnalyser
{
public:

  AFSKAnalyser(int bufferSize, int fftSize, int signalSize); //
  ~AFSKAnalyser(); //

  void signalGet(void *signal);
  bool configure(AudioInI2S& input, int carrierFreq[], int carrierFreqSize, int deltaFreq);

private:
  void fft(void*inputBuffer, void* outputBuffer, int fftBufferSize);    
  unsigned char freqDetect(void* refBuffer, void* pointBuffer, int carrierFreqSize);
  bool isPeak (const void* inputBuffer, int index, int windowSize, double stdDev);
  // void normalEquation(int vectorLength, float window[], void* ordinate, void* slope);
  void normalEquation(int vectorLength, float window[], void* slope);
  double standardDeviation(void* inputBuffer, int bufferSize);
  double average(void* inputBuffer, int bufferSize);

  
  //BUFFER Sizes
  int _fftSize;
  int _bufferSize; //Already usable bufferSize
  //ASFK
  int _signalSize;
  int _sampleRate;
  int _carrierFreqSize;
  int _windowSize;
  // float _ordRight;
  float _slopeRight;
  // float _ordLeft;
  float _slopeLeft;
  //BUFFERS
  void* _sampleBuffer;
  void* _fftBuffer;
  void* _spectrumBuffer;
  void* _pointFreq;
  void* _windowBuffer;
  void* _windowBufferFloat;
  void* _signalBuffer;
  //FFT
  arm_rfft_instance_q31 _S31;
};

#endif