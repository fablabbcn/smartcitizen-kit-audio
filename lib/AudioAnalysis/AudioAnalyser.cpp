#include "AudioAnalyser.h"

void AudioAnalyser::scaling(void *vector, int vectorSize, double factor, bool multDiv){
  // SCALE signal by factor
  q31_t* _vectDW = (q31_t*) vector;
  for (int i = 0; i<vectorSize;i++){
    if (multDiv) {
      *_vectDW *= factor;
    }
    else {
      *_vectDW /= factor;
    }
    _vectDW++;
  }
}

void AudioAnalyser::window(void *vector, int vectorSize){
  q31_t* srcW = (q31_t*)vector;

  //Apply hann window in time-domain
  for (int i = 0; i < vectorSize; i ++) {
    double window = HANN[i];
    (*srcW) *= window;
    srcW++;
  }
}

void AudioAnalyser::scalingandwindow(void *vector, int vectorSize){
  // SCALE signal by factor
  q31_t* _vectDW = (q31_t*) vector;
  for (int i = 0; i<vectorSize;i++){
    double window = HANN_REDUCED[i];
    *_vectDW *= window;
    _vectDW++;
  }
}

void AudioAnalyser::equalising(void *inputBuffer, int inputSize){
  //Deconvolution of the spectrumBuffer by division of the microphone frequency response

  q31_t* spBE = (q31_t*)inputBuffer;

  for (int i = 0; i < inputSize; i ++) {
    double equalfactor = EQUALTAB[i];
    *spBE /= equalfactor;
    spBE++;
  }
}
