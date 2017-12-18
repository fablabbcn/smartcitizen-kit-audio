#include "AFSKAnalyser.h"

AFSKAnalyser::AFSKAnalyser(int bufferSize, int fftSize, int signalSize) :
  //BUFFER Sizes
  _fftSize(fftSize),
  _bufferSize(bufferSize),
  _signalSize(signalSize),
  _sampleRate(-1),
  _carrierFreqSize(-1),
  _pointFreq(NULL),
  _windowSize(-1),
  // _ordRight(-1),
  _slopeRight(-1),
  // _ordLeft(-1),
  _slopeLeft(-1),
  //BUFFERs
  _sampleBuffer(NULL),
  _fftBuffer(NULL),
  _spectrumBuffer(NULL),
  _windowBuffer(NULL),
  _windowBufferFloat(NULL),
  _signalBuffer(NULL)
{
}

AFSKAnalyser::~AFSKAnalyser()
{
  if (_sampleBuffer){
    free(_sampleBuffer);
  }

  if (_fftBuffer) {
    free(_fftBuffer);
  }

  if (_spectrumBuffer) {
    free(_spectrumBuffer);
  }

}

bool AFSKAnalyser::configure(AudioInI2S& input, int carrierFreq[], int carrierFreqSize, int minDeltaFreq){
  _sampleRate = input.sampleRate();
  _carrierFreqSize = carrierFreqSize;
  int _minDeltaFreq = minDeltaFreq;

  //Initialize fft
  if (ARM_MATH_SUCCESS != arm_rfft_init_q31(&_S31, _fftSize, 0, 1)) {
    return false;
  }

  double _freqResolution = _sampleRate/_fftSize;
  _windowSize = round(_minDeltaFreq/_freqResolution);
  // _windowSize = 6;

  // SerialUSB.println("WindowSize\t" + String(_windowSize));

  //Allocate frequency index pointers
  _pointFreq = calloc(_carrierFreqSize, sizeof(uint8_t)); // Index frequency

  //Allocate time buffer
  _sampleBuffer = calloc(_bufferSize, sizeof(q31_t));
  _fftBuffer = calloc(_fftSize, sizeof(q31_t));

  //Allocate frequency buffers
  _spectrumBuffer = calloc(_fftSize/2, sizeof(q31_t));
  _windowBuffer = calloc(_windowSize, sizeof(q31_t));
  _windowBufferFloat = calloc(_windowSize, sizeof(float)); 
  // _signalBuffer = calloc(_signalSize, sizeof(int));

  // Determine the index of carrierFrequencies with respect to fftSize Vector and SampleRate
  int32_t* ptFreq = (int32_t*) _pointFreq;

  for (int i = 0; i < _carrierFreqSize; i++) {
    ptFreq[i] = round(_fftSize/2*carrierFreq[i])/(_sampleRate/2); 
    SerialUSB.println(ptFreq[i]);
  }

  //Free all buffers in case of bad allocation
  if (_sampleBuffer == NULL || _fftBuffer == NULL || _spectrumBuffer == NULL) {

    if (_sampleBuffer) {
      free(_sampleBuffer);
      _sampleBuffer = NULL;
    }

    if (_fftBuffer) {
      free(_fftBuffer);
      _fftBuffer = NULL;
    }

    if (_spectrumBuffer) {
      free(_spectrumBuffer);
      _spectrumBuffer = NULL;
    }

    return false;
  }
  return true;
}

void AFSKAnalyser::signalGet(void *signal){

  if (audioInI2SObject.readBuffer(_sampleBuffer,_bufferSize)){
    // uint32_t timePre = micros();
    
    // Downscale the sample buffer for proper functioning and apply Hann window
    scalingandwindow(_sampleBuffer, _bufferSize);
    // SerialUSB.println(micros()-timePre);

    // FFT - EQUALIZATION
    fft(_sampleBuffer, _spectrumBuffer, _fftSize);
    // SerialUSB.println(micros()-timePre);

    // equalising(_spectrumBuffer, _fftSize/2);
    // SerialUSB.println(micros()-timePre);

    // Detect Peak on _spectrumBuffer and assign them to the mask
    unsigned char _signal = freqDetect(_spectrumBuffer, _pointFreq, _carrierFreqSize);
    
    SerialUSB.println(_signal);
  }
}

void AFSKAnalyser::fft(void *inputBuffer, void* outputBuffer, int fftBufferSize){
  //_inputBuffer is already treated for FFT (usable samples, averaged, windowed)
    
  // Calculate FFTBuffer ((r-i,r-i...))
  arm_rfft_q31(&_S31, (q31_t*)inputBuffer, (q31_t*)_fftBuffer);

  //Calculate spectrumBuffer and normalize
  const q31_t* _pfftBuffer = (const q31_t*)_fftBuffer;
  q31_t* _pspectrumBuffer = (q31_t*) outputBuffer;
    

  for (int i = 0; i < fftBufferSize; i +=2) {
    *_pspectrumBuffer = (*_pfftBuffer) * (*_pfftBuffer);
    _pfftBuffer++;
    
    *_pspectrumBuffer += (*_pfftBuffer) * (*_pfftBuffer);
    *_pspectrumBuffer = sqrt(*_pspectrumBuffer);

    //Normalize SpectrumBuffer
    if (i) {
      *_pspectrumBuffer = 2 * (*_pspectrumBuffer);
    }

    _pfftBuffer++;
    _pspectrumBuffer++;
  } 
}

unsigned char AFSKAnalyser::freqDetect(void* refBuffer, void* indexBuffer, int carrierFreqSize) {

  q31_t* _inputBuffer = (q31_t*) _spectrumBuffer;
  const int* _indexFreq = (const int*) indexBuffer;
  double _iBufferDB = 0;

  unsigned char _flags = 0;
  int _powerof2 = 0;

  // Calculate stdDev
  double _stdDev = 0;
  _stdDev = standardDeviation(_inputBuffer, _fftSize/2);

  // Loop over the amount of carrier frequencies and retrieve the active ones, giving back a bitmask
  for (int i = 0; i<carrierFreqSize; i++) {

    
    if (_inputBuffer[_indexFreq[i]]) { //If the _indexFreq[i]th component in _inputBuffer is non-zero
      _powerof2 = pow(2,i);
      /* TO ERASE
      // CONVERT TO DB THE VECTOR COMPONENT
      _inputBufferDB = FULL_SCALE_DBSPL-(FULL_SCALE_DBFS-20*log10(sqrt(2)*CONST_FACTOR*(_inputBuffer[_indexFreq[i]])));
      */
      
      if (isPeak(_inputBuffer, _indexFreq[i], _windowSize, _stdDev)) { // REPLACE WITH PEAK DETECTION ALGORITHM
        _flags |= _powerof2;
      } else {
        _flags &= ~_powerof2;
      }
    
    } else {
      _flags &= ~_powerof2;
    }

  }

  return _flags;
}

bool AFSKAnalyser::isPeak (const void* inputBuffer, int index, int windowSize, double stdDev) {
  uint32_t timePreAlloc = micros();

  bool _isPeak = false;

  //Determine the indexes without counting the middle one
  int _indexLeft = max(0,index-windowSize/2);
  int _indexRight = min(_fftSize/2, index+windowSize/2);

  // SerialUSB.println("windowSize\t" + String(windowSize));
  // SerialUSB.println("_index\t" + String(index));
  // SerialUSB.println("_indexLeft\t" + String(_indexLeft));
  // SerialUSB.println("_indexRight\t" + String(_indexRight));

  // Buffers
  q31_t* _inBuffer = (q31_t*) inputBuffer;
  q31_t* _winBuffer = (q31_t*) _windowBuffer;
  float* _winBufferFloat = (float*) _windowBufferFloat;

  // Max of window variables
  q31_t _maxWindow = _winBuffer[_indexLeft];
  int _indexMax = _indexLeft;

  // SerialUSB.println("Time after allocation\t"+String(micros()-timePreAlloc));

  // SerialUSB.println("_windowBuffer");
  //Extract the _windowBuffer and get the peak (value,index)
  for (int i = 0; i<(_indexRight-_indexLeft)+1 ; i++) {
    //Copy the value to the window
    _winBuffer[i] = _inBuffer[_indexLeft+i];
    // SerialUSB.println(_winBuffer[i]);

    //Check for the max value and update if needed
    if (_winBuffer[i] > _maxWindow) {
      _maxWindow = _winBuffer[i];
      _indexMax = _indexLeft + i;
    }
  }

  // SerialUSB.println("MAX\t" + String(_maxWindow));
  // SerialUSB.println("_indexMax\t"+String(_indexMax));

  //Convert to float since there is no INV matrix function for q31_t type in the arm cortex
  arm_q31_to_float(_winBuffer, _winBufferFloat, windowSize+1);
  // SerialUSB.println("Converted to float");
  // SerialUSB.println("Time after float \t"+String(micros()-timePreAlloc));
  int _mLeft = _indexMax - _indexLeft+1;
  // SerialUSB.println("_mLeft\t" + String(_mLeft));

  if (_mLeft>1) {
    //Initialise Left Matrix
    float _windowLeft [_mLeft];
    // SerialUSB.println("_windowLeft");

    for (int i = 0 ; i<_mLeft; i++){
      _windowLeft[i] = _winBuffer[i];
      // SerialUSB.println(_windowLeft[i]);
    }

    // normalEquation(_mLeft,_windowLeft, &_ordLeft, &_slopeLeft);
    // SerialUSB.println("Time before normal \t"+String(micros()-timePreAlloc));

    normalEquation(_mLeft,_windowLeft, &_slopeLeft);
    // SerialUSB.println("Time after normal \t"+String(micros()-timePreAlloc));

    // SerialUSB.println("ordinate left\t" + String(_ordLeft));
    // SerialUSB.println("slope left\t" + String(_slopeLeft));

  } else {
    _slopeLeft = 1;
  }

  int _mRight = _indexRight-_indexMax+1;
  // SerialUSB.println("_mRight\t" + String(_mRight));

  if (_mRight>1) {

    float _windowRight [_mRight];
    // SerialUSB.println("_windowRight");

    for (int i = 0; i<_mRight; i++){
      _windowRight[i] = _winBuffer[i+_mLeft-1];
      // SerialUSB.println(_windowRight[i]);
    }

    // normalEquation(_mRight,_windowRight, &_ordRight, &_slopeRight);
    // SerialUSB.println("Time before normal \t"+String(micros()-timePreAlloc));

    normalEquation(_mRight,_windowRight, &_slopeRight);
    // SerialUSB.println("Time after normal \t"+String(micros()-timePreAlloc));

    // SerialUSB.println("ordinate right\t" + String(_ordRight));
    // SerialUSB.println("slope right\t" + String(_slopeRight));
  } else {
    _slopeRight = -1;
  }

  if (_slopeLeft>0 && _slopeRight<0 && _maxWindow>stdDev*FACTOR_NOISE ) {
    // SerialUSB.println("It's a peak");
    _isPeak = true;
  }

  return _isPeak;
}

void AFSKAnalyser::normalEquation(int vectorLength, float window[], void* slope) {
  // Linear regression of buffer points
  // We basicly aim to do this:
  // window = vector of points to be regressed with equation like window = ordinate + slope*index
  // vectorLength = length(window);
  // X = [ones(vectorLength, 1) [1:1:vectorLength]'];
  // index = [1:1:vectorLength]
  // Solve the normal equation INV(X'*X)*X'*window

  // float* ord = (float*) ordinate;
  float* slp = (float*) slope;

  // SerialUSB.println("ord\t" + String(*ord));
  // SerialUSB.println("slp\t" + String(*slp));

  // INITIALISE ALL MATRIXES
  float _XN [vectorLength*2];
  float _XT [vectorLength*2];
  float _XTM [4];
  float _XTMI [4];
  float _XTMIT [vectorLength*2];
  float _theta [2];

  // arm_matrix_instance_f32 _F32Win;  //SIDE MATRIX X
  // arm_matrix_instance_f32 _F32X;  //SIDE MATRIX X
  // arm_matrix_instance_f32 _F32XT; //SIDE MATRIX X TRANSPOSED
  arm_matrix_instance_f32 _F32XTM; //SIDE MATRIX TRANSPOSED multiplied by SIDE MATRIX X
  arm_matrix_instance_f32 _F32XTMI; //INV(SIDE MATRIX TRANSPOSED multiplied by SIDE MATRIX X)
  // arm_matrix_instance_f32 _F32XTMIT; //INV(SIDE MATRIX TRANSPOSED multiplied by SIDE MATRIX X)*TRANSPOSED
  // arm_matrix_instance_f32 _F32THETA; //FINAL RESULT 
  
  // arm_mat_init_f32 (&_F32Win, vectorLength, 1, (float*) window);
  // arm_mat_init_f32 (&_F32X, vectorLength, 2, (float*) _XN);
  // arm_mat_init_f32 (&_F32XT, 2, vectorLength, (float*) _XT);
  arm_mat_init_f32 (&_F32XTM, 2, 2, (float*) _XTM);
  arm_mat_init_f32 (&_F32XTMI, 2, 2, (float*) _XTMI);
  // arm_mat_init_f32 (&_F32XTMIT, 2, vectorLength, (float*) _XTMIT);
  // arm_mat_init_f32 (&_F32THETA, 2, 1, (float*) _theta);

  // SerialUSB.println("vectorLength\t" + String(vectorLength));
  
  // SerialUSB.println("_XN");
  for (int i = 0; i<vectorLength; i++){
    _XN[i] = 1;
    _XN[i+vectorLength] = i+1;
    // SerialUSB.println(String(_XN[i])+"\t"+_XN[i+vectorLength]);
  }

  int _counterUP = 0;
  int _counterDOWN = vectorLength;
  //Calculate Transpose
  // arm_mat_trans_f32(&_F32X, &_F32XT);
  for (int i = 0; i < vectorLength*2; i+=2) {
    _XT[i] = _XN[_counterUP];
    _counterUP ++;
    _XT[i+1] = _XN[_counterDOWN];
    _counterDOWN ++;
  }

  // SerialUSB.println("_XN'");
  // for (int i = 0; i<vectorLength*2; i+=2) {
  //   SerialUSB.print(String(_XT[i]) + "\t");
  // }

  // SerialUSB.println("");
  // for (int i = 1; i<vectorLength*2+1; i+=2) {
  //   SerialUSB.print(String(_XT[i]) + "\t");
  // }

  //Calculate of Transpose (X) * (X)
  // arm_mat_mult_f32(&_F32XT, &_F32X, &_F32XTM);
  int sumCol1 = 0;
  int sumCol2 = 0;

  for (int i = 0; i<2; i++) {

    for (int j = 0; j <vectorLength; j++) {
      sumCol1 += _XT[j*2+i] * _XN[j];
      sumCol2 += _XT[j*2+i] * _XN[j+vectorLength];
    }

    _XTM[i] = sumCol1;
    _XTM[i+2] = sumCol2;
    sumCol1 = 0;
    sumCol2 = 0;
  }

  // SerialUSB.println("");
  // SerialUSB.println("_XN'* _XN");
  // for (int i = 0; i<2; i++) {
  //   SerialUSB.println(String(_XTM[i])+"\t"+String(_XTM[i+2]));
  // }

  //Calculate Inverse((Transpose(X) * X)
  arm_mat_inverse_f32(&_F32XTM, &_F32XTMI);

  // SerialUSB.println("");
  // SerialUSB.println("INV(_XN'* _XN)");
  // for (int i = 0; i<2; i++) {
  //   SerialUSB.println(String(_XTMI[i])+"\t"+String(_XTMI[i+2]));
  // }

  //Calculate (Inverse((Transpose(X) * X)) *  Transpose(X))
  // arm_mat_mult_f32(&_F32XTMI, &_F32XT, &_F32XTMIT);  

  for (int j = 0; j <vectorLength*2+1; j+=2) {
    _XTMIT[j] = _XTMI[0] * _XT[j] + _XTMI[2]*_XT[j+1];
    _XTMIT[j+1] = _XTMI[1] * _XT[j] + _XTMI[3]*_XT[j+1];
  }

 
  //Calculate ((Inverse((Transpose(A) * A)) *  Transpose(A)) * B)
  // arm_mat_mult_f32(&_F32XTMIT, &_F32Win, &_F32THETA);

  // _theta [0] = 0;
  _theta [1] = 0;

  for (int i = 0; i<vectorLength; i++) {
    // _theta[0] += _XTMIT[i*2]* window[i];
    _theta[1] += _XTMIT[i*2+1]* window[i];
  }

  // SerialUSB.println("");
  // SerialUSB.println("THETA");
  // for (int i = 0; i<2; i++) {
  //   SerialUSB.println(_theta[i]);
  // }

  // *ord = _theta[0];
  *slp = _theta[1];
}

double AFSKAnalyser::standardDeviation(void* inputBuffer, int bufferSize) {
  const q31_t* _inBuf = (const q31_t*) inputBuffer;
  double _stdDev = 0;
  double _avg = 0;

  _avg = average(inputBuffer, bufferSize);
  // SerialUSB.println("buffer");

  for (int i = 0; i < bufferSize; i++) {
    // SerialUSB.println(*_inBuf);
    _stdDev += (*_inBuf-_avg)*(*_inBuf-_avg);
    _inBuf++;
  }

  _stdDev = sqrt(_stdDev/bufferSize);
  // SerialUSB.println("stdDev");
  // SerialUSB.println(_stdDev);

  return _stdDev;
}

double AFSKAnalyser::average(void* inputBuffer, int bufferSize) {

  const q31_t* inBuf = (const q31_t*) inputBuffer;
  double _avg = 0;
  for (int i = 0; i < bufferSize; i++) {
    _avg += *inBuf;
    inBuf++;
  }

  _avg = _avg/bufferSize;

  return _avg;
}
