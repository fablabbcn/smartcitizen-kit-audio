// This sketch is to be used with the native port of the Arduino Zero

/*
Circuit:
 * Arduino/Genuino Zero
 * ICS43432:
   * GND connected GND
   * 3.3V connected 3.3V
   * WS connected to pin 0
   * CLK connected to pin 1
   * SD connected to pin 9
*/

#include "FFTAnalyser.h"

///// FFT Parameters
const int fftSize = 512;
const int bitsPerSample = 32;
const int channels = 2;
const int bufferSize = 512;
const int sampleRate = 44100;

#define RED 6
#define GREEN 12
#define BLUE 10

///// OUTPUT
int spectrum[fftSize/2];
float resultdB = 0;
int timer = 0; 

class NoiseClass{

FFTAnalyser _fftAnalyser;

public:

    NoiseClass(int sampleRate, int bufferSize, int fftSize, WeightingType weightingType) :
        _bufferSize(bufferSize),
        _sampleRate(sampleRate),
        _fftSize(fftSize),
        _weightingType(weightingType)
    {        
    }

    ~NoiseClass(){
        // free(_fftAnalyser);
    }

    enum NoiseState{
        INITIAL,
        READINGBUFFER,
        READINGDONE,
        ALLOCATINGBUFFER
    };

    bool init(){
        bool _initI2SOK = _fftAnalyser.initI2S(_sampleRate,32);
        bool _initFFTOK = _fftAnalyser.initFFT(_fftSize);
        if (_initFFTOK && _initI2SOK){
            return true;
        }
        return false;
    }
    void end(){
        // Rien de rien
    }

    float getReading(){
        float _noise = 0;

        switch (_noiseState) {
            case INITIAL:

                if (init()){
                    _noiseState = ALLOCATINGBUFFER;
                }
                break;

            case ALLOCATINGBUFFER:

                if(_fftAnalyser.allocateBuffer(_bufferSize, _fftSize, _weightingType)){
                    _noiseState = READINGBUFFER;
                }
                break;

            case READINGBUFFER:

                _noise = _fftAnalyser.getReading();
                if (_noise>0){
                    _noiseState = READINGDONE;
                }
                break;

            case READINGDONE:

                if(_fftAnalyser.terminateBuffer()){
                    _noiseState = ALLOCATINGBUFFER;
                }
                break;
        }

        return _noise;
    } 

private:
    int _bufferSize;
    int _fftSize;
    int _sampleRate;
    bool _bufferTerminated = true;
    WeightingType _weightingType;
    NoiseState _noiseState = INITIAL;


    uint32_t FreeRamMem() {
        uint32_t stackTop;
        uint32_t heapTop;

        // Current position of the stack
        stackTop = (uint32_t) &stackTop;

        // Current position of heap
        void* hTop = malloc(1);
        heapTop = (uint32_t) hTop;
        free(hTop);

        // The difference is the free, available ram
        return stackTop - heapTop;
    }
};

NoiseClass noisedBA(sampleRate, bufferSize, fftSize, A_WEIGHTING);

uint32_t FreeRamMem() {
    uint32_t stackTop;
    uint32_t heapTop;

    // Current position of the stack
    stackTop = (uint32_t) &stackTop;

    // Current position of heap
    void* hTop = malloc(1);
    heapTop = (uint32_t) hTop;
    free(hTop);

    // The difference is the free, available ram
    return stackTop - heapTop;
}

void setup() {
    // Provisional For SCK
    pinMode(3, OUTPUT);
    digitalWrite(3, HIGH);
    pinMode(4, OUTPUT);
    digitalWrite(4, HIGH);

    // BLINK LED
    pinMode(RED, OUTPUT); //ROJO
    pinMode(GREEN, OUTPUT); //VERDE
    pinMode(BLUE, OUTPUT); //BLUE
    digitalWrite(RED, HIGH);
    digitalWrite(GREEN, HIGH);
    digitalWrite(BLUE, HIGH);

    delay(2000);

    // SerialUSB.println(FreeRamMem());
}

void loop() {
    
    float resultdBA = noisedBA.getReading();

    if (resultdBA) {
        SerialUSB.println(resultdBA);
        digitalWrite(GREEN, LOW);
        delay(20);
        digitalWrite(GREEN, HIGH);
        delay(20);
    }

    // SerialUSB.println(FreeRamMem());

    // Do something else
    // while (timer<200000) {
    //     timer++;
    // }
    // timer = 0;

}