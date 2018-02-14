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

    void init(){
        _fftAnalyser.initI2S(_sampleRate,32);
        _fftAnalyser.configure(_bufferSize, _fftSize, _weightingType);     
        SerialUSB.println(FreeRamMem());
    }
    void end(){
        // Rien de rien
    }

    float getReading(){
        // SerialUSB.println("GetReading");

        float _noise = 0;
        // Modify it so that it doesn't come back before it get the data
        if (_fftAnalyser.bufferFilled()) {
            _noise = _fftAnalyser.getReading();
            SerialUSB.println("bufferFilled");
            SerialUSB.println(FreeRamMem());
            // _fftAnalyser->terminate();
        }
        return _noise;
    } 

private:
    int _bufferSize;
    int _fftSize;
    int _sampleRate;

    WeightingType _weightingType;

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

// NoiseClass noisedBA(sampleRate, bufferSize, fftSize, A_WEIGHTING);
// NoiseClass noisedBC(sampleRate, bufferSize, fftSize, C_WEIGHTING);
// NoiseClass noisedBZ(sampleRate, bufferSize, fftSize, Z_WEIGHTING);

NoiseClass noise(sampleRate, bufferSize, fftSize, A_WEIGHTING);

void setup() {
    // Provisional For SCK
    pinMode(3, OUTPUT);
    digitalWrite(3, HIGH);
    pinMode(4, OUTPUT);
    digitalWrite(4, HIGH);

    // BLINK LED
    pinMode(6, OUTPUT); //ROJO
    pinMode(12, OUTPUT); //VERDE
    pinMode(10, OUTPUT); //BLUE
    digitalWrite(6, HIGH);
    digitalWrite(12, HIGH);
    digitalWrite(10, HIGH);

    delay(2000);

    // SerialUSB.println(FreeRamMem());
    noise.init();
}

void loop() {
    
    // SerialUSB.println(FreeRamMem());
    float resultDB = noise.getReading();
    SerialUSB.println(resultDB);

    // Make the led blink
    digitalWrite(6, HIGH);
    digitalWrite(12, HIGH);
    digitalWrite(10, HIGH);
    delay(20);
    digitalWrite(6, HIGH);
    digitalWrite(12, LOW);
    digitalWrite(10, HIGH);
    delay(20);
    
    // SerialUSB.println(FreeRamMem());
    // noise->end();

    // Do something else
    // while (timer<200000) {
    //     timer++;
    // }
    // timer = 0;

}