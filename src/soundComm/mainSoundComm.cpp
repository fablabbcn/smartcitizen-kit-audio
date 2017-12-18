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

#include "AudioAnalyser.h"
#include "AFSKAnalyser.h"
#include "AudioInI2S.h"

///// FFT Parameters
const int fftSize = 128;
const int bitsPerSample = 32;
const int channels = 2;
const int bufferSize = 256;
const int sampleRate = 20000;

///// OUTPUT
const int signalSize = 2;
int signal[signalSize];

// CARRIER FREQUENCIES VECTOR 
int timer = 0;
const int carrierFreqSize = 2;
int carrierFreq[carrierFreqSize] = {5000, 8000};
int minDeltaFreq = 500;

///// DEFINE ANALYSER
AFSKAnalyser afskAnalyser(bufferSize, fftSize, signalSize);

void setup() {

    while (timer < 30000000){
        timer++;
    }
    timer = 0;

	// Open SerialUSB communications
	SerialUSB.begin(115200);

    // BLINK LED
    pinMode(LED_BUILTIN, OUTPUT);

 	// Configure Analysis
    if(!audioInI2SObject.begin(sampleRate, bitsPerSample)){
        SerialUSB.println("Failed to init I2S");
    }

    if(!afskAnalyser.configure(audioInI2SObject, carrierFreq, carrierFreqSize, minDeltaFreq)){
        SerialUSB.println("Failed to init Analyser");
    }

    SerialUSB.println("*******");
    SerialUSB.println("Init Audio OK");
}

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

void loop() {

    //// Use this timer if you want to test some "time-spacing" between sensor readings
    // while (timer < 30000000){
    //     timer++;
    // }
    // timer = 0;

    //// READ WORD
    afskAnalyser.signalGet(signal);

    //// Make the LED blink
    digitalWrite(LED_BUILTIN, HIGH);
    delay(15);                  
    digitalWrite(LED_BUILTIN, LOW);
    delay(15);         

    // SerialUSB.println("FreeRamMem\t" + String(FreeRamMem()));              

}