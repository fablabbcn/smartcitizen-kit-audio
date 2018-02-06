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

#include "AFSKAnalyser.h"

///// FFT Parameters
const int fftSize = 512; // CAREFUL IF YOU MODIFY IT!
const int bitsPerSample = 32;
const int channels = 2;
const int bufferSize = 512;
const int sampleRate = 22050; // CAREFUL IF YOU MODIFY IT!

///// OUTPUT
unsigned char signal;

// CARRIER FREQUENCIES VECTOR 
int timer = 0;
const int carrierFreqSize = 8;
int carrierFreq[carrierFreqSize] = {3000,4100,5200,6300,7400,8500,9600,10700};

///// DEFINE ANALYSER
AFSKAnalyser afskAnalyser(bufferSize, fftSize);

void setup() {

    // FOR SCK
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

 	// Configure Analysis
    afskAnalyser.configure(sampleRate, bitsPerSample, carrierFreq, carrierFreqSize);

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

    //// READ WORD
    signal = afskAnalyser.signalGet();

    // SerialUSB.println("FreeRamMem\t" + String(FreeRamMem()));              

}