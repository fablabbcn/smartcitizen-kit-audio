// This sketch is meant for reading audio over 

#include "FFTAnalyser.h"

///// FFT Parameters
const int fftSize = 512;
const int bitsPerSample = 32;
const int channels = 1;
const int bufferSize = 512;
const int sampleRate = 44100;

///// OUTPUT
int spectrum[fftSize/2];
float resultdB = 0;

///// DEFINE ANALYSER
// Declare weighting type in the last argument. For reference below
/*
enum WeightingType{
    A_WEIGHTING,
    C_WEIGHTING,
    Z_WEIGHTING
*/
FFTAnalyser fftAnalyser(bufferSize, fftSize, A_WEIGHTING);

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

    // Init I2S and configure fft analyser, buffers..
    fftAnalyser.configure(sampleRate, bitsPerSample);
}

void loop() {

    if (fftAnalyser.bufferFilled()) {

        // Get only dB results
        resultdB = fftAnalyser.getReading();
        // Get dB and specturm
        // resultdB = fftAnalyser.getReading(spectrum);
        //// Make the LED blink
        digitalWrite(12, LOW); // TURN IT ON
        delay(20);          
        digitalWrite(12, HIGH); // TURN IT OFF
        delay(20);   
        SerialUSB.println(resultdB); 

        // Print out the spectrum
        // SerialUSB.println("Buffer Results (arduino)");    
        // for (int i = 0; i < fftSize/6; i++) {
        //     SerialUSB.print((i * sampleRate) / fftSize);
        //     SerialUSB.print("\t");
        //     SerialUSB.print(spectrum[i]);
        //     SerialUSB.println("");
        // }
        // SerialUSB.println("--");

    } 
}