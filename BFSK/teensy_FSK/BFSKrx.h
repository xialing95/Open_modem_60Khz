/*   MIT Future Ocean Lab  
--------------------------------------------------------------------------------
 Project:       FOL OpenModem
 Version:       V1
 Design:        OpenModem Hardware V2
 Substrate:     Teensy 4.0
--------------------------------------------------------------------------------
 Module:        nFSK demodulation Library
 Filename:      BFSKrx.cpp
 Created:       June 10 2020
 Author:        Charlene Xia (cxia_1@mit.edu)
--------------------------------------------------------------------------------
 Description:   BFSKrx is a Teensy 4.0 executable that operates with the FOL 
                OpenModem V2 hardware. It is works with the processing board 
                connected to the receiver board. The main purpose is to ...

 1. Hardware connection and interface between Processing & Recevier board
 2. Decode received nFSK signal via Goertzel Algorithm and output received 
    data

 In a little more detail.....
 - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    A. How the Library Works


    B. Hardware connections:


    C. Quick Summary of objects and functions, details above each function:

 - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
 Notes: 
    
    Todo: ADD ring buffer, threshold detection, windowing 
          ADD calculation for coefficients

*/

#ifndef BFSKRX_H
#define BFSKRX_H

//TODO: #include "CircularBuffer.h"
#include "arduino.h"
#include "ADC.h"
#include "ADC_util.h"

/*
 User Control Variables
 Details on how to change variables in Pubpub page.
 - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    Digital Pin Out
    analogInPin

*/

// hardware pin setup
#define digitalOutPin1 7
#define digitalOutPin2 8

// ADC hardwae setup 
#define analogInPin A0 // pin 14 
#define samplingFreq 300000 //increase for 44khz/48khz FSK
#define USE_ADC_0
ADC *adc = new ADC();

// FSK Receiver setting 
#define N_FSK 2
int package_length;
int outputValue = 0;  

// Goertzel Filter variable 
// TODO: CircularBuffer<uint16_t, BUFFER_SIZE> BUFFER;
#define BUFFER_SIZE 200 //120 switch to 80 instead of 20
#define THRESHOLD_VALUE 2000000 //1000000

// Goertzel Filter variable in ADC IRS
volatile uint16_t buffer_adc_0_count = 0;
float coeff[N_FSK];
float mag[N_FSK];
float q1[N_FSK], q2[N_FSK], q0[N_FSK];

// decision maker in PIT IRS
float usec;
uint32_t cycles;
volatile int start_check = 0;
volatile int currBit;
volatile int prevBit;
volatile int bitCount = 0;

// external function
void BFSKrxSetup(uint16_t, uint16_t, float, int);
void BFSKrxADCSetup();
void BFSKrxStart();
void BFSKrxStop();

// Trigger release setup
void releaseTrigger();
void resetCode();

// Please set the agreed upon TX and RX code for release
#define codeByteLength 2
uint8_t receivedCode[codeByteLength];
uint8_t startCode[codeByteLength] = {
//  0xFF, //BIN 11111111
//  0x95, //BIN 10101001 
  0xAA, //BIN 10101010
  0xAA, //BIN 10101010
};

// Hardware Motor Trigger
#define triggerPin 4
#define triggerDuration 30000 //use 30000 milliseconds 30 seconds

// internal function
void adc_isr();
void PIT_setup();
void mod_isr();
 
#endif

/*
 object class: BFSKtx
 Description:   Setting up frequencies and hardware pinmode for transmission.
                

 - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
 Variable:

    bit_duration: the period of the one bit in milliseconds. Usually you want
    to think of transmission as x bit/bytes/baud per seconds.
    calculation is 1000 msec/sec / (N bit/sec)
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
NOTE: not sure why i convert bit_duration to milliseconds, then switch it back
to second in modBit.
*/
void BFSKrxSetup(float freqCoeff0, float freqCoeff1, float bitRate, int packageBit){
    pinMode(analogInPin, INPUT);
    pinMode(digitalOutPin1, OUTPUT);
    pinMode(digitalOutPin2, OUTPUT);

    coeff[0] = freqCoeff0; 
    coeff[1] = freqCoeff1;
    
    package_length = packageBit;
    usec = (float)1000000.0 / (float)bitRate;
    cycles = float(24000000 / 1000000) * usec - 1;
}

/*
 Function: BFSKrxADCSetup()
 Description:   Setting up Teensy 4.0 ADC for sampling
                
 - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
 Setting up ADC:

    Detail in datasheet page:
    Also described in Pubpub page.
*/
void BFSKrxADCSetup(){
    // setup ADC0 configuration
    adc->adc0->setAveraging(1);
    adc->adc0->setResolution(8);
    adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_HIGH_SPEED);
    adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED);
}

/*
 Function: BFSKrxStart()
 Description:   Starting ADC sampling and interrupts. 

 - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
 Setting up ADC:

    Detail in datasheet page:
    Also described in Pubpub page.
*/
void BFSKrxStart(){
    // Setup ADC0 interrupt start sampling
    adc->adc0->stopQuadTimer();
    adc->adc0->startSingleRead(analogInPin);
    adc->adc0->enableInterrupts(adc_isr);
    adc->adc0->startQuadTimer(samplingFreq);

    PIT_setup();
    // start PIT interrupt 
    // enable timer and interrupt
    PIT_TCTRL0 = 0x03;
    // enable IRQ
    NVIC_ENABLE_IRQ(IRQ_PIT);  
}

/*
 Function: BFSKrxStop()
 Description:   Stop ADC sampling and Interrupt.

 - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
 Setting up ADC:

    Detail in datasheet page:
    Also described in Pubpub page.
*/
void BFSKrxStop(){
  // Stop ADC0 stop interrupt sampling
  adc->adc0->stopTimer();
  adc->adc0->disableInterrupts();

  // disable timer and interrupt 
  PIT_TCTRL0 = 0;
  
  // disable IRQ_PIT
  NVIC_DISABLE_IRQ(IRQ_PIT);
}

/*
 Function: BFSKrxStop()
 Description:   Stop ADC sampling and Interrupt.

 - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
 Setting up ADC:

    Detail in datasheet page:
    Also described in Pubpub page.
*/
void BPSKrxStop(){
  // Stop ADC0 stop interrupt sampling
  adc->adc0->stopTimer();
  adc->adc0->disableInterrupts();

  // disable timer and interrupt 
  PIT_TCTRL0 = 0;
  
  // disable IRQ_PIT
  NVIC_DISABLE_IRQ(IRQ_PIT);
}

/*
 Function adc_isr()
 Description:   Interrupt service routing. Triggered by PIT, when called
                it toggles the transmission pins high and low for the right  
                number of times, then disable the PIT timer and interrupts. 

 - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
 Setting up Goertzel Algorithm:
    
    Detail in Pubpub Page. 
*/
void adc_isr(){
    // read a new value
    uint8_t adc_val = adc->adc0->readSingle();
    
    // check the ADC sampling rate
    //digitalWriteFast(digitalOutPin2, !digitalRead(digitalOutPin2));
    // check ADC value, serial output
//    adc->adc0->isComplete();
//    //Serial.println(adc_val);
//    analogWrite(A9, map(adc_val, 0, 254, 0, 64));
    
    // saving the ADC value to the Buffer
    for (int i = 0; i < N_FSK; i++){
        // should just added the - 2 hmmmm. check the counting on this again
        if (buffer_adc_0_count < BUFFER_SIZE*N_FSK-2){
            q0[i] = float(adc_val) + coeff[i] * q1[i] - q2[i];
            q2[i] = q1[i];
            q1[i] = q0[i];
            buffer_adc_0_count++;
        } else {
            // reset buffer count
            buffer_adc_0_count = 0;

            // calculate goertzel magnitude, normalized by dividing via Buffer size
            mag[i] = q1[i] * q1[i] + q2[i] * q2[i] - q1[i]*q2[i]*coeff[i];
            
            // reset goertzel variable
            q1[i] = 0;
            q2[i] = 0;    
        }
    }

      // Threshold Value        
  if (mag[0] > THRESHOLD_VALUE && mag[1] < THRESHOLD_VALUE){ 
      digitalWriteFast(digitalOutPin1, 0);
      //digitalWriteFast(digitalOutPin2, 1);
      //Serial.println(1);
  } else if (mag[1] > THRESHOLD_VALUE && mag[0] < THRESHOLD_VALUE){
      digitalWriteFast(digitalOutPin1, 1);
      //digitalWriteFast(digitalOutPin2, 0);
      //Serial.println(0);
  } else {
//l;      digitalWriteFast(digitalOutPin1, 0);
//      digitalWriteFast(digitalOutPin2, 0);
      //Serial.println(1);
  }

    // Add low pass filter?
    // Add decision slicer?
    
    #if defined(__IMXRT1062__)  // Teensy 4.0
        asm("DSB");
    #endif
}

/*
 Function PIT_setup
 Description:    

 - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
*/

// setup PIT timer for time slicer
void PIT_setup(){
  // setting the clock on for PIT
  // CCM_CCGR_1_PIT(CCM_CCGR_ON) = 0x3000
  CCM_CCGR1 |= CCM_CCGR1_PIT(CCM_CCGR_ON);

  // Turn on PIT module
  PIT_MCR = 0x00;

  // set the Timer Load Value Register
  PIT_LDVAL0 = cycles;

  // attach interrupt vector 
  attachInterruptVector(IRQ_PIT, &mod_isr);

  // set priority
  NVIC_SET_PRIORITY(IRQ_PIT, 255);
}

/*
 Function mod_isr()
 Description:   Receiving the bits, saving into array and comparing to triggerCode

 - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
*/

// add if start bit 
void mod_isr(){
  // reset the TFLG 
  PIT_TFLG0 = 1;

  // Test checking decision slicer rate
  // digitalWriteFast(digitalOutPin2, !digitalReadFast(digitalOutPin2));
   
  currBit = digitalReadFast(digitalOutPin1);

  // For debugging the magnitude value.    
//  Serial.print(mag[0]);
//  Serial.print(",");
//  Serial.print(mag[1]);
//  Serial.print(",");
//  Serial.println(3000000);

  // Debuging serial bit received
  Serial.print(currBit);
  Serial.print(",");

  if (currBit && start_check == 0){
    start_check = 1;
    // reset/zero receivedCode memory bytes
    memset(receivedCode, 0, sizeof(receivedCode));
    Serial.println("Recevied starting"); 
  }
  
  if (start_check){
    if (currBit) {
        bitSet(receivedCode[bitCount/8], bitCount%8);
        bitCount ++;

      }
      
    else if (!currBit) {
        bitClear(receivedCode[bitCount/8], bitCount%8);
        bitCount ++;
      }
      
    // debugging using serial print
//    Serial.print("DATA: ");
//    for (int i = 0 ; i < sizeof(receivedCode); i++) {
//      Serial.print(receivedCode[i], BIN);
//    }
//    Serial.print(" Bitcount: ");
//    Serial.println(bitCount);

  } 
  
  if (bitCount == codeByteLength*8+1) {
    if (memcmp(receivedCode, startCode, sizeof(startCode)) == 0){
      Serial.println("Signal Correct");
      releaseTrigger();
      resetCode();
    }
    else if (memcmp(receivedCode, startCode, sizeof(startCode)) != 0) {
      Serial.println("Signal Error, restart");
      resetCode();
    }
  } else if(bitCount > codeByteLength*8){
    //Serial.println("Signal Overflow, restart");
    resetCode();
  } else {
    //Serial.println("Receiver Error");
  }
}

void resetCode() {
  bitCount = 0; 
  memset(receivedCode, 0, sizeof(receivedCode));
}

void releaseTrigger() {
  pinMode(triggerPin, OUTPUT);
//  Serial.println("Trigger Start");
//  digitalWrite(triggerPin, HIGH); //pin 4 HIGH
//  digitalWrite(13, HIGH); //pin 4 HIGH
//  delay(triggerDuration); // for 30000 for 30 second hold
//  digitalWrite(triggerPin, LOW);
//  digitalWrite(13, LOW); //pin 4 HIGH
//  Serial.println("Trigger End");

  for (int i = 0; i <= 255; i++) {
    digitalWrite(triggerPin, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(10);                       // wait for a second
    digitalWrite(triggerPin, LOW);    // turn the LED off by making the voltage LOW
    delay(10);                       // wait for a second
  }
}
