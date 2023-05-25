#include "BFSKrx.h"

void setup() {
  Serial.begin(115200);
  Serial.println("OpenModem FSK RX");
//  // BFSKrxSetup takes in coeff of 0 bit freq, 1 bit freq, bit rate and code length
//  //BFSKrxSetup(float(0.13), float(-0.37), float (100), 100); // 24khz, 28khz 
//
//  // BFSKrxSetup for 44khz, 48kHz, 5hz data rate
//  BFSKrxSetup(float(1.1756), float(1.0), float (100), 50); // 44khz, 48khz 
//  BFSKrxADCSetup();
//  BFSKrxStart();

//Transmission freq = 1/(bitrate/2)
  BFSKrxSetup(float(0.4363), float(0.3129), float (10), 16); // 64khz, 67khz 
  //BFSKrxSetup(float(0.4363), float(0.2507), float (10), 16); // 65khz, 68khz 

  BFSKrxADCSetup();
  BFSKrxStart();
}

void loop() {
}
