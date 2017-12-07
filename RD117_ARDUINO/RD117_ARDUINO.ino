/** \file RD117_LILYPAD.ino ******************************************************
*
* Project: MAXREFDES117#
* Filename: RD117_LILYPAD.ino
* Description: This module contains the Main application for the MAXREFDES117 example program.
*
* Revision History:
*\n 1-18-2016 Rev 01.00 GL Initial release.
*\n
*
* --------------------------------------------------------------------
*
* This code follows the following naming conventions:
*
* char              ch_pmod_value
* char (array)      s_pmod_s_string[16]
* float             f_pmod_value
* int32_t           n_pmod_value
* int32_t (array)   an_pmod_value[16]
* int16_t           w_pmod_value
* int16_t (array)   aw_pmod_value[16]
* uint16_t          uw_pmod_value
* uint16_t (array)  auw_pmod_value[16]
* uint8_t           uch_pmod_value
* uint8_t (array)   auch_pmod_buffer[16]
* uint32_t          un_pmod_value
* int32_t *         pn_pmod_value
*
* ------------------------------------------------------------------------- */
/*******************************************************************************
* Copyright (C) 2016 Maxim Integrated Products, Inc., All Rights Reserved.
*
* Permission is hereby granted, free of charge, to any person obtaining a
* copy of this software and associated documentation files (the "Software"),
* to deal in the Software without restriction, including without limitation
* the rights to use, copy, modify, merge, publish, distribute, sublicense,
* and/or sell copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included
* in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
* OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
* IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
* OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
*
* Except as contained in this notice, the name of Maxim Integrated
* Products, Inc. shall not be used except as stated in the Maxim Integrated
* Products, Inc. Branding Policy.
*
* The mere transfer of this software does not imply any licenses
* of trade secrets, proprietary technology, copyrights, patents,
* trademarks, maskwork rights, or any other form of intellectual
* property whatsoever. Maxim Integrated Products, Inc. retains all
* ownership rights.
*******************************************************************************
*/
/*!\mainpage Main Page
*
* \section intro_sec Introduction
*
* This is the code documentation for the MAXREFDES117# subsystem reference design.
* 
*  The Files page contains the File List page and the Globals page.
* 
*  The Globals page contains the Functions, Variables, and Macros sub-pages.
*
* \image html MAXREFDES117_Block_Diagram.png "MAXREFDES117# System Block Diagram"
* 
* \image html MAXREFDES117_firmware_Flowchart.png "MAXREFDES117# Firmware Flowchart"
*
*/
#include <Arduino.h>
#include "algorithm.h"
#include "max30102.h"

#include <SPI.h>
#include <ADXL362.h>

bool isInitial;
bool isStable;

ADXL362 xl;

//if Adafruit Flora development board is chosen, include NeoPixel library and define an NeoPixel object
#if defined(ARDUINO_AVR_FLORA8)
#include "adafruit_neopixel.h"
#define BRIGHTNESS_DIVISOR 8  //to lower the max brightness of the neopixel LED
Adafruit_NeoPixel LED = Adafruit_NeoPixel(1, 8, NEO_GRB + NEO_KHZ800);
#endif

#define MAX_BRIGHTNESS 255
#define PPG_INTERRUPT 8
#define ADXL_CS 10
#define BUTTON_PIN  3
#define STABLE_MAX  30
#define STABLE_MIN  -30

#if defined(ARDUINO_AVR_UNO)
//Arduino Uno doesn't have enough SRAM to store 100 samples of IR led data and red led data in 32-bit format
//To solve this problem, 16-bit MSB of the sampled data will be truncated.  Samples become 16-bit data.
uint16_t aun_ir_buffer[100]; //infrared LED sensor data
uint16_t aun_red_buffer[100];  //red LED sensor data
#else
uint32_t aun_ir_buffer[100]; //infrared LED sensor data
uint32_t aun_red_buffer[100];  //red LED sensor data
#endif
int32_t n_ir_buffer_length; //data length
int32_t n_spo2;  //SPO2 value
int8_t ch_spo2_valid;  //indicator to show if the SPO2 calculation is valid
int32_t n_heart_rate; //heart rate value
int8_t  ch_hr_valid;  //indicator to show if the heart rate calculation is valid
uint8_t uch_dummy;

int buttonState = 0;

// the setup routine runs once when you press reset:
void setup() {

//#if defined(ARDUINO_AVR_LILYPAD_USB)    
//  pinMode(13, OUTPUT);  //LED output pin on Lilypad
//#endif
//
//#if defined(ARDUINO_AVR_FLORA8)
//  //Initialize the LED
//  LED.begin();
//  LED.show();
//#endif

  maxim_max30102_reset(); //resets the MAX30102
  // initialize serial communication at 115200 bits per second:
  Serial.begin(9600);

  pinMode(BUTTON_PIN, INPUT);

  //for ADXL
  xl.begin(ADXL_CS);       // Setup SPI protocol, issue device soft reset
  xl.beginMeasure();              // Switch ADXL362 to measure mode
  isInitial = true;
  //for PPG
  pinMode(PPG_INTERRUPT, INPUT);  //pin D10 connects to the interrupt output pin of the MAX30102
  delay(1000);
  maxim_max30102_read_reg(REG_INTR_STATUS_1,&uch_dummy);  //Reads/clears the interrupt status register
//  while(Serial.available()==0)  //wait until user presses a key
//  {
//    Serial.write(27);       // ESC command
//    Serial.print(F("[2J"));    // clear screen command
//    Serial.println(F("Press any key to start conversion"));
//    delay(1000);
//  }
  uch_dummy=Serial.read();
  maxim_max30102_init();  //initialize the MAX30102
  delay(1000);
  maxim_max30102_LED_turnOFF();
}

// the loop routine runs over and over again forever:
void loop() {
  buttonState = digitalRead(BUTTON_PIN);
  int16_t i;
  int16_t PPG_i;
  //if botton is pressed -- > check stable --> Start recording data
  if(buttonState == HIGH){
    isInitial = true;
    Serial.print(buttonState);
    isStable = checkStable();
    if (isStable == true){
      
      Serial.print("stable\n");
      getPPG();
    }
    else{
      Serial.print("not stable\n");
    }
  }
  
    // read all three axis in burst to ensure all measurements correspond to same sample time
//  checkStable();
//  getPPG();
}



///////////////////////////////////// Fcx /////////////////////////////////

void getADXLRawData(){
  int16_t XValue, YValue, ZValue, Temperature;
  xl.readXYZTData(XValue, YValue, ZValue, Temperature);  
  Serial.print("XVALUE=");
  Serial.print(XValue);   
  Serial.print("\tYVALUE=");
  Serial.print(YValue);  
  Serial.print("\tZVALUE=");
  Serial.print(ZValue);  
  Serial.print("\n"); 
  delay(500); // Arbitrary delay to make serial monitor easier to observe
}

uint8_t checkStable(){
  bool isStable;
  int16_t initialXValue, initialYValue, initialZValue;
  int16_t XValue, YValue, ZValue, Temperature;
  int16_t i;
  int16_t getRidOfInitData = 3;
  for (i=0; i<getRidOfInitData; i++){
    if(isInitial == true && i == 2){
      xl.readXYZTData(initialXValue, initialYValue, initialZValue, Temperature);  
      Serial.print(isInitial);
      isInitial = false;
      i = 0;
    }
    
  }

//Serial.print("initialXValue=");
//  Serial.print(initialXValue);   
//  Serial.print("\tinitialYValue=");
//  Serial.print(initialYValue);  
//  Serial.print("\ttinitialZValue=");
//  Serial.print(initialZValue);  
//  Serial.print("\n"); 

  delay(1000); 
  //start recording next data
  xl.readXYZTData(XValue, YValue, ZValue, Temperature); 
  
  if (((XValue - initialXValue) < STABLE_MAX && (XValue - initialXValue) > STABLE_MIN)
      && ((YValue - initialYValue) < STABLE_MAX && (YValue - initialYValue) > STABLE_MIN)
      && ((ZValue - initialZValue) < STABLE_MAX && (ZValue - initialZValue) > STABLE_MIN)
      ) {
    isStable = true;
  }
  else{
    isStable = false;
  }
  return isStable;
}


void getPPG(){
  uint32_t un_min, un_max, un_prev_data, un_brightness;  //variables to calculate the on-board LED brightness that reflects the heartbeats
  int32_t i;
  float f_temp;
  
  un_brightness=0;
  un_min=0x3FFFF;
  un_max=0;
  
  n_ir_buffer_length=100;  //buffer length of 100 stores 4 seconds of samples running at 25sps
  
  maxim_max30102_LED_turnON();
  delay(100);   
  //read the first 100 samples, and determine the signal range
  for(i=0;i<n_ir_buffer_length;i++)
  {
    while(digitalRead(PPG_INTERRUPT)==1);  //wait until the interrupt pin asserts
    maxim_max30102_read_fifo((aun_red_buffer+i), (aun_ir_buffer+i));  //read from MAX30102 FIFO
    
    if(un_min>aun_red_buffer[i])
      un_min=aun_red_buffer[i];  //update signal min
    if(un_max<aun_red_buffer[i])
      un_max=aun_red_buffer[i];  //update signal max
//    Serial.print(F("red="));
    Serial.print(aun_red_buffer[i], DEC);
    Serial.print("\n");
//    Serial.print(F(", ir="));
//    Serial.println(aun_ir_buffer[i], DEC);
  }
  delay(100); 
  maxim_max30102_LED_turnOFF();
}




 
