 

  
#include <SPI.h>  
#include <ADXL362.h>
#include <Arduino.h>
#include "algorithm.h"
#include "max30102.h"


#define PPG_INTERRUPT 8


const int chipSelectPin_Temp = 10;  
const int chipSelectPin_ADXL = 7;

ADXL362 xl;

int16_t temp;
int16_t XValue, YValue, ZValue, Temperature;

int32_t n_ir_buffer_length; //data length
uint8_t uch_dummy;
uint32_t aun_ir_buffer[100]; //infrared LED sensor data
uint32_t aun_red_buffer[100];  //red LED sensor data
  
void setup() {  
  Serial.begin(9600);  
  SPI.setDataMode(SPI_MODE3);// ADT7320 Supports only MODE3 SPI  
  // start the SPI library:  
  SPI.begin();  
  // initalize the  data ready and chip select pins:  
  pinMode(chipSelectPin_Temp, OUTPUT);  

  xl.begin(chipSelectPin_ADXL);   // Setup SPI protocol, issue device soft reset
  xl.beginMeasure();              // Switch ADXL362 to measure mode  

  maxim_max30102_reset(); //resets the MAX30102

  pinMode(PPG_INTERRUPT, INPUT);  //pin D10 connects to the interrupt output pin of the MAX30102
  delay(1000);
  maxim_max30102_read_reg(REG_INTR_STATUS_1,&uch_dummy);  //Reads/clears the interrupt status register

  uch_dummy=Serial.read();
  maxim_max30102_init();  //initialize the MAX30102
  delay(1000);
}  
  
  
void loop() {  
    //Read the temperature data  
    int tempData = readRegister(0x50, 2); // 0x50 is read commad for 0x02 register  
    tempData = tempData/8;// MSB bit15 and LSB bit4 so received value need to be divide/8  
  
  
    // convert the temperature to celsius and display it:  
    float realTemp = (float)tempData * 0.0625;  
    Serial.print("Temp[C]=");  
    Serial.print(realTemp);  
    Serial.print("\tRXdata>>3=");  
    Serial.println(tempData);  
    delay(500);  
  
  readADXLRawData();
  delay(1000);
  getPPG();
  delay(100);
}  


void getPPG(){
  uint32_t un_min, un_max, un_prev_data, un_brightness;  //variables to calculate the on-board LED brightness that reflects the heartbeats
  int32_t i;
  float f_temp;
  
  un_brightness=0;
  un_min=0x3FFFF;
  un_max=0;
  
  n_ir_buffer_length=100;  //buffer length of 100 stores 4 seconds of samples running at 25sps
  
//  maxim_max30102_LED_turnON();
//  delay(1000);   
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
//  delay(100); 
//  maxim_max30102_LED_turnOFF();
}


void readADXLRawData(){
  // read all three axis in burst to ensure all measurements correspond to same sample time
  xl.readXYZTData(XValue, YValue, ZValue, Temperature);  
  Serial.print("XVALUE=");
  Serial.print(XValue);   
  Serial.print("\tYVALUE=");
  Serial.print(YValue);  
  Serial.print("\tZVALUE=");
  Serial.print(ZValue);  
  Serial.print("\tTEMPERATURE=");
  Serial.println(Temperature);   
  delay(100);                // Arbitrary delay to make serial monitor easier to observe
}
  
//Read from register from the ADT7320:  
unsigned int readRegister(byte thisRegister, int bytesToRead ) {  
  byte inByte = 0;          // incoming byte from the SPI  
  unsigned int result = 0;  // result to return  
  // take the chip select low to select the device:  
  digitalWrite(chipSelectPin_Temp, LOW);  
  // send the device the register you want to read:  
  SPI.transfer(thisRegister);  
  // send a value of 0 to read the first byte returned:  
  result = SPI.transfer(0xFF);  
  // decrement the number of bytes left to read:  
  bytesToRead--;  
  // if you still have another byte to read:  
  if (bytesToRead > 0) {  
    // shift the first byte left, then get the second byte:  
    result = result << 8;  
    inByte = SPI.transfer(0xFF);  
    // combine the byte you just got with the previous one:  
    result = result | inByte;  
    // decrement the number of bytes left to read:  
    bytesToRead--;  
  }  
  // take the chip select high to de-select:  
  digitalWrite(chipSelectPin_Temp, HIGH);  
  // return the result:  
  return(result);  
}  
  
//Sends a write command to ADT7320  
void writeRegister(byte thisRegister, byte thisValue) {  
  // take the chip select low to select the device:  
  digitalWrite(chipSelectPin_Temp, LOW);  
  SPI.transfer(thisRegister); //Send register location  
  SPI.transfer(thisValue);  //Send value to record into register  

  // take the chip select high to de-select:  
  digitalWrite(chipSelectPin_Temp, HIGH);  
}  







