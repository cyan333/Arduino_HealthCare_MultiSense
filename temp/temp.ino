//ADT7320 Interfacing with Arduino LaunchPAD (Rev 0)  
// Released under CC-SA Creative commons Share Alike Licence  

  
#include <SPI.h>  
#include <ADXL362.h>


const int chipSelectPin_Temp = 10;  
const int chipSelectPin_ADXL = 7;

ADXL362 xl;

int16_t temp;
int16_t XValue, YValue, ZValue, Temperature;
  
void setup() {  
  Serial.begin(9600);  
  SPI.setDataMode(SPI_MODE3);// ADT7320 Supports only MODE3 SPI  
  // start the SPI library:  
  SPI.begin();  
  // initalize the  data ready and chip select pins:  
  pinMode(chipSelectPin_Temp, OUTPUT);  

  xl.begin(chipSelectPin_ADXL);   // Setup SPI protocol, issue device soft reset
  xl.beginMeasure();              // Switch ADXL362 to measure mode  
  
  delay(100);  
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
  delay(100);
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







