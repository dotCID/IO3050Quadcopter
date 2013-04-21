/*	Class to read gyroscope/accelerometer combo for the IO3050 Quadcopter project
	Written by Bart van der Werf & Marien ".CID" Wolthuis
	Partially based upon the work of from www.den-uijl.nl/electronics/gyro.html
*/

#include "Arduino.h"
#include "Wire.h"
#include "DeviceReader.h"


int _gyroResult[3],_accelResult[3];
byte* _readout;

DeviceReader::DeviceReader(){
	Wire.begin();
}

void DeviceReader::setup(){
	//Serial.println("DeviceReader is intialising the IMU");
	this->_writeTo(0x53,0x31,0x09); //Set accelerometer to 11bit, +/-4g
	this->_writeTo(0x53,0x2D,0x08); //Set accelerometer to measure mode
	this->_writeTo(0x68,0x16,0x1A); //Set gyro to +/-2000deg/sec and 98Hz low pass filter
	this->_writeTo(0x68,0x15,0x09); //Set gyro to 100Hz sample rate
	
	delay(100);
	//Serial.println("DeviceReader finished initialising");
}

void DeviceReader::_writeTo(byte device, byte toAddress, byte val) {
  Wire.beginTransmission(device);  
  Wire.write(toAddress);        
  Wire.write(val);        
  Wire.endTransmission();
}

void DeviceReader::_readFrom(byte device, byte fromAddress, int num, byte result[]) {
  Wire.beginTransmission(device);
  Wire.write(fromAddress);
  Wire.endTransmission();
  Wire.requestFrom((int)device, num);
  int i = 0;
  while(Wire.available()) {
    result[i] = Wire.read();
    i++;
  }
  
  _readout = result;
}

void DeviceReader::retrGyroReadings() {
  byte buffer[6];
  this->_readFrom(0x68,0x1D,6,buffer);
  _gyroResult[0] = (((int)buffer[0]) << 8 ) | buffer[1];
  _gyroResult[1] = (((int)buffer[2]) << 8 ) | buffer[3];
  _gyroResult[2] = (((int)buffer[4]) << 8 ) | buffer[5];
} 

void DeviceReader::retrAccelReadings() {
  byte buffer[6];
  this->_readFrom(0x53,0x32,6,buffer);
  _accelResult[0] = (((int)buffer[1]) << 8 ) | buffer[0];
  _accelResult[1] = (((int)buffer[3]) << 8 ) | buffer[2];
  _accelResult[2] = (((int)buffer[5]) << 8 ) | buffer[4];
}


int DeviceReader::getGyroReadingX(){
	return _gyroResult[0];
}

int DeviceReader::getGyroReadingY(){
	return _gyroResult[1];
}

int DeviceReader::getGyroReadingZ(){
	return _gyroResult[2];
}

int DeviceReader::getAccelReadingX(){
	return _accelResult[1]*-1;	// Seems to be switched with Y, so 1 = X
}

int DeviceReader::getAccelReadingY(){
	return _accelResult[0];
}

int DeviceReader::getAccelReadingZ(){
	return _accelResult[2];
}