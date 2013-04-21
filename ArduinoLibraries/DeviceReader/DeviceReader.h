/*	
	Class to read gyroscope/accelerometer combo for the IO3050 Quadcopter project
	Written by Bart van der Werf & Marien ".CID" Wolthuis
*/

#ifndef DeviceReader_h
#define DeviceReader_h

#include "Arduino.h"
#include "Wire.h"

class DeviceReader{
	private:
		void _writeTo(byte device, byte toAddress, byte val);
		void _readFrom(byte device, byte fromAddress, int num, byte result[]);
		int _gyroResult[3],_accelResult[3];
		int _summedGyroXVal, _summedGyroYVal, _summedGyroZVal;
		int _summedAccelXVal, _summedAccelYVal, _summedAccelZVal;
	public:
		DeviceReader();
		void setup();
		void retrGyroReadings();
		void retrAccelReadings();
		
		int getGyroReadingX();
		int getGyroReadingY();
		int getGyroReadingZ();
		
		int getAccelReadingX();
		int getAccelReadingY();
		int getAccelReadingZ();
};
#endif;