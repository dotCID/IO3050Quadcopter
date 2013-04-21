/* Test sketch to get ouput from the IMU for the IO3050 Quadcopter Project
   Written by Marien ".CID" Wolthuis
*/

#include <DeviceReader.h>
#include <Wire.h>

String incomingString;

DeviceReader imu;

void setup()
{
  // Required for I/O from Serial monitor
  Serial.begin(9600);
  // Print a startup message
  Serial.println("initializing");
  
  imu.setup();
}

void loop()
{
  imu.calcGyroReadings();
  imu.calcAccelReadings();
  
  int gyro[3];
  gyro[0] = imu.getGyroReadingX(); 
  gyro[1] = imu.getGyroReadingY(); 
  gyro[2] = imu.getGyroReadingZ(); 
        
  Serial.print("[x] =");
  Serial.print(gyro[0]);
  Serial.print("\t[y] =");
  Serial.print(gyro[1]);
  Serial.print("\t[z] =");
  Serial.print(gyro[2]);

        
  int accel[3];
  accel[0] = imu.getAccelReadingX();
  accel[1] = imu.getAccelReadingY(); 
  accel[2] = imu.getAccelReadingZ();  
        
  Serial.print("\t[x] =");
  Serial.print(accel[0]);
  Serial.print("\t[y] =");
  Serial.print(accel[1]);
  Serial.print("\t[z] =");
  Serial.println(accel[2]);
  
  delay(250);
}

