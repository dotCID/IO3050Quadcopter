/* Class to test using interrupts with the reading of the IMU and Kalman filtering.
   Written by Marien ".CID" Wolthuis
*/

#include <Wire.h>
#include <DeviceReader.h>
#include <KalmanSmall.h>

float ang_rate_x = 0;
double phi_x = 0;
float p_bias_x = 0;
  
float ang_rate_y = 0;
double phi_y = 0;
float p_bias_y = 0;

float ang_rate_z = 0;
double phi_z = 0;
float p_bias_z = 0;
 
DeviceReader imu;
KalmanSmall filter;

float neutralPoint_x = 0;
float neutralPoint_y = 0;
float neutralPoint_z = 0;

boolean calibrated = false;		// true when finished with calibration

void setup(){
  Serial.begin(115200);
  imu.setup();
  	
  //.CID: use timer 2 for read interrupts at 100Hz. // prescaler of 1024, count to 155
  noInterrupts();				
  TCCR2A  = 0;								// disable timer
  TCCR2B  = 0;								// disable timer
  TCNT2	  = 0;								// counter value 0
  OCR2A	  = 155;							// count to 155
  TCCR2A |= (1<<WGM21);			        	// CTC mode
  TCCR2B |= (1<<CS22)|(1<<CS21)|(1<<CS20);	// 1024 prescaler
  TIMSK2 |= (1<<OCIE2A);					// enable timer compare interrupt  
  interrupts();	
  
  delay(10000);
}

int count = 0;
float phi_y_old =0;
float phi_x_old = 0;

void loop(){
  float tmp = phi_x_old / phi_x;
  if(tmp>1 && count<100){
     neutralPoint_x += phi_x;
     count++;
  }
  
  if(count == 100){
     neutralPoint_x /= count;
     count = 101; 
	 calibrated = true;
  }
  
  Serial.print("{");
  Serial.print("phi_y");
  Serial.print(",T,");
  Serial.print(phi_y);
  Serial.println("}");
  
  Serial.print("{");
  Serial.print("phi_x");
  Serial.print(",T,");
  Serial.print(phi_x);
  Serial.println("}");
  
  delay(100);
}

ISR(TIMER2_COMPA_vect){     // TODO: very dirty fix!
  interrupts();
  imu.retrGyroReadings();
  imu.retrAccelReadings(); 
  noInterrupts();
    
  phi_y_old = phi_y;
  phi_x_old = phi_x;
  filter.kalman(imu.getGyroReadingX(), imu.getAccelReadingX(), &phi_x, &ang_rate_x, &p_bias_x);
  filter.kalman(imu.getGyroReadingY(), imu.getAccelReadingY(), &phi_y, &ang_rate_y, &p_bias_y);
  filter.kalman(imu.getGyroReadingX(), imu.getAccelReadingZ(), &phi_z, &ang_rate_z, &p_bias_z);
}
