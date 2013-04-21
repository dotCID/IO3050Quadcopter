/* Class to test using interrupts with the reading of the IMU and Kalman filtering.
   Written by Marien ".CID" Wolthuis
*/

#include <Wire.h>
#include <DeviceReader.h>

#define C1 100
#define C2 10000000
#define P2PHI 0.01


float ang_rate_x = 0;
float phi_x = 0;
float p_bias_x = 0;
  
float ang_rate_y = 0;
float phi_y = 0;
float p_bias_y = 0;

float ang_rate_z = 0;
float phi_z = 0;
float p_bias_z = 0;
 
DeviceReader imu;


int counter =0;
unsigned long lastLoop = 0;

unsigned long time = millis();
  
void setup(){
  Serial.begin(115200);
  	
  //.CID: use timer 2 for read interrupts at 100Hz. // prescaler of 1024, count to 155
  noInterrupts();				
  TCCR2A  = 0;					// disable timer
  TCCR2B  = 0;					// disable timer
  TCNT2	  = 0;					// counter value 0
  OCR2A	  = 155;				// count to 155
  TCCR2A |= (1<<WGM21);			        // CTC mode
  TCCR2B |= (1<<CS22)|(1<<CS21)|(1<<CS20);	// 1024 prescaler
  TIMSK2 |= (1<<OCIE2A);			// enable timer compare interrupt
  interrupts();					
  
  imu.setup();
}

void loop(){
  Serial.print("{");
  Serial.print("phi_x");
  Serial.print(",T,");
  Serial.print(phi_x);
  Serial.println("}");

  Serial.print("{");
  Serial.print("phi_y");
  Serial.print(",T,");
  Serial.print(phi_y);
  Serial.println("}");
  
  Serial.print("{");
  Serial.print("phi_z");
  Serial.print(",T,");
  Serial.print(phi_z);
  Serial.println("}");
}

ISR(TIMER2_COMPA_vect){     // TODO: very dirty fix!
  counter++;
  interrupts();
  imu.retrGyroReadings();
  imu.retrAccelReadings(); 
  noInterrupts();
  kalmanX(imu.getGyroReadingX(), imu.getAccelReadingY());
  kalmanY(imu.getGyroReadingY(), imu.getAccelReadingX());
  kalmanZ(imu.getGyroReadingZ(), imu.getAccelReadingZ());
}


void kalmanX(float sens_phi_x,float sens_rate_x){
    ang_rate_x = sens_rate_x - p_bias_x;
    phi_x = phi_x + ang_rate_x * P2PHI;  // integrate ang_rate to obtain phi
    phi_x = phi_x - (phi_x - sens_phi_x) / C1;
    p_bias_x = p_bias_x + (phi_x - sens_phi_x) / C2;
    
}

void kalmanY(float sens_phi_y,float sens_rate_y){
    ang_rate_y = sens_rate_y - p_bias_y;
    phi_y = phi_y + ang_rate_y * P2PHI;  // integrate ang_rate to obtain phi
    phi_y = phi_y - (phi_y - sens_phi_y) / C1;
    p_bias_y = p_bias_y + (phi_y - sens_phi_y) / C2;

}

void kalmanZ(float sens_phi_z,float sens_rate_z){
    ang_rate_z = sens_rate_z - p_bias_z;
    phi_z = phi_z + ang_rate_z * P2PHI;  // integrate ang_rate to obtain phi
    phi_z = phi_z - (phi_z - sens_phi_z) / C1;
    p_bias_z = p_bias_z + (phi_z - sens_phi_z) / C2;
}
