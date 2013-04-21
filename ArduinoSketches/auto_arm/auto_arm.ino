/* sketch to test automatic arming of the IO3050 Quadcopter project
   Written by Marien ".CID"  Wolthuis
*/

#include <Wire.h>
#include <Ardruino.h>
#include <Servo.h>

Servo ENG_1;
Servo ENG_2;
Servo ENG_3;
Servo ENG_4;

void setup(){
  ENG_1.attach(9);
  ENG_2.attach(10);
  ENG_3.attach(11);
  ENG_4.attach(12);
  
  delay(1000);
  Write(0);
  delay(1000);
  Write(180);
  delay(1000);
  Write(20);
  delay(1000);
  Write(30);
}

void loop(){
  Write(40);
  delay(1000);
  Write(30);
  delay(1000);
}

void Write(int val){
    ENG_1.write(val);
    ENG_2.write(val);
    ENG_3.write(val);
    ENG_4.write(val); 
}
