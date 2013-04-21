// Need the Servo library
#include <Servo.h>
#include <Engine.h>

// This is our motor.
//Engine ENG_1(11);
Servo testServo;
//Engine ENG_2(10);
//Engine ENG_3(11);
//Engine ENG_4(12);

String incomingString;


void setup()
{
  Serial.begin(9600);
  // Print a startup message
  Serial.println("initializing");
  testServo.attach(12);
}


void loop()
{
  // If there is incoming value
  if(Serial.available() > 0)
  {
    // read the value
    char ch = Serial.read();
  
    if (ch != 10){ // ch 10 = \n
      incomingString += ch;
    }else{
      int val = incomingString.toInt();
      Serial.println(val);
    
      if (val > -1 && val < 181)
     {
       Serial.println("Value is between 0 and 180");
  //     ENG_1.setSpeed(val);
       testServo.write(val);
    //   ENG_2.setSpeed(val);
    //   ENG_3.setSpeed(val);
    //   ENG_4.setSpeed(val);
     }
     else if( val == -3){
         Serial.println("Arming engines");
//         ENG_1.arm();

     //    ENG_2.arm();
     //    ENG_3.arm();
     //    ENG_4.arm();
     }else
     {
       Serial.println("Value is NOT between 0 and 180");
     }
      incomingString = "";
    }
  }
}

/*
void Write(int val){
    ENG_1.write(val);
    ENG_2.write(val);
    ENG_3.write(val);
    ENG_4.write(val); 
}*/
