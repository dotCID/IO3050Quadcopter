/*
	Main class for the IO3050 Quadcopter project
	Written by Marien ".CID" Wolthuis
*/

#include <Wire.h>
#include <Math.h>
#include <Servo.h>
#include <DeviceReader.h>
#include <PID_v1.h>

#define ENG_1_PIN 9
#define ENG_2_PIN 10
#define ENG_3_PIN 11
#define ENG_4_PIN 12

#define C1 175
#define C2 10000000
#define P2PHI 0.01

// combined speeds per two engines
#define LIFTOFF_SPEED 176
#define FLY_SPEED 180

// values for PIDs
#define MIN_CORRECTED_SPEED 38
#define MAX_CORRECTED_SPEED 62
#define PID_SAMPLE_TIME 10

float  ang_rate_x = 0;
double phi_x      = 0;
double phi_x_abs  = 0;
float  p_bias_x   = 0;

float  ang_rate_y = 0;
double phi_y      = 0;
double phi_y_abs  = 0;
float  p_bias_y   = 0;
 
DeviceReader imu;

Servo ENG_1;
Servo ENG_2;
Servo ENG_3;
Servo ENG_4;

double engineRatio_13		= 50;	// ratio of 1 vs 3, 30 = {30% to 1, 70% to 3}
double engineSpeedTotal_13	= 60;	
double engineSpeed_1;	
double engineSpeed_3;

double engineRatio_24		= 50;
double engineSpeedTotal_24	= 60;
double engineSpeed_2;
double engineSpeed_4;


double neutralPoint_x = 0;
double neutralPoint_x_corr = 0;

double neutralPoint_y = 0;
double neutralPoint_y_corr = 0;


bool calibrated_x = false;		// true when finished with calibration
bool calibrated_y = false;
bool started	  = false;		// true when everything is initiated and liftoff can be commenced
bool liftedOff	  = false;		// true when predetermined minimum hover speed was achieved

bool killed     = false;        // true when engines are killed
bool armed      = false;        // true when finished arming
bool compute    = false;        // safety/debugging measure, when false PIDs will not be active

bool debugMode	= false;		// controls serial output of variables, graphing data, etc.

double Kp_x	= 0.021;			// stable single axis: 0.025
double Ki_x	= 0.001;			// ssa: 0.001
double Kd_x	= 0.00015;			// ssa:0.0001
double pid_x_neg_target = 0.0;	// 0 because neutralPoint should compensate

double Kp_y	= 0.011;			// ssa: 0.014
double Ki_y	= 0.001;			// ssa: 0.001
double Kd_y	= 0.0015;			// ssa: 0.001
double pid_y_neg_target = 0.0;	

PID pid_x_pos(&phi_x, &engineRatio_13, &neutralPoint_x, Kp_x, Ki_x, Kd_x, REVERSE);
PID pid_x_neg(&phi_x_abs, &engineRatio_13, &pid_x_neg_target, Kp_x, Ki_x, Kd_x, DIRECT);

PID pid_y_pos(&phi_y, &engineRatio_24, &neutralPoint_y, Kp_y, Ki_y, Kd_y, REVERSE);
PID pid_y_neg(&phi_y_abs, &engineRatio_24, &pid_y_neg_target, Kp_y, Ki_y, Kd_y, DIRECT);

int count_x = 0;      			// counts number of completed calibration loops
int count_y = 0;
int countFlightTime = 0;

float phi_x_old = 0;
float phi_y_old = 0;
String incomingString;

/*
	The setup function sets up the following:
		connects to the serial if debugMode is enabled
		sets up the IMU
		attaches the engines and makes sure they receive signal 0
		sets the output limits and sample time of the PIDs
		configures timer2 for 100Hz interrupts
		arms the ESCs
*/
void setup(){
  delay(5000);
  
  if(debugMode){
	Serial.begin(115200);
  
	Serial.println("Setting up the IMU and attaching engines");
  };
  
  imu.setup();
  
  ENG_1.attach(ENG_1_PIN);
  ENG_1.write(0);
  
  ENG_2.attach(ENG_2_PIN);
  ENG_2.write(0);
  
  ENG_3.attach(ENG_3_PIN);
  ENG_3.write(0);
  
  ENG_4.attach(ENG_4_PIN);
  ENG_4.write(0);
  
  pid_x_pos.SetOutputLimits(MIN_CORRECTED_SPEED,MAX_CORRECTED_SPEED);
  pid_x_neg.SetOutputLimits(MIN_CORRECTED_SPEED,MAX_CORRECTED_SPEED);
  pid_x_pos.SetSampleTime(PID_SAMPLE_TIME);
  pid_x_neg.SetSampleTime(PID_SAMPLE_TIME);
  
  pid_y_pos.SetOutputLimits(MIN_CORRECTED_SPEED,MAX_CORRECTED_SPEED);
  pid_y_neg.SetOutputLimits(MIN_CORRECTED_SPEED,MAX_CORRECTED_SPEED);
  pid_y_pos.SetSampleTime(PID_SAMPLE_TIME);
  pid_y_neg.SetSampleTime(PID_SAMPLE_TIME);
  
  
  // prescaler of 1024, count to 155 = 100Hz
  noInterrupts();				
  TCCR2A  = 0;								// disable timer
  TCCR2B  = 0;								// disable timer
  TCNT2	  = 0;								// counter value 0
  OCR2A	  = 155;							// count to 155
  TCCR2A |= (1<<WGM21);			        	// CTC mode
  TCCR2B |= (1<<CS22)|(1<<CS21)|(1<<CS20);	// 1024 prescaler
  TIMSK2 |= (1<<OCIE2A);					// enable timer compare interrupt  
  interrupts();	
    
  armESCs();
  
  delay(10000);								// to give the gyro a chance to stabilise
}

/*
	The main loop of the Arduino program. It calls all functions needed for manual control, calibration, liftoff and flight as well as the safety feature
*/
void loop(){
  serialControl();

  calibrateIMU_x();
  calibrateIMU_y();
  
  liftOff();

  activatePIDs();
  
  fly();
  
  safetyChecks();
}

/*
	Interrupt Service Routine to read the IMU and calculate Kalman and PID values
	Pre: phi is old, system works with previous values
	Post: old phi is saved, phi is updated, PIDs computed current state and corrected for that
*/
ISR(TIMER2_COMPA_vect){     
  if(armed){
    interrupts();				// TODO: dirty fix?
    imu.retrGyroReadings();
    imu.retrAccelReadings(); 
       
    phi_x_old = phi_x;
    phi_y_old = phi_y;
    
    kalmanSmallX(); 
    kalmanSmallY();
    
    if(compute){
      if(phi_x >= neutralPoint_x){
		pid_x_pos.Compute();
      }else{
        phi_x_abs = phi_x;
		phi_x_abs -= neutralPoint_x;
        if(phi_x_abs < 0){
            phi_x_abs *= -1;
        }
        pid_x_neg.Compute();
      }
      
	  if(phi_y >= neutralPoint_y){
		pid_y_pos.Compute();
      }else{
        phi_y_abs = phi_y;
		phi_y_abs -= neutralPoint_y;
        if(phi_y_abs < 0){
            phi_y_abs *= -1;
        }
        pid_y_neg.Compute();
      }
    };
	
	if(liftedOff && !killed){
		countFlightTime++;
	}
  }
}

/*
	Wrapper function to siplify syntax of motor control
	old syntax: ENG_x.write(speed)
	new syntax: write(engNo, speed)
*/
void write(int ENG,int val){
	if(val >= 20 && val <=125 || !armed){
		if(ENG == 1){
			ENG_1.write(val);
            if(!armed && debugMode){ 
				Serial.print("ENG_1 wrote ");Serial.println(val);
			};
		}else if(ENG == 2){
			ENG_2.write(val);
            if(!armed && debugMode){ 
				Serial.print("ENG_2 wrote ");Serial.println(val); 
			};
		}else if(ENG == 3){
			ENG_3.write(val);
            if(!armed && debugMode){ 
				Serial.print("ENG_3 wrote ");Serial.println(val); 
			};
		}else if(ENG == 4){
			ENG_4.write(val);
            if(!armed && debugMode){ 
				Serial.print("ENG_4 wrote ");Serial.println(val); Serial.println("");
			};
		}		
	}
}

/*
	function to automatically arm the ESCs and set engines to lowest speed
	pre: ESCs are ready for commands, waiting for arming
	post: ESCs are armed, engines spinning at signal 30
*/
void armESCs(){
	if(!armed){        
		if(debugMode){
			Serial.println("Arming engines");
		};
		
		delay(3000);
		write(1,180);
		write(2,180);
		write(3,180);
		write(4,180);
			
		delay(3000);
		write(1,20);
		write(2,20);
		write(3,20);
		write(4,20);
			
		delay(4000);
		write(1,engineSpeedTotal_13/2);
		write(2,engineSpeedTotal_13/2);
		write(3,engineSpeedTotal_24/2);
		write(4,engineSpeedTotal_24/2);
			
		delay(2000);
		
		armed = true;
		started = true;
	}
}

/*
	Function to read and process input received through the serial connection
	Reads characters, if they're not a newline it will save the character in a string
	If the newline is read, check if the string contains K values and if so, use that input to configure the PIDs
		Possible commands: 	Kp_x = [double]
							Ki_x = [double]
							Kd_x = [double]
							Kp_y = [double]
							Ki_y = [double]
							Kd_y = [double]
	If not a K value, process the string as an integer and check if it is a control signal
		Possible controls:	-1	:	stop engines
							-2  :   restart engines
	
	If the input is none of the above, print a message stating input could not be parsed
*/
void serialControl(){
	if(debugMode && Serial.available() > 0){
		char ch = Serial.read();
		if (ch != 10){
                  incomingString += ch;
		} else {
			if(incomingString.startsWith("Kp_x = ")){
				incomingString = incomingString.substring(7);
				Kp_x = stringToDouble(incomingString);
				
				pid_x_pos.SetTunings(Kp_x, Ki_x, Kd_x);
				pid_x_neg.SetTunings(Kp_x, Ki_x, Kd_x);
				Serial.print("New Kp_x: "); Serial.println(incomingString);
			}else if(incomingString.startsWith("Ki_x = ")){
				incomingString = incomingString.substring(7);
				Ki_x = stringToDouble(incomingString);
				
				pid_x_pos.SetTunings(Kp_x, Ki_x, Kd_x);
				pid_x_neg.SetTunings(Kp_x, Ki_x, Kd_x);
				Serial.print("New Ki_x: "); Serial.println(incomingString);
			}else if(incomingString.startsWith("Kd_x = ")){
				incomingString = incomingString.substring(7);
				Kd_x = stringToDouble(incomingString);
				
				pid_x_pos.SetTunings(Kp_x, Ki_x, Kd_x);
				pid_x_neg.SetTunings(Kp_x, Ki_x, Kd_x);
				Serial.print("New Kd_x: "); Serial.println(incomingString);
			}else if(incomingString.startsWith("Kp_y = ")){
				incomingString = incomingString.substring(7);
				Kp_y = stringToDouble(incomingString);
				
				pid_y_pos.SetTunings(Kp_y, Ki_y, Kd_y);
				pid_y_neg.SetTunings(Kp_y, Ki_y, Kd_y);
				Serial.print("New Kp_y: "); Serial.println(incomingString);
			}else if(incomingString.startsWith("Ki_y = ")){
				incomingString = incomingString.substring(7);
				Ki_y = stringToDouble(incomingString);
				
				pid_y_pos.SetTunings(Kp_y, Ki_y, Kd_y);
				pid_y_neg.SetTunings(Kp_y, Ki_y, Kd_y);
				Serial.print("New Ki_y: "); Serial.println(incomingString);
			}else if(incomingString.startsWith("Kd_y = ")){
				incomingString = incomingString.substring(7);
				Kd_y = stringToDouble(incomingString);
				
				pid_y_pos.SetTunings(Kp_y, Ki_y, Kd_y);
				pid_y_neg.SetTunings(Kp_y, Ki_y, Kd_y);
				Serial.print("New Kd_y: "); Serial.println(incomingString);
			
			}else{
				int val = incomingString.toInt();
				if(val==-1){
				   emergencyStop("Received kill request from user");
				 }else if(val == -2){
				   Serial.println("Reviving..");
				   engineSpeedTotal_13 = 50;
				   engineRatio_13 = 50;
				   engineSpeedTotal_24 = 50;
				   engineRatio_24 = 50;
				   
				   write(1,30);
				   write(2,30);
				   write(3,30);
				   write(4,30);
				   
				   phi_x = neutralPoint_x;
				   phi_y = neutralPoint_y;
				   
				   armed       = true;
				   calibrated_x  = true;
                   calibrated_y = true;
				   started     = true;
				   liftedOff   = false;
				   
				   killed      = false;
				}else{
					Serial.print("Could not parse the input: ");Serial.println(incomingString);
				}
			}
		    incomingString = "";
		}
	}
}

/*
	Function to calibrate the X-axis IMU
	Pre: no neutralPoint_x, count_x = 0
	Post: neutralPoint_x has been computed, count_x = 110
*/
void calibrateIMU_x(){
	if(armed && !calibrated_x){
		float tmp_x = phi_x_old / phi_x;
		if(tmp_x>1 && count_x<100){
			neutralPoint_x += phi_x;
			count_x++;
			
            if(debugMode){
				Serial.print("Calibration status X: ");Serial.print(count_x);Serial.println("/100");
			};
		}
  
		if(count_x == 100){
			neutralPoint_x /= count_x;
			neutralPoint_x_corr = neutralPoint_x + 400;
			count_x = 110; 
			calibrated_x = true;
		}
		
		if(debugMode && calibrated_x){
			Serial.println("Calibration phase concluded for X axis");
		};
        delay(100);
	}
}

/*
	Function to calibrate the Y-axis IMU
	Pre: no neutralPoint_y, count_y = 0
	Post: neutralPoint_y has been computed, count_y = 120
*/
void calibrateIMU_y(){
	if(armed && !calibrated_y){
		float tmp_y = phi_y_old / phi_y;
		if(tmp_y>1 && count_y<100){
			neutralPoint_y += phi_y;
			count_y++;
            
			if(debugMode){
				Serial.print("Calibration status Y: ");Serial.print(count_y);Serial.println("/100");
			};
		}
  
		if(count_y == 100){
			neutralPoint_y /= count_y;
			neutralPoint_y_corr = neutralPoint_y + 400;
			count_y = 120; 
			calibrated_y = true;
		}
		
		if(debugMode && calibrated_y){
			Serial.println("Calibration phase concluded for Y axis");
		};
        
		delay(100);
	}
}

/*
	Function to speed the engines (slowly) to predefined LIFTOFF_SPEED
	Pre: engines spin at minimum speed
	Post: engines spin at LIFTOFF_SPEED/2
*/
void liftOff(){
	if(calibrated_x && calibrated_y && !liftedOff){
		engineSpeed_1 = engineSpeedTotal_13 * (engineRatio_13/100);
		engineSpeed_3 = engineSpeedTotal_13 - engineSpeed_1;
		
		engineSpeed_2 = engineSpeedTotal_24 * (engineRatio_24/100.0);
		engineSpeed_4 = engineSpeedTotal_24 - engineSpeed_2;
		
		write(1,engineSpeed_1);
		write(2,engineSpeed_2);
		write(3,engineSpeed_3);
		write(4,engineSpeed_4);
		
		
		if(engineSpeedTotal_13 < LIFTOFF_SPEED){
			engineSpeedTotal_13 += 1;	// increments of 0.5%
			delay(40);
		}else if(engineSpeedTotal_13 >= LIFTOFF_SPEED){
			engineSpeedTotal_13 = LIFTOFF_SPEED;
		}
		
		if(engineSpeedTotal_24 < LIFTOFF_SPEED){
			engineSpeedTotal_24 += 1;	// increments of 0.5%
			delay(40);
		}else if(engineSpeedTotal_24 >= LIFTOFF_SPEED){
			engineSpeedTotal_24 = LIFTOFF_SPEED;
		}
		
		if(engineSpeedTotal_24 == LIFTOFF_SPEED && engineSpeedTotal_13 == LIFTOFF_SPEED){
			if(debugMode){
				Serial.print("Liftoff achieved. ("); Serial.print(LIFTOFF_SPEED); Serial.println(")");
			};
			
			liftedOff = true;
		}
	}
}

/*
	Function to activate the PIDs
	Pre: engines turn at the same speed, no angle adjustment
	Post: engines can turn at variable speed, angle adjustment through PIDs
*/
void activatePIDs(){
	if(calibrated_x && calibrated_y && started && liftedOff && !compute){
		pid_x_pos.SetMode(AUTOMATIC);
		pid_x_neg.SetMode(AUTOMATIC);
		pid_y_pos.SetMode(AUTOMATIC);
		pid_y_neg.SetMode(AUTOMATIC);
		
		compute = true;
        
		if(debugMode){
			Serial.println("Finished setup, flying..");
		};
	}
}

/*
	Function to speed up the engines to predetermined FLY_SPEED and adjust speed based upon the PID's speed adjustments
	Pre: engines either spin below FLY_SPEED (t < 10s) or are not updated to suit PID calculated speeds
	Post: engines spin with average speed equal to FLY_SPEED (t>=10s) and are updated to stabilise
*/
void fly(){
	if(liftedOff && !killed){
		if(engineSpeedTotal_13 < FLY_SPEED && countFlightTime > 1000){
			engineSpeedTotal_13 += 1;	// increments of 0.5%
			delay(70);
		}else if(engineSpeedTotal_13 >= FLY_SPEED){
			engineSpeedTotal_13 = FLY_SPEED;
		}
		
		if(engineSpeedTotal_24 < FLY_SPEED && countFlightTime > 1000){
			engineSpeedTotal_24 += 1;	// increments of 0.5%
			delay(70);
		}else if(engineSpeedTotal_24 >= FLY_SPEED){
			engineSpeedTotal_24 = FLY_SPEED;
		}
		
		engineSpeed_1 = engineSpeedTotal_13 * (engineRatio_13/100.0);
		engineSpeed_3 = engineSpeedTotal_13 - engineSpeed_1;
		
		engineSpeed_2 = engineSpeedTotal_24 * (engineRatio_24/100.0);
		engineSpeed_4 = engineSpeedTotal_24 - engineSpeed_2;
		
		write(1,engineSpeed_1);
		write(2,engineSpeed_2);
		write(3,engineSpeed_3);
		write(4,engineSpeed_4);
			
        if(debugMode){
			graphRoutine();
		};
		
		if(countFlightTime > 4000){	// should stop flying after set time
			killed = true;
			write(1,20);
			write(2,20);
			write(3,20);
			write(4,20);
		}
	}
}

/*
	Function to check the current engine speeds and angle of the craft for possible malfunctions
	Within bounds: craft stays flying
	Outside: calls emergencyStop() with a message to user
*/
void safetyChecks(){
	if(liftedOff && !killed){
		if((engineSpeed_1 > LIFTOFF_SPEED * 0.7) || (engineSpeed_2 > LIFTOFF_SPEED * 0.7) || (engineSpeed_3 > LIFTOFF_SPEED * 0.7) || (engineSpeed_4 > LIFTOFF_SPEED * 0.7)){
			emergencyStop("SPEED OVER LIMIT");
		}
		
		if((phi_x - neutralPoint_x >170) ||(phi_x - neutralPoint_x <-170)){
			emergencyStop("ANGLE OUT OF BOUNDS");
		}
		
		if((phi_y - neutralPoint_y >170) ||(phi_y - neutralPoint_y <-170)){
			emergencyStop("ANGLE OUT OF BOUNDS");
		}
	}
}

/*
	Function to stop the engines and print a message over serial if debugMode is enabled
	Pre: craft is malfunctioning
	Post: engines are killed, Arduino is waiting for input over serial or restart
*/
void emergencyStop(String message){
	if(debugMode){
		Serial.print("Possible malfunction. Stopping engines. Message: ");
		Serial.println(message);
	};
	
	write(1, 20);
	write(2, 20);
	write(3, 20);
	write(4, 20);
	killed = true;
}

/*
	Function to read the X-axis data from the IMU and process the data through a Kalman filter
	Pre: phi_x is old
	Post: phi_x is updated with fresh data from the IMU
*/
void kalmanSmallX(){
    double sens_rate = imu.getAccelReadingX();
    double sens_phi = imu.getGyroReadingX();
    
    ang_rate_x = sens_rate - p_bias_x;
    phi_x = phi_x + ang_rate_x * P2PHI;  // integrate ang_rate to obtain phi
    phi_x = phi_x - (phi_x - sens_phi) / C1;
    p_bias_x = p_bias_x + (phi_x - sens_phi) / C2; 
}

/*
	Function to read the Y-axis data from the IMU and process the data through a Kalman filter
	Pre: phi_y is old
	Post: phi_y is updated with fresh data from the IMU
*/
void kalmanSmallY(){
    double sens_rate = imu.getAccelReadingY();
    double sens_phi = imu.getGyroReadingY();
    
    ang_rate_y = sens_rate - p_bias_y;
    phi_y = phi_y + ang_rate_y * P2PHI;  // integrate ang_rate to obtain phi
    phi_y = phi_y - (phi_y - sens_phi) / C1;
    p_bias_y = p_bias_y + (phi_y - sens_phi) / C2; 
}

/*
	Function to parse a double from an input string
	Pre: input is a String
	Post: input is a double
*/
double stringToDouble(String input){
    boolean underOne = false;
    if(input.startsWith("0")) underOne = true;
	int dot = input.indexOf(".");
	String before = input.substring(0,dot);
	String after = input.substring(dot+1);
    int decimals = after.length();
        
    if(!underOne){
		long bigNumber = before.toInt() * pow(10,dot-1);
		for(int i=0;i<decimals;i++){
			bigNumber *= 10; 
		}
		bigNumber += after.toInt();
	  
		double result = (double)bigNumber / (double)pow(10,dot+decimals-1);

		return result;
	}else{
		long bigNumber = 1;
		for(int i=0;i<decimals;i++){
			bigNumber *= 10; 
		}
		bigNumber += after.toInt();
	  
		double result = (double)bigNumber / (double)pow(10,dot+decimals-1);
	  
		result -= 1;
  
		return result;
	}
}

/*
	Function to control what graphing data is returned over the serial connection
*/
void graphRoutine(){
  if(debugMode){
	Serial.println(dataGraph(phi_x-neutralPoint_x,"phi_x"));
	Serial.println(dataGraph(phi_y-neutralPoint_y,"phi_y"));
  
  //Serial.println(dataGraph(ENG_1.read(),"ENG_1"));
  //Serial.println(dataGraph(ENG_2.read(),"ENG_2"));
  //Serial.println(dataGraph(ENG_3.read(),"ENG_3"));
  //Serial.println(dataGraph(ENG_4.read(),"ENG_4"));
  };
}

/*
	Function to construct graphing syntax from a data point and the name if the data sequence
*/
String dataGraph(int data, String name){
       String result ="{"+name+",T,"+data+"}";
       
       return result;
}