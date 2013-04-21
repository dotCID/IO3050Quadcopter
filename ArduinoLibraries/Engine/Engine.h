/*
	Engine.h - Library for controlling ESCs for the IO3050 Quadrotor project at the TU Delft.
	Written by Marien ".CID"  Wolthuis - 07/03/2013
 */
#ifndef Engine_h
#define Engine_h

#include "Arduino.h"
#include "Servo.h"

class Engine{
	private:
		void _setPin(int pinIn);			// saves the pin of the motor controller
		int _pin;
		int _speed;						// speed as percentage of max
		int _minSpeed;
		int _maxSpeed;
		int _killSpeed;					// amount of speed lost per second
		int _startSpeed;				// amount of speed gained per second
		Servo _esc;						// ESC connection
	public:
		Engine(int pinIn);
		void arm();						// arms the ESC
		void setSpeedPCT(int pct);		// sets speed to a certain percentage of max
		void setSpeed(int spd);			// sets speed as servo value
		void speedOffset(int offset);	// speeds up the engine by an offset percentage
		void kill();					// kills the engine by gradually decreasing power
		void start(int tgtPct);			// will slowly increase up to target percentage of max power
		int getPin();					// returns pin of the motor controller
		int getSpeed();					// returns current percentage of max power
		void setKillSpeed(int ksIn);
		int getKillSpeed();
		void setStartSpeed(int stIn);
		int getStartSpeed();
		int getPercentage();			// return the percentage of maximum power on which the engine is turning
};

#endif