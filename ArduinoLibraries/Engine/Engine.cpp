/*
	Engine.cpp - Library for controlling ESCs for the IO3050 Quadrotor project at the TU Delft.
	Written by Marien ".CID"  Wolthuis - 07/03/2013
*/
#include "Arduino.h"
#include "Engine.h"
#include "Servo.h"

int _minSpeed,_maxSpeed,_armSpeed,_speed,_killSpeed,_startSpeed,_pin;
Servo _esc;

Engine::Engine(int pinIn){
	_minSpeed = 30;
	_maxSpeed = 125;	
	_speed = 0;
	_killSpeed = 10;		// degrees per iteration
	_startSpeed = 10;
	_pin = pinIn;
	
	this->_esc.attach(_pin);
	this->_esc.write(0);
};

void Engine::_setPin(int pinIn){
	_pin = pinIn;
}

void Engine::arm(){
	Serial.print("Starting arm sequence for engine on pin ");
	delay(5000);
	 
	Serial.println(_pin);
	
	this->_esc.write(0);
	delay(1000);
	this->_esc.write(180);
	delay(1000);
	this->_esc.write(20);
	delay(1000);
	this->_esc.write(30); 	// TODO: remove to prevent auto spinning
	
	Serial.println("Finished arming");
}

void Engine::setSpeedPCT(int pct){
	if(pct<=100 && pct>=0){
		_speed = _minSpeed + (int)((_maxSpeed - _minSpeed) * (pct/100));
	}else if(pct>100){
		_speed = _maxSpeed;
	}else if(pct<0){
		_speed = _minSpeed;
	}
	this->_esc.write(_speed);
};

void Engine::setSpeed(int spd){
	/*if(spd <= _maxSpeed && spd >= _minSpeed){
		_speed = spd;
	}else if(spd > _maxSpeed){
		_speed = _maxSpeed;
	}else{
		_speed = _minSpeed;
	}*/
	this->_esc.write(_speed);	
}

void Engine::speedOffset(int offset){
	if(_speed+offset<=_maxSpeed && _speed+offset>=_minSpeed){
		_speed += offset;
	}else if(_speed+offset>_maxSpeed){
		_speed = _maxSpeed;
	}else if(_speed+offset<_minSpeed){
		_speed = _minSpeed;
	}

	this->_esc.write(_speed);
};

// needs testing to find appropriate speed
void Engine::kill(){
	if(_speed>_minSpeed){
		this->speedOffset(-_killSpeed);
	}
};

// TODO: needs testing to find appropriate speed
void Engine::start(int tgtPct){
	if(_speed<(_maxSpeed*tgtPct/100)){
		this->speedOffset(_startSpeed);
	};
}

int Engine::getPin(){
	return _pin;
}

int Engine::getSpeed(){
	return _speed;
}

void Engine::setKillSpeed(int ksIn){
	_killSpeed = ksIn;
}

int Engine::getKillSpeed(){
	return _killSpeed;
}

void Engine::setStartSpeed(int stIn){
	_startSpeed = stIn;
}

int Engine::getStartSpeed(){
	return _startSpeed;
}

int Engine::getPercentage(){
	return _speed/_maxSpeed*100;
}