/* 	Class to perform Kalman filtering on the readouts from the gyroscope/acceletrometer combo for the IO3050 Quadcopter project
	Written by Bart van der Werf & Marien ".CID" Wolthuis
*/

//code adapted from www.den-uijl.nl/electronics/gyro.html

#include "Arduino.h"
#include "Math.h"
#include "Kalman.h"

float _timeStep,_pitchGyro,_pitchAccel,_pitchPrediction,_rollGyro,_rollAccel,_rollPrediction,_giroVar,_deltaGiroVar,_accelVar,_Pxx,_Pvv,_Pxv,_kx,_kv;
float _gyroBias[3];
float _accelBias[3];

float _gyroSum[3];
float _accelSum[3];
int _sumCounter;
int _counterLimit;
unsigned long _timer;

Kalman::Kalman() {
	_timeStep = 0.02;
	_pitchGyro = 0;
	_pitchAccel = 0;
	_pitchPrediction = 0; //Output of Kalman filter
	_rollGyro = 0;
	_rollAccel = 0;
	_rollPrediction = 0;  //Output of Kalman filter
	_giroVar = 0.1;
	_deltaGiroVar = 0.1;
	_accelVar = 5;
	_Pxx = 0.1; // angle variance
	_Pvv = 0.1; // angle change rate variance
	_Pxv = 0.1; // angle and angle change rate covariance
	
	_sumCounter = 0;
	_counterLimit = 50;
}

void Kalman::predict(int* gyroReading, int* accelReading) {
  if(_sumCounter>_counterLimit){ this->_calcBias(); };
  _timer = millis();
  
  int* gyroResult = gyroReading;
  int* accelResult = accelReading;
  
   _gyroSum[0] += gyroResult[0];
   _gyroSum[1] += gyroResult[1];
   _gyroSum[2] += gyroResult[2];
   
   _accelSum[0] += accelResult[0];
   _accelSum[1] += accelResult[1];
   _accelSum[2] += accelResult[2];
   
   _sumCounter++;
  
  float biasGyroX = _gyroBias[0];
  float biasGyroY = _gyroBias[1];
  float biasGyroZ = _gyroBias[2];
  
  float biasAccelX = _accelBias[0];
  float biasAccelY = _accelBias[1];
  float biasAccelZ = _accelBias[2];
  
  _pitchAccel = atan2((accelResult[1] - biasAccelY) / 256, (accelResult[2] - biasAccelZ) / 256) * 360.0 / (2*PI);
  _pitchGyro = _pitchGyro + ((gyroResult[0] - biasGyroX) / 14.375) * _timeStep;
  _pitchPrediction = _pitchPrediction + ((gyroResult[0] - biasGyroX) / 14.375) * _timeStep;
  
  _rollAccel = atan2((accelResult[0] - biasAccelX) / 256, (accelResult[2] - biasAccelZ) / 256) * 360.0 / (2*PI);
  _rollGyro = _rollGyro - ((gyroResult[1] - biasGyroY) / 14.375) * _timeStep; 
  _rollPrediction = _rollPrediction - ((gyroResult[1] - biasGyroY) / 14.375) * _timeStep;
  
  _Pxx += _timeStep * (2 * _Pxv + _timeStep * _Pvv);
  _Pxv += _timeStep * _Pvv;
  _Pxx += _timeStep * _giroVar;
  _Pvv += _timeStep * _deltaGiroVar;
  _kx = _Pxx * (1 / (_Pxx + _accelVar));
  _kv = _Pxv * (1 / (_Pxx + _accelVar));
  
  _pitchPrediction += (_pitchAccel - _pitchPrediction) * _kx;
  _rollPrediction += (_rollAccel - _rollPrediction) * _kx;
  
  _Pxx *= (1 - _kx);
  _Pxv *= (1 - _kx);
  _Pvv -= _kv * _Pxv;
  
  _timer = millis() - _timer;
  _timer = (_timeStep * 1000) - _timer;
}


/*	Function to calculate bias based on counterLimit
	Pre: _GyroSum and _AccelSum contain data sums over  reading iterations
	Post: _biasGyro and _biasAccel are averaged over these
*/
void Kalman::_calcBias(){
	_gyroBias[0]  = _gyroSum[0] / (float)_counterLimit;
	_gyroBias[1]  = _gyroSum[1] / (float)_counterLimit;
	_gyroBias[2]  = _gyroSum[2] / (float)_counterLimit;
	_accelBias[0] = _accelSum[0] / (float)_counterLimit;
	_accelBias[1] = _accelSum[1] / (float)_counterLimit;
	_accelBias[2] = (_accelSum[2] / (float)_counterLimit) - 256;		// TODO: figure out why 256
	
	_gyroSum[0]  = 0;
	_gyroSum[1]  = 0;
	_gyroSum[2]  = 0;
	_accelSum[1] = 0;
	_accelSum[2] = 0;
	_accelSum[3] = 0;
	_sumCounter  = 0;
	Serial.println("Bias calculated (private)");
}

/*	Function to calculate bias based on input value
	Pre: _GyroSum and _AccelSum contain data sums over  reading iterations
	Post: _biasGyro and _biasAccel are averaged over these
*/
void Kalman::calcBias(int counterLimit){	
	_gyroBias[0]  = _gyroSum[0] / (float)counterLimit;
	_gyroBias[1]  = _gyroSum[1] / (float)counterLimit;
	_gyroBias[2]  = _gyroSum[2] / (float)counterLimit;
	_accelBias[0] = _accelSum[0] / (float)counterLimit;
	_accelBias[1] = _accelSum[1] / (float)counterLimit;
	_accelBias[2] = (_accelSum[2] / (float)counterLimit) - 256;		// TODO: figure out why 256
	
	_gyroSum[0]  = 0;
	_gyroSum[1]  = 0;
	_gyroSum[2]  = 0;
	_accelSum[1] = 0;
	_accelSum[2] = 0;
	_accelSum[3] = 0;
	_sumCounter  = 0;
	Serial.println("Bias calculated (public)");
}

float Kalman::getPitchPred(){
	return _pitchPrediction;
}

float Kalman::getRollPred(){
	return _rollPrediction;
}

void Kalman::setGyroSum(int* gyroSumIn){
	_gyroSum[0] = gyroSumIn[0];
	_gyroSum[1] = gyroSumIn[1];
	_gyroSum[2] = gyroSumIn[2];
}

void Kalman::setAccelSum(int* accelSumIn){
	_accelSum[0] = accelSumIn[0];
	_accelSum[1] = accelSumIn[1];
	_accelSum[2] = accelSumIn[2];
}