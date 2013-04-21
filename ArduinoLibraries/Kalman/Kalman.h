/* 	Class to perform Kalman filtering on the readouts from the gyroscope/acceletrometer combo for the IO3050 Quadcopter project
	Written by Bart van der Werf & Marien ".CID" Wolthuis
*/

#ifndef Kalman_h
#define Kalman_h

#include "Arduino.h"
#include "Math.h"


class Kalman{
	private:
		float _timeStep,_pitchGyro,_pitchAccel,_pitchPrediction,_rollGyro,_rollAccel,_rollPrediction,_giroVar,_deltaGiroVar,_accelVar,_Pxx,_Pvv,_Pxv,_kx,_kv;
		unsigned long _timer;
		void _calcBias();
		float _gyroBias[3];
		float _accelBias[3];
		float _gyroSum[3];
		float _accelSum[3];
		int _sumCounter;
	public:
		Kalman();
		void predict(int* gyroReading, int* accelReading);
		float getPitchPred();
		float getRollPred();
		void setGyroSum(int* gyroSumIn);
		void setAccelSum(int* accelSumIn);
		void calcBias(int counterLimit);
};

#endif