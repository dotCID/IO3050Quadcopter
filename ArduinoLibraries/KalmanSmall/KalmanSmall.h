/*  Class to filter with Kalman, easy and with pointers instead of variables
	Written by Marien ".CID" Wolthuis
*/

#ifndef KalmanSmall_h
#define KalmanSmall_h

#include "Arduino.h"

class KalmanSmall{
	private:
	public:
		KalmanSmall();
		void kalman(float sens_phi, float sens_rate, double* phi, float* ang_rate, float* p_bias);
};
#endif;