/*  Class to filter with Kalman, easy and with pointers instead of variables
	Written by Marien ".CID" Wolthuis
*/

#include "KalmanSmall.h"
#include "Arduino.h"

#define C1 175
#define C2 10000000
#define P2PHI 0.01

KalmanSmall::KalmanSmall(){
	// nothing to define here, move along
}

void KalmanSmall::kalman(float sens_phi, float sens_rate, double* phi, float* ang_rate, float* p_bias){
    ang_rate[0] = sens_rate - p_bias[0];
    phi[0] = phi[0] + ang_rate[0] * P2PHI;  // integrate ang_rate to obtain phi
    phi[0] = phi[0] - (phi[0] - sens_phi) / C1;
    p_bias[0] = p_bias[0] + (phi[0] - sens_phi) / C2;   
}