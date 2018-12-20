#ifndef _KALMAN_H_
#define _KALMAN_H_

typedef struct{
    /* Kalman filter variables */
    double Q_angle; //Process noise variance for the accelerometer
    double Q_bias; //Process noise variance for the gyro bias
    double R_measure; // Measurement noise variance - this is actually the variance of the measurement noise

    double angle; // The angle calculated by the Kalman filter - part of the 2x1 state vector
    double bias; // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
    double rate; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate

    double P[2][2]; //P���� Error covariance matrix - This is a 2x2 matrix
    double K[2]; // Kalman gain - This is a 2x1 vector
    double y; // Angle difference
    double S; // Estimate error
} KALMAN;

void KalmanInit(KALMAN* klm);
void KalmanCalc(KALMAN* klm, double NewAngle, double NewRate, double dt);
void SetAngle(KALMAN* klm,double newAngle);
#endif

