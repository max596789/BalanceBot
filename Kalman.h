/*
 * Kalman.h
 *
 * Created: 1/12/2015 10:41:03 PM
 *  Author: Jacob
 */ 


#ifndef KALMAN_H_
#define KALMAN_H_

float Q_angle; // Process noise variance for the accelerometer
float Q_bias; // Process noise variance for the gyro bias
float R_measure; // Measurement noise variance - this is actually the variance of the measurement noise

float angle_x; // The angle calculated by the Kalman filter - part of the 2x1 state vector
float bias_x; // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
float rate_x; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate

float P_x[2][2]; // Error covariance matrix - This is a 2x2 matrix

float angle_y; // The angle calculated by the Kalman filter - part of the 2x1 state vector
float bias_y; // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
float rate_y; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate

float P_y[2][2]; // Error covariance matrix - This is a 2x2 matrix
 
void initKalmanFilter();
void setAngle_X(float newAngle);
void setAngle_Y(float newAngle);
float getKalmanAngle_X(float newAngle, float newRate, float dt);
float getKalmanAngle_Y(float newAngle, float newRate, float dt);

#endif /* KALMAN_H_ */