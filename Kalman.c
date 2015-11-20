/*
 * Kalman.c
 *
 * Created: 1/12/2015 10:40:52 PM
 *  Author: Jacob
 */ 

#include "Kalman.h"

void initKalmanFilter()
{
	Q_angle = 0.001f;
	Q_bias = 0.003f;
	R_measure = 0.03f;
	angle_x = 0.0f;
	bias_x = 0.0f;
	P_x[0][0] = 0.0f; // Since we assume that the bias is 0 
	P_x[0][1] = 0.0f; 
	P_x[1][0] = 0.0f;
	P_x[1][1] = 0.0f;
	
	angle_y = 0.0f;
	bias_y = 0.0f;
	P_y[0][0] = 0.0f; // Since we assume that the bias is 0
	P_y[0][1] = 0.0f;
	P_y[1][0] = 0.0f;
	P_y[1][1] = 0.0f;
}

// Used to set angle, this should be set as the starting angle
void setAngle_X(float newAngle) 
{ angle_x = newAngle; }
	
// Used to set angle, this should be set as the starting angle
void setAngle_Y(float newAngle)
{ angle_y = newAngle; }

// The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
float getKalmanAngle_X(float newAngle, float newRate, float dt)
{
	// Discrete Kalman filter time update equations - Time Update ("Predict")
	// Update xhat - Project the state ahead
	/* Step 1 */
	rate_x = newRate - bias_x;
	angle_x += dt * rate_x;

	// Update estimation error covariance - Project the error covariance ahead
	/* Step 2 */
	P_x[0][0] += dt * (dt*P_x[1][1] - P_x[0][1] - P_x[1][0] + Q_angle);
	P_x[0][1] -= dt * P_x[1][1];
	P_x[1][0] -= dt * P_x[1][1];
	P_x[1][1] += Q_bias * dt;

	// Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
	// Calculate Kalman gain - Compute the Kalman gain
	/* Step 4 */
	float S = P_x[0][0] + R_measure; // Estimate error
	/* Step 5 */
	float K[2]; // Kalman gain - This is a 2x1 vector
	K[0] = P_x[0][0] / S;
	K[1] = P_x[1][0] / S;

	// Calculate angle and bias - Update estimate with measurement zk (newAngle)
	/* Step 3 */
	float y = newAngle - angle_x; // Angle difference
	/* Step 6 */
	angle_x += K[0] * y;
	bias_x += K[1] * y;

	// Calculate estimation error covariance - Update the error covariance
	/* Step 7 */
	float P00_temp = P_x[0][0];
	float P01_temp = P_x[0][1];

	P_x[0][0] -= K[0] * P00_temp;
	P_x[0][1] -= K[0] * P01_temp;
	P_x[1][0] -= K[1] * P00_temp;
	P_x[1][1] -= K[1] * P01_temp;

	return angle_x;
}

// The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
float getKalmanAngle_Y(float newAngle, float newRate, float dt)
{
	// Discrete Kalman filter time update equations - Time Update ("Predict")
	// Update xhat - Project the state ahead
	/* Step 1 */
	rate_y = newRate - bias_y;
	angle_y += dt * rate_y;

	// Update estimation error covariance - Project the error covariance ahead
	/* Step 2 */
	P_y[0][0] += dt * (dt*P_y[1][1] - P_y[0][1] - P_y[1][0] + Q_angle);
	P_y[0][1] -= dt * P_y[1][1];
	P_y[1][0] -= dt * P_y[1][1];
	P_y[1][1] += Q_bias * dt;

	// Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
	// Calculate Kalman gain - Compute the Kalman gain
	/* Step 4 */
	float S = P_y[0][0] + R_measure; // Estimate error
	/* Step 5 */
	float K[2]; // Kalman gain - This is a 2x1 vector
	K[0] = P_y[0][0] / S;
	K[1] = P_y[1][0] / S;

	// Calculate angle and bias - Update estimate with measurement zk (newAngle)
	/* Step 3 */
	float y = newAngle - angle_y; // Angle difference
	/* Step 6 */
	angle_y += K[0] * y;
	bias_y += K[1] * y;

	// Calculate estimation error covariance - Update the error covariance
	/* Step 7 */
	float P00_temp = P_y[0][0];
	float P01_temp = P_y[0][1];

	P_y[0][0] -= K[0] * P00_temp;
	P_y[0][1] -= K[0] * P01_temp;
	P_y[1][0] -= K[1] * P00_temp;
	P_y[1][1] -= K[1] * P01_temp;

	return angle_y;
}