/*
 * Kalman.c
 *
 *  Created on: February 23, 2015
 *      Author: Léo Statkus
 *
 *
 *  See http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it/
 *  for more explanation about the Kalman filter
 */


/* Includes ------------------------------------------------------------------*/
#include "Kalman.h"


/* Global variables ----------------------------------------------------------*/
float actual_angle = 0.0f; // Reset the angle
float bias = 0.0f; // Reset bias
float rate;

float P[2][2] = {{0.0f, 0.0f},
				 {0.0f, 0.0f}};  // Since we assume that the bias is 0 and we know the starting angle (use setAngle), the error covariance matrix is set like so - see: http://en.wikipedia.org/wiki/Kalman_filter#Example_application.2C_technical




/**
  * @brief Calculate angle tilt with the Kalman filter
  * @param  newAngle : angle calculate with the accelerometer, newRate : angle rate measured with the gyrometer
  * @retval The angle tilt
  */
float getAngle(float newAngle, float newRate) {
	// KasBot V2  -  Kalman filter module - http://www.x-firm.com/?page_id=145
	// Modified by Kristian Lauszus
	// See my blog post for more information: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it

	// Discrete Kalman filter time update equations - Time Update ("Predict")
	// Update xhat - Project the state ahead
	/* Step 1 */
	rate = newRate - bias;
	actual_angle += dt * rate;

	// Update estimation error covariance - Project the error covariance ahead
	/* Step 2 */
	P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
	P[0][1] -= dt * P[1][1];
	P[1][0] -= dt * P[1][1];
	P[1][1] += Q_bias * dt;

	// Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
	// Calculate Kalman gain - Compute the Kalman gain
	/* Step 4 */
	float S = P[0][0] + R_measure; // Estimate error
	/* Step 5 */
	float K[2]; // Kalman gain - This is a 2x1 vector
	K[0] = P[0][0] / S;
	K[1] = P[1][0] / S;

	// Calculate angle and bias - Update estimate with measurement zk (newAngle)
	/* Step 3 */
	float y = newAngle - actual_angle; // Angle difference
	/* Step 6 */
	actual_angle += K[0] * y;
	bias += K[1] * y;

	// Calculate estimation error covariance - Update the error covariance
	/* Step 7 */
	float P00_temp = P[0][0];
	float P01_temp = P[0][1];

	P[0][0] -= K[0] * P00_temp;
	P[0][1] -= K[0] * P01_temp;
	P[1][0] -= K[1] * P00_temp;
	P[1][1] -= K[1] * P01_temp;

	return actual_angle;
};


/**
  * @brief Used to set angle, this should be set as the starting angle
  * @param newAngle : starting angle in degree
  * @retval None
  */
void setAngle(float newAngle) {
	actual_angle = newAngle;
};


/**
  * @brief Return the unbiased rate
  * @param None
  * @retval The unbiased rate
  */
float getRate() {
	return rate;
};



