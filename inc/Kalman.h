/*
 * Kalman.h
 *
 *  Created on: February 23, 2015
 *      Author: Léo Statkus
 *
 *
 *  See http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it/
 *  for more explanation about the Kalman filter
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _Kalman_h
#define _Kalman_h


/* Define --------------------------------------------------------------------*/
#define Q_angle   (float) 0.001f
#define Q_bias    (float) 0.003f
#define R_measure (float) 0.03f
#define dt		  (float) 0.01f



/* Kalman filter functions ****************************************************/
float getAngle(float newAngle, float newRate);	// The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
void setAngle(float newAngle); // Used to set angle, this should be set as the starting angle
float getRate(); // Return the unbiased rate


#endif
