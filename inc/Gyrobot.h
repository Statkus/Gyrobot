/*
 * Gyrobot.h
 *
 *  Created on: September 14, 2014
 *      Author: Léo Statkus
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef GYROBOT_H_
#define GYROBOT_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32f30x.h"
#include "stm32f3_discovery_l3gd20.h"
#include "stm32f3_discovery_lsm303dlhc.h"


/* Define --------------------------------------------------------------------*/
#define g			   (float)	   9.80665f
#define DegreeToRadian (float)	   0.0174532925f  				// PI/180
#define RadianToDegree (float)	   57.2957795f;					// 180/PI




/* Initialization and Configuration functions *********************************/
void RCC_Configuration(void);
void GPIOs_Configuration (void);
void Timer2_Configuration(void);
void Timer4_Configuration(void);
void USART_Configuration(void);
//void ADC_Configuration(void);

/* Gyrometer, accelerometer and magnetometer functions ************************/
void Gyro_Configuration(void);
void Accelero_Magneto_Configuration(void);
void GyroReadAngRate(int16_t* pfData);
void AcceleroReadAcc(int16_t* pfData);
void MagnetoReadMag(float* pfData);
void ReadAccMag(float* pfData_Acc, float* pfData_Mag);


#endif /* GYROBOT_H_ */
