/********************************************************************************************************************************************
*																																																																						*
*********************************************************************************************************************************************
*		System Name: 																																																														*
*																																																																						*
*		Version: 1.0																																																														*			
*																																																																						*		
*		Module Name: MPU6050																																																										*	
*																																																																						*
*		File Name: mpu6050_driver.h																																																							*
*																																																																						*
*		Lead Author: Burak YILDIRIM.																																																						*
*																																																																						*
*		Modification Dates: 																																																										*
*																																																																						*	
*		Description:																																																														*
*																																																																						*
																																																																						*	
*																																																																						*
* 																																																																					*
*		Notes:																																																																	*
*																																																																						*
*	 [1]																																																																			*
*	 																																																																					*
*	 [2]																																																										    							*
*********************************************************************************************************************************************
*																																																																						*	
********************************************************************************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
/*																																						*/
#ifndef __mpu6050_driver
#define __mpu6050_driver
/*																																						*/
/* Includes ------------------------------------------------------------------*/
/*																																						*/
#include "main.h"
/*																																						*/
/*																																						*/
/* Private includes ----------------------------------------------------------*/
/*																																						*/
/*																																						*/
/* Exported types ------------------------------------------------------------*/
/*																																						*/

/*																																						*/
/* Exported constants --------------------------------------------------------*/
/*																																						*/

/*																																						*/
/* Exported macro ------------------------------------------------------------*/
/*																																						*/

/*																																						*/
/* Exported functions prototypes ---------------------------------------------*/
/*																																						*/
void MPU6050_Init(void)	;
void MPU6050_Read_Acell(double *accel_angle_x, double *accel_angle_y) ;
void MPU6050_Read_Gyro(double *gyro_angle_x, double *gyro_angle_y, double *gyro_angle_z);
void complementaryFilter(double *accel_angle_x, double *accel_angle_y, double *Roll_Axis, double *Pitch_Axis);
/*																																						*/
/* Private defines -----------------------------------------------------------*/
/*																																						*/
#define MPU6050_SDA      GPIO_PIN_10
#define MPU6050_SCL      GPIO_PIN_11

#define I2C_STANDARD_CLOCKSPEED 100000
#define I2C_MASTER_ADDRESS1 0
#define I2C_MASTER_ADDRESS2 0

#define I2C_BUSY 													10

#define ALPHA 														0.96
#define PI 																3.14
#define TAMACI 														180.0
#define DIKACI 														90.0
#define SAMPLE_TIME    										0.04 // örnekleme frkansi

/*************************MPU6050 REGISTER ADRESS************************/
#define MPUADDRESS 							((uint8_t)0x68) // MPU6050 I2C adresi

#define PWR_MGHT_1_RA 					((uint8_t)0x6B)		// PWR_MGHT_1 Register Adresi
#define PWR_MGHT_1_DATA0 				((uint8_t)0x00)		// PWR_MGHT_1 gönderilen data (cihaz resetlenme ,uyku modu kapali,cihaz belirlenen hizda çalisir )

#define AFS_SEL 													16384.0 // Duyarlilik Ölçegi Faktörü
#define ACCEL_CONFIG_RA 				((uint8_t)0x1C)		//ACCEL_CONFIG Register Adresi
#define ACCEL_CONFIG_DATA_2G 		((uint8_t)0x00)		// ACCEL_CONFIG gönderilen data (selft test kapali, tam ölçek araligi ±2 G-Force)
#define ACCEL_XOUT_H_RA 				((uint8_t)0x3B) 
#define ACCEL_XOUT_L_RA 				((uint8_t)0x3C) 
#define ACCEL_YOUT_H_RA 				((uint8_t)0x3D) 
#define ACCEL_YOUT_L_RA 				((uint8_t)0x3E)
#define ACCEL_ZOUT_H_RA 				((uint8_t)0x3F) 
#define ACCEL_ZOUT_L_RA 				((uint8_t)0x40)

#define FS_SEL_0 												 	131.0 	//Gyroscope ADC Word Length 16 bits Sensitivity Scale Factor
#define OFFSET													 	360 		// açisal hizda olusan bir yanliskli karsilasiyoruz bunu engellemek için offset degeri kullandim bu cozumu ben buldum ama gayet saglikli calisiyor
#define GYRO_CONFIG_RA 					((uint8_t)0x1B) //GYRO_CONFIG Register Adresi
#define GYRO_CONFIG_DATA_250 		((uint8_t)0x00) // ACCEL_CONFIG gönderilen data (selft test kapali, tam ölçek araligi ±250deg/sn )
#define GYRO_XOUT_H_RA 					((uint8_t)0x43)
#define GYRO_XOUT_L_RA 					((uint8_t)0x44)
#define GYRO_YOUT_H_RA 					((uint8_t)0x45)
#define GYRO_YOUT_L_RA 					((uint8_t)0x46)
#define GYRO_ZOUT_H_RA 					((uint8_t)0x47)
#define GYRO_ZOUT_L_RA 					((uint8_t)0x48)
/*																																						*/
#endif
