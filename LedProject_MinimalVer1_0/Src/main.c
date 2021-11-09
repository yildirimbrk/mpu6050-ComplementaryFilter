/********************************************************************************************************************************************
*																																																																						*
*********************************************************************************************************************************************
*		System Name: 																																																														*
*																																																																						*
*		Version: 1.0																																																														*			
*																																																																						*		
*		Module Name:  																																																													*	
*																																																																						*
*		File Name: main.c																																																												*
*																																																																						*
*		Lead Author: Burak YILDIRIM.																																																						*
*																																																																						*
*		Modification 																																																														*
*																																																																						*	
*		Description:																																																														*
*																																																																						*
*		Notes:																																																																	*
*																																																																						*
*	 [1] 																																																																			*
*																																																																						*
*	 [2]  																																																																		*
*********************************************************************************************************************************************
*																																																																						*	
********************************************************************************************************************************************/
/* Includes ------------------------------------------------------------------*/
/*																																						*/
#include "main.h"

/*																																						*/
/* Private includes ----------------------------------------------------------*/
/*																																						*/
#include "mpu6050_driver.h"
#include "timerInit.h"
#include "Hc05.h"
/*																																						*/
/* Private typedef -----------------------------------------------------------*/
/*																																						*/

/*																																						*/
/* Private define ------------------------------------------------------------*/
/*																																						*/

/*																																						*/
/* Private macro -------------------------------------------------------------*/
/*																																						*/

/*																																						*/
/* Private variables ---------------------------------------------------------*/
/*																																						*/
extern uint32_t currentTime;

uint32_t  timeIntervalMpu6050=0;
uint32_t  timeIntervalHc05=0;

double accel_angle_x,accel_angle_y ;
double gyro_angle_x,gyro_angle_y,gyro_angle_z = 0;
double Roll_Axis,Pitch_Axis = 0 ;
/*																																						*/
/* Private function prototypes -----------------------------------------------*/

/* Private function-----------------------------------------------------------*/
/*																																            */

int main(void)
{
  HAL_Init();
  SystemClock_Config();
	timer2_Init();
  MPU6050_Init();
	Hc05_Init();
	
  while (1)
  {
		/*0.04 saniye'lik örnekleme frekansi ile Açi degerlerini al*/
		if((currentTime - timeIntervalMpu6050) > 40)
		{
			timeIntervalMpu6050 = currentTime;
			
			MPU6050_Read_Acell(&accel_angle_x,&accel_angle_y); //
			MPU6050_Read_Gyro(&gyro_angle_x,&gyro_angle_y,&gyro_angle_z);
			complementaryFilter(&accel_angle_x,&accel_angle_y,&Roll_Axis,&Pitch_Axis);

		}
		
		/*0.1 saniyede Terminal ekrana verileri gönder.*/
		if((currentTime-timeIntervalHc05)>100)
		{
			timeIntervalHc05 = currentTime ;
			
			Hc05_WriteDoubleValue(accel_angle_y); //terminal ekrana Gyro açisini yazdir
			Hc05_WriteStrValue("			");
			Hc05_WriteDoubleValue(gyro_angle_y);  //terminal ekrana ivme açisini yazdir
			Hc05_WriteStrValue("			");
			Hc05_WriteDoubleValue(Pitch_Axis); //terminal ekrana filitre açisini yazdir
			Hc05_WriteStrValue("\r\n");
			
		}
		
  }
}
/*																																            */
















