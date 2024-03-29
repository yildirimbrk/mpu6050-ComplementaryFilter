/********************************************************************************************************************************************
*																																																																						*
*********************************************************************************************************************************************
*		System Name: 																																																														*
*																																																																						*
*		Version: 1.0																																																														*			
*																																																																						*		
*		Module Name:	MPU6050																																																										*	
*																																																																						*
*		File Name: 	mpu6050_driver.c																																																						*
*																																																																						*
*		Lead Author: Burak YILDIRIM.																																																						*
*																																																																						*
*		Modification :																																																													*
*																																																																						*	
*		Description:																																																														*
*																																																																						*
*		bu dosya da mpu-6050 gyro ve ivme �l�er sens�r� kullanilarak a�isal islemler yapilmistir																								*
*		mpu6050 sens�r� I2C aray�z� kullanmaktadir 																																															*
*		jiroskop sens�rlerinin ve ivme sens�rlerinin dezavantajlari g�z �n�ne alinarak bu iki sens�r verilerini 																*
*		filtreleme algoritmasi kullanarak olusan hatali �l��mler en aza indirgenme hedeflenmistir																								*
*																																																																						*
* 																																																																					*
*		Notes:																																																																	*
*																																																																						*
*	 [1] 																																																																			*
*																																																																						*
*			Projede Drivers/STM32F4xx_HAL_Driver altinda I2C icin kullanilacak .c dosyalarini  ekle:																							*
*				stm32f4xx_hal_i2c.c																																																									*
*				stm32f4xx_hal_i2c_ex.c																																																							*
*																																																																						*
*				stm32f4xx_hal_conf.h icerisindeki 																																																	*
*				#define HAL_I2C_MODULE_ENABLED satirini aktiflestir.																																								*
*																																																																						*			
*  [2]																																																																			*
*				MPU6050 sens�r� i�in konfig�rasyon degerleri src/mpu6050_driver.h dosyasinda tanimlanmistir			  																	*
*	 																																																																					*
*	  																																																																				*
*********************************************************************************************************************************************
*																																																																						*	
********************************************************************************************************************************************/
/* Includes ------------------------------------------------------------------*/
/*																																						*/
#include "mpu6050_driver.h"

/*																																						*/
/* Private includes ----------------------------------------------------------*/
/*																																						*/
#include "math.h"
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
I2C_HandleTypeDef I2C2_InitStruct ;

uint8_t pwr_mght_1[2] 	= {PWR_MGHT_1_RA,PWR_MGHT_1_DATA0}	; 				// mod�l a�ma-kapama Registerlari
uint8_t accel_config[2] = {ACCEL_CONFIG_RA, ACCEL_CONFIG_DATA_2G} ;		// ivme konfig register lari
uint8_t accel_xout[2] 	= {ACCEL_XOUT_H_RA,ACCEL_XOUT_L_RA} ;        	// X ekseni ivme deger Registerlari
uint8_t accel_yout[2] 	= {ACCEL_YOUT_H_RA,ACCEL_YOUT_L_RA} ;        	// Y ekseni ivme deger Registerlari
uint8_t accel_zout[2] 	= {ACCEL_ZOUT_H_RA,ACCEL_ZOUT_L_RA} ;        	// Z ekseni ivme deger Registerlari
uint8_t gyro_config[2] 	= {GYRO_CONFIG_RA, GYRO_CONFIG_DATA_250} ;  	// gyro konfig register lari
uint8_t gyro_xout[2] 		= {GYRO_XOUT_H_RA,GYRO_XOUT_L_RA} ;					 	// X ekseni gyro deger Registerlari
uint8_t gyro_yout[2] 		= {GYRO_YOUT_H_RA, GYRO_YOUT_L_RA} ; 				 	// Y ekseni gyro deger Registerlari
uint8_t gyro_zout[2] 		= {GYRO_ZOUT_H_RA, GYRO_ZOUT_L_RA} ;					// Z ekseni gyro deger Registerlari

uint8_t accel_x_msb_bit, accel_y_msb_bit, accel_z_msb_bit ;  // MSB bitleri
uint8_t accel_x_lsb_bit,accel_y_lsb_bit,accel_z_lsb_bit ;    //	LSB bitleri
uint8_t gyro_x_msb_bit, gyro_y_msb_bit, gyro_z_msb_bit = 0 ; // MSB bitleri
uint8_t gyro_x_lsb_bit, gyro_y_lsb_bit, gyro_z_lsb_bit = 0 ; // MSB bitleri

int16_t accel_x,accel_y,accel_z ;    		// ham ivme verileri
int16_t gyro_X,gyro_Y,gyro_Z = 0 ; 			 // ham gyro verileri

double g_force_x, g_force_y, g_force_z = 0;
double angular_velocity_x, angular_velocity_y, angular_velocity_z = 0;
double R=0; // birleske vekt�r
double prev_gyro_angle_x, prev_gyro_angle_y, prev_gyro_angle_z;
double filtered_gyro_angle_x, filtered_gyro_angle_y = 0;
/*																																						*/
/* Private function prototypes -----------------------------------------------*/
/*																																						*/
/*																																						*/
/* Private function-----------------------------------------------------------*/
/*																																            */

/**
  * @brief MPU6050 mod�l� i�in �n ayarlarin konfig�rasyonu
  * @param yok 
  * @retval yok
	* @note STM32F407VG I2C �evre birimi �n ayari; 
					I2C_2 hatti kullanilmistir
					PB10   ------> I2C2_SCL
					PB11   ------> I2C2_SDA
					I2C_STANDARD_CLOCKSPEED =100000
		@note MPU6050 sens�r �n ayari;
					Jiroskop Duyarlilik �l�egi Fakt�r� = 131
					Ivme Duyarlilik �l�egi Fakt�r� 		 = 16384
  */	
void MPU6050_Init(void)	
{
	__HAL_RCC_GPIOB_CLK_ENABLE()	;
	__HAL_RCC_I2C2_CLK_ENABLE()	;
	
	GPIO_InitTypeDef	I2C2_GPIOInitStruct ;
	
	I2C2_GPIOInitStruct.Alternate =	GPIO_AF4_I2C2 ;
	I2C2_GPIOInitStruct.Mode 			= GPIO_MODE_AF_OD	;
	I2C2_GPIOInitStruct.Pin				= MPU6050_SDA | MPU6050_SCL ;
	I2C2_GPIOInitStruct.Pull			= GPIO_PULLUP ;
	I2C2_GPIOInitStruct.Speed			= GPIO_SPEED_FREQ_HIGH ;
	HAL_GPIO_Init(GPIOB,&I2C2_GPIOInitStruct) ;
	
	
	I2C2_InitStruct.Instance = I2C2 ;
	
	I2C2_InitStruct.Init.AddressingMode 	= I2C_ADDRESSINGMODE_7BIT	;
	I2C2_InitStruct.Init.ClockSpeed 			= I2C_STANDARD_CLOCKSPEED ;
	I2C2_InitStruct.Init.DualAddressMode 	=	I2C_DUALADDRESS_DISABLE ;
	I2C2_InitStruct.Init.DutyCycle 				= I2C_DUTYCYCLE_2 ;
	I2C2_InitStruct.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE ;
	I2C2_InitStruct.Init.NoStretchMode		= I2C_NOSTRETCH_DISABLE	;
	I2C2_InitStruct.Init.OwnAddress1			= I2C_MASTER_ADDRESS1 ;
	I2C2_InitStruct.Init.OwnAddress2			= I2C_MASTER_ADDRESS2 ;
	HAL_I2C_Init(&I2C2_InitStruct);
	
	HAL_I2C_Master_Transmit(&I2C2_InitStruct, MPUADDRESS<<1, (uint8_t *) pwr_mght_1, 2, I2C_BUSY) ;
	
	HAL_I2C_Master_Transmit(&I2C2_InitStruct, MPUADDRESS<<1, (uint8_t *) accel_config, 2, I2C_BUSY) ;
	
	HAL_I2C_Master_Transmit(&I2C2_InitStruct, MPUADDRESS<<1,(uint8_t *) gyro_config, 2, I2C_BUSY) ; 
	
}

/**
  * @brief Bu fonksiyon ivme sens�rlerinden elde edilen G kuvvetlerini 
	*				 x ve y a�i degerlerine d�n�st�rerek kullanica sunar
  * @param X ekseni ivme a�isini g�steren bir accel_angle_x pointer 
  * @param Y ekseni ivme a�isini g�steren bir accel_angle_y pointer 
  * @retval yok 
  */
void MPU6050_Read_Acell(double *accel_angle_x, double *accel_angle_y) 
{

	HAL_I2C_Master_Transmit(&I2C2_InitStruct, MPUADDRESS<<1, &accel_xout[0], 1, I2C_BUSY) ; //   -> accel_xout_h 
	HAL_I2C_Master_Receive(&I2C2_InitStruct, MPUADDRESS<<1, &accel_x_msb_bit, 1, I2C_BUSY) ;
	
	HAL_I2C_Master_Transmit(&I2C2_InitStruct, MPUADDRESS<<1, &accel_xout[1], 1, I2C_BUSY) ; //  -> accel_xout_l
	HAL_I2C_Master_Receive(&I2C2_InitStruct, MPUADDRESS<<1, &accel_x_lsb_bit, 1, I2C_BUSY) ;
		
	HAL_I2C_Master_Transmit(&I2C2_InitStruct,MPUADDRESS<<1,&accel_yout[0],1,I2C_BUSY) ;	//   -> accel_yout_h 
	HAL_I2C_Master_Receive(&I2C2_InitStruct,MPUADDRESS<<1,&accel_y_msb_bit,1,I2C_BUSY) ;
	
	HAL_I2C_Master_Transmit(&I2C2_InitStruct,MPUADDRESS<<1,&accel_yout[1],1,I2C_BUSY) ;	//  -> accel_yout_l
	HAL_I2C_Master_Receive(&I2C2_InitStruct,MPUADDRESS<<1,&accel_y_lsb_bit,1,I2C_BUSY) ;
	
	HAL_I2C_Master_Transmit(&I2C2_InitStruct, MPUADDRESS<<1, &accel_zout[0], 1, I2C_BUSY) ;	//   -> accel_zout_h 
	HAL_I2C_Master_Receive(&I2C2_InitStruct, MPUADDRESS<<1, &accel_z_msb_bit, 1, I2C_BUSY) ;

	HAL_I2C_Master_Transmit(&I2C2_InitStruct, MPUADDRESS<<1, &accel_zout[1], 1, I2C_BUSY) ;	//  -> accel_zout_l
	HAL_I2C_Master_Receive(&I2C2_InitStruct, MPUADDRESS<<1, &accel_z_lsb_bit, 1, I2C_BUSY) ;

	accel_x =( (accel_x_msb_bit<<8) | accel_x_lsb_bit ) ;
	accel_y =( (accel_y_msb_bit<<8) | accel_y_lsb_bit ) ;
	accel_z =( (accel_z_msb_bit<<8) | accel_z_lsb_bit ) ;
	
	g_force_x = accel_x / AFS_SEL ; // X G-force degeri
	g_force_y = accel_y / AFS_SEL ;	// Y G-force degeri
	g_force_z = accel_z / AFS_SEL ;	// Z G-force degeri
	
	R = (double) sqrt( ( pow(g_force_x, 2) ) + ( pow(g_force_y, 2) ) + ( pow(g_force_z, 2) ) ) ;	//birleske vekt�r
	*accel_angle_y = (double) ( DIKACI - (acos(g_force_x / R) * TAMACI / PI) ) ; // bileske vekt�r�n X ekseniyle olan a�isi (y ekseni a�isi)
	*accel_angle_x = (double) ( DIKACI - (acos(g_force_y / R) * TAMACI / PI) ) ; // bileske vekt�r�n Y ekseniyle olan a�isi	(x ekseni a�isi)
	
}

/**
  * @brief 	Bu fonksiyon jiroskop sens�rlerinden elde edilen a�isal hiz degerlirini 
	*				 	x ve y a�i degerlerine d�n�st�rerek kullanica sunar
  * @param 	X ekseni jiroskop a�isini g�steren bir gyro_angle_x pointer 
  * @param 	Y ekseni jiroskop a�isini g�steren bir gyro_angle_y pointer 
  * @retval yok 
	* @note 	jiroskop elde edilen jiroskop verileri filitrelenmemistir.
  */
void MPU6050_Read_Gyro(double *gyro_angle_x, double *gyro_angle_y, double *gyro_angle_z)
{ 

	HAL_I2C_Master_Transmit(&I2C2_InitStruct, MPUADDRESS<<1, &gyro_xout[0], 1, I2C_BUSY) ;	// -> gyro_xout_h
	HAL_I2C_Master_Receive(&I2C2_InitStruct, MPUADDRESS<<1, &gyro_x_msb_bit, 1, I2C_BUSY) ;
	
	HAL_I2C_Master_Transmit(&I2C2_InitStruct, MPUADDRESS<<1, &gyro_xout[1], 1, I2C_BUSY) ;	// -> gyro_xout_l
	HAL_I2C_Master_Receive(&I2C2_InitStruct, MPUADDRESS<<1, &gyro_x_lsb_bit, 1, I2C_BUSY) ;
	
	HAL_I2C_Master_Transmit(&I2C2_InitStruct, MPUADDRESS<<1, &gyro_yout[0], 1, I2C_BUSY) ;	// -> gyro_yout_h
	HAL_I2C_Master_Receive(&I2C2_InitStruct, MPUADDRESS<<1, &gyro_y_msb_bit, 1, 10) ;

	HAL_I2C_Master_Transmit(&I2C2_InitStruct, MPUADDRESS<<1, &gyro_yout[1], 1, I2C_BUSY) ;	// -> gyro_yout_l
	HAL_I2C_Master_Receive(&I2C2_InitStruct, MPUADDRESS<<1, &gyro_y_lsb_bit, 1, I2C_BUSY) ;
	
	HAL_I2C_Master_Transmit(&I2C2_InitStruct, MPUADDRESS<<1, &gyro_zout[0], 1, I2C_BUSY) ;	// -> gyro_zout_h
	HAL_I2C_Master_Receive(&I2C2_InitStruct, MPUADDRESS<<1, &gyro_z_msb_bit, 1, I2C_BUSY) ;

	HAL_I2C_Master_Transmit(&I2C2_InitStruct, MPUADDRESS<<1, &gyro_zout[1], 1, I2C_BUSY) ;	// -> gyro_zout_l
	HAL_I2C_Master_Receive(&I2C2_InitStruct, MPUADDRESS<<1, &gyro_z_lsb_bit, 1, I2C_BUSY) ;
	
	gyro_X = ( (gyro_x_msb_bit<<8) | gyro_x_lsb_bit ) ;
	gyro_Y = ( (gyro_y_msb_bit<<8) | gyro_y_lsb_bit ) ;
	gyro_Z = ( (gyro_z_msb_bit<<8) | gyro_z_lsb_bit );
	
	angular_velocity_x = (double) ( (gyro_X / FS_SEL_0 )) ; // X pozisyonunun a�isal hizi deg/s
	angular_velocity_y = (double) ( (gyro_Y / FS_SEL_0 )) ;	// Y pozisyonunun a�isal hizi deg/s
	angular_velocity_z = (double) ( (gyro_Z / FS_SEL_0 )) ;	// Z pozisyonunun a�isal hizi deg/s
	
	/*filitrelenmemis gyro a�isi*/
	*gyro_angle_x = prev_gyro_angle_x +(SAMPLE_TIME * angular_velocity_x) ; //	a�i = �nceki gyro A�isi +(sn * (deg/sn) = deg)
	prev_gyro_angle_x = *gyro_angle_x;
	*gyro_angle_y = prev_gyro_angle_y + (SAMPLE_TIME * angular_velocity_y) ;
	prev_gyro_angle_y = *gyro_angle_y;
	*gyro_angle_z = prev_gyro_angle_z + (SAMPLE_TIME * angular_velocity_z) ; 
  prev_gyro_angle_z = *gyro_angle_z;
	  
}

/**
  * @brief 	Bu fonksiyon ivme ve gyro sens�r�n� al�ak ge�irgen filitreleme 
	*				 	ile birlestirerek Roll_Axis(x ekseni) ve Pitch_Axis (y ekseni) eksenlerini kullanica sunar
  * @param 	X ekseni ivme a�isi accel_angle_x pointer 
  * @param 	Y ekseni ivme a�isi bir gyro_angle_y pointer 
  * @param 	Roll ekseni a�isini g�steren bir Roll_Axis pointer
  * @param 	Pitch ekseni a�isini g�steren bir Pitch_Axis pointer
  * @retval yok 
	* @note  	jiroskop a�isi filitrelenmis jiroskop verisi olmasi gereklidir 
  */
void complementaryFilter(double *accel_angle_x, double *accel_angle_y, double *Roll_Axis, double *Pitch_Axis)
{
	/*Complemantry Filtresi
		Filtrelenen A�i = a � (filtrelenmis Jiroskop A�isi) + (1 - a) � (Ivme�l�er A�isi)
		a 			= T/T+DELTA_T 
		T 			= gyro sens�r�m�z deg/s yani saniye cinsinden hesap yaptigi i�in 1 saniye
		DELTA_T = gyro sens�r�n�n �rnekleme peryodu yani 0.04 saniye 	*/
 
	filtered_gyro_angle_x = angular_velocity_x*SAMPLE_TIME + filtered_gyro_angle_x;
	filtered_gyro_angle_y = angular_velocity_y*SAMPLE_TIME + filtered_gyro_angle_y;
	
	*Roll_Axis = ( ALPHA *  (filtered_gyro_angle_x)  ) + ( (1 - ALPHA) * (*accel_angle_x) )  ; 
	*Pitch_Axis = ( ALPHA *  (filtered_gyro_angle_y) ) + ( (1 - ALPHA) * (*accel_angle_y) )  ; 
	
	filtered_gyro_angle_x = *Roll_Axis ;
	filtered_gyro_angle_y = *Pitch_Axis ;
	
}
/*																																						*/
