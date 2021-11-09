/********************************************************************************************************************************************
*																																																																						*
*********************************************************************************************************************************************
*		System Name: 																																																														*
*																																																																						*
*		Version: 1.0																																																														*			
*																																																																						*		
*		Module Name:																																																														*	
*																																																																						*
*		File Name: 	timerInit.c																																																									*
*																																																																						*
*		Lead Author: Burak YILDIRIM.																																																						*
*																																																																						*
*		Modification :																																																													*
*																																																																						*	
*		Description:																																																														*
*																																																																						*
*				Mikrodenetleyicinin mevcut programi çalistirmaya baslamasindan bu yana geçen milisaniye sayisini döndürür.												  *										
*																																																																						*
*		Notes:																																																																	*
*																																																																						*
*	 [1] 																																																																			*
*																																																																						*
*			Projede Drivers/STM32F4xx_HAL_Driver altinda TIM icin kullanilacak .c dosyalarini  ekle:																							*
*				stm32f4xx_hal_tim.c																																																									*
*				stm32f4xx_hal_tim_ex.c																																																							*
*																																																																						*
*				stm32f4xx_hal_conf.h icerisindeki 																																																	*
*				#define HAL_TIM_MODULE_ENABLED satirini aktiflestir.																																								*
*		[2]																																																																			*
*	 																																																																					*
*	   																																																																				*
*********************************************************************************************************************************************
*																																																																						*	
********************************************************************************************************************************************/
/* Includes ------------------------------------------------------------------*/
/*																																						*/
#include "timerInit.h"
/*																																						*/
/* Private includes ----------------------------------------------------------*/
/*																																						*/
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
TIM_HandleTypeDef TIM2_InitStruct;

volatile uint32_t currentTime  =0;
/*																																						*/
/* Private function prototypes -----------------------------------------------*/
/*																																						*/
/*																																						*/
/* Private function-----------------------------------------------------------*/
/*																																            */

/**
  * @brief Timer 2 kesme fonksiyonu 
  * @param yok 
  * @retval yok
	* @note 4294967295 degerine ulasinca currentTime sayisi tasacak (sifira dönecek) 
  */	
void TIM2_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&TIM2_InitStruct);
	currentTime +=1 ;

}

/**
  * @brief timer çevre birimi için ön ayarlarin konfigürasyonu
  * @param yok 
  * @retval yok
	* @note STM32F407VG TIM çevre birimi ön ayari; 
	*				TIM2 hatti kullanilmistir
	*				Ana clock frekansi= 16Mhz 
	*				Prescaler 				=16
	*				Counter Mode  		=	up
	*				Period        		=	1000
	*				
  */	
void timer2_Init(void)
{
	
	__HAL_RCC_TIM2_CLK_ENABLE() ; 
	
	TIM2_InitStruct.Instance 					= TIM2 ;
	TIM2_InitStruct.Init.Prescaler 		= TIM2_PRESCALER ; 
	TIM2_InitStruct.Init.CounterMode  = TIM_COUNTERMODE_UP ;
	TIM2_InitStruct.Init.Period 			= TIM2_PERIOD;
	HAL_TIM_Base_Init(&TIM2_InitStruct) ;
	
	HAL_TIM_Base_Start_IT(&TIM2_InitStruct) ; 

	HAL_NVIC_SetPriority(TIM2_IRQn, 1, 1) ; 
	HAL_NVIC_EnableIRQ(TIM2_IRQn) ; 

	HAL_TIM_Base_Start(&TIM2_InitStruct) ; 
}

