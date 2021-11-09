/********************************************************************************************************************************************
*																																																																						*
*********************************************************************************************************************************************
*		System Name: 																																																														*
*																																																																						*
*		Version: 1.0																																																														*			
*																																																																						*		
*		Module Name: HC05 																																																											*	
*																																																																						*
*		File Name: Hc05.c																																																												*
*																																																																						*
*		Lead Author: Burak YILDIRIM.																																																						*
*																																																																						*
*		Modification 																																																														*
*																																																																						*	
*		Description:																																																														*
*				HC05 modülü UART seri haberlesme protokolünü kullanmaktadir 																																				*
* 		  HC05 modülü ile bluetooth kablosuz haberlesme ile veri aktarimi saglamaktadir																												*
*		Notes:																																																																	*
*																																																																						*
*	 [1] 																																																																			*
*																																																																						*
*			Projede Drivers/STM32F4xx_HAL_Driver altinda UART,USART  icin kullanilacak .c dosyalarini  ekle:																			*
*				stm32f4xx_hal_uart.c																																																								*
*				stm32f4xx_hal_uart_ex.c																																																							*
*				stm32f4xx_hal_usart.c																																																								*
*				stm32f4xx_hal_usart_ex.c																																																						*
*																																																																						*
*				stm32f4xx_hal_conf.h icerisindeki 																																																	*
*				 #define HAL_UART_MODULE_ENABLED  																																																	*	 
* 			 #define HAL_USART_MODULE_ENABLED 																																																	*
*	 [2]  																																																																		*
*********************************************************************************************************************************************
*																																																																						*	
********************************************************************************************************************************************/
/* Includes ------------------------------------------------------------------*/
/*																																						*/
#include "Hc05.h"
/*																																						*/
/* Private includes ----------------------------------------------------------*/
/*																																						*/
#include "string.h"
#include "stdio.h"
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
UART_HandleTypeDef USART1InitStruct;
/*																																						*/
/* Private function prototypes -----------------------------------------------*/

/* Private function-----------------------------------------------------------*/
/*																																            */

/**
  * @brief 	HC05 modülü için ön ayarlarin konfigürasyonu
  * @param 	yok 
  * @retval yok
	* @note 	STM32F407VG UART çevre birimi ön ayari; 
	*					USART_1 hatti kullanilmistir
	*					PB6  ------> USART_TX
	*					PB7  ------> USART_RX
	*					BAUD RATE 	 = 9600
	*					Bit uzunlugu = 8
	*					Stop Bit     = 1
  */
void Hc05_Init(void)
{
	
	__HAL_RCC_USART1_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	
	GPIO_InitTypeDef uart1PinStruct;
	
	uart1PinStruct.Pin 				= HC05_TXPIN | HC05_RXPIN;
  uart1PinStruct.Mode 			= GPIO_MODE_AF_PP;
	uart1PinStruct.Alternate 	= GPIO_AF7_USART1;
  uart1PinStruct.Pull 			= GPIO_PULLUP;
  uart1PinStruct.Speed 			= GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOB, &uart1PinStruct);

  USART1InitStruct.Instance 					= USART1;
  USART1InitStruct.Init.BaudRate 			= BAUDRATE;
  USART1InitStruct.Init.WordLength 		= UART_WORDLENGTH_8B;
  USART1InitStruct.Init.StopBits 			= UART_STOPBITS_1;
  USART1InitStruct.Init.Parity 				= UART_PARITY_NONE;
  USART1InitStruct.Init.Mode 					= UART_MODE_TX_RX;
  USART1InitStruct.Init.HwFlowCtl 		= UART_HWCONTROL_NONE;
  USART1InitStruct.Init.OverSampling 	= UART_OVERSAMPLING_16;
	HAL_UART_Init(&USART1InitStruct);
	
}

/**
  * @brief 	HC05 modülü ile double cinsindeki verileri gönderme fonksiyonu
  * @param 	Gönderilmek istenilen veri double cinsinden veri  
  * @retval yok
  */
void Hc05_WriteDoubleValue(double doubleValue)
{
	char sendTxt[50];
	
	sprintf(sendTxt,"%.2f",doubleValue);
	HAL_UART_Transmit(&USART1InitStruct,(uint8_t*) sendTxt,strlen(sendTxt),0xffff);
}

/**
  * @brief 	HC05 modülü ile string cinsindeki verileri gönderme fonksiyonu
  * @param 	Gönderilmek istenilen string veri 
  * @retval yok
  */
void Hc05_WriteStrValue(char *strValue)
{
	char sendTxt[50];
	
	sprintf(sendTxt,"%s",strValue);
	HAL_UART_Transmit(&USART1InitStruct,(uint8_t*) sendTxt,strlen(sendTxt),0xffff);
}
/*																																            */

