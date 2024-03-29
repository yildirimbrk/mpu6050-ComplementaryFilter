/********************************************************************************************************************************************
*																																																																						*
*********************************************************************************************************************************************
*		System Name: 																																																														*
*																																																																						*
*		Version: 1.0																																																														*			
*																																																																						*		
*		Module Name:																																																														*	
*																																																																						*
*		File Name: 																																																															*
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
#ifndef __uartInit
#define __uartInit
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
void Hc05_Init(void);
void Hc05_WriteDoubleValue(double doubleValue);
void Hc05_WriteStrValue(char *strValue);
/*																																						*/
/* Private defines -----------------------------------------------------------*/
/*																																						*/
#define HC05_TXPIN GPIO_PIN_6
#define HC05_RXPIN GPIO_PIN_7
#define BAUDRATE 9600
/*																																						*/

#endif
