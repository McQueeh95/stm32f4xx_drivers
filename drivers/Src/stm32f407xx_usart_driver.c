/*
 * stm32f407xx_usart_driver.c
 *
 *  Created on: Oct 20, 2025
 *      Author: Sashko
 */

#include "stm32f407xx_usart_driver.h"

void USART_PeriClockControl(USART_RegDef_t	*pUASRTx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		if(pUASRTx == USART1)
		{
			USART1_PCLOCK_EN();
		}
		else if(pUASRTx == USART2)
		{
			USART2_PCLOCK_EN();
		}
		else if(pUASRTx == USART3)
		{
			USART3_PCLOCK_EN();
		}
		else if(pUASRTx == UART4)
		{
			UART4_PCLOCK_EN();
		}
		else if(pUASRTx == UART5)
		{
			UART5_PCLOCK_EN();
		}
		else if(pUASRTx == USART6)
		{
			USART6_PCLOCK_EN();
		}
	}
	else
	{
		if(pUASRTx == USART1)
		{
			USART1_PCLOCK_DI();
		}
		else if(pUASRTx == USART2)
		{
			USART2_PCLOCK_DI();
		}
		else if(pUASRTx == USART3)
		{
			USART3_PCLOCK_DI();
		}
		else if(pUASRTx == UART4)
		{
			UART4_PCLOCK_DI();
		}
		else if(pUASRTx == UART5)
		{
			UART5_PCLOCK_DI();
		}
		else if(pUASRTx == USART6)
		{
			USART6_PCLOCK_DI();
		}
	}
}

void USART_Init(USART_Handle_t *pUSARTHandle);
void USART_DeInit(USART_Handle_t *pUSARTHandle)
{
	if(pUASRTx == USART1)
	{
		USART1_PCLOCK_DI();
	}
	else if(pUASRTx == USART2)
	{
		USART2_PCLOCK_DI();
	}
	else if(pUASRTx == USART3)
	{
		USART3_PCLOCK_DI();
	}
	else if(pUASRTx == UART4)
	{
		UART4_PCLOCK_DI();
	}
	else if(pUASRTx == UART5)
	{
		UART5_PCLOCK_DI();
	}
	else if(pUASRTx == USART6)
	{
		USART6_PCLOCK_DI();
	}
}
