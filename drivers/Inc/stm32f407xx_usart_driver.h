/*
 * stm32f407xx_usart_driver.h
 *
 *  Created on: Sep 5, 2025
 *      Author: Sashko
 */

#ifndef INC_STM32F407XX_USART_DRIVER_H_
#define INC_STM32F407XX_USART_DRIVER_H_

#include "stm32f407xx.h"

//Configuration structure for the USARTx peripheral
typedef struct{
	uint8_t 	USART_MODE;
	uint32_t	USART_Baud;
	uint8_t		USART_NoOfStopBits;
	uint8_t		USART_WordLength;
	uint8_t 	USART_ParityControl;
	uint8_t 	USART_HWFlowControl;
}USART_Config_t;

typedef struct{
	USART_RegDef_t *pUSARTx;
	USART_Config_t	USART_Config;
}USART_Handle_t;

//Peripheral clock setup
void USART_PeriClockControl(USART_RegDef_t	*pUASRTx, uint8_t EnOrDi);

// Init and De-Init
void USART_Init(USART_Handle_t *pUSARTHandle);
void USART_DeInit(USART_Handle_t *pUSARTHandle);

// Data Send and Receive
void USART_SendData(USART_RegDef_t *pUSARTx, uint8_t *pTxBuffer, uint32_t Len);
void USART_ReceiveData(USART_RegDef_t *pUSARTx, uint8_t *pRxBuffer, uint32_t Len);
uint8_t USART_SendDataIT(USART_RegDef_t *pUSARTx, uint8_t *pTxBuffer, uint32_t Len);
uint8_t USART_ReceiveDataIT(USART_RegDef_t *pUSARTx, uint8_t *pRxBuffer, uint32_t Len);

//IRQ Configuration and ISR handling
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi); //+
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority); //+
void USART_IRQHandling(USART_Handle_t *pHandle);

//Other APIs
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi); //+
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint8_t StatusFlagName); //+
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t StatusFlagName); //+

//Application callback
void USART_ApllicationEventCallBack(USART_Handle_t *pHandle, uint8_t AppEv);

#endif /* INC_STM32F407XX_USART_DRIVER_H_ */
