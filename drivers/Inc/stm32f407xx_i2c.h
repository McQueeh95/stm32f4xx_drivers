/*
 * stm32f407xx_i2c.h
 *
 *  Created on: Aug 13, 2025
 *      Author: Sashko
 */

#ifndef INC_STM32F407XX_I2C_H_
#define INC_STM32F407XX_I2C_H_
#include "stm32f407xx.h"
/*
 * Configuration structure for I2Cx peripheral
 */
typedef struct
{
	uint32_t	I2C_SCLSpeed;
	uint8_t		I2C_DeviceAddress;
	uint8_t		I2C_ACKControl;
	uint16_t	I2C_FMDutyCycle;
}I2C_Config_t;

/*
 * Handle structure for I2Cx peripheral
 */
typedef struct
{
	I2C_RegDef_t 	*pI2Cx;
	I2C_Config_t 	I2C_Config;
	uint8_t 		*pTxBuffer; //To store current TxBuffer of app
	uint8_t 		*pRxBuffer; //To store current RxBuffer of app
	uint32_t 		TxLen;		//To store current len of TxBuffer
	uint32_t 		RxLen;		//To store current len of RxBuffer
	uint8_t 		TxRxState;	//To store TxRxState (0 - Ready, 1 - BusyInRx, 2 - BusyInTx)
	uint8_t		 	DevAddr;	//To store slave device addr
	uint32_t 		RxSize;		//To store RxSize
	uint8_t			Sr;			//Is repeated start enabled
}I2C_Handle_t;

/*
 * I2C application state
 */
#define I2C_READY			(0)
#define I2C_BUSY_IN_RX		(1)
#define I2C_BUSY_IN_TX		(2)


/*
 * @I2C_SCLSpeed
 */
#define I2C_SCL_SPEED_SM	100000
#define I2C_SCL_SPEED_FM2K	200000
#define I2C_SCL_SPEED_FM4K	400000

/*
 * @I2C_AckControl
 */
#define I2C_ACK_EN			1
#define I2C_ACK_DI			0

/*
 * @I2C_DutyCycle
 */
#define I2C_FM_DUTY_2		0
#define I2C_FM_DUTY_16_9	1

/*
 * I2C related flag status
 */
#define I2C_FLAG_SB			(1 << I2C_SR1_SB)
#define I2C_FLAG_ADDR		(1 << I2C_SR1_ADDR)
#define I2C_FLAG_BTF		(1 << I2C_SR1_BTF)
#define I2C_FLAG_ADD10		(1 << I2C_SR1_ADD10)
#define I2C_FLAG_STOPF		(1 << I2C_SR1_STOPF)
#define I2C_FLAG_RxNE		(1 << I2C_SR1_RxNE)
#define I2C_FLAG_TxE		(1 << I2C_SR1_TxE)
#define I2C_FLAG_BERR		(1 << I2C_SR1_BERR)
#define I2C_FLAG_ARLO		(1 << I2C_SR1_ARLO)
#define I2C_FLAG_AF			(1 << I2C_SR1_AF)
#define I2C_FLAG_OVR		(1 << I2C_SR1_OVR)
#define I2C_FLAG_TIMEOUT	(1 << I2C_SR1_TIMEOUT)

#define I2C_FLAG_MSL		(1 << I2C_SR2_MSL)
#define I2C_FLAG_BUSY		(1 << I2C_SR2_BUSY)
#define I2C_FLAG_TRA		(1 << I2C_SR2_TRA)
#define I2C_FLAG_GENCALL	(1 << I2C_SR2_GENCALL)
#define I2C_DUALF			(1 << I2C_SR2_DUALF)

#define I2C_DISABLE_SR		RESET
#define I2C_ENABLE_SR		SET

/*
 *  I2C application events macros
 */
#define I2C_EV_TX_CMPLT		0
#define I2C_EV_STOP			1
#define I2C_EV_RX_CMPLT		2
#define I2C_ERROR_BERR  	3
#define I2C_ERROR_ARLO  	4
#define I2C_ERROR_AF    	5
#define I2C_ERROR_OVR   	6
#define I2C_ERROR_TIMEOUT 	7
#define I2C_EV_DATA_REQ		8
#define I2C_EV_DATA_RCV		9

/************************************************
 * 			APIs supported by this driver
 ************************************************/

/*
 * Peripheral clock setup
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

//Init De-init
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

/*
 * Data send and receive
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Sr);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Sr);

void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle);
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle);

/*
 * Data send and receive Interrupt based
 */
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Sr);
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Sr);

void I2C_SlaveSendData(I2C_RegDef_t *pI2C, uint8_t data);
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2C);
/*
 * IRQ Configuration and ISR handling
 */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);

/*
 * Other peripheral control APIs
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName);
void I2C_Ack_Control(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);
void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);
void I2C_SlaveControlCallBackEvent(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);
/*
 * Application callback
 */

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv);


#endif /* INC_STM32F407XX_I2C_H_ */
