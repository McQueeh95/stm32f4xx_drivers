/*
 * stm32f407xx_i2c.c
 *
 *  Created on: Aug 13, 2025
 *      Author: Sashko
 */

#include "stm32f407xx_i2c.h"

uint16_t AHB_PreScaler[8] = {2, 4, 8, 16, 64, 128, 256, 512};
uint8_t APB1_PreScaler[4] = {2, 4, 8, 16};

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddress, uint8_t WriteorRead);
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle);

static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle);


static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_START);
}

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
		//pI2cBaseAddress->CR1 |= I2C_CR1_PE_Bit_Mask;
	}else
	{
		pI2Cx->CR1 &= ~(1 << 0);
	}
}
static void I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr, uint8_t WriteorRead)
{
	SlaveAddr = SlaveAddr << 1;
	if(WriteorRead == I2C_MASTER_WRITE)
	{
		SlaveAddr &= ~(1); //Slave address + r/nw bit=0
	}
	else
	{
		SlaveAddr |= 1; //Slave address + r/nw bit=1
	}

	pI2Cx->DR = SlaveAddr;
}

static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle)
{
	uint32_t dummyRead;
	if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
	{
		//Device in master mode
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			//In reception
			if(pI2CHandle->RxSize == 1)
			{
				//Disable ACKing(send NACK after first byte)
				I2C_Ack_Control(pI2CHandle->pI2Cx, DISABLE);

				dummyRead = pI2CHandle->pI2Cx->SR1;
				dummyRead = pI2CHandle->pI2Cx->SR2;
				(void)dummyRead;
			}
		}
		else
		{
			//Clear ADDR flag
			dummyRead = pI2CHandle->pI2Cx->SR1;
			dummyRead = pI2CHandle->pI2Cx->SR2;
			(void)dummyRead;
		}
	}
	else
	{
		dummyRead = pI2CHandle->pI2Cx->SR1;
		dummyRead = pI2CHandle->pI2Cx->SR2;
		(void)dummyRead;
	}

}

void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}

void I2C_SlaveControlCallBackEvent(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);
		pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);
		pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);
	}
	else
	{
		pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN);
		pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN);
		pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITERREN);
	}
}

void I2C_Ack_Control(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == I2C_ACK_EN)
	{
		pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
	}
	else
	{
		pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
	}
}

void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLOCK_EN();
		}
		else if(pI2Cx == I2C2)
		{
			I2C2_PCLOCK_EN();
		}
		else if(pI2Cx == I2C3)
		{
			I2C3_PCLOCK_EN();
		}
	}
	else
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLOCK_DI();
		}
		else if(pI2Cx == I2C2)
		{
			I2C2_PCLOCK_DI();
		}
		else if(pI2Cx == I2C3)
		{
			I2C3_PCLOCK_DI();
		}
	}
}

void RCC_GetPLLOutputClk(void)
{
	return;
}

uint32_t RCC_GetPCLKValue(void)
{
	uint32_t pclk1, sysclk;

	uint8_t clksrc, temp, ahbp, apb1p;
	clksrc = ((RCC->CFGR >> 2) & 0x3);

	if(clksrc == 0)
	{
		sysclk = 16000000;
	}
	else if(clksrc == 1)
	{
		sysclk = 8000000;
	}
	else if(clksrc == 2)
	{
		RCC_GetPLLOutputClk();
	}

	//for ahbp
	temp = ((RCC->CFGR >> 4) & 0xF);

	if(temp < 8)
	{
		ahbp = 1;
	}
	else
	{
		ahbp = AHB_PreScaler[temp-8];
	}

	//for apbp
	temp = ((RCC->CFGR >> 10) & 0x7);
	if(temp < 4)
	{
		apb1p = 1;
	}
	else
	{
		apb1p = APB1_PreScaler[temp-4];
	}

	pclk1 = (sysclk / ahbp) / apb1p;

	return pclk1;
}

void I2C_Init(I2C_Handle_t *pI2CHandle)
{


	uint32_t tempreg = 0;

	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);

	//ack
	tempreg |= pI2CHandle->I2C_Config.I2C_ACKControl << I2C_CR1_ACK;
	pI2CHandle->pI2Cx->CR1 = tempreg;

	//configure the FREQ of CR2
	tempreg = 0;
	tempreg |= (RCC_GetPCLKValue() / 1000000U);
	pI2CHandle->pI2Cx->CR2 = (tempreg & 0x3F);

	//set address in slave mode
	tempreg = 0;
	tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
	tempreg |= (1 << 14);
	pI2CHandle->pI2Cx->OAR1 = tempreg;

	//CCR calculations
	uint16_t ccr_val = 0;
	tempreg = 0;

	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		//standard mode
		ccr_val = (RCC_GetPCLKValue() / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed) );
		tempreg |= (ccr_val & 0xFFF);
	}
	else
	{
		//fast mode
		tempreg |= (1 << I2C_CCR_FS);
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << I2C_CCR_DUTY);
		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			ccr_val = (RCC_GetPCLKValue() / (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}
		else
		{
			ccr_val = (RCC_GetPCLKValue() / (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}
		tempreg |= (ccr_val & 0xFFF);
	}
	pI2CHandle->pI2Cx->CCR = tempreg;

	//TRISE configuration
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		//mode is standard

		tempreg = (RCC_GetPCLKValue() / 1000000U) + 1;
	}
	else
	{
		//mode is fast
		tempreg = ((RCC_GetPCLKValue() * 300) / 1000000000U) + 1;
	}
	pI2CHandle->pI2Cx->TRISE = (tempreg & 0x3F);
}

void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{
	if(pI2Cx == I2C1)
	{
		I2C1_PCLOCK_DI();
	}
	else if(pI2Cx == I2C2)
	{
		I2C2_PCLOCK_DI();
	}
	else if(pI2Cx == I2C3)
	{
		I2C3_PCLOCK_DI();
	}
}

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName)
{
	if(pI2Cx->SR1 & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
	//Generate START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//Confirm start generated by reading SB flag in SR1
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

	//Send the address of the slave with r/nw bit set to w(0) (total 8 bits)
	I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, SlaveAddr, I2C_MASTER_WRITE);

	//Confirm address phase is completed by checking the ADDR flag in the SR1
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

	I2C_ClearADDRFlag(pI2CHandle);

	//send data until Len becomes 0

	while(Len > 0)
	{
		while( !I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TxE));
		pI2CHandle->pI2Cx->DR = *pTxBuffer;
		pTxBuffer++;
		Len--;
	}

	//when Len == 0 wait for TXE=1 and BTF=1 before generating the STOP condition
	while( !I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TxE));
	while( !I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF));

	//Generate STOP condition
	if(Sr == I2C_DISABLE_SR)
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
}

void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
	//Generate start condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//Confirm start condition generation by checking the SB flag in the SR1
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

	//Send the address of the slave with r/nw bit set to R(1)
	I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, SlaveAddr, I2C_MASTER_READ);

	//Wait until address phase is completed by checking ADDR flag in SR1

	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

	//read only 1 byte from the slave
	if(Len == 1)
	{
		//Disable Acking
		I2C_Ack_Control(pI2CHandle->pI2Cx, I2C_ACK_DI);

		//Clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		//wait until RXNE is 1
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RxNE));

		//generate STOP condition
		if(Sr == I2C_DISABLE_SR)
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

		//read data into buffer
		*pRxBuffer = pI2CHandle->pI2Cx->DR;
	}

	//read from slave when len > 1
	if(Len > 1)
	{
		//Clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		//read data until Len is 0
		for( uint32_t i = Len; i > 0; i--)
		{
			//wait until RXNE is 1
			while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RxNE));

			if(i == 2) //if last 2 bytes are remaining
			{
				//Disable Acking
				I2C_Ack_Control(pI2CHandle->pI2Cx, I2C_ACK_DI);
				//generate STOP condition
				if(Sr == I2C_DISABLE_SR)
					I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
			}

			//read data from data register into the buffer
			*pRxBuffer = pI2CHandle->pI2Cx->DR;
			//increment the buffer address
			pRxBuffer++;
		}
	}
	//re-enable ACKING
	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_EN)
	{
		I2C_Ack_Control(pI2CHandle->pI2Cx, I2C_ACK_EN);
	}

}

uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
	uint8_t busystate = pI2CHandle->TxRxState;

	if((busystate != I2C_BUSY_IN_RX) && (busystate != I2C_BUSY_IN_TX))
	{
		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->TxLen = Len;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;

		//Generate START condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//Enable Buff interrupt
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Enable Event interrupt
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);

		//Enable Error interrupt
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
	}

	return busystate;
}

uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
	uint8_t busystate = pI2CHandle->TxRxState;

	if((busystate != I2C_BUSY_IN_RX) && (busystate != I2C_BUSY_IN_TX))
	{
		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->RxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
		pI2CHandle->RxSize = Len;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Generate START condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//Enable Buff interrupt
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Enable Event interrupt
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);

		//Enable Error interrupt
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
	}

	return busystate;
}



void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//Program ISER0
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber <= 63)
		{
			//Program ISER1
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}
		else if(IRQNumber > 63 && IRQNumber <= 95)
		{
			//Program ISER2
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		}
	}
	else
	{
		if(IRQNumber <= 31)
		{
			//Program ICER0
			*NVIC_ICER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber <= 63)
		{
			//Program ICER1
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		}
		else if(IRQNumber > 63 && IRQNumber <= 95)
		{
			//Program ICER2
			*NVIC_ICER2 |= (1 << (IRQNumber % 64));
		}
	}
}
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4;
	uint8_t section = IRQNumber % 4;
	uint8_t shift_amount = ((8 * section) + (8 - NO_PR_BITS_IMPLEMENTED));
	*((NVIC_PR_BASEADDR) + iprx) |= (IRQPriority << shift_amount);
}

void I2C_SlaveSendData(I2C_RegDef_t *pI2C, uint8_t data)
{
	pI2C->DR = data;
}

uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2C)
{
	return (uint8_t)pI2C->DR;
}

void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	//Interrupt handling for both master and slave mode of device

	uint32_t temp1, temp2, temp3;

	temp1 = (pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITEVTEN));
	temp2 = (pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITBUFEN));

	temp3 = (pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_SB));
	//Handle by interrupt generated by SB event
	if(temp1 && temp3)
	{
		//Interrupt generated by SB event, now execute the address phase
		//WILL NOT BE EXECUTED IN SLAVE MODE BECAUSE SB FOR SLAVE IS ALWAYS ZERO
		uint8_t writeOrRead = 0;
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			writeOrRead = I2C_MASTER_READ;
		}
		I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, pI2CHandle->DevAddr, writeOrRead);
	}

	temp3 = (pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_ADDR));
	//Handle by interrupt generated by ADDR event
	if(temp1 && temp3)
	{
		//ADDR flag is set
		I2C_ClearADDRFlag(pI2CHandle);
	}

	temp3 = (pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_BTF));
	//Handle by interrupt generated by BTF event
	if(temp1 && temp3)
	{
		//BTF flag is set
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			//confirm TXE is also set
			if(pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TxE))
			{
				if(pI2CHandle->TxLen == 0)
				{
					//Close the communication
					//generate stop condition and reset all handler values after notify an application the transmission is over
					if(pI2CHandle->Sr == I2C_DISABLE_SR)
						I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

					I2C_CloseSendData(pI2CHandle);

					I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);
				}
			}
		}
		else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			;
		}
	}

	temp3 = (pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_STOPF));
	//Handle by interrupt generated by STOPF event
	if(temp1 && temp3)
	{
		//STOPF flag is set
		//WILL NOT BE EXECUTED IN MASTER MODE
		//Clear the STOPF flag (read SR1 after write to CR1)
		pI2CHandle->pI2Cx->CR1 |= 0x0000;
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);
	}

	temp3 = (pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TxE));

	//Handle by interrupt generated by TxE event
	if(temp1 && temp2 && temp3)
	{
		//Check device mode
		if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
		{
			//TxE flag is set
			//Do data transmission
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
			{
				I2C_MasterHandleTXEInterrupt(pI2CHandle);
			}
		}
		else
		{
			//For Slave Request for data
			if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA))
			{
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_REQ);
			}
		}
	}

	temp3 = (pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_RxNE));
	//Handle by interrupt generated by RxNE event
	if(temp1 && temp2 && temp3)
	{
		//RxNE flag is set
		if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
		{
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
			{
				I2C_MasterHandleRXNEInterrupt(pI2CHandle);
			}
		}
		else
		{
			//Slave Send data
			//check if the salve in receiver mode
			if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA))
			{
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_RCV);
			}
		}
	}
}

void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle)
{
	//Clear all interrupt bits
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;
	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;

	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_EN)
		I2C_Ack_Control(pI2CHandle->pI2Cx, ENABLE);
}

void I2C_CloseSendData(I2C_Handle_t *pI2CHandle)
{
	//Clear all interrupt bits
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->TxLen = 0;

}

static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle)
{
	if(pI2CHandle->RxSize == 1)
	{
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;

		pI2CHandle->RxLen--;
	}
	if(pI2CHandle->RxSize > 1)
	{
		if(pI2CHandle->RxLen == 2)
		{
			I2C_Ack_Control(pI2CHandle->pI2Cx, DISABLE);
		}

		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
		pI2CHandle->pRxBuffer++;
		pI2CHandle->RxLen--;
	}

	if(pI2CHandle->RxLen == 0)
	{
		//Close communication and notify app
		if(pI2CHandle->Sr == I2C_DISABLE_SR)
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

		I2C_CloseReceiveData(pI2CHandle);

		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);
	}
}
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle)
{

	if(pI2CHandle->TxLen > 0)
	{
		//load data in the DR
		pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);

		//decrement the TxLen
		pI2CHandle->TxLen--;

		//increment the buffer address
		pI2CHandle->pTxBuffer++;
	}

}

void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	uint32_t temp1, temp2;

	//Check if error is enabled
	temp2 = (pI2CHandle->pI2Cx->CR2) & (1 << I2C_CR2_ITERREN);

	//Check for bus error
	temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_BERR);
	if(temp1 && temp2)
	{
		//Bus error

		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_BERR);

		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_BERR);
	}

	//Check for arbitration error
	temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_ARLO);

	if(temp1 && temp2)
	{
		//Arbitration lost error
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_ARLO);

		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_ARLO);
	}

	//Check for ACK failure error
	temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_AF);

	if(temp1 && temp2)
	{
		//Arbitration lost error
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_AF);

		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_AF);
	}

	//Check for over/underrun error
	temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_OVR);

	if(temp1 && temp2)
	{
		//Over/underrun error
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_OVR);

		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_OVR);
	}

	//Check for time out error
	temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_TIMEOUT);

	if(temp1 && temp2)
	{
		//Time out error
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_TIMEOUT);

		I2C_ApplicationEventCallback(pI2CHandle, I2C_SR1_TIMEOUT);
	}
}







