/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: Jul 28, 2025
 *      Author: Sashko
 */

#include "stm32f407xx_spi_driver.h"

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLOCK_EN();
		}
		else if(pSPIx == SPI2)
		{
			SPI2_PCLOCK_EN();
		}
		else if(pSPIx == SPI3)
		{
			SPI3_PCLOCK_EN();
		}
	}
	else
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLOCK_DI();
		}
		else if(pSPIx == SPI2)
		{
			SPI2_PCLOCK_DI();
		}
		else if(pSPIx == SPI3)
		{
			SPI3_PCLOCK_DI();
		}
	}

}

//Init De-init
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	//enable RCC
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);
	uint32_t tempreg = 0;

	//Configure device mode
	tempreg |= (pSPIHandle->SPI_Config.SPI_DeviceMode << SPI_CR1_MSTR);

	//Configure device mode bus config
	if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		//BIDI should be cleared
		tempreg &= ~(0x1 << SPI_CR1_BIDIMODE);
	}
	else if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		//BIDI should be set
		tempreg |= (1 << SPI_CR1_BIDIMODE);
	}
	else if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		//BIDI clear
		tempreg &= ~(0x1 << SPI_CR1_BIDIMODE);
		//RXONLY set
		tempreg |= (0x1 << SPI_CR1_RXONLY);
	}

	tempreg |= (pSPIHandle->SPI_Config.SPI_Speed << SPI_CR1_BR);

	tempreg |= (pSPIHandle->SPI_Config.SPI_DFF << SPI_CR1_DFF);

	tempreg |= (pSPIHandle->SPI_Config.SPI_CPHA << SPI_CR1_CPHA);

	tempreg |= (pSPIHandle->SPI_Config.SPI_CPOL << SPI_CR1_CPOL);

	tempreg |= (pSPIHandle->SPI_Config.SPI_SSM << SPI_CR1_SSM);

	pSPIHandle->pSPIx->CR1 = tempreg;
}

void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if(pSPIx == SPI1)
	{
		SPI1_PCLOCK_DI();
	}
	if(pSPIx == SPI2)
	{
		SPI2_PCLOCK_DI();
	}
	if(pSPIx == SPI3)
	{
		SPI3_PCLOCK_DI();
	}
}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}
/*
 * Data send and receive
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t* pTxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		while( SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET );

		if(pSPIx->CR1 & (1 << SPI_CR1_DFF))
		{
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			Len -= 2;
			(uint16_t*)pTxBuffer++;

		}
		else
		{
			pSPIx->DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}
	}
}
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t* pRxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == (uint8_t)FLAG_RESET);

		if((pSPIx->CR1 & (1 << SPI_CR1_DFF)))
		{
			*((uint16_t*)pRxBuffer) = (uint16_t)pSPIx->DR;
			Len -= 2;
			(uint16_t*)pRxBuffer++;
		}
		else
		{
			*pRxBuffer = (uint8_t)pSPIx->DR;
			Len--;
			pRxBuffer++;
		}

	}
}

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t* pTxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->TxState;
	if(state != SPI_BUSY_IN_TX)
	{
		//Save the Tx buffer address and Len information in global variables
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;
		//Set SPI state as busy
		pSPIHandle->TxState = SPI_BUSY_IN_TX;
		//Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);
	}
	return state;
}
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t* pRxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->RxState;
	if(state != SPI_BUSY_IN_RX)
	{
		//Save the Tx buffer address and Len information in global variables
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;
		//Set SPI state as busy
		pSPIHandle->RxState = SPI_BUSY_IN_RX;
		//Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);
	}
	return state;
}

/*
 * IRQ Configuration and ISR handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4;
	uint8_t section = IRQNumber % 4;
	uint8_t shift_amount = ((8 * section) + (8 - NO_PR_BITS_IMPLEMENTED));
	*((NVIC_PR_BASEADDR) + iprx) |= (IRQPriority << shift_amount);
}

void SPI_IRQHandling(SPI_Handle_t *pHandle)
{
	uint8_t temp1, temp2;
	//check for TXE
	temp1 = (pHandle->pSPIx->SR) & (1 << SPI_SR_TXE);
	temp2 = (pHandle->pSPIx->CR2) & (1 << SPI_CR2_TXEIE);
	if(temp1 && temp2)
	{
		spi_txe_interrupt_handle(pHandle);
	}

	//check for RXNE
	temp1 = (pHandle->pSPIx->SR) & (1 << SPI_SR_RXNE);
	temp2 = (pHandle->pSPIx->CR2) & (1 << SPI_CR2_RXNEIE);
	if(temp1 && temp2)
	{
		spi_rxne_interrupt_handle(pHandle);
	}

	//check for overrun flag
	temp1 = (pHandle->pSPIx->SR) & (1 << SPI_SR_OVR);
	temp2 = (pHandle->pSPIx->CR2) & (1 << SPI_CR2_ERRIE);
	if(temp1 && temp2)
	{
		spi_ovr_err_interrupt_handle(pHandle);
	}

}

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}
	else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	}
	else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}

void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	}
	else
	{
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}

/*
 * Helper functions
 */
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	//check the DFF(data frame format) size
	if(pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF))
	{
		pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen -= 2;
		(uint16_t*)pSPIHandle->pTxBuffer++;
	}
	else
	{
		pSPIHandle->pSPIx->DR = *(pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer++;
	}

	if(pSPIHandle->TxLen == 0)
	{
		//TxLen zero, closing spi transmission  and inform the app that TX is over

		//prevents interrupts from setting up of TXE flag
		SPI_CloseTransmission(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
	}

}
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	//Check data format
	if((pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF)))
	{
		//16 bit
		//read form Data register to RX buffer
		*((uint16_t*)pSPIHandle->pRxBuffer) = (uint16_t)pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen -= 2;
		(uint16_t*)pSPIHandle->pRxBuffer++;
	}
	else
	{
		//8 bit
		//read form Data register to RX buffer
		*(pSPIHandle->pRxBuffer) = (uint8_t)pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->pRxBuffer++;
	}

	//Close the communication
	if(pSPIHandle->RxLen == 0)
	{
		//Set bit as 0 so we don't get an exception next time DR is not zero
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
	}
}
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp;
	//Clear the overrun flag (Read value from DR)
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	(void)temp;
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_CMPLT);
}

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;
}
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
}
void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;
}

__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv)
{
	//Weak implementation, an application should override this function in order to use
}

