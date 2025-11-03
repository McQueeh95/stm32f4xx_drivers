/*
 * stm32f407xx_gpio.c
 *
 *  Created on: Jul 23, 2025
 *      Author: Sashko
 */

#include "stm32f407xx_gpio.h"


//Peripheral clock setup

/******************************************
 * @fn					- GPIO_PeriClockControl
 *
 * @brief				- Enable or disables peripheral clock for the give GPIO port
 *
 * @param[in]			- base address of the GPIO peripheral
 * @param[in]			- ENABLE or DISABLE macros
 *
 * @return				- none
 *
 * @Note				- none
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLOCK_EN();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLOCK_EN();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLOCK_EN();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLOCK_EN();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLOCK_EN();
		}
		else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLOCK_EN();
		}
		else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLOCK_EN();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLOCK_EN();
		}
		else if(pGPIOx == GPIOI)
		{
			GPIOI_PCLOCK_EN();
		}
	}
	else
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLOCK_DI();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLOCK_DI();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLOCK_DI();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLOCK_DI();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLOCK_DI();
		}
		else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLOCK_DI();
		}
		else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLOCK_DI();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLOCK_DI();
		}
		else if(pGPIOx == GPIOI)
		{
			GPIOI_PCLOCK_DI();
		}
	}
}

/******************************************
 * @fn					- GPIO_Init
 *
 * @brief				- Initialize Register for given GPIO
 *
 * @param[in]			- base address of the GPIO peripheral
 * @param[in]			-
 * @param[in]			-
 *
 * @return				- none
 *
 * @Note				- none
 */
//Init De-init
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	//enable RCC
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);
	uint32_t temp = 0;
	//configure the mode of GPIO pin

	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
		pGPIOHandle->pGPIOx->MODER |= temp; //setting
	}
	else
	{
		//Interrupt mode
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			//configure the FTSR(Falling Trigger Selection Register)
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			//configure the RTSR(Rising Trigger Selection Register)
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			//configure the FTSR and RTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		//configure the GPIO port selection in SYSCFG_EXTICR(external interrupts)
		uint8_t temp1 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4);
		uint8_t temp2 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4);
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);;
		SYSCFG_PCLOCK_EN();
		SYSCFG->EXTICR[temp1] = portcode << (temp2 * 4);
		//enable exit interrupt delivery using IMR
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}
	temp = 0;
	//configure the speed of GPIO pin
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
	pGPIOHandle->pGPIOx->OSPEEDR |= temp; //setting

	temp = 0;

	//configure pupd settings
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
	pGPIOHandle->pGPIOx->PUPDR |= temp; //setting

	temp = 0;

	//configure the output type
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
	pGPIOHandle->pGPIOx->OTYPER |= temp; //setting

	temp = 0;

	//configure alt functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		uint8_t temp1, temp2;
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2));
		pGPIOHandle->pGPIOx->AFR[temp1] |= temp;
	}
}
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
			{
				GPIOA_REG_RESET();
			}
			else if(pGPIOx == GPIOB)
			{
				GPIOB_REG_RESET();
			}
			else if(pGPIOx == GPIOC)
			{
				GPIOC_REG_RESET();
			}
			else if(pGPIOx == GPIOD)
			{
				GPIOD_REG_RESET();
			}
			else if(pGPIOx == GPIOE)
			{
				GPIOE_REG_RESET();
			}
			else if(pGPIOx == GPIOF)
			{
				GPIOF_REG_RESET();
			}
			else if(pGPIOx == GPIOG)
			{
				GPIOG_REG_RESET();
			}
			else if(pGPIOx == GPIOH)
			{
				GPIOH_REG_RESET();
			}
			else if(pGPIOx == GPIOI)
			{
				GPIOI_REG_RESET();
			}
}

//Data read/write
/******************************************
 * @fn					- GPIO_ReadFromInputPin
 *
 * @brief				- Reads value from selected pin position
 *
 * @param[in]			- base address of the GPIO peripheral
 * @param[in]			- Number of pin to get value from
 * @param[in]			-
 *
 * @return				- 0 or 1
 *
 * @Note				- none
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value = 0;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & (0x1));
	return value;
}

//Data read/write
/******************************************
 * @fn					- GPIO_ReadFromInputPort
 *
 * @brief				- Reads value from selected port
 *
 * @param[in]			- base address of the GPIO peripheral
 * @param[in]			-
 * @param[in]			-
 *
 * @return				- IDR register value for given GPIO
 *
 * @Note				- none
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value = 0;
	value = (uint16_t)pGPIOx->IDR;
	return value;
}

/******************************************
 * @fn					- GPIO_WriteToOutputPin
 *
 * @brief				- Writes value in the given pin number
 *
 * @param[in]			- base address of the GPIO peripheral
 * @param[in]			- Number of pin to write value to
 * @param[in]			- Value to write into given pin (0 or 1)
 *
 * @return				- none
 *
 * @Note				- none
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if(Value == GPIO_PIN_SET)
	{
		pGPIOx->ODR |= (1 << PinNumber);
	}
	else
	{
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}
/******************************************
 * @fn					- GPIO_WriteToOutputPort
 *
 * @brief				- Writes value into given port
 *
 * @param[in]			- base address of the GPIO peripheral
 * @param[in]			- Value to write into given port
 * @param[in]			-
 *
 * @return				- none
 *
 * @Note				- none
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR = Value;
}

/******************************************
 * @fn					- GPIO_ToggleOutputPin
 *
 * @brief				- Changes value of given pin to opposite (0 to 1, 1 to 0)
 *
 * @param[in]			- base address of the GPIO peripheral
 * @param[in]			- number of pin to toggle
 * @param[in]			-
 *
 * @return				- none
 *
 * @Note				- none
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}

//IRQ configuration and ISR handling
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4;
	uint8_t section = IRQNumber % 4;
	uint8_t shift_amount = ((8 * section) + (8 - NO_PR_BITS_IMPLEMENTED));
	*((NVIC_PR_BASEADDR) + iprx) |= (IRQPriority << shift_amount);
}

void GPIO_IRQHandling(uint8_t PinNumber)
{
	//clear the EXTI PR register according to the pin number
	if(EXTI->PR & (1 << PinNumber))
	{
		EXTI->PR |= (1 << PinNumber);
	}
}







