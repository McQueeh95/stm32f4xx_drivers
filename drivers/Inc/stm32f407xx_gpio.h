/*
 * stm32f407xx_gpio.h
 *
 *  Created on: Jul 23, 2025
 *      Author: Sashko
 */

#ifndef INC_STM32F407XX_GPIO_H_
#define INC_STM32F407XX_GPIO_H_

#include "stm32f407xx.h"

/*
 * Structure for configuration of the GPIO. When initialize GPIO pass this structure with values
 */
typedef struct
{
	uint8_t GPIO_PinNumber;			//possible values from @GPIO_PIN_NUMBERS
	uint8_t GPIO_PinMode;			//possible values from @GPIO_PIN_MODES
	uint8_t GPIO_PinSpeed;			//possible values from @GPIO_PIN_SPEED
	uint8_t	GPIO_PinPuPdControl;	//possible values from @GPIO_PIN_PUPD
	uint8_t GPIO_PinOPType;			//possible values from @GPIO_PIN_OP_TYPE
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;


/*
 * Handle structure for a GPIO pin. Call any attribute of initialize GPIO
 */

typedef struct
{
	GPIO_RegDef_t *pGPIOx; // Holds the base address of the GPIO port to which the pin belongs
	GPIO_PinConfig_t GPIO_PinConfig; //Holds GPIO pin configuration settings
}GPIO_Handle_t;

/*
 * @GPIO_PIN_NUMBERS
 * GPIO pin number
 */
#define GPIO_PIN_NO_0 		0
#define GPIO_PIN_NO_1 		1
#define GPIO_PIN_NO_2 		2
#define GPIO_PIN_NO_3 		3
#define GPIO_PIN_NO_4 		4
#define GPIO_PIN_NO_5 		5
#define GPIO_PIN_NO_6 		6
#define GPIO_PIN_NO_7 		7
#define GPIO_PIN_NO_8 		8
#define GPIO_PIN_NO_9 		9
#define GPIO_PIN_NO_10 		10
#define GPIO_PIN_NO_11 		11
#define GPIO_PIN_NO_12 		12
#define GPIO_PIN_NO_13 		13
#define GPIO_PIN_NO_14 		14
#define GPIO_PIN_NO_15 		15

/*
 * @GPIO_PIN_MODES
 * GPIO pin possible modes
 */
#define GPIO_MODE_IN 		0
#define GPIO_MODE_OUT 		1
#define GPIO_MODE_ALTFN 	2
#define GPIO_MODE_ANALOG 	3
#define GPIO_MODE_IT_FT		4
#define GPIO_MODE_IT_RT		5
#define GPIO_MODE_IT_RFT	6

/*
 * @GPIO_PIN_OP_TYPE
 * GPIO pin possible output types
 */
#define GPIO_OP_TYPE_PP		0 //Usually use this (pin actively outputs HIGH and LOW)
#define GPIO_OP_TYPE_OD		1 //Pin is never high (either LOW or floating) use in I2C

/*
 * @GPIO_PIN_SPEED
 * GPIO pin output speed
 */
#define GPIO_OP_SPEED_LOW		0
#define GPIO_OP_SPEED_MEDIUM	1
#define GPIO_OP_SPEED_FAST		2
#define GPIO_OP_SPEED_HIGH		3

/*
 * @GPIO_PIN_PUPD
 * GPIO pin pull up AND pull down configuration macros
 */
#define GPIO_PIN_NO_PUPD			0 //Set when GPIO driven from external resistor
#define GPIO_PIN_PU					1 //Set when we use internal resistor of GPIO and pin in HIGH state
#define GPIO_PIN_PD					2 //Set when we use internal resistor of GPIO and pin in LOW state



/************************************************
 * 			APIs supported by this driver
 ************************************************/

//Peripheral clock setup
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

//Init De-init
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

//Data read/write
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

//IRQ configuration and ISR handling
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);


#endif /* INC_STM32F407XX_GPIO_H_ */
