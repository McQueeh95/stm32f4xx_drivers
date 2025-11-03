/*
 * 001led_toggle.c
 *
 *  Created on: Jul 24, 2025
 *      Author: Sashko
 */

#include "stm32f407xx.h"
#include "stm32f407xx_gpio.h"

void delay(void)
{
	for(uint32_t i = 0; i < 500000/2; i++);
}

int main(void)
{
	GPIO_Handle_t gpio_led;
	gpio_led.pGPIOx = GPIOD;
	gpio_led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	gpio_led.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpio_led.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	gpio_led.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	GPIO_PeriClockControl(gpio_led.pGPIOx, ENABLE);
	GPIO_Init(&gpio_led);

	while(1)
	{
		GPIO_ToggleOutputPin(gpio_led.pGPIOx, gpio_led.GPIO_PinConfig.GPIO_PinNumber);
		delay();
	}
	for(;;);
	return 0;
}
