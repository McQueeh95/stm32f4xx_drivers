/*
 * 011i2c_master_rx_testing.c
 *
 *  Created on: Aug 17, 2025
 *      Author: Sashko
 */


// PB6 -> SCL
// PB9 -> SDA

#include "stm32f407xx.h"
#include <string.h>

I2C_Handle_t I2C1Handle;

#define SLAVE_ADDR 0x68
uint8_t read_buff[32];

void delay(void)
{
	for(uint32_t i = 0; i < 500000/2; i++);
}

void I2C1_GPIOInits(void)
{
	GPIO_Handle_t I2CPins;

	I2CPins.pGPIOx = GPIOB;
	I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
	I2CPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OP_SPEED_FAST;

	//SCL
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&I2CPins);

	//SDA
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_9;
	GPIO_Init(&I2CPins);
}

void I2C1_Inits(void)
{
	I2C1Handle.pI2Cx = I2C1;
	I2C1Handle.I2C_Config.I2C_ACKControl = I2C_ACK_EN;
	I2C1Handle.I2C_Config.I2C_DeviceAddress = 0x61;
	I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

	I2C_Init(&I2C1Handle);
}

void GPIO_ButtonInit(void){
	GPIO_Handle_t gpio_button;
	gpio_button.pGPIOx = GPIOA;
	gpio_button.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	gpio_button.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	gpio_button.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_NO_PUPD;

	GPIO_Init(&gpio_button);

	GPIO_PeriClockControl(GPIOA, ENABLE);
}

int main(void)
{
	uint8_t message_len, command;

	GPIO_ButtonInit();

	//I2C pins init
	I2C1_GPIOInits();

	//I2C peripheral configuration
	I2C1_Inits();

	//Enable peripheral
	I2C_PeripheralControl(I2C1, ENABLE);

	I2C_Ack_Control(I2C1, I2C_ACK_EN);
	while(1)
	{
		while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

		delay();

		command = 0x51;

		I2C_MasterSendData(&I2C1Handle, &command, 1, SLAVE_ADDR, I2C_ENABLE_SR);
		I2C_MasterReceiveData(&I2C1Handle, &message_len, 1, SLAVE_ADDR, I2C_ENABLE_SR);

		command = 0x52;
		I2C_MasterSendData(&I2C1Handle, &command, 1, SLAVE_ADDR, I2C_ENABLE_SR);
		I2C_MasterReceiveData(&I2C1Handle, read_buff, message_len, SLAVE_ADDR, I2C_DISABLE_SR);
	}

	return 0;
}
