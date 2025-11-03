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

#define SLAVE_ADDR 	0x68
#define MY_ADDR 	SLAVE_ADDR
uint8_t tx_buff[] = "HiHiHiHiH";
uint8_t commandCode = 0;
uint8_t Cnt = 0;

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
	I2C1Handle.I2C_Config.I2C_DeviceAddress = MY_ADDR;
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
	//uint8_t message_len, command;

	GPIO_ButtonInit();

	//I2C pins init
	I2C1_GPIOInits();

	//I2C peripheral configuration
	I2C1_Inits();

	//I2C IRQ Configuration
	I2C_IRQInterruptConfig(IRQ_NO_I2C1EV, ENABLE);
	I2C_IRQInterruptConfig(IRQ_NO_I2C1ER, ENABLE);

	I2C_SlaveControlCallBackEvent(I2C1, ENABLE);

	//Enable peripheral
	I2C_PeripheralControl(I2C1, ENABLE);

	I2C_Ack_Control(I2C1, I2C_ACK_EN);

	while(1);

	return 0;
}

void I2C1_EV_IRQHandler(void)
{
	I2C_EV_IRQHandling(&I2C1Handle);
}
void I2C1_ER_IRQHandler(void)
{
	I2C_ER_IRQHandling(&I2C1Handle);
}

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv)
{
	if(AppEv == I2C_EV_DATA_REQ)
	{
		//Master wants data
		if(commandCode == 0x51)
		{
			I2C_SlaveSendData(I2C1, (strlen((char*)tx_buff)));
		}
		else if(commandCode == 0x52)
		{
			//Send the content of tx
			I2C_SlaveSendData(I2C1, tx_buff[Cnt++]);
		}
	}
	else if(AppEv == I2C_EV_DATA_RCV)
	{
		commandCode = I2C_SlaveReceiveData(I2C1);
	}
	else if(AppEv == I2C_ERROR_AF)
	{
		//during slave txing
		//Master has sent NACK, master doesn't need any data
		commandCode = 0xff;
		Cnt = 0;
	}
	else if(AppEv == I2C_EV_STOP)
	{
		//Slave reception
		//Master has ended I2C communication
	}
}
