/*
 * 007spi_tx_testing.c
 *
 *  Created on: Jul 29, 2025
 *      Author: Sashko
 */

#include "stm32f407xx.h"
#include <string.h>

//command codes
#define COMMAND_LED_CTRL      		0x50
#define COMMAND_SENSOR_READ      	0x51
#define COMMAND_LED_READ      		0x52
#define COMMAND_PRINT      			0x53
#define COMMAND_ID_READ      		0x54

#define LED_ON     1
#define LED_OFF    0

//arduino analog pins
#define ANALOG_PIN0 	0
#define ANALOG_PIN1 	1
#define ANALOG_PIN2 	2
#define ANALOG_PIN3 	3
#define ANALOG_PIN4 	4

//arduino led

#define LED_PIN  9

/*
 * PB14 -> SPI2_MISO +
 * PB15 -> SPI2_MOSI +
 * PB13 -> SPI2_SCLK +
 * PB12 -> SPI2_NSS
 * ALT function mode 5
 */
void SPI2_GPIOInits(void)
{
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OP_SPEED_FAST;

	//SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);

	//MISO
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&SPIPins);

	//NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);
}

void SPI2_Init(void)
{
	SPI_Handle_t SPI2Handle;
	SPI2Handle.pSPIx = SPI2;
	SPI2Handle.SPI_Config.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2Handle.SPI_Config.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPI_Config.SPI_Speed = SPI_SCLK_SPEED_DIV32;
	SPI2Handle.SPI_Config.SPI_DFF = SPI_DFF_8BITS;
	SPI2Handle.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Handle.SPI_Config.SPI_CPHA = SPI_CPHA_LOW;
	SPI2Handle.SPI_Config.SPI_SSM = SPI_SSM_DI;

	SPI_Init(&SPI2Handle);
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

void delay(void)
{
	for(uint32_t i = 0; i < 500000/2; i++);
}

uint8_t SPI_VerifyResponse(uint8_t ackbyte)
{

	if(ackbyte == (uint8_t)0xF5)
	{
		return 1;
	}
	return 0;
}

int main(void)
{
	uint8_t dummy_write = 0xFF;

	uint8_t dummy_read;

	//char user_data[] = "Hello World";

	GPIO_ButtonInit();

	//initializes the GPIO pins to behave as SPI2 pins
	SPI2_GPIOInits();

	//initializes the SPI2 peripheral parameters
	SPI2_Init();

	//NSS signal internally high and avoids MODF error
	//SPI_SSIConfig(SPI2, ENABLE);
	SPI_SSOEConfig(SPI2, ENABLE);
	while(1){

		//wait until the button is pressed
		while(! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

		//avoids de-bouncing of the button
		delay();

		//enable the SPI2 peripheral
		SPI_PeripheralControl(SPI2, ENABLE);

		uint8_t commandcode = COMMAND_LED_CTRL;
		uint8_t ackbyte;
		uint8_t args[2];

		SPI_SendData(SPI2, &commandcode, 1);

		//Dummy read to clear off the RXNE register
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		//Send CMD_LED_CTRL command to the slave <pin no> <value>

		//Send some dummy bytes to get an acknowledge byte from the slave's data register
		SPI_SendData(SPI2, &dummy_write, 1);

		//Get an acknowledge byte
		SPI_ReceiveData(SPI2, &ackbyte, 1);

		//Checks whether akbyte byte is 0xF5(can proceed)
		if(SPI_VerifyResponse(ackbyte))
		{
			//send arguments
			args[0] = LED_PIN;
			args[1] = LED_ON;
			SPI_SendData(SPI2, args, 2);
			SPI_ReceiveData(SPI2, args, 2);
		}

		//uint8_t dataLen = strlen(user_data);
		//SPI_SendData(SPI2, &dataLen, 1);

		//SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));

		//CMD SENSOR_READ <analog pin number>
		while(! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

		delay();

		commandcode = COMMAND_SENSOR_READ;

		//Send command
		SPI_SendData(SPI2, &commandcode, 1);

		//Received byte because of send need to clear off RXNE register
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		//Send dummy byte to get acknowledge byte from slave
		SPI_SendData(SPI2, &dummy_write, 1);

		//Read an acknowledge byte
		SPI_ReceiveData(SPI2, &ackbyte, 1);

		//Checks whether akbyte byte is 0xF5(can proceed)
		if(SPI_VerifyResponse(ackbyte))
		{
			args[0] = ANALOG_PIN0;
			//Send one byte of argument
			SPI_SendData(SPI2, args, 1);
			//Because we sent now we need to receive byte to clear RXNE
			SPI_ReceiveData(SPI2, &dummy_read, 1);

			//Insert some delay because slave need to convert analog to digital and send data back
			delay();

			//This command return the value of the sensor so we need to read it, for that send some dummy byte
			SPI_SendData(SPI2, &dummy_write, 1);

			uint8_t analog_read;

			//Get a value of the sensor pin
			SPI_ReceiveData(SPI2, &analog_read, 1);
		}

		//CMD LED_READ <analog pin number>
		while(! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

		delay();

		commandcode = COMMAND_LED_READ;

		//Send command
		SPI_SendData(SPI2, &commandcode, 1);

		//Received byte because of send need to clear off RXNE register
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		//Send dummy byte to get acknowledge byte from slave
		SPI_SendData(SPI2, &dummy_write, 1);

		//Read an acknowledge byte
		SPI_ReceiveData(SPI2, &ackbyte, 1);

		//Checks whether akbyte byte is 0xF5(can proceed)
		if(SPI_VerifyResponse(ackbyte))
		{
			args[0] = LED_PIN;
			//Send one byte of argument
			SPI_SendData(SPI2, args, 1);
			//Because we sent now we need to receive byte to clear RXNE
			SPI_ReceiveData(SPI2, &dummy_read, 1);

			//This command return the value of the led pin of the slave
			SPI_SendData(SPI2, &dummy_write, 1);

			uint8_t led_status;

			//Get current value of the pin
			SPI_ReceiveData(SPI2, &led_status, 1);
		}

		//CMD PRINT <len(of message)> <message>
		while(! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

		delay();

		commandcode = COMMAND_PRINT;

		//Sends code of the command
		SPI_SendData(SPI2, &commandcode, 1);

		//Need to read dummy_byte to clear off RXNE
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		//To receive ack-byte we need to send dummy-byte
		SPI_SendData(SPI2, &dummy_write, 1);

		//Gets an ack-byte
		SPI_ReceiveData(SPI2, &ackbyte, 1);

		if(SPI_VerifyResponse(ackbyte))
		{
			uint8_t message_to_send[] = "Sending message to the slave!";
			args[0] = strlen((char*)message_to_send);
			//Send len of the message
			SPI_SendData(SPI2, args, 1);
			//Clear RXNE
			SPI_ReceiveData(SPI2, &dummy_read, 1);

			//Send message to the slave
			for(int i = 0; i < args[0]; i++)
			{
				SPI_SendData(SPI2, &message_to_send[i], 1);
				SPI_ReceiveData(SPI2, &dummy_read, 1);
			}
		}

		//CMD ID_READ
		while(! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

		delay();

		commandcode = COMMAND_ID_READ;

		//Send command
		SPI_SendData(SPI2, &commandcode, 1);

		//Received byte because of send need to clear off RXNE register
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		//Send dummy byte to get acknowledge byte from slave
		SPI_SendData(SPI2, &dummy_write, 1);

		//Read an acknowledge byte
		SPI_ReceiveData(SPI2, &ackbyte, 1);

		uint8_t id[11];
		uint32_t i=0;
		//Checks whether akbyte byte is 0xF5(can proceed)
		if(SPI_VerifyResponse(ackbyte))
		{
			//read 10 bytes id from the slave
			for(i = 0; i < 10; i++)
			{
				SPI_SendData(SPI2, &dummy_write, 1);
				SPI_ReceiveData(SPI2, &id[i], 1);;
			}
			id[10] = '\0';
		}


		//Confirms SPI is not busy
		while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG) );

		//Close SPI disable peripheral
		SPI_PeripheralControl(SPI2, DISABLE);
	}




	return 0;
}
