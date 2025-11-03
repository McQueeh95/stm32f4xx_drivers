/*
 * stm32f407xx.h
 *
 *  Created on: Jul 22, 2025
 *      Author: Sashko
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>
#include <stddef.h>

#define __vo volatile
#define __weak __attribute__((weak))
/************************Processor specific details************************/
/*
 * ARM Cortex Mx Processor NVIC ISERx register addresses
 */
#define NVIC_ISER0				((__vo uint32_t*)0xE000E100)
#define NVIC_ISER1				((__vo uint32_t*)0xE000E104)
#define NVIC_ISER2				((__vo uint32_t*)0xE000E108)
#define NVIC_ISER3				((__vo uint32_t*)0xE000E10C)

/*
 * ARM Cortex Mx Processor NVIC ICERx register addresses
 */
#define NVIC_ICER0				((__vo uint32_t*)0XE000E180)
#define NVIC_ICER1				((__vo uint32_t*)0XE000E184)
#define NVIC_ICER2				((__vo uint32_t*)0XE000E188)
#define NVIC_ICER3				((__vo uint32_t*)0XE000E18C)

/*
 * ARM Cortex Mx Processor Priority register addresses
 */
#define NVIC_PR_BASEADDR		((__vo uint32_t*)0xE000E400)

#define NO_PR_BITS_IMPLEMENTED	4

/*
 * Base addresses of FLASH and SRAM memories
 */
#define FLASH_BASEADDR			(0x08000000UL)					//FLASH base address
#define SRAM1_BASEADDR			(0x20000000UL)					//SRAM1 base address
#define SRAM2_BASEADDR			((SRAM1_BASEADDR) + (0x1C000))	//SRAM2 base address
#define ROM_BASEADDR			(0x1FFF0000L)					//ROM base address
#define SRAM_BASEADDR			(SRAM1_BASEADDR)				//SRAM base address


/*
 * AHBx and APBx Bus Peripheral base addresses
 */

#define PERIPH_BASEADDR			(0x40000000UL)
#define APB1_BASEADDR			(PERIPH_BASEADDR)
#define APB2_BASEADDR			(0x40010000UL)
#define AHB1_BASEADDR			(0x40020000UL)
#define AHB2_BASEADDR			(0x50000000UL)
#define AHB3_BASEADDR			(0xA0000000UL)

/*
 * Base addresses of peripherals which are hanging on AHB1 bus
 */

#define GPIOA_BASEADDR			(AHB1_BASEADDR)
#define GPIOB_BASEADDR			((AHB1_BASEADDR) + (0x0400))
#define GPIOC_BASEADDR			((AHB1_BASEADDR) + (0x0800))
#define GPIOD_BASEADDR			((AHB1_BASEADDR) + (0x0C00))
#define GPIOE_BASEADDR			((AHB1_BASEADDR) + (0x1000))
#define GPIOF_BASEADDR			((AHB1_BASEADDR) + (0x1400))
#define GPIOG_BASEADDR			((AHB1_BASEADDR) + (0x1800))
#define GPIOH_BASEADDR			((AHB1_BASEADDR) + (0x1C00))
#define GPIOI_BASEADDR			((AHB1_BASEADDR) + (0x2000))

#define RCC_BASEADDR			(0x40023800UL)

/*
 * Base addresses of peripherals which are hanging on APB1 bus
 */

#define I2C1_BASEADDR			((APB1_BASEADDR) + (0x5400))
#define I2C2_BASEADDR			((APB1_BASEADDR) + (0x5800))
#define I2C3_BASEADDR			((APB1_BASEADDR) + (0x5C00))

#define SPI2_BASEADDR			((APB1_BASEADDR) + (0x3800))
#define SPI3_BASEADDR			((APB1_BASEADDR) + (0x3C00))

#define USART2_BASEADDR			((APB1_BASEADDR) + (0x4400))
#define USART3_BASEADDR			((APB1_BASEADDR) + (0x4800))
#define UART4_BASEADDR			((APB1_BASEADDR) + (0x4C00))
#define UART5_BASEADDR			((APB1_BASEADDR) + (0x5000))

/*
 * Base addresses of peripherals which are hanging on APB2 bus
 */

#define EXTI_BASEADDR			((APB2_BASEADDR) + (0x3C00))
#define SPI1_BASEADDR			((APB2_BASEADDR) + (0x3000))
#define SYSCFG_BASEADDR			((APB2_BASEADDR) + (0x3800))
#define USART1_BASEADDR			((APB2_BASEADDR) + (0x1000))
#define USART6_BASEADDR			((APB2_BASEADDR) + (0x1400))

/****************************** peripheral registers definition structures ******************************/

typedef struct
{
	__vo uint32_t MODER; 	//mode register
	__vo uint32_t OTYPER;	//output type
	__vo uint32_t OSPEEDR;	//output speed
	__vo uint32_t PUPDR;	//pull-up/pull-down
	__vo uint32_t IDR;		//input data
	__vo uint32_t ODR;		//output data
	__vo uint32_t BSRR;		//port bit set/reset
	__vo uint32_t LCKR;		//configuration lock
	__vo uint32_t AFR[2];	//0->alternate function low(0-7), 1->alternate function high(8-15)
}GPIO_RegDef_t;

/*
 * peripheral definition structure for RCC
 */
typedef struct
{
	__vo uint32_t CR;			//clock control
	__vo uint32_t PLLCFGR;		//PLL configuration
	__vo uint32_t CFGR;			//clock configuration
	__vo uint32_t CIR;			//clock interrupt
	__vo uint32_t AHB1RSTR;		//AHB1 peripheral reset
	__vo uint32_t AHB2RSTR;		//AHB2 peripheral reset
	__vo uint32_t AHB3RSTR;		//AHB3 peripheral reset
	uint32_t RESERVED0;
	__vo uint32_t APB1RSTR;		//APB1 peripheral reset
	__vo uint32_t APB2RSTR;		//APB2 peripheral reset
	uint32_t RESERVED1[2];
	__vo uint32_t AHB1ENR;		//AHB1 peripheral clock
	__vo uint32_t AHB2ENR ;		//AHB2 peripheral clock enable
	__vo uint32_t AHB3ENR;		//AHB3 peripheral clock enable
	uint32_t RESERVED2;
	__vo uint32_t APB1ENR;		//APB1 peripheral clock enable
	__vo uint32_t APB2ENR;		//APB2 peripheral clock enable
	uint32_t RESERVED3[2];
	__vo uint32_t AHB1LPENR;		//AHB1 peripheral clock enable in low power mode
	__vo uint32_t AHB2LPENR;		//AHB2 peripheral clock enable in low power mode
	__vo uint32_t AHB3LPENR; 	//AHB3 peripheral clock enable in low power mode
	uint32_t RESERVED4;
	__vo uint32_t APB1LPENR;		//APB1 peripheral clock enable in low power mode
	__vo uint32_t APB2LPENR;		//APB2 peripheral clock enable in low power mode
	uint32_t RESERVED5[2];
	__vo uint32_t BDCR;			//Backup domain control
	__vo uint32_t CSR;			//clock control & status
	uint32_t RESERVED6[2];
	__vo uint32_t SSCGR;			//spread spectrum clock generation
	__vo uint32_t PLLI2SCFGR;	//PLLI2S configuration
	__vo uint32_t PLLSAICFGR;	//PLL configuration
	__vo uint32_t DCKCFGR;		// Dedicated Clock Configuration
}RCC_RegDef_t;

/*
 * peripheral definition structure for EXTI
 */
typedef struct
{
	__vo uint32_t IMR;			//Interrupt mask
	__vo uint32_t EMR;			//Event mask
	__vo uint32_t RTSR;			//Rising trigger selection
	__vo uint32_t FTSR;			//Falling trigger selection
	__vo uint32_t SWIER;		//Software interrupt event
	__vo uint32_t PR;			//Pending register
}EXTI_RegDef_t;

/*
 * peripheral definition structure for SYSCFG
 */
typedef struct
{
	__vo uint32_t MEMRMP;
	__vo uint32_t PMC;
	__vo uint32_t EXTICR[4];
	uint32_t RESEVERD[2];
	__vo uint32_t CMPCR;
}SYSCFG_RegDef_t;

/*
 * Peripheral definition structure for SPI
 */
typedef struct
{
	__vo uint32_t CR1;		//control 1
	__vo uint32_t CR2;		//control 2
	__vo uint32_t SR;		//status
	__vo uint32_t DR;		//data
	__vo uint32_t CRCPR;	//CRC polynomial
	__vo uint32_t RXCRCR;	//RX CRC
	__vo uint32_t TXCRCR;	//TX CRC
	__vo uint32_t I2CCFGR;	//I2C configuration
	__vo uint32_t I2CPR;	//I2C prescaler

}SPI_RegDef_t;

/*
 * Peripheral definition structure for I2C
 */
typedef struct
{
	__vo uint32_t CR1; 		//control 1
	__vo uint32_t CR2; 		//control 2
	__vo uint32_t OAR1; 	//own address 1
	__vo uint32_t OAR2;		//own address 2
	__vo uint32_t DR;		//data
	__vo uint32_t SR1;		//status 1
	__vo uint32_t SR2;		//status 2
	__vo uint32_t CCR;		//clock control
	__vo uint32_t TRISE;	//trise
	__vo uint32_t FLTR;		//fltr
}I2C_RegDef_t;

/*
 * Peripheral definition structure for USART
 */
typedef struct
{
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t BRR;
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t CR3;
	__vo uint32_t GTPR;
}USART_RegDef_t;


/*
 * Peripheral definitions
 */

#define GPIOA 					((GPIO_RegDef_t*)(GPIOA_BASEADDR))
#define GPIOB 					((GPIO_RegDef_t*)(GPIOB_BASEADDR))
#define GPIOC					((GPIO_RegDef_t*)(GPIOC_BASEADDR))
#define GPIOD 					((GPIO_RegDef_t*)(GPIOD_BASEADDR))
#define GPIOE 					((GPIO_RegDef_t*)(GPIOE_BASEADDR))
#define GPIOF 					((GPIO_RegDef_t*)(GPIOF_BASEADDR))
#define GPIOG 					((GPIO_RegDef_t*)(GPIOG_BASEADDR))
#define GPIOH 					((GPIO_RegDef_t*)(GPIOH_BASEADDR))
#define GPIOI 					((GPIO_RegDef_t*)(GPIOI_BASEADDR))

#define RCC 					((RCC_RegDef_t*)(RCC_BASEADDR))

#define EXTI					((EXTI_RegDef_t*)(EXTI_BASEADDR))

#define SYSCFG					((SYSCFG_RegDef_t*)(SYSCFG_BASEADDR))

#define SPI1					((SPI_RegDef_t*)(SPI1_BASEADDR))
#define SPI2					((SPI_RegDef_t*)(SPI2_BASEADDR))
#define SPI3					((SPI_RegDef_t*)(SPI3_BASEADDR))

#define I2C1					((I2C_RegDef_t*)(I2C1_BASEADDR))
#define I2C2					((I2C_RegDef_t*)(I2C2_BASEADDR))
#define I2C3					((I2C_RegDef_t*)(I2C3_BASEADDR))

#define USART1					((USART_RegDef_t*)(USART1_BASEADDR))
#define USART2					((USART_RegDef_t*)(USART2_BASEADDR))
#define USART3					((USART_RegDef_t*)(USART3_BASEADDR))
#define UART4					((USART_RegDef_t*)(UART4_BASEADDR))
#define UART5					((USART_RegDef_t*)(UART5_BASEADDR))
#define USART6					((USART_RegDef_t*)(USART6_BASEADDR))

/*
 * Clock enable macros for GPIOx peripherals
 */

#define GPIOA_PCLOCK_EN()	(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLOCK_EN()	(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLOCK_EN()	(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLOCK_EN()	(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLOCK_EN()	(RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLOCK_EN()	(RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLOCK_EN()	(RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLOCK_EN()	(RCC->AHB1ENR |= (1 << 7))
#define GPIOI_PCLOCK_EN()	(RCC->AHB1ENR |= (1 << 8))

/*
 * Clock enable macros for I2Cx peripherals
 */

#define I2C1_PCLOCK_EN()			(RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLOCK_EN()			(RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLOCK_EN()			(RCC->APB1ENR |= (1 << 23))

/*
 * Clock enable macros for SPIx peripherals
 */

#define SPI1_PCLOCK_EN()			(RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLOCK_EN()			(RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLOCK_EN()			(RCC->APB1ENR |= (1 << 15))

/*
 * Clock enable macros for USARTx peripherals
 */

#define USART1_PCLOCK_EN()			(RCC->APB2ENR |= (1 << 4))
#define USART2_PCLOCK_EN()			(RCC->APB1ENR |= (1 << 17))
#define USART3_PCLOCK_EN()			(RCC->APB1ENR |= (1 << 18))
#define UART4_PCLOCK_EN()			(RCC->APB1ENR |= (1 << 19))
#define UART5_PCLOCK_EN()			(RCC->APB1ENR |= (1 << 20))
#define USART6_PCLOCK_EN()			(RCC->APB2ENR |= (1 << 5))

/*
 * Clock enable macros for SYSCFG peripheral
 */

#define SYSCFG_PCLOCK_EN()			(RCC->APB2ENR |= (1 << 14))

/*
 * Clock disable macros for GPIOx peripherals
 */

#define GPIOA_PCLOCK_DI()	(RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLOCK_DI()	(RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLOCK_DI()	(RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLOCK_DI()	(RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLOCK_DI()	(RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLOCK_DI()	(RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLOCK_DI()	(RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLOCK_DI()	(RCC->AHB1ENR &= ~(1 << 7))
#define GPIOI_PCLOCK_DI()	(RCC->AHB1ENR &= ~(1 << 8))

/*
 * Clock disable macros for I2Cx peripherals
 */

#define I2C1_PCLOCK_DI()			(RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLOCK_DI()			(RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLOCK_DI()			(RCC->APB1ENR &= ~(1 << 23))

/*
 * Clock disable macros for SPIx peripherals
 */

#define SPI1_PCLOCK_DI()			(RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLOCK_DI()			(RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLOCK_DI()			(RCC->APB1ENR &= ~(1 << 15))

/*
 * Clock disable macros for USARTx peripherals
 */

#define USART1_PCLOCK_DI()			(RCC->APB2ENR &= ~(1 << 4))
#define USART2_PCLOCK_DI()			(RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLOCK_DI()			(RCC->APB1ENR &= ~(1 << 18))
#define UART4_PCLOCK_DI()			(RCC->APB1ENR &= ~(1 << 19))
#define UART5_PCLOCK_DI()			(RCC->APB1ENR &= ~(1 << 20))
#define USART6_PCLOCK_DI()			(RCC->APB2ENR &= ~(1 << 5))

/*
 * Clock disable macros for SYSCFG peripheral
 */

#define SYSCFG_PCLOCK_DI()			(RCC->APB2ENR &= ~(1 << 14))

/*
 * Macros to reset GPIOx peripherals
 */
#define GPIOA_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); } while(0)
#define GPIOB_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); } while(0)
#define GPIOC_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); } while(0)
#define GPIOD_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); } while(0)
#define GPIOE_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4)); } while(0)
#define GPIOF_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5)); } while(0)
#define GPIOG_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6)); } while(0)
#define GPIOH_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7)); } while(0)
#define GPIOI_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 8)); (RCC->AHB1RSTR &= ~(1 << 8)); } while(0)

/*
 * Macros returns a code (0 - 8) for given GPIO base address
 */
#define GPIO_BASEADDR_TO_CODE(x)	( (x == GPIOA) ? 0 :\
									(x == GPIOB) ? 1 :\
									(x == GPIOC) ? 2 :\
									(x == GPIOD) ? 3 :\
									(x == GPIOE) ? 4 :\
									(x == GPIOF) ? 5 :\
									(x == GPIOG) ? 6 :\
									(x == GPIOH) ? 7 :\
									(x == GPIOI) ? 8 :0 )

/*
 * Macros IRQ numbers GPIOS
 */

#define IRQ_NO_EXTI0		6
#define IRQ_NO_EXTI1		7
#define IRQ_NO_EXTI2		8
#define IRQ_NO_EXTI3		9
#define IRQ_NO_EXTI4		10
#define IRQ_NO_EXTI5_9		23
#define IRQ_NO_EXTI15_10	40

/*
 * Macros IQR numbers for SPI
 */

#define IRQ_NO_SPI1			35
#define IRQ_NO_SPI2			36
#define IRQ_NO_SPI3			51

/*
 * Macros IQR numbers for I2C
 */

#define IRQ_NO_I2C1EV		31
#define IRQ_NO_I2C1ER		32
#define IRQ_NO_I2C2EV		33
#define IRQ_NO_I2C2ER		34
#define IRQ_NO_I2C3EV		72
#define IRQ_NO_I2C3ER		73

/*
 * Macros for the possible priority levels of interrupt
 */
#define NVIC_IRQ_PRI0		0
#define NVIC_IRQ_PRI1		1
#define NVIC_IRQ_PRI2		2
#define NVIC_IRQ_PRI3		3
#define NVIC_IRQ_PRI4		4
#define NVIC_IRQ_PRI5		5
#define NVIC_IRQ_PRI6		6
#define NVIC_IRQ_PRI7		7
#define NVIC_IRQ_PRI8		8
#define NVIC_IRQ_PRI9		9
#define NVIC_IRQ_PRI10		10
#define NVIC_IRQ_PRI11		11
#define NVIC_IRQ_PRI12		12
#define NVIC_IRQ_PRI13		13
#define NVIC_IRQ_PRI14		14
#define NVIC_IRQ_PRI15		15



/*
 * Generic macros
 */

#define ENABLE 			(1)
#define DISABLE 		(0)
#define	SET 			ENABLE
#define RESET 			DISABLE
#define GPIO_PIN_SET 	SET
#define GPIO_PIN_RESET	RESET
#define FLAG_SET		SET
#define FLAG_RESET		RESET

/*
 * Bit positions of SPI peripheral
 */
#define SPI_CR1_CPHA		(0)
#define SPI_CR1_CPOL		(1)
#define SPI_CR1_MSTR		(2)
#define SPI_CR1_BR			(3)
#define SPI_CR1_SPE			(6)
#define SPI_CR1_SSI			(8)
#define SPI_CR1_SSM			(9)
#define SPI_CR1_RXONLY		(10)
#define SPI_CR1_DFF			(11)
#define SPI_CR1_BIDIOE		(14)
#define SPI_CR1_BIDIMODE	(15)

#define SPI_CR2_RXDMAEN		(0)
#define SPI_CR2_TXDMAEN		(1)
#define SPI_CR2_SSOE		(2)
#define SPI_CR2_FRF			(4)
#define SPI_CR2_ERRIE		(5)
#define SPI_CR2_RXNEIE		(6)
#define SPI_CR2_TXEIE		(7)

#define SPI_SR_RXNE			(0)
#define SPI_SR_TXE			(1)
#define SPI_SR_OVR			(6)
#define SPI_SR_BSY			(7)

/*
 * Bit positions of I2C peripheral
 */
#define I2C_CR1_PE			(0)
#define I2C_CR1_SMBUS		(1)
#define I2C_CR1_SMBTYPE		(3)
#define I2C_CR1_ENARP		(4)
#define I2C_CR1_ENPEC		(5)
#define I2C_CR1_ENGC		(6)
#define I2C_CR1_NOSTRETCH	(7)
#define I2C_CR1_START		(8)
#define I2C_CR1_STOP		(9)
#define I2C_CR1_ACK			(10)
#define I2C_CR1_POS			(11)
#define I2C_CR1_PEC			(12)
#define I2C_CR1_ALERT		(13)
#define I2C_CR1_SWRST		(15)

#define I2C_CR2_FREQ		(0)
#define I2C_CR2_ITERREN		(8)
#define I2C_CR2_ITEVTEN		(9)
#define I2C_CR2_ITBUFEN		(10)
#define I2C_CR2_DMAEN		(11)
#define I2C_CR2_LAST		(12)

#define I2C_SR1_SB			(0)
#define I2C_SR1_ADDR		(1)
#define I2C_SR1_BTF			(2)
#define I2C_SR1_ADD10		(3)
#define I2C_SR1_STOPF		(4)
#define I2C_SR1_RxNE		(6)
#define I2C_SR1_TxE			(7)
#define I2C_SR1_BERR		(8)
#define I2C_SR1_ARLO		(9)
#define I2C_SR1_AF			(10)
#define I2C_SR1_OVR			(11)
#define I2C_SR1_PECERR		(12)
#define I2C_SR1_TIMEOUT		(14)

#define I2C_SR2_MSL			(0)
#define I2C_SR2_BUSY		(1)
#define I2C_SR2_TRA			(2)
#define I2C_SR2_GENCALL		(4)
#define I2C_SR2_DUALF		(7)

#define I2C_CCR_CCR			(0)
#define I2C_CCR_DUTY		(14)
#define I2C_CCR_FS			(15)

#define I2C_MASTER_WRITE 	(0)
#define I2C_MASTER_READ 	(1)


#include <stm32f407xx_gpio.h>
#include <stm32f407xx_spi_driver.h>
#include <stm32f407xx_i2c.h>
#include <stm32f407xx_usart_driver.h>
#endif /* INC_STM32F407XX_H_ */
