/*
 * stm32f407xx.h
 *
 *  Created on: 06-Nov-2019
 *      Author: ATIQUE
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include<stdint.h>

#define __vo volatile
/*****************************************START:Processor Specific Details*****************************************/
/*
 * NVIC ISERx registers address
 * Interrupt set-Enable Register
 */

#define NVIC_ISER0					((__vo uint32_t*)0xE000E100)
#define NVIC_ISER1					((__vo uint32_t*)0xE000E104)
#define NVIC_ISER2					((__vo uint32_t*)0xE000E108)
#define NVIC_ISER3					((__vo uint_2_t*)0xE000E10C)

/*
 * NVIC ICERx registers address
 */

#define NVIC_ICER0					((__vo uint32_t*)0xE000E180)
#define NVIC_ICER1					((__vo uint32_t*)0xE000E184)
#define NVIC_ICER2					((__vo uint32_t*)0xE000E188)
#define NVIC_ICER3					((__vo uint32_t*)0xE000E18C)

/*
 * NVIC IPRX REGISTER ADDRESS
 */

#define NVIC_PR_BASE_ADDR			((__vo uint32_t*)0xE000E400)


#define NO_PR_NITS_IMPLEMENTED		4



/* BASE ADDRESSES OF FLASH AND SRAM MEMOERIES*/

#define FLASH_BASEADDR				0x08000000U //FLASH BASE ADDRESS
#define SRAM1_BASEADDR              0x20000000U //112KB
#define SRAM2_BASEADDR				0x2001C000U
#define ROM_BASEADDR				0x1FFF0000U
#define SRAM_BASEADDR				SRAM1_BASEADDR

/* BASE ADDRESS OF AHBx AND APBx */

#define PERIPH_BASE					0x40000000U
#define APB1PERIPH_BASE 			PERIPH_BASE
#define APB2PERIPH_BASE 			(PERIPH_BASE + 0x00010000U)
#define AHB1PERIPH_BASE 			(PERIPH_BASE + 0x00020000U)
#define AHB2PERIPH_BASE 			(PERIPH_BASE + 0x10000000U)

/*BASE ADDRESS OF PERIPHERALS ON AHB1 BUS */

#define GPIOA_BASEADDR				AHB1PERIPH_BASE
#define GPIOB_BASEADDR				(AHB1PERIPH_BASE + 0x00000400U)
#define GPIOC_BASEADDR				(AHB1PERIPH_BASE + 0x00000800U)
#define GPIOD_BASEADDR				(AHB1PERIPH_BASE + 0x00000C00U)
#define GPIOE_BASEADDR				(AHB1PERIPH_BASE + 0x00001000U)
#define GPIOF_BASEADDR				(AHB1PERIPH_BASE + 0x00001400U)
#define GPIOG_BASEADDR				(AHB1PERIPH_BASE + 0x00001800U)
#define GPIOH_BASEADDR				(AHB1PERIPH_BASE + 0x00001C00U)
#define GPIOI_BASEADDR				(AHB1PERIPH_BASE + 0x00002000U)
#define RCC_BASEADDR				(AHB1PERIPH_BASE + 0X00003800U)


/* BASE ADDRESS OF PERIPHERALS ON APB1 BUS */

#define I2C1_BASEADDR				(APB1PERIPH_BASE + 0X00005400U)
#define I2C2_BASEADDR				(APB1PERIPH_BASE + 0X00005800U)
#define I2C3_BASEADDR				(APB1PERIPH_BASE + 0X00005C00U)
#define SPI2_BASEADDR				(APB1PERIPH_BASE + 0X00003800U)
#define SPI3_BASEADDR				(APB1PERIPH_BASE + 0X00003C00U)
#define USART2_BASEADDR				(APB1PERIPH_BASE + 0X00004400U)
#define USART3_BASEADDR				(APB1PERIPH_BASE + 0X00004800U)
#define UART4_BASEADDR				(APB1PERIPH_BASE + 0X00004C00U)
#define UART5_BASEADDR				(APB1PERIPH_BASE + 0X00005000U)
#define SYSCFG_BASRADDR				(APB2PERIPH_BASE + 0x00003800U)

/* BASE ADDRESS OF PERIPHERAL IN APB2 BUS*/
// APB2_ADDR =

#define SPI1_BASEADDR				(APB2PERIPH_BASE + 0x00003000U)
#define USART1_BASEADDR				(APB2PERIPH_BASE + 0x00001000U)
#define USART6_BASEADDR				(APB2PERIPH_BASE + 0x00001400U)
#define EXTI_BASEADDR				(APB2PERIPH_BASE + 0x00003C00U)
#define SYSCFG_BASEADDR				(APB2PERIPH_BASE + 0x00003800U)


/* GPIOx PERIPHERAL REGISTER DEFINITION STRUCTURE */

/*
 * Register address = GPIOA + offset;
 * MODER_BASEA DDR = 0x40000000U + offset;
 */

typedef struct
{
	__vo uint32_t MODER;
	__vo uint32_t OTYPER;
	__vo uint32_t OSPEEDR;
	__vo uint32_t PUPDR;
	__vo uint32_t IDR;
	__vo uint32_t ODR;
	__vo uint32_t BSRR;
	__vo uint32_t LCKR;
	__vo uint32_t AFR[2];       /*AFR[0] -> AFRL  AFR[1] -> AFRH ALTERNATIVE FUNCTION REGISTER*/
}GPIO_RegDef_t;

/*
 * RCC PERIPHERAL REGISTER DEFINITION STRUCTURE
 */

typedef struct
{
	__vo uint32_t CR;
	__vo uint32_t PLLCFGR;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t AHB1RSTR;
	__vo uint32_t AHB2RSTR;
	__vo uint32_t AHB3RSTR;
	uint32_t RESERVED0;
	__vo uint32_t APB1RSTR;
	__vo uint32_t APB2RSTR;
	uint32_t RESERVED1;
	uint32_t RESERVED2;
	__vo uint32_t AHB1ENR;
	__vo uint32_t ABH2ENR;
	__vo uint32_t AHB3ENR;
	uint32_t RESERVED3;
	__vo uint32_t APB1ENR;
	__vo uint32_t APB2ENR;
	uint32_t RESERVED4;
	uint32_t RESERVED5;
	__vo uint32_t AHB1LPENR;
	__vo uint32_t AHB2LPENR;
	__vo uint32_t AHB3LPENR;
	uint32_t RESERVED6;
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	uint32_t RESERVED7;
	uint32_t RESERVED8;
	__vo uint32_t SSCGR;
	__vo uint32_t PLLI2SCFGR;
	__vo uint32_t PLLSAICFGR;
	__vo uint32_t DCKCFGR;
}RCC_RegDef_t;


/* EXTI PERIPHERAL STRUCTURE */

typedef struct
{
	__vo uint32_t IMR; /* Interupt Mask Register */
	__vo uint32_t EMR;
	__vo uint32_t RTSR; /* Rising Trigger Selection Register */
	__vo uint32_t FTSR; /* Falling Trigger Selection Register */
	__vo uint32_t SWIER;
	__vo uint32_t PR; /*Pending  Register*/
}EXTI_RegDef_t;


/* SYSCFG PERIPHERAL DEFINATION */

typedef struct
{
	__vo uint32_t MEMRMP;		//0x00
	__vo uint32_t PMC;			//0x04
	__vo uint32_t EXTICR[4];	//0x08 - 0x14
	uint32_t RESERVED1[2];		//0X18 - 0x1C
	__vo uint32_t CMPCR;		//0x20
	uint32_t RESERVED2[2];		//0x24 - 0x28
	__vo uint32_t CFGR;			//0x2C
}SYSCFG_RegDef_t;


/*
 * PERIPHERAL DEFINATION OF SPI
 */

typedef struct
{
	__vo uint32_t CR1; //0x00
	__vo uint32_t CR2; //0x04
	__vo uint32_t SR;  //0x08
	__vo uint32_t DR;  //0x0C
	__vo uint32_t CRCPR;//0x10
	__vo uint32_t RXCRCR;//0x14
	__vo uint32_t TXCRCR;//0x18
	__vo uint32_t I2SCFGR;//0x1C
	__vo uint32_t I2SPR; //0x20
}SPI_RegDef_t;

/*
 * PERIPHERAL DEFINATION OF SPI
 */

typedef struct
{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t OAR1;
	__vo uint32_t OAR2;
	__vo uint32_t DR;
	__vo uint32_t SR1;
	__vo uint32_t SR2;
	__vo uint32_t CCR;
	__vo uint32_t TRISE;
	__vo uint32_t FLTR;
}I2C_RegDef_t;
/* PERIPHERAL DEFINATION */

#define GPIOA						((GPIO_RegDef_t*) GPIOA_BASEADDR)
#define GPIOB						((GPIO_RegDef_t*) GPIOB_BASEADDR)
#define GPIOC						((GPIO_RegDef_t*) GPIOC_BASEADDR)
#define GPIOD						((GPIO_RegDef_t*) GPIOD_BASEADDR)
#define GPIOE						((GPIO_RegDef_t*) GPIOE_BASEADDR)
#define GPIOF						((GPIO_RegDef_t*) GPIOF_BASEADDR)
#define GPIOG						((GPIO_RegDef_t*) GPIOG_BASEADDR)
#define GPIOH						((GPIO_RegDef_t*) GPIOH_BASEADDR)
#define GPIOI						((GPIO_RegDef_t*) GPIOI_BASEADDR)

/*
 * SYSCGF clock configuration for EXTx clock
 */

#define SYSCFG						((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)
#define EXTI						((EXTI_RegDef_t*) EXTI_BASEADDR)
#define RCC							((RCC_RegDef_t*)RCC_BASEADDR)
#define SPI1						((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2						((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3						((SPI_RegDef_t*)SPI3_BASEADDR)
#define I2C1						((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2						((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3						((I2C_RegDef_t*)I2C3_BASEADDR)
/* CLOCK ENABLE MACROS FOR GPIO PERIPHERALS */

/*
 * ALL THE PERIPHERAL CLOCKS ARE PRESENT IN THE RCC REGISTER BLOCK.
 * ALL ARE WELL DEFIENED WITH RESPECT TO ON WHICH BUS THE PERIPHERAL RESISED
 * Eg. GPIOx [A,..,H,I] are on AHB1 bus, clocks can be enabled from AHB1ENR
 */

#define GPIOA_PCLK_EN()				(RCC->AHB1ENR |=(1<<0))
#define GPIOB_PCLK_EN()				(RCC->AHB1ENR |=(1<<1))
#define GPIOC_PCLK_EN()				(RCC->AHB1ENR |=(1<<2))
#define GPIOD_PCLK_EN()				(RCC->AHB1ENR |=(1<<3))
#define GPIOE_PCLK_EN()				(RCC->AHB1ENR |=(1<<4))
#define GPIOF_PCLK_EN()				(RCC->AHB1ENR |=(1<<5))
#define GPIOG_PCLK_EN()				(RCC->AHB1ENR |=(1<<6))
#define GPIOH_PCLK_EN()				(RCC->AHB1ENR |=(1<<7))
#define GPIOI_PCLK_EN()				(RCC->AHB1ENR |=(1<<8))


/*CLOCK ENABLE FOR I2C PERIPHERAL */

#define I2C1_PCLK_EN()				(RCC->APB1ENR |=(1<21))
#define I2C2_PCLK_EN()				(RCC->APB1ENR |=(1<22))
#define I2C3_PCLK_EN()				(RCC->APB1ENR |=(1<23))

/*CLOCK ENABLE FOR SPIx PERIPHERAL */

#define SPI1_PCLK_EN()				(RCC->APB2ENR |= (1<<12))
#define SPI2_PCLK_EN()				(RCC->APB1ENR |= (1<<14))
#define SPI3_PCLK_EN()				(RCC->APB1ENR |= (1<<15))

/*CLOCK ENABLE FOR USARTx AND UARTx */

#define	USART1_PCLK_EN()			(RCC->APB2ENR |= (1<<4))
#define	USART2_PCLK_EN()			(RCC->APB1ENR |= (1<<17))
#define	USART3_PCLK_EN()			(RCC->APB1ENR |= (1<<18))
#define	UART4_PCLK_EN()				(RCC->APB1ENR |= (1<<19))
#define	UART1_PCLK_EN()				(RCC->APB1ENR |= (1<<20))
#define	USART6_PCLK_EN()			(RCC->APB2ENR |= (1<<5))



/* CLOCK ENABLE FOR SYSCFG */

#define SYSCFG_PCLK_EN()			(RCC->APB2ENR |=(1<<14))

/* CLOCK DISABLE MACROS FOR GPIO PERIPHERALS */
/*
 * DISABLE THE CLOCK FOR GPIOx
 */
#define GPIOA_PCLK_DI()				(RCC->AHB1ENR &= ~(1<<0))
#define GPIOB_PCLK_DI()				(RCC->AHB1ENR &= ~(1<<1))
#define GPIOC_PCLK_DI()				(RCC->AHB1ENR &= ~(1<<2))
#define GPIOD_PCLK_DI()				(RCC->AHB1ENR &= ~(1<<3))
#define GPIOE_PCLK_DI()				(RCC->AHB1ENR &= ~(1<<4))
#define GPIOF_PCLK_DI()				(RCC->AHB1ENR &= ~(1<<5))
#define GPIOG_PCLK_DI()				(RCC->AHB1ENR &= ~(1<<6))
#define GPIOH_PCLK_DI()				(RCC->AHB1ENR &= ~(1<<7))
#define GPIOI_PCLK_DI()				(RCC->AHB1ENR &= ~(1<<8))

/*CLOCK DISABLE FOR I2C PERIPHERAL */

#define I2C1_PCLK_DI()				(RCC->APB1ENR &= ~(1<21))
#define I2C2_PCLK_DI()				(RCC->APB1ENR &= ~(1<22))
#define I2C3_PCLK_DI()				(RCC->APB1ENR &= ~(1<23))

/*CLOCK DISABLE FOR SPIx PERIPHERAL */

#define SPI1_PCLK_DI()				(RCC->APB2ENR &= ~(1<<12))
#define SPI2_PCLK_DI()				(RCC->APB1ENR &= ~(1<<14))
#define SPI3_PCLK_DI()				(RCC->APB1ENR &= ~(1<<15))

/*CLOCK DISABLE FOR USARTx AND UARTx */

#define	USART1_PCLK_DI()			(RCC->APB2ENR &= ~(1<<4))
#define	USART2_PCLK_DI()			(RCC->APB1ENR &= ~(1<<17))
#define	USART3_PCLK_DI()			(RCC->APB1ENR &= ~(1<<18))
#define	UART4_PCLK_DI()				(RCC->APB1ENR &= ~(1<<19))
#define	UART1_PCLK_DI()				(RCC->APB1ENR &= ~(1<<20))
#define	USART6_PCLK_DI()			(RCC->APB2ENR &= ~(1<<5))

/*PORT RESET USING RCC*/

/*
 * RCC (Reset and Clock Control)
 * AHB[n]RSTR is used to de-initialize the port and start with new configuration
 */

#define GPIOA_REG_RESET()			do{(RCC->AHB1RSTR |= (1<<0)); (RCC->AHB1RSTR &= ~(1<<0));}while(0)
#define GPIOB_REG_RESET()			do{(RCC->AHB1RSTR |= (1<<2)); (RCC->AHB1RSTR &= ~(1<<2));}while(0)
#define GPIOC_REG_RESET()			do{(RCC->AHB1RSTR |= (1<<3)); (RCC->AHB1RSTR &= ~(1<<3));}while(0)
#define GPIOD_REG_RESET()			do{(RCC->AHB1RSTR |= (1<<4)); (RCC->AHB1RSTR &= ~(1<<4));}while(0)
#define GPIOE_REG_RESET()			do{(RCC->AHB1RSTR |= (1<<5)); (RCC->AHB1RSTR &= ~(1<<5));}while(0)
#define GPIOF_REG_RESET()			do{(RCC->AHB1RSTR |= (1<<6)); (RCC->AHB1RSTR &= ~(1<<6));}while(0)
#define GPIOG_REG_RESET()			do{(RCC->AHB1RSTR |= (1<<7)); (RCC->AHB1RSTR &= ~(1<<7));}while(0)
#define GPIOH_REG_RESET()			do{(RCC->AHB1RSTR |= (1<<8)); (RCC->AHB1RSTR &= ~(1<<8));}while(0)
#define GPIOI_REG_RESET()			do{(RCC->AHB1RSTR |= (1<<9)); (RCC->AHB1RSTR &= ~(1<<9));}while(0)

/*
 * De-initialize clock for SPI
 */
#define SPI1_REG_RESET()			do{(RCC->APB2RSTR |= (1<<12)); (RCC -> APB2RSTR |= ~(1<<12));}while(0)
#define SPI2_REG_RESET()			do{(RCC->APB1RSTR |= (1<<14)); (RCC -> APB1RSTR |= ~(1<<14));}while(0)
#define SPI3_REG_RESET()			do{(RCC->APB1RSTR |= (1<<15)); (RCC -> APB1RSTR |= ~(1<<15));}while(0)
#define SPI4_REG_RESET()			do{(RCC->APB2RSTR |= (1<<13)); (RCC -> APB2RSTR |= ~(1<<13));}while(0)

/*
 * De-initialize clock for I2C
 */
#define I2C1_REG_RESET()			do{(RCC->APB1RSTR |= (1<<21)); (RCC -> APB1RSTR |= ~(1<<21));}while(0)
#define I2C2_REG_RESET()			do{(RCC->APB1RSTR |= (1<<22)); (RCC -> APB1RSTR |= ~(1<<22));}while(0)
#define I2C3_REG_RESET()			do{(RCC->APB1RSTR |= (1<<23)); (RCC -> APB1RSTR |= ~(1<<23));}while(0)
/*
 * Port selection for interrupt in SYSCFG
 */
#define GPIO_BASEADDER_TO_CODE(x)		((x == GPIOA) ? 0:\
										 (x == GPIOB) ? 1:\
										 (x == GPIOC) ? 2:\
										 (x == GPIOD) ? 3:\
										 (x == GPIOE) ? 4:\
										 (x == GPIOF) ? 5:\
										 (x == GPIOG) ? 6:\
										 (x == GPIOH) ? 7:0)

/*
 * IRQ numbers for all EXTI[n] lines
 * this the priority number.
 * They are specific to MCU family.
 * Refer Vector table.
 */

#define 	IRQ_NO_EXTI0				6
#define 	IRQ_NO_EXTI1				7
#define 	IRQ_NO_EXTI2				8
#define 	IRQ_NO_EXTI3				9
#define 	IRQ_NO_EXTI4				10
#define 	IRQ_NO_EXTI9_5				23
#define 	IRQ_NO_EXTI15_10			40
#define		IRQ_NO_SPI1					35
#define		IRQ_NO_SPI2					36
#define		IRQ_NO_SPI3					51

#define NVIC_IRQ_PRIORITY_0				0
#define NVIC_IRQ_PRIORITY_1				1
#define NVIC_IRQ_PRIORITY_2				2
#define NVIC_IRQ_PRIORITY_3				3
#define NVIC_IRQ_PRIORITY_4				4
#define NVIC_IRQ_PRIORITY_5				5
#define NVIC_IRQ_PRIORITY_6				6
#define NVIC_IRQ_PRIORITY_7				7
#define NVIC_IRQ_PRIORITY_8				8
#define NVIC_IRQ_PRIORITY_9				9
#define NVIC_IRQ_PRIORITY_10			10
#define NVIC_IRQ_PRIORITY_11			11
#define NVIC_IRQ_PRIORITY_12			12
#define NVIC_IRQ_PRIORITY_13			13
#define NVIC_IRQ_PRIORITY_14			14
#define NVIC_IRQ_PRIORITY_15			15


//SPI CR1 REGISTER BIT

#define SPI_CR1_CPHA					0
#define SPI_CR1_CPOL                    1
#define SPI_CR1_MSTR					2
#define SPI_CR1_BR						3
#define SPI_CR1_SPE						6
#define SPI_CR1_LSBFIRST				7
#define SPI_CR1_SSI						8
#define SPI_CR1_SSM						9
#define SPI_CR1_RXONLY					10
#define SPI_CR1_DFF						11
#define SPI_CR1_CRCNEXT					12
#define SPI_CR1_CRCEN					13
#define SPI_CR1_BIDIOE					14
#define SPI_CR1_BIDIMODE				15

//SPI CR2 REGISTER BIT

#define SPI_CR2_RDXMAEN					0
#define SPI_CR2_XTDMAEN					1
#define SPI_CR2_SSOE					2
#define SPI_CR2_FRF						4
#define SPI_CR2_ERRIE					5
#define SPI_CR2_RXNEIE					6
#define SPI_CR2_TXEIE					7

//SPI SR REGISTER BIT
#define SPI_SR_RXNE						0
#define SPI_SR_TXE						1
#define SPI_SR_CHSIDE					2
#define SPI_SR_UDR						3
#define SPI_SR_CRCERR					4
#define SPI_SR_MODF						5
#define SPI_SR_OVR						6
#define SPI_SR_BSY						7
#define SPI_SR_FRE						8

//I2C CR1 REGISTER BITS
#define I2C_CR1_PE						0
#define I2C_CR1_SMBUS					1
#define I2C_CR1_SMBTYPE					3
#define I2C_CR1_ENARP					4
#define I2C_CR1_ENPEC					5
#define I2C_CR1_ENGC					6
#define I2C_CR1_NOSTRETCH				7
#define I2C_CR1_START					8
#define I2C_CR1_STOP					9
#define I2C_CR1_ACK						10
#define I2C_CR1_POS						11
#define I2C_CR1_PEC						12
#define I2C_CR1_ALERT					13
#define I2C_CR1_SWRST					15

//I2C CR2 REGISTER BITS
#define I2C_CR2_FREQ					0
#define I2C_CR2_ITERREN					8
#define I2C_CR2_ITEVTEN					9
#define I2C_CR2_ITBUFEN					10
#define I2C_CR2_DMAEN					11
#define I2C_CR2_LAST					12

//I2C SR1 REGISTER BITS

#define I2C_SR1_SB						0
#define I2C_SR1_ADDR					1
#define I2C_SR1_BTF						2
#define I2C_SR1_ADD10					3
#define I2C_SR1_STOPF					4
#define I2C_SR1_RXNE					6
#define I2C_SR1_TXE						7
#define I2C_SR1_BERR					8
#define I2C_SR1_ARLO					9
#define I2C_SR1_AF						10
#define I2C_SR1_OVR						11
#define I2C_SR1_PECERR					12
#define I2C_SR1_TIMEOUT					14
#define I2C_SR1_SMBALERT				15

//U2C SR2 REGISTER BITS
#define I2C_SR2_MSL						0
#define I2C_SR2_BUSY					1
#define I2C_SR2_TRA						2
#define I2C_SR2_GENCALL					4
#define I2C_SR2_SMBDEFAULT				5
#define I2C_SR2_DUALF					7
#define I2C_SR2_PEC						8

//I2C CCR REGISTER BITS
#define I2C_CCR_CCR						0
#define I2C_CCR_DUTY					14
#define I2C_CCR_FS						15

//I2C TRISE REGISTER BITS
#define I2C_TRISE_TRISE					0
//SOME GENERIC MACROS//

#define ENABLE 						1
#define DISABLE 					0
#define SET 						ENABLE
#define RESET						DISABLE
#define GPIO_PIN_SET				SET
#define GPIO_PIN_RESET				RESET
#define FLAG_SET					SET
#define FLAG_RESET					RESET
#define TRUE						1

#endif /* INC_STM32F407XX_H_ */
