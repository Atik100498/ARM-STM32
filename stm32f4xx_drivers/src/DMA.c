/*
 * DMA.c
 *
 *  Created on: 10-Mar-2020
 *      Author: ATIQUE
 */


/*
 * DMA_i2c.c
 *
 *  Created on: 10-Mar-2020
 *      Author: ATIQUE
 */

#include "stm32f407xx.h"
#include "stm32f407xx_dma_driver.h"
#include "stm32f407xx_i2c_driver.h"
#include "stm32f407xx_gpio_driver.h"
#include "stdint.h"

DMA_Handle_t DMAi2c;

void button(void)
{
	GPIO_Handle_t push;
	push.pGPIOx = GPIOA;
	push.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_0;
	push.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	push.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	push.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	push.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;

	GPIO_PClkControl(GPIOA,ENABLE);
	GPIO_Init(&push);
}

void com_but_pin(void)
{
	GPIO_Handle_t com_but;
	com_but.pGPIOx = GPIOB;
	com_but.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	com_but.GPIO_PinConfig.GPIO_PinAltFunMode = AF4;
	com_but.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	com_but.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	com_but.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;

	GPIO_PClkControl(GPIOB,ENABLE);
	com_but.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_6;//SCL
	GPIO_Init(&com_but);

	com_but.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_7;//SDA
	GPIO_Init(&com_but);
}

int main(void)
{
	char buff[2] = {0xd0,0x31};
//	button();
//	com_but_pin();


//	GPIO_Handle_t adcPin;
//	adcPin.pGPIOx = GPIOA;
//	adcPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_1;
//	adcPin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ANALOG;
//	adcPin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
//	adcPin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
//
//	GPIO_PClkControl(GPIOA,ENABLE);
//	GPIO_Init(&adcPin);

	I2C_Handle_t com;
	com.pI2Cx = I2C1;
	com.I2C_Config.I2C_ACKControl = ENABLE;
	com.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

	I2C_PClkControl(I2C1,ENABLE);
	I2C_Init(&com);
	I2C_PeripheralControl(I2C1,ENABLE);

	//set as global
	DMAi2c.pDMAx = DMA1;
	DMAi2c.pDMAConfig.ChannelSelect = 1;
	DMAi2c.pDMAConfig.DataFlowDirection = DIR_M2P;
	DMAi2c.pDMAConfig.DirectModeEnable = 0;
	DMAi2c.pDMAConfig.MemoryIncrementEnable = 1;
	DMAi2c.pDMAConfig.NumberOfTransaction = 2;//address + data
	DMAi2c.pDMAConfig.PeripheralDataSize = PSIZE_8BIT;
	DMAi2c.pDMAConfig.PeripheralIncrementEnable = 0;
	DMAi2c.pDMAConfig.SelectPriority = PRIORITY_VH;
	DMAi2c.pDMAConfig.StreamSelect = STREAM7;
	//source destination address
	DMAi2c.pDMAConfig.SourceAddress= &buff;
	//Destination destination address
	DMAi2c.pDMAConfig.DestinationAddress = 0x40005410;
	//enable interrupt at its default priority
	DMA_IRQITConfig(IRQ_NO_DMA1_Stream7,ENABLE);
	DMAEnable(DMA1,ENABLE);//RCC CLOCK
	DMAInit(&DMAi2c);//configuration
	DMAPeripheralEnable(DMA1,ENABLE);//DMAEN
	while(1);

	return 0;
}

void DMA1_Stream7_IRQHandler(void)
{
	DMAInterruptHandle(&DMAi2c);
	DMAPeripheralEnable(DMA1,DISABLE);
}
