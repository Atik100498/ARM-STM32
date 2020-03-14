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

uint32_t baseaddr = 0x20001250;

void delay()
{
	uint16_t i = 0;
	for(i=0;i<=500;i++);
}
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
	char buff[5] = "atik";
	com_but_pin();
	button();
	I2C_Handle_t com;
	com.pI2Cx = I2C1;
	com.I2C_Config.I2C_ACKControl = ENABLE;
	com.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

	I2C_PClkControl(I2C1,ENABLE);
	I2C_Init(&com);


	//set as global
	DMAi2c.pDMAx = DMA1;
	DMAi2c.pDMAConfig.ChannelSelect = 1;
	DMAi2c.pDMAConfig.DataFlowDirection = DIR_M2P;
	DMAi2c.pDMAConfig.DirectModeEnable = 0;
	DMAi2c.pDMAConfig.MemoryIncrementEnable = 1;
	DMAi2c.pDMAConfig.NumberOfTransaction = 4;//data
	DMAi2c.pDMAConfig.PeripheralDataSize = PSIZE_8BIT;
	DMAi2c.pDMAConfig.PeripheralIncrementEnable = 0;
	DMAi2c.pDMAConfig.SelectPriority = PRIORITY_VH;
	DMAi2c.pDMAConfig.StreamSelect = STREAM6;
	//source destination address
	DMAi2c.pDMAConfig.SourceAddress= baseaddr;
	//Destination destination address
	DMAi2c.pDMAConfig.DestinationAddress = 0x40005410;
	//enable interrupt at its default priority
	DMA_IRQITConfig(IRQ_NO_DMA1_Stream6,ENABLE);
	DMAEnable(DMA1,ENABLE);//RCC CLOCK
	DMAInit(&DMAi2c);//configuration


	int i = 0;
	for(i=0;i<5;i++)
	{
		*((uint8_t*)baseaddr + i) = buff[i];
	}

	while(1)
	{
		while(!GPIO_ReadPin(GPIOA,GPIO_PIN_0));

			delay();
			I2C_PeripheralControl(I2C1,ENABLE);
			DMAPeripheralEnable(DMA1,ENABLE);
			I2C_MasterDMASendData(&com,0x68);

	}

	return 0;
}

void DMA1_Stream6_IRQHandler(void)
{
	DMAInterruptHandle(&DMAi2c);
	DMAPeripheralEnable(DMA1,DISABLE);
}
