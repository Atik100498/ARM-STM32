/*
 * arduino_spi_com.c
 *
 *  Created on: 16-Dec-2019
 *      Author: ATIQUE
 */




#include "stm32f407xx.h"
#include "stm32f407xx_spi_driver.h"
#include "stm32f407xx_gpio_driver.h"
#include<string.h>

void delay()
{
	for(uint32_t i = 0; i<=50000*2;i++);
}
void SPI2_GPIOInits(void)
{
	GPIO_Handle_t SPIPins;
	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = AF5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP; //Push Pull configuration for SPIx
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;// this config. does not matter
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW; // this config. does not matter
	GPIOB_PCLK_EN();
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;//SCLK
	GPIO_Init(&SPIPins);

	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_15;//MOSI
	GPIO_Init(&SPIPins);

	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_14;//MISO
	//GPIO_Init(&SPIPins);

	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;//NSS
	GPIO_Init(&SPIPins);
}

void SPI2_Init(void)
{
	SPI_Handle_t SPI2_Handle;
	SPI2_Handle.pSPIx = SPI2;
	SPI2_Handle.SPIConfig.SPI_BusConfig = SPI_CONFIG_FD;
	SPI2_Handle.SPIConfig. SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2_Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8;
	SPI2_Handle.SPIConfig.SPI_DFF = SPI_DFF_8BIT;
	SPI2_Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2_Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2_Handle.SPIConfig.SPI_SSM = SPI_SSM_HARDWARE;

	SPI_Init(&SPI2_Handle);
}


int main(void)
{
	GPIO_Handle_t but;
	but.pGPIOx = GPIOA;
	but.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	but.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	but.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	but.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_0;

	GPIO_PClkControl(GPIOA,ENABLE);
	GPIO_Init(&but);


	char user_data[] ="\r1234\r";
	uint8_t len = strlen(user_data);
	SPI2_GPIOInits();

	SPI_PClkControl(SPI2,ENABLE);
	SPI2_Init();

	SPI_SSOEConfig(SPI2,ENABLE);



	while(1)
	{
		SPI_PeripheralControl(SPI2,ENABLE); // SPE (SPI ENABLE) bit in CR1 register
		if(GPIO_ReadPin(GPIOA,GPIO_PIN_0) == SET)
		{

			SPI_SendData(SPI2,&len,1);

			SPI_SendData(SPI2,user_data,strlen(user_data));


			delay();

		//SPI_PeripheralControl(SPI2,DISABLE); // DISBALE the SPE bit
		}

	}

	return 0;
}
