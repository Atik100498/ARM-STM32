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

void SPI2_GPIOInits(void)
{
	GPIO_Handle_t SPIPins;
	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = AF5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP; //Push Pull configuration for SPIx
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;// this config. does not matter
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH; // this config. does not matter

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
	SPI_Handle_t *SPI2_Handle;
	SPI2_Handle->pSPIx = SPI2;
	SPI2_Handle->SPIConfig.SPI_BusConfig = SPI_CONFIG_FD;
	SPI2_Handle->SPIConfig. SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2_Handle->SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2;
	SPI2_Handle->SPIConfig.SPI_DFF = SPI_DFF_8BIT;
	SPI2_Handle->SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2_Handle->SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2_Handle->SPIConfig.SPI_SSM = SPI_SSM_SOFTWARE;

	SPI_Init(SPI2_Handle);
}


int main(void)
{
	char user_data[] = "Hello world";
	SPI2_GPIOInits();
	GPIOB_PCLK_EN();

	SPI2_Init();

	SPI_SSOEConfig(SPI2,ENABLE);
	SPI2_PCLK_EN();

	SPI_PeripheralControl(SPI2,ENABLE); // SPE (SPI ENABLE) bit in CR1 register
	SPI_SendData(SPI2,(uint8_t*)user_data,strlen(user_data));
	SPI_PeripheralControl(SPI2,DISABLE); // DISBALE the SPE bit
	return 0;
}
