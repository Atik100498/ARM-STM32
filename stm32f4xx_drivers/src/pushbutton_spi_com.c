/*
 * pushbutton_spi_com.c
 *
 *  Created on: 15-Dec-2019
 *      Author: ATIQUE
 */

#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_spi_driver.h"
#include <string.h>

/*
 * Push button:- PA1
 * PA4 - NSS
 * PA5 - SCK
 * PA6 - MISO
 * PA7 - MOSI
 */

SPI_Handle_t SPI1_Handle;

void SPI_GPIOCnfig(void)
{

	GPIO_Handle_t SPIPins;
	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = AF5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP; //Push Pull configuration for SPIx
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;// this config. does not matter
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH; // this config. does not matter
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
void SPI1_init(void)
{
	memset(&SPI1_Handle,0,sizeof(SPI1_Handle));
	SPI1_Handle.pSPIx = SPI2;
	SPI1_Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI1_Handle.SPIConfig.SPI_BusConfig = SPI_CONFIG_FD;
	SPI1_Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8;
	SPI1_Handle.SPIConfig.SPI_DFF = SPI_DFF_8BIT;
	SPI1_Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI1_Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI1_Handle.SPIConfig.SPI_SSM = SPI_SSM_HARDWARE;

	SPI_PClkControl(SPI2,ENABLE); // CLOCK SET
	SPI_Init(&SPI1_Handle); //

}

void GPIO_PushButton(void)
{
	GPIO_Handle_t GPIO_PushButton;
	memset(&GPIO_PushButton,0,sizeof(GPIO_PushButton));

	GPIO_PushButton.pGPIOx = GPIOA;
	GPIO_PushButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_0;
	GPIO_PushButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GPIO_PushButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GPIO_PushButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PClkControl(GPIOA,ENABLE);
	GPIO_Init(&GPIO_PushButton);
	//
	//Interrupt handling
	GPIO_IRQITConfig(IRQ_NO_EXTI0,ENABLE);
}
int main(void)
{
	//Push button
	GPIO_PushButton();
	//SPI button config
	SPI_GPIOCnfig();
	//SPI initialization
	SPI1_init();
	SPI_SSOEConfig(SPI2,ENABLE);//SSM = 0,SSOE = 1 for NSS pin HIGH
	SPI_PeripheralControl(SPI2,ENABLE);

	while(0);
	return 0;

}

void EXTI0_IRQHandler(void)
{
	GPIO_ReadHandling(GPIO_PIN_0);

	//Action on interrupt
	char send_data[] = "hello world";
	SPI_SendData(SPI2,(uint8_t*)send_data,strlen(send_data));

}

