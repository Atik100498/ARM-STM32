/*
 * SPI_COM.C
 *
 *  Created on: 15-Dec-2019
 *      Author: ATIQUE
 */

#include "stm32f407xx.h"
#include "stm32f407xx_spi_driver.h"
#include "stm32f407xx_gpio_driver.h"
#include<string.h>

void SPI2_GPIOInits(void)
{
	GPIO_Handle_t SPIPins;
	memset(&SPIPins,0,sizeof(SPIPins));
	SPIPins.pGPIOx = GPIOA;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = AF5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;// this config. does not matter
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH; // this config. does not matter


	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_5;//SCLK
	GPIO_Init(&SPIPins);

	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_15;//MOSI
	GPIO_Init(&SPIPins);

	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_14;//MISO
	//GPIO_Init(&SPIPins);

	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;//NSS
	//GPIO_Init(&SPIPins);
}



void SPI2_Init(void)
{
	SPI_Handle_t SPI2_Handle;
	memset(&SPI2_Handle,0,sizeof(SPI2_Handle));
	SPI2_Handle.pSPIx = SPI2;
	SPI2_Handle.SPIConfig.SPI_BusConfig = SPI_CONFIG_FD;//0x00
	SPI2_Handle.SPIConfig. SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;//0x01
	SPI2_Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8;//0x03
	SPI2_Handle.SPIConfig.SPI_DFF = SPI_DFF_8BIT;//0x01
	SPI2_Handle.SPIConfig.SPI_CPOL = SPI_CPOL_HIGH;//0x01
	SPI2_Handle.SPIConfig.SPI_CPHA = SPI_CPHA_HIGH;//0x01
	SPI2_Handle.SPIConfig.SPI_SSM = SPI_SSM_SOFTWARE;//0x01
	SPI_Init(&SPI2_Handle);

}


int main(void)
{


	//GPIO_PClkControl(GPIOA,ENABLE);
	GPIO_Handle_t gp;
	memset(&gp,0,sizeof(gp));
	gp.pGPIOx = GPIOA;
	gp.GPIO_PinConfig.GPIO_PinMode = GPIO_PIN_0;
	gp.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	gp.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	gp.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GPIO_PClkControl(GPIOA,ENABLE);
	GPIO_Init(&gp);
	GPIO_IRQITConfig(IRQ_NO_EXTI0,ENABLE);


	SPI2_GPIOInits();

	SPI_PClkControl(SPI2,ENABLE);
	SPI2_Init();
	SPI_SSIConfig(SPI2,ENABLE);


	while(1)
	{
		int i=0;
	}

	/*
	while(1)
	{
		SPI_PeripheralControl(SPI2,ENABLE); // SPE (SPI ENABLE) bit in CR1 register
		char user_data[] = "1234";
		SPI_SendData(SPI2,(uint8_t*)user_data,strlen(user_data));

		SPI_PeripheralControl(SPI2,DISABLE); // DISBALE the SPE bit
	}
	*/
	return 0;
}

void EXTI0_IRQHandler(void)
{

	GPIO_ReadHandling(GPIO_PIN_0);

	SPI_PeripheralControl(SPI2,ENABLE); // SPE (SPI ENABLE) bit in CR1 register
	char user_data[] = "1234";
	SPI_SendData(SPI2,(uint8_t*)user_data,strlen(user_data));

	//SPI_PeripheralControl(SPI2,DISABLE); // DISBALE the SPE bit
}
