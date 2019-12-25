/*
 * 001ledtoggle.c
 *
 *  Created on: 08-Nov-2019
 *      Author: ATIQUE
 */

#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"
#include<string.h>

void delay()
{
	for(uint32_t i = 0;i<=500000; i++);
}
int main(void)
{
	GPIO_Handle_t GpioLed;
	GPIO_PClkControl(GPIOD,ENABLE);

	memset(&GpioLed,0,sizeof(GpioLed));
	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;


	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_15;
	GPIO_Init(&GpioLed);
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_14;
	GPIO_Init(&GpioLed);
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	GPIO_Init(&GpioLed);
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	GPIO_Init(&GpioLed);

	GPIO_Handle_t but;
	memset(&but,0,sizeof(but));
	but.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	but.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_0;
	//but.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	but.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	but.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	but.pGPIOx=  GPIOA;
	GPIO_PClkControl(GPIOA,ENABLE);
	GPIO_Init(&but);

	while(1)
	{
		if(GPIO_ReadPin(GPIOA,GPIO_PIN_0) == 1)
		{
		GPIO_ToggleOutputPin(GPIOD,GPIO_PIN_15);
		delay();
		}
	}
	return 0;
}
