/*
 * led_blink.c
 *
 *  Created on: 29-Nov-2019
 *      Author: ATIQUE
 */


#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"

void delay()
{

	for(uint32_t i = 0; i <= 500000 ; i++);
}
int main(void)
{
	GPIO_Handle_t xGPIO;

	xGPIO.pGPIOx = GPIOD;
	xGPIO.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	xGPIO.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	xGPIO.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	xGPIO.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;


	GPIO_PClkControl(GPIOD,ENABLE);

	xGPIO.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	GPIO_Init(&xGPIO);
	xGPIO.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	GPIO_Init(&xGPIO);
	xGPIO.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_14;
	GPIO_Init(&xGPIO);
	xGPIO.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_15;
	GPIO_Init(&xGPIO);


	while(1)
	{

		GPIO_WritePin(GPIOD,GPIO_PIN_12,GPIO_PIN_SET);
		GPIO_WritePin(GPIOD,GPIO_PIN_13,GPIO_PIN_SET);
		GPIO_WritePin(GPIOD,GPIO_PIN_14,GPIO_PIN_SET);
		GPIO_WritePin(GPIOD,GPIO_PIN_15,GPIO_PIN_SET);
		delay();
		GPIO_WritePin(GPIOD,GPIO_PIN_12,GPIO_PIN_RESET);
		GPIO_WritePin(GPIOD,GPIO_PIN_13,GPIO_PIN_RESET);
		GPIO_WritePin(GPIOD,GPIO_PIN_14,GPIO_PIN_RESET);
		GPIO_WritePin(GPIOD,GPIO_PIN_15,GPIO_PIN_RESET);
		delay();

	}
	return 0;
}
