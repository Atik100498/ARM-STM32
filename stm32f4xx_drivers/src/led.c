/*
 * led.c
 *
 *  Created on: 22-Dec-2019
 *      Author: ATIQUE
 */


#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"


void delay()
{
	for(int i = 0; i<=500000; i++);
}
int main(void)
{

	GPIO_Handle_t led;
	led.pGPIOx = GPIOD;
	led.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	led.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	led.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	led.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;


	GPIO_PClkControl(GPIOD,ENABLE);

	led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_15;
	GPIO_Init(&led);
	led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_14;
	GPIO_Init(&led);
	led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	GPIO_Init(&led);
	led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	GPIO_Init(&led);


	while(1)
	{
		GPIO_WritePin(GPIOD,GPIO_PIN_15,GPIO_PIN_SET);
		delay();
		GPIO_WritePin(GPIOD,GPIO_PIN_14,GPIO_PIN_SET);
		delay();
		GPIO_WritePin(GPIOD,GPIO_PIN_13,GPIO_PIN_SET);
		delay();
		GPIO_WritePin(GPIOD,GPIO_PIN_12,GPIO_PIN_SET);
		delay();
		GPIO_WritePin(GPIOD,GPIO_PIN_15,GPIO_PIN_RESET);
		delay();
		GPIO_WritePin(GPIOD,GPIO_PIN_14,GPIO_PIN_RESET);
		delay();
		GPIO_WritePin(GPIOD,GPIO_PIN_13,GPIO_PIN_RESET);
		delay();
		GPIO_WritePin(GPIOD,GPIO_PIN_12,GPIO_PIN_RESET);
		delay();
	}
	return 0;
}
