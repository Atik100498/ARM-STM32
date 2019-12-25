/*
 * gpio_interrupt.c
 *
 *  Created on: 21-Dec-2019
 *      Author: ATIQUE
 */

#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"

void delay()
{
	for(uint32_t i = 0; i<=500000; i++);
}

int main(void)
{

	GPIO_Handle_t led;

	led.pGPIOx = GPIOD;
	led.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	led.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	led.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	led.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;

	GPIO_PClkControl(GPIOD,ENABLE);
	GPIO_Init(&led);

	//INTRRUPT PIN LED
	led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	GPIO_Init(&led);

	GPIO_Handle_t button;
	button.pGPIOx = GPIOA;
	button.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_0;
	button.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	button.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	button.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PD;
	button.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;

	GPIO_PClkControl(GPIOA,ENABLE);
	GPIO_Init(&button);

	//enable intrrupt
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI0,NVIC_IRQ_PRIORITY_15);
	GPIO_IRQITConfig(IRQ_NO_EXTI0,ENABLE);
	while(1)
	{
		GPIO_ToggleOutputPin(GPIOD,GPIO_PIN_13);
		delay();
	}
}

void EXTI0_IRQHandler(void)
{
	GPIO_ReadHandling(GPIO_PIN_0);
	GPIO_ToggleOutputPin(GPIOD,GPIO_PIN_12);
	delay();

}
