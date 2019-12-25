/*
 * interrupt.c
 *
 *  Created on: 10-Nov-2019
 *      Author: ATIQUE
 */


#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"
#include<string.h>

void delay(void)
{
	for(uint32_t i =0;i<=500000; i++);
}
int main(void)
{

	GPIO_Handle_t pGPIO;
	/*
	 * this will set all the memory attributes of the pGPIO to desiered value
	 * func:  memset(arg[0],arg[1],arg[2])
	 *
	 * Library : string.h
	 * param arg[0] : address of the variable
	 * param arg[1] : desired value
	 * param arg[2] : size of the memory
	 */
	memset(&pGPIO,0,sizeof(pGPIO));
	pGPIO.pGPIOx = GPIOA;
	pGPIO.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_0;
	pGPIO.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_RT;//ERROR
	pGPIO.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	pGPIO.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;

	GPIO_PClkControl(GPIOA,ENABLE);
	GPIO_Init(&pGPIO);

	GPIO_Handle_t pLed;
	memset(&pLed,0,sizeof(pLed));

	pLed.pGPIOx = GPIOD;
	pLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	pLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	pLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	pLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	pLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;

	GPIO_PClkControl(GPIOD,ENABLE);
	GPIO_Init(&pLed);

	//IRQ config
	pLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	GPIO_Init(&pLed);

	//not nessesary in normal use
	//GPIO_IRQPriorityConfig(IRQ_NO_EXTI0,NVIC_IRQ_PRIORITY_15);
	GPIO_IRQITConfig(IRQ_NO_EXTI0,ENABLE);

	while(1)
	{
		GPIO_WritePin(GPIOD,GPIO_PIN_12,GPIO_PIN_RESET);
		GPIO_WritePin(GPIOD,GPIO_PIN_13,GPIO_PIN_RESET);

	}
	return 0;
}

/*
 * EXTI0 HANDDLER COPIED FROM THE START UP FILE
 */
void EXTI0_IRQHandler(void)
{
	//Handle the interrupt
	GPIO_ReadHandling(GPIO_PIN_0);

	uint8_t i=0;
	for(i = 0; i<=10;i++)
	{
	GPIO_WritePin(GPIOD,GPIO_PIN_12,GPIO_PIN_SET);
	GPIO_WritePin(GPIOD,GPIO_PIN_13,GPIO_PIN_RESET);
	delay();
	GPIO_WritePin(GPIOD,GPIO_PIN_13,GPIO_PIN_SET);
	GPIO_WritePin(GPIOD,GPIO_PIN_12,GPIO_PIN_RESET);
	delay();
	}


}
