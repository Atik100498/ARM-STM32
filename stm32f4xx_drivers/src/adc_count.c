/*
 * adc_count.c
 *
 *  Created on: 15-Feb-2020
 *      Author: ATIQUE
 */

#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_adc_driver.h"


int main(void)
{
	uint16_t count = 0;
	GPIO_Handle_t adcPin;
	adcPin.pGPIOx = GPIOA;
	adcPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_1;
	adcPin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ANALOG;
	adcPin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	adcPin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;

	GPIO_PClkControl(GPIOA,ENABLE);
	GPIO_Init(&adcPin);

	ADC_Handle_t adc;
	adc.pADCx = ADC1;
	adc.PConfig.ADC_Resolution = 0x01;
	adc.PConfig.ADC_ContinuousENABLE = 0;
	adc.PConfig.ADC_NoofConvertion = 0;
	adc.PConfig.ADC_ChannelSelect = 0x01;
	adc.PConfig.ADC_ChannelSampling = 0x0;

	ADC_PClkControl(ADC1,ENABLE);
	ADC_Init(&adc);



	while(1)
	{
		ADC_PeripheralControl(ADC1,ENABLE);
		count = ADC_ReadCount(&adc);
		ADC_PeripheralControl(ADC1,DISABLE);
		//(void)count;
	}
	return 0;
}
