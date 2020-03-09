/*
 * i2c_send_string.c
 *
 *  Created on: 19-Jan-2020
 *      Author: ATIQUE
 */


#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_i2c_driver.h"
#include "stm32f407xx_adc_driver.h"
#include<string.h>

void delay()
{
	for(uint16_t i = 0;i<=50000;i++);
}
void button(void)
{
	GPIO_Handle_t push;
	push.pGPIOx = GPIOA;
	push.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_0;
	push.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	push.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	push.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	push.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;

	GPIO_PClkControl(GPIOA,ENABLE);
	GPIO_Init(&push);
}

void com_but_pin(void)
{
	GPIO_Handle_t com_but;
	com_but.pGPIOx = GPIOB;
	com_but.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	com_but.GPIO_PinConfig.GPIO_PinAltFunMode = AF4;
	com_but.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	com_but.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	com_but.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;

	GPIO_PClkControl(GPIOB,ENABLE);
	com_but.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_6;//SCL
	GPIO_Init(&com_but);

	com_but.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_7;//SDA
	GPIO_Init(&com_but);
}

int main(void)
{
	button();
	com_but_pin();

	//uint16_t count = 0;
	int i=0;
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
	adc.PConfig.ADC_Resolution = 0x0;
	adc.PConfig.ADC_ContinuousENABLE = 0;
	adc.PConfig.ADC_NoofConvertion = 0;
	adc.PConfig.ADC_ChannelSelect = 0x01;
	adc.PConfig.ADC_ChannelSampling = 0x0;

	ADC_PClkControl(ADC1,ENABLE);
	ADC_Init(&adc);

	I2C_Handle_t com;
	com.pI2Cx = I2C1;
	com.I2C_Config.I2C_ACKControl = ENABLE;
	com.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

	I2C_PClkControl(I2C1,ENABLE);
	I2C_Init(&com);

	uint8_t slave_addr = 0x68;
	char send_data[10];
	union
	{
		uint16_t count;
		char send_data[3];
	}a;
	I2C_PeripheralControl(I2C1,ENABLE);
	while(1)
	{
		ADC_PeripheralControl(ADC1,ENABLE);
		a.count = ADC_ReadCount(&adc);
		ADC_PeripheralControl(ADC1,DISABLE);
		//a.count = 200;
		I2C_MasterSendData(&com,(uint8_t*)a.send_data,strlen(send_data),slave_addr);
		delay();
		delay();
		delay();

	}
	return 0;
}
