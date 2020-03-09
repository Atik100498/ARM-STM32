/*
 * i2c_recieve_string.c
 *
 *  Created on: 19-Jan-2020
 *      Author: ATIQUE
 */


/*
 * i2c_send_string.c
 *
 *  Created on: 19-Jan-2020
 *      Author: ATIQUE
 */


#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_i2c_driver.h"
#include<string.h>
#include<stdio.h>

extern void initialise_monitor_handles();

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
	initialise_monitor_handles();
	printf("SEMIHOSTING INIT!");
	uint8_t command_code;
	char rec_len;
	uint8_t slave_addr = 0x68;
	char rec_buf[32];

	button();
	com_but_pin();

	I2C_Handle_t com;
	com.pI2Cx = I2C1;
	com.I2C_Config.I2C_ACKControl = ENABLE;
	com.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

	I2C_PClkControl(I2C1,ENABLE);
	I2C_Init(&com);
	I2C_PeripheralControl(I2C1,ENABLE);
	 I2C_ManageAck(com.pI2Cx,ENABLE);

	 command_code = 0x51;


	while(1)
	{

		while(GPIO_ReadPin(GPIOA,GPIO_PIN_0) == GPIO_PIN_SET)
		{
			//I2C_MasterSendData(&com,&a,1,slave_addr);//correct
			//delay();
			//rec_len
			I2C_MasterSendData(&com,&command_code,1,slave_addr);//correct
			I2C_MasterRecieveData(&com,&rec_len,1,slave_addr);//correct

			command_code = 0x52;

			I2C_MasterSendData(&com,&command_code,1,slave_addr);//correct
			I2C_MasterRecieveData(&com,&rec_buf,rec_len,slave_addr);


		}



	}

	return 0;
}
