/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: 07-Nov-2019
 *      Author: ATIQUE
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include "stm32f407xx.h"


typedef struct
{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFunMode;
	uint8_t GPIO_PinSpeed;

}GPIO_PinConfig;

/*
 * This is for handling all the parameters of the GPIO
 */

typedef struct
{
	GPIO_RegDef_t *pGPIOx; /*This is for port name and initializes the gpio registers and their address */
	GPIO_PinConfig GPIO_PinConfig;/*GPIOx register configuration*/

}GPIO_Handle_t;

/*POSSIBLE GPIO PORT PIN NUMBERS*/
#define GPIO_PIN_0					0
#define GPIO_PIN_1					1
#define GPIO_PIN_2					2
#define GPIO_PIN_3					3
#define GPIO_PIN_4					4
#define GPIO_PIN_5					5
#define GPIO_PIN_6					6
#define GPIO_PIN_7					7
#define GPIO_PIN_8					8
#define GPIO_PIN_9					9
#define GPIO_PIN_10					10
#define GPIO_PIN_11					11
#define GPIO_PIN_12					12
#define GPIO_PIN_13					13
#define GPIO_PIN_14					14
#define GPIO_PIN_15					15

/* GPIO POSSIBLE MODE */
#define GPIO_MODE_IN				0
#define GPIO_MODE_OUT				1
#define GPIO_MODE_ALTFN				2
#define GPIO_MODE_ANALOG			3
#define GPIO_MODE_IT_FT				4
#define GPIO_MODE_IT_RT				5
#define GPIO_MODE_IT_RFT			6


/*POSSBLE PIN OUTPUT TYPES*/
#define GPIO_OP_TYPE_PP				0
#define GPIO_OP_TYPE_OD				1

/*POSSIBLE OUTPUT SPEED*/

#define GPIO_SPEED_LOW				0
#define GPIO_SPEED_MEDIUM			1
#define GPIO_SPEED_HIGH				2

/*POSSIBLE PULL UP/DOWM CONFIGURATION*/
#define GPIO_NO_PUPD				0
#define GPIO_PIN_PU					1
#define GPIO_PIN_PD					2

/*POSSBLE ALTERNATE FUCNTION*/

#define AF0							0
#define AF1							1
#define AF2							2
#define AF3							3
#define AF4							4
#define AF5							5
#define AF6							6
#define AF7							7
#define AF8							8
#define AF9							9
#define AF10						10
#define AF11						11
#define AF12						12
#define AF13						13
#define AF14						14
#define AF15						15

/*API*/

void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIO);//PERIPHERAL RESET IN RCC WILL RESET ALL THE REGIATER OF THAT PORT WILL BE RESET


void GPIO_PClkControl(GPIO_RegDef_t *pGPIO, uint8_t EnorDi);


uint8_t GPIO_ReadPin(GPIO_RegDef_t *pGPIO, uint8_t PinNumber);
uint16_t GPIO_ReadPort(GPIO_RegDef_t *pGPIO);
void GPIO_WritePin(GPIO_RegDef_t *pGPIO, uint8_t PinNumber, uint8_t Value);
void GPIO_WritePort(GPIO_RegDef_t *pGPIO, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIO, uint8_t PinNumber);


/* Only for IRQ NUMBER */
void GPIO_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi);
/* For IRQ number priorirty */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority);
void GPIO_ReadHandling(uint8_t PinNumber);


#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
