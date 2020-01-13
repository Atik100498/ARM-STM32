/*
 * stm32f407xx_i2c_driver.h
 *
 *  Created on: 12-Jan-2020
 *      Author: ATIQUE
 */

#ifndef INC_STM32F407XX_I2C_DRIVER_H_
#define INC_STM32F407XX_I2C_DRIVER_H_

#include "stm32f407xx.h"
typedef struct
{
	uint32_t I2C_SCLSpeed;
	uint8_t I2C_DeviceAddress;
	uint8_t I2C_ACKControl;
	uint16_t I2C_FMDutyCycle;
}I2C_Config_t;

typedef struct
{
	I2C_RegDef_t *pI2Cx;
	I2C_Config_t I2C_Config;
}I2C_Handle_t;

#define I2C_SCL_SPEED_SM				100000
#define I2C_SCL_SPEED_FM				400000

#define I2C_ACK_ENABLE					1
#define I2C_ACK_DISABLE					0

#define I2C_FM_DUTY_2					0
#define I2C_FM_DUTY_16_9				1


/*
 * @I2C clock control
 */
void I2C_PClkControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

/*
 * I2C initialize and de-initialize
 */
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);//PERIPHERAL RESET IN RCC WILL RESET ALL THE REGIATER OF THAT PORT WILL BE RESET


void I2C_MasterSendData(I2C_Handle_t *pI2CHandle,uint8_t *pTxBuffer,uint32_t len,uint8_t SlaveAddr);
/*
 * I2C peripheral control API
 */

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx,uint8_t EnorDi);


/*
 * interupt handdling
 */

/* Only for IRQ NUMBER */
void I2C_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi);
/* For IRQ number priorirty */
void I2C_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority);


#endif /* INC_STM32F407XX_I2C_DRIVER_H_ */
