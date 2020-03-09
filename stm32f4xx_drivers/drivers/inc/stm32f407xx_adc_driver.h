/*
 * stm32f407xx_adc_driver.h
 *
 *  Created on: 15-Feb-2020
 *      Author: ATIQUE
 */

#ifndef INC_STM32F407XX_ADC_DRIVER_H_
#define INC_STM32F407XX_ADC_DRIVER_H_


#include "stm32f407xx.h"

typedef struct
{
	uint8_t ADC_Resolution;
	uint8_t ADC_DataAlignment;
	uint8_t ADC_ChannelSelect;
	uint8_t ADC_ChannelSampling;
	uint8_t ADC_ContinuousENABLE;
	uint8_t ADC_NoofConvertion;
}ADC_Config_t;

typedef struct
{
	ADC_RegDef_t *pADCx;
	ADC_Config_t PConfig;
}ADC_Handle_t;



void ADC_PClkControl(ADC_RegDef_t *pADCx, uint8_t EnorDi);
void ADC_Init(ADC_Handle_t *pADCHandle);
void ADC_DeInit(ADC_RegDef_t *pADCx);

uint16_t ADC_ReadCount(ADC_Handle_t *pHandle);

void ADC_PeripheralControl(ADC_RegDef_t *pADCx,uint8_t EnorDi);

#endif /* INC_STM32F407XX_ADC_DRIVER_H_ */
