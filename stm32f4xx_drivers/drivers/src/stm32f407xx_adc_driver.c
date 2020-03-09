/*
 * stm32f407xx_adc_driver.c
 *
 *  Created on: 15-Feb-2020
 *      Author: ATIQUE
 */

#include "stm32f407xx.h"
#include "stm32f407xx_adc_driver.h"

void ADC_PClkControl(ADC_RegDef_t *pADCx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pADCx == ADC1)
		{
			ADC1_PCLK_EN();
		}

	}else
	{
		if(pADCx == ADC1)
		{
			ADC1_PCLK_DI();
		}
	}

}
void ADC_Init(ADC_Handle_t *pADCHandle)
{
	//uint32_t tempreg=0;
	pADCHandle->pADCx->CR1 |= (pADCHandle->PConfig.ADC_Resolution << 24);

	pADCHandle->pADCx->CR2 |= (pADCHandle->PConfig.ADC_DataAlignment << 11);

	pADCHandle->pADCx->SQR1 |= (pADCHandle->PConfig.ADC_NoofConvertion << 20);

	/* for now only convertion 1 */
	pADCHandle->pADCx->SQR3 |=(pADCHandle->PConfig.ADC_ChannelSelect << 0);

	pADCHandle->pADCx->SMPR2 |=(pADCHandle->PConfig.ADC_ChannelSampling << 0);
	//enable scan bit
	pADCHandle->pADCx->CR1 |=(1<<8);

}
void ADC_DeInit(ADC_RegDef_t *pADCx)
{
	if(pADCx == ADC1)
	{
		ADC1_REG_RESET();
	}
}

void ADC_PeripheralControl(ADC_RegDef_t *pADCx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
		{
			pADCx->CR2 |= (1 << 0);
		}
		else
		{
			pADCx->CR2 &= ~(1 << 0);
		}
}

uint16_t ADC_ReadCount(ADC_Handle_t *pHandle)
{
	//start converstion
	pHandle->pADCx->CR2 |=(1 << 30);
	while(!(pHandle->pADCx->SR & (1<<4)));//check if started
	while(!(pHandle->pADCx->SR & (1<<1)));
	return pHandle->pADCx->DR;
}
