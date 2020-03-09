/*
 * stm32f407xx_dma_driver.c
 *
 *  Created on: 08-Mar-2020
 *      Author: ATIQUE
 */

#include "stm32f407xx.h"
#include "stm32f407xx_dma_driver.h"

//RCC clock
void DMAEnable(DMA_RegDef_t *pDMAx,uint8_t ENOrDi)
{
	if(ENOrDi == ENABLE)
	{
		if(pDMAx == DMA1)
		{
			DMA1_PCLK_EN();
		}
		if(pDMAx == DMA2)
		{
			DMA2_PCLK_EN();
		}
	}else
	{
		if(pDMAx == DMA1)
		{
			DMA1_PCLK_DI();
		}
		if(pDMAx == DMA2)
		{
			DMA2_PCLK_DI();
		}
	}
}
// DMA_EN (local enable)
void DMAPeripheralEnable(DMA_RegDef_t *pDMAx,uint8_t ENOrDi)
{
	if(ENOrDi == ENABLE)
	{
		pDMAx->S0CR |= (1<<0);
	}else
	{
		pDMAx->S0CR &= ~(1<<0);
	}
}
void DMAInit(DMA_Handle_t *pDMAHanlde)
{
	if(pDMAHanlde->pDMAConfig.StreamSelect == STREAM0)
	{
		pDMAHanlde->pDMAx->S0CR |= (pDMAHanlde->pDMAConfig.ChannelSelect << 25);
		pDMAHanlde->pDMAx->S0CR |= (pDMAHanlde->pDMAConfig.SelectPriority << 16);
		pDMAHanlde->pDMAx->S0CR |= (pDMAHanlde->pDMAConfig.PeripheralDataSize << 11);
		pDMAHanlde->pDMAx->S0CR |= (pDMAHanlde->pDMAConfig.MemoryIncrementEnable << 10);
		pDMAHanlde->pDMAx->S0CR |= (pDMAHanlde->pDMAConfig.PeripheralDataSize << 9);
		pDMAHanlde->pDMAx->S0CR |= (pDMAHanlde->pDMAConfig.DataFlowDirection << 6);
		pDMAHanlde->pDMAx->S0M0AR |= (pDMAHanlde->pDMAConfig.SourceAddress << 0);
		pDMAHanlde->pDMAx->S0PAR |= (pDMAHanlde->pDMAConfig.DestinationAddress << 0);
		pDMAHanlde->pDMAx->S0NDTR |= (pDMAHanlde->pDMAConfig.NumberOfTransaction << 0);
		pDMAHanlde->pDMAx->S0M0AR &= ~(1 << 2);
		//Enable direct mode interrupt
		//1.TCIE - Transfer complete interrupt enable
		//2. DMEIE - Direct mode interrupt enable
		pDMAHanlde->pDMAx->S0CR |= 0x12;
	}
}

void DMA_InterruptFlagClear(DMA_RegDef_t *pDMAx)
{
	pDMAx->LIFCR = 0;
	pDMAx->HIFCR = 0;
}
void DMA_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	/*
	 * Processor side configuration.
	 * Refer CORTEX M4 generic user guide.
	 * STM32F4 family has up to 82 IRQ numbers thus limited up to ISER2
	 */
	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			// program ISER0
			*NVIC_ISER0 |= (1<< IRQNumber); //Inerrupt set-enable register
		}
		else if(IRQNumber >31 && IRQNumber <64)
		{
			// program ISER1
			*NVIC_ISER1 |= (1<< (IRQNumber % 32));
		}
		else if(IRQNumber > 64 && IRQNumber <96)
		{
			// program ISER2
			*NVIC_ISER2 |= (1<< (IRQNumber % 64));
		}
	}
	else//CLEAR THE BIT / DISABLE THEM
	{
		if(IRQNumber <= 31)
		{
			// program ISER0
			*NVIC_ICER0 |= (1<< IRQNumber);
		}
		else if(IRQNumber >31 && IRQNumber <64)
		{
			// program ISER1
			*NVIC_ICER1 |= (1<< (IRQNumber % 32));
		}
		else if(IRQNumber > 64 && IRQNumber <96)
		{
			// program ISER2
			*NVIC_ICER1 |= (1<< (IRQNumber % 64));
		}
	}
}

void DMA_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = (8 * iprx_section) + (8 + NO_PR_NITS_IMPLEMENTED) ;
	*(NVIC_PR_BASE_ADDR + (iprx)) |= (IRQPriority<< shift_amount);
}
