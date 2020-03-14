/*
 * stm32f407xx_dma_driver.h
 *
 *  Created on: 08-Mar-2020
 *      Author: ATIQUE
 */

#ifndef INC_STM32F407XX_DMA_DRIVER_H_
#define INC_STM32F407XX_DMA_DRIVER_H_


#include "stm32f407xx.h"

typedef struct
{
	uint8_t StreamSelect;
	uint8_t ChannelSelect; //CHSEL
	uint8_t PeripheralDataSize;//PSIZE destination and peripheral size same in direct mode
	uint8_t MemoryIncrementEnable;//MINC
	uint8_t PeripheralIncrementEnable;//PINC
	uint8_t DataFlowDirection;//DIR
	uint8_t DirectModeEnable;//DMDIS = 0 IN SxFCR
	uint32_t SourceAddress;
	uint32_t DestinationAddress;
	uint16_t NumberOfTransaction;//SxNDTR
	uint8_t EnableAllInterrupt;//TCIE = 1,TEIE = 1, DMEIE = 1
	uint8_t SelectPriority;
	uint8_t DMATransferCompeleteFLag;
	uint8_t DMADirectModeErrorFlag;
}DMA_Config_t;

typedef struct
{
	DMA_RegDef_t *pDMAx;//DMA1,DMA2
	DMA_Config_t pDMAConfig;
}DMA_Handle_t;


void DMAEnable(DMA_RegDef_t *pDMAx,uint8_t ENOrDi);//RCC clock
void DMAPeripheralEnable(DMA_RegDef_t *pDMAx,uint8_t ENOrDi);// DMA_EN (local enable)
void DMAInit(DMA_Handle_t *pDMAHanlde);


void DMA_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi);
/* For IRQ number priorirty */
void DMA_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority);

void DMA_InterruptFlagClear(DMA_RegDef_t *pDMAx);

void DMAInterruptHandle(DMA_Handle_t *pDMAHandle);

void DMA1_Stream0_IRQHandler(void);
void DMA1_Stream1_IRQHandler(void);
void DMA1_Stream2_IRQHandler(void);
void DMA1_Stream3_IRQHandler(void);
void DMA1_Stream4_IRQHandler(void);
void DMA1_Stream5_IRQHandler(void);
void DMA1_Stream6_IRQHandler(void);
void DMA1_Stream7_IRQHandler(void);

void DMA2_Stream0_IRQHandler(void);
void DMA2_Stream1_IRQHandler(void);
void DMA2_Stream2_IRQHandler(void);
void DMA2_Stream3_IRQHandler(void);
void DMA2_Stream4_IRQHandler(void);
void DMA2_Stream5_IRQHandler(void);
void DMA2_Stream6_IRQHandler(void);
void DMA2_Stream7_IRQHandler(void);


#define STREAM0			0
#define STREAM6			6
#define STREAM7			7

#define PRIORITY_VH		3
#define PRIORITY_H		2
#define PRIORITY_M		1
#define PRIORITY_L		0

#define PSIZE_8BIT		0
#define PSIZE_16BIT		1
#define PSIZE_32BIT		2

#define DIR_P2M			0
#define DIR_M2P			1
#define DIR_M2M			2

#endif /* INC_STM32F407XX_DMA_DRIVER_H_ */
