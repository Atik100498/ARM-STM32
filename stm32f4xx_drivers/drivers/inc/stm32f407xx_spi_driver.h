/*
 * stm32f407xx_spi_driver.h
 *
 *  Created on: 11-Dec-2019
 *      Author: ATIQUE
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_

#include "stm32f407xx.h"

/*
 * Spi configuration structure
 */
typedef struct
{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
}SPI_Config_t;

/*
 * Handle Structure for SPI
 */

typedef struct
{
	SPI_RegDef_t *pSPIx;
	SPI_Config_t SPIConfig;
	uint8_t	*pTxBuffer;
	uint8_t	*pRxBuffer;
	uint32_t Txlen;
	uint32_t Rxlen;
	uint8_t TxState;
	uint8_t RxState;
}SPI_Handle_t;


/*
 * @SPI DeviceMode
 */

#define SPI_DEVICE_MODE_MASTER				1
#define SPI_DEVICE_MODE_SLAVE				0

/*
 * @SPI_BusConfig
 */

#define SPI_CONFIG_FD						1
#define SPI_CONFIG_HD						2
#define SPI_CONFIG_SIMPLEX_TXONLY			3
#define SPI_CONFIG_SIMPLEX_RXONLY			4

/*
 * @SPI_SclkSpeed
 */
#define SPI_SCLK_SPEED_DIV2					0
#define SPI_SCLK_SPEED_DIV4					1
#define SPI_SCLK_SPEED_DIV8					2
#define SPI_SCLK_SPEED_DIV16				3
#define SPI_SCLK_SPEED_DIV32				4
#define SPI_SCLK_SPEED_DIV64				5
#define SPI_SCLK_SPEED_DIV128				6
#define SPI_SCLK_SPEED_DIV256				7

/*
 * @SPI_DFF
 */

#define SPI_DFF_8BIT						0
#define SPI_DFF_16BIT						1

/*
 * @SPI_CPOL
 */
#define SPI_CPOL_LOW						0
#define SPI_CPOL_HIGH						1

/*
 * @SPI_CPHA
 */
#define SPI_CPHA_LOW						0
#define SPI_CPHA_HIGH						1

/*
 * @SPI_SSM
 */
#define SPI_SSM_SOFTWARE					1
#define SPI_SSM_HARDWARE					0

/*
 * SPI Related flag
 */

#define SPI_TXE_FLAG						(1 << SPI_SR_TXE)
#define SPI_RXEN_FLAG						(1 << SPI_SR_RXNE)
#define SPI_BUSY_FLAG						(1 << SPI_SR_BSY)

/*
 * Application state
 */

#define SPI_READY							0
#define SPI_BUSY_IN_RX						1
#define SPI_BUSY_IN_TX						2
/*
 * @API supports fir SPI
 */

/*
 * @SPI clock control
 */
void SPI_PClkControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

/*
 * SPI initialize and de-initialize
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);//PERIPHERAL RESET IN RCC WILL RESET ALL THE REGIATER OF THAT PORT WILL BE RESET

/*
 * SPI peripheral control API
 */

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx,uint8_t EnorDi);

/*
 * SSI config when SSM = 1
 * managing the NSS pin using software
 * SSM = 1 , SSI = 0,MODEF(Mode Fault) error arises
 */

void SPI_SSIConfig(SPI_RegDef_t *pSPIx,uint8_t EnorDi);

/*
 * SSOE when SSM = 0
 * SSOE (Slave Select Output enable)
 * managing the NSS output using hardware configuration
 */

void SPI_SSOEConfig(SPI_RegDef_t *pSPIx,uint8_t EnorDi);
/*
 * Data send and receive.
 *
 * There are 3 types of data send and receive in SPI,CAN,I2C.
 * 1. Polling			(Blocking type)
 * 2. Interrupt			(Non-Blocking type)
 * 3. DMA
 */

void SPI_SendData(SPI_RegDef_t *pSPIx,uint8_t *pTxBuffer, uint32_t len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx,uint8_t *pRxBuffer, uint32_t len);

/*
 * interrupt based API
 */
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPI_Handle,uint8_t *pTxBuffer, uint32_t len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPI_Handle,uint8_t *pRxBuffer, uint32_t len);
/*
 * interupt handdling
 */

/* Only for IRQ NUMBER */
void SPI_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi);
/* For IRQ number priorirty */
void SPI_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);

#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
