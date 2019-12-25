/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: 11-Dec-2019
 *      Author: ATIQUE
 */

#include "stm32f407xx_spi_driver.h"
#include "stm32f407xx.h"

/*
 * Enabling the SPI through the SPE bit in CR1 register.
 * before making this bit
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
		{
			pSPIx->CR1 |= (1 << SPI_CR1_SPE); // EnorDi - ENABLE, DISABLE
		}
		else
		{
			pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
		}
}


void SPI_SSIConfig(SPI_RegDef_t *pSPIx,uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SSI); // EnorDi - ENABLE, DISABLE
	}
	else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}

void SPI_SSOEConfig(SPI_RegDef_t *pSPIx,uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE); // EnorDi - ENABLE, DISABLE
	}
	else
	{
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}


void SPI_PClkControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}
		else if (pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}
		else if(pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		}
	}
	else if(EnorDi == DISABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_DI();
		}
		else if (pSPIx == SPI2)
		{
			SPI2_PCLK_DI();
		}
		else if(pSPIx == SPI3)
		{
			SPI3_PCLK_DI();
		}
	}
}

/*
 * SPI initialize and de-initialize
 */
void SPI_Init(SPI_Handle_t *pSPIHandle)
{

	//FOR CR1 REGISTER BIT USE  device specific library
	uint32_t tempreg = 0;


	//Device mode master or slave
	tempreg |= (pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR);

	/*
	 * Bus configuration
	 * 1. FULL DUPLEX
	 * 2. HALF DUPLEX
	 * 3.SIMPLEX Tx only
	 * 4.SIMPLEX Rx ONLY
	 */

	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_CONFIG_FD)
	{
		tempreg &= ~( 1<< SPI_CR1_BIDIMODE);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_CONFIG_HD)
	{
		tempreg |= (1 << SPI_CR1_BIDIMODE);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_CONFIG_SIMPLEX_RXONLY)
	{
		tempreg &= ~( 1<< SPI_CR1_BIDIMODE); //BIDIMODE RESET FOR 2 LINE COMM.
		tempreg |= (1 << SPI_CR1_RXONLY); //RXONLY SET
	}
	/*
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_CONFIG_SIMPLEX_TXONLY)
	{
		tempreg |= ~(1 << SPI_CR1_BIDIMODE);
		tempreg |= ~(1 << SPI_CR1_RXONLY);
	}
    */
	//BAUD RATE
	tempreg |= (pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR);

	//DATA FRAME FORMAT
	tempreg |= (pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF);

	//CLOCK POLARITY

	tempreg |= (pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL);

	//CLOCK PHASE
	tempreg |= (pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA);

	// SOFTWARE SLAVE MANAGEMENT
	tempreg |= (pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM);


	pSPIHandle->pSPIx->CR1 = tempreg;
	//tempreg = 0;
}


void SPI_DeInit(SPI_RegDef_t *pSPIx)//PERIPHERAL RESET IN RCC WILL RESET ALL THE REGIATER OF THAT PORT WILL BE RESET
{
	if(pSPIx == SPI1)
	{
		SPI1_REG_RESET();
	}
	else if (pSPIx == SPI2)
	{
		SPI2_REG_RESET();
	}
	else if(pSPIx == SPI3)
	{
		SPI3_REG_RESET();
	}


}

/*
 * Data send and receive.
 *
 * There are 3 types of data send and receive in SPI,CAN,I2C.
 * 1. Polling			(Blocking type)
 * 2. Interrupt			(Non-Blocking type)
 * 3. DMA
 */

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}
/*
 * Note - This is blocking call (Polling type)
 */
void SPI_SendData(SPI_RegDef_t *pSPIx,uint8_t *pTxBuffer, uint32_t len)
{
	while(len > 0)
	{
		//1. wait until TXE is set
		while(SPI_GetFlagStatus(pSPIx,SPI_TXE_FLAG) == FLAG_RESET);

		// 2. Check the DFF bit field in CR1 for 8 bit or 16 bit
		if(pSPIx->CR1 & (1<<SPI_CR1_DFF))
		{
			//16 bit DFF
			//1.Load data into DFF
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			len--;
			len--;
			(uint16_t*)pTxBuffer ++;
		}else
		{
			pSPIx->DR = *(pTxBuffer);
			len--;
			pTxBuffer ++;
		}



	}
}
void SPI_ReceiveData(SPI_RegDef_t *pSPIx,uint8_t *pRxBuffer, uint32_t len)
{

	while(len > 0)
	{
		while(SPI_GetFlagStatus(pSPIx,SPI_SR_RXNE) == FLAG_RESET);

		if(pSPIx->CR1 & (1 << SPI_CR1_DFF))
		{
			*((uint16_t*)pRxBuffer) = pSPIx->DR;
			len -= 2;
			(uint16_t*)pRxBuffer ++;
		}
		else
		{
			*(pRxBuffer) = pSPIx->DR;
			len -= 1;
			pRxBuffer ++;
		}
	}
}

/*
 * interupt handdling
 */

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPI_Handle,uint8_t *pTxBuffer, uint32_t len)
{
	uint8_t state = pSPI_Handle->TxState;
	if(state != SPI_BUSY_IN_TX)
	{
		pSPI_Handle->pTxBuffer = pTxBuffer;
		pSPI_Handle->Txlen = len;

		pSPI_Handle->TxState = SPI_BUSY_IN_TX;
		//Enable the interrupt
		pSPI_Handle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);
		//Now, data transmission will be handled by the ISR of SPIx
	}

	return state;
}

uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPI_Handle,uint8_t *pRxBuffer, uint32_t len)
{
	uint8_t state = pSPI_Handle->RxState;
		if(state != SPI_BUSY_IN_RX)
		{
			pSPI_Handle->pRxBuffer = pRxBuffer;
			pSPI_Handle->Rxlen = len;

			pSPI_Handle->RxState = SPI_BUSY_IN_RX;
			//Enable the interrupt
			pSPI_Handle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);
			//Now, the reception will be handled by ISR of SPIx
		}

		return state;
}
/* Only for IRQ NUMBER */
void SPI_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi)
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

void SPI_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = (8 * iprx_section) + (8 + NO_PR_NITS_IMPLEMENTED) ;
	*(NVIC_PR_BASE_ADDR + (iprx)) |= (IRQPriority<< shift_amount);
}
void SPI_IRQHandling(SPI_Handle_t *pHandle)
{

	//check for flag
	uint8_t temp1, temp2;
	temp1 =  pHandle->pSPIx->SR & (1 << SPI_SR_TXE);
	temp2 =  pHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);

	if(temp1 && temp2)
	{
		//spi_txe_handle();
	}

	temp1 =  pHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
	temp2 =  pHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);

	if(temp1 && temp2)
	{
		//spi_rxne_handle();
	}

}
