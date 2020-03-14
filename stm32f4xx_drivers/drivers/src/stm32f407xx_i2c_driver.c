/*
 * stm32f407xx_i2c_driver.c
 *
 *  Created on: 12-Jan-2020
 *      Author: ATIQUE
 */

#include "stm32f407xx.h"
#include  "stm32f407xx_i2c_driver.h"


void I2C_PClkControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_EN();
		}
		else if (pI2Cx == I2C2)
		{
			I2C2_PCLK_EN();
		}
		else if(pI2Cx == I2C3)
		{
			I2C3_PCLK_EN();
		}
	}
	else if(EnorDi == DISABLE)
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_DI();
		}
		else if (pI2Cx == I2C2)
		{
			I2C2_PCLK_DI();
		}
		else if(pI2Cx == I2C3)
		{
			I2C3_PCLK_DI();
		}
	}
}

uint16_t AHB_PreScalar[8] = {2,4,16,64,128,256,512};
uint8_t APB1_PreScalar[4] = {2,4,8,16};
uint32_t RCC_SystemClock()
{
	uint32_t pclk1,Systemclk;
	uint8_t clkscr,temp,ahbps,apb1ps;

	clkscr = ((RCC->CFGR >> 2) & 0x03); // geting bit3 and bit2 ans masking them with 0011;

	if(clkscr == 0) // HSI
	{
		Systemclk = 16000000U;
	}
	else if(clkscr == 1)//HSE
	{
		Systemclk = 8000000U;
	}
	else
	{
		//pll clock selection
		// not implimenting
	}

	temp = ((RCC->CFGR >> 4) & 0x0F);

	if(temp > 8)
	{
		ahbps = 1;
	}else if(temp >= 8)
	{
		ahbps = AHB_PreScalar[temp - 8];
	}

	//find the apb1 pre-scalar

	temp = ((RCC->CFGR >> 10) & 0x07);

	if(temp < 4)
	{
		apb1ps = 1;
	}else if( temp >= 4)
	{
		apb1ps = APB1_PreScalar[temp-4];
	}

	pclk1 = (Systemclk/ahbps)/apb1ps;
	return pclk1;
}
void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	uint32_t tempreg = 0;
	//ENABLE ACKING
	tempreg |= (pI2CHandle->I2C_Config.I2C_ACKControl << I2C_CR1_ACK);

	// SETTIG THE FERQ BIT IN CR2 -> SET IT TO THE APB1 BUS CLOCK FREQUENCY
	pI2CHandle->pI2Cx->CR1 = tempreg;
	tempreg = 0;
	tempreg = RCC_SystemClock()/1000000U;
	pI2CHandle->pI2Cx->CR2 |= (tempreg & 0x3F);
	tempreg = 0;
	tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAddress;
	tempreg |= (1 << 14);
	pI2CHandle->pI2Cx->OAR1 |= (tempreg << 1);


	//configuring the CCR
	tempreg = 0;
	uint16_t ccr_value = 0;
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		ccr_value = (RCC_SystemClock()/(pI2CHandle->I2C_Config.I2C_SCLSpeed * 2));
		pI2CHandle->pI2Cx->CCR |= (ccr_value & 0x0FFF);
	}else
	{
		//configure FM mode
		pI2CHandle->pI2Cx->CCR |= (1 << 15); // SET F/S (BIT 15) of ccr register

		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			//config the DUTY bit in CCR register
			pI2CHandle->pI2Cx->CCR |= (I2C_FM_DUTY_2 << I2C_CCR_DUTY);
			// set the CCR bits
			ccr_value = (RCC_SystemClock()/(pI2CHandle->I2C_Config.I2C_SCLSpeed * 3));
			pI2CHandle->pI2Cx->CCR |= (ccr_value & 0x0FFF);
		}else
		{
			//config the DUTY bit in CCR register
			pI2CHandle->pI2Cx->CCR |= (I2C_FM_DUTY_2 << I2C_CCR_DUTY);
			// set the CCR bits
			ccr_value = (RCC_SystemClock()/(pI2CHandle->I2C_Config.I2C_SCLSpeed * 25));
			pI2CHandle->pI2Cx->CCR |= (ccr_value & 0x0FFF);
		}
	}
	//configuring TRISE
	tempreg = 0;
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		//uint8_t trise;
		tempreg = (RCC_SystemClock()/1000000U)+1;
	}else
	{
		tempreg = (RCC_SystemClock() * 300/1000000U)+1;
	}

	pI2CHandle->pI2Cx->TRISE = (tempreg & 0x3F);
}
void I2C_DeInit(I2C_RegDef_t *pI2Cx)//PERIPHERAL RESET IN RCC WILL RESET ALL THE REGIATER OF THAT PORT WILL BE RESET
{
	if(pI2Cx == I2C1)
	{
		I2C1_REG_RESET();
	}
	else if (pI2Cx == I2C2)
	{
		I2C2_REG_RESET();
	}
	else if(pI2Cx == I2C3)
	{
		I2C3_REG_RESET();
	}


}

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_START); // GENERATED ONLY WHEN PE = 0 (I2C ENABLE BIT)
}

static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_STOP); // GENERATED ONLY WHEN PE = 0 (I2C ENABLE BIT)
	while(!(pI2Cx->CR1 & (1<<I2C_CR1_STOP)));
}

static void I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2Cx,uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr &= ~(1 << 0); // clearing the bit 0 for Write operation
	pI2Cx->DR = SlaveAddr;
}

static void I2C_ExecuteAddressPhase_ReadData(I2C_RegDef_t *pI2Cx,uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr |= (1 << 0); // set the bit 0 for Read operation
	pI2Cx->DR = SlaveAddr;
}

static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx)
{
	uint16_t dummy = pI2Cx->SR1;
	dummy = pI2Cx->SR2;
	(void)dummy;
}

void I2C_ManageAck(I2C_RegDef_t *pI2Cx,uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
	}
	else
	{
		pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
	}
}
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle,uint8_t *pTxBuffer,uint32_t len,uint8_t SlaveAddr)
{
	//1. Generate start condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2.confirm that start generation is completed by checking the SB flag in the SR1
	while(!(pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_SB)));

	//3. send the address of the slave with r/w bit to w(0) (total 8bit)
	I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx,SlaveAddr);

	//4. confirm that the address pahse is completed by checking the AADR flag in the SR1
	while(!(pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_ADDR)));

	//5. clear the ADDR flag according to software sequence
	I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

	//6. send data untill len becomes 0
	while(len > 0)
	{
		while(!(pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE)));
		pI2CHandle->pI2Cx->DR = *pTxBuffer;
		len --;
		pTxBuffer ++;
	}
	//7. wait until TXE = 1 and BTF = 1 then initialize the stop condition
	while(!((pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE)) && (pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_BTF)) ));
	//8.generate the stop condition
	I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

}
void I2C_MasterDMASendData(I2C_Handle_t *pI2CHandle,uint8_t SlaveAddr)
{
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2.confirm that start generation is completed by checking the SB flag in the SR1
	while(!(pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_SB)));

	//3. send the address of the slave with r/w bit to w(0) (total 8bit)
	I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx,SlaveAddr);

	//ENABLE DMA
	pI2CHandle->pI2Cx->CR2 |= (1<<11);
	//4. confirm that the address pahse is completed by checking the AADR flag in the SR1
	while(!(pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_ADDR)));

	//5. clear the ADDR flag according to software sequence
	I2C_ClearADDRFlag(pI2CHandle->pI2Cx);
}

void I2C_MasterRecieveData(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer,uint32_t len,uint8_t SlaveAddr)
{
	//1. Generate start condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2.confirm that start generation is completed by checking the SB flag in the SR1
	while(!(pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_SB)));

	//3. send the address of the slave with r/w bit to w(0) (total 8bit)
	I2C_ExecuteAddressPhase_ReadData(pI2CHandle->pI2Cx,SlaveAddr);

	//4. confirm that the address pahse is completed by checking the AADR flag in the SR1
	while(!(pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_ADDR)));

	if(len == 1)
	{
		pI2CHandle->pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
		I2C_ClearADDRFlag(pI2CHandle->pI2Cx);
		while(!(pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_RXNE)));
		*pRxBuffer = pI2CHandle->pI2Cx->DR;
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

	}

	if(len > 1)
	{
		I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

		for(uint32_t i = len ; i > 0; i--)
		{
			while(!(pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_RXNE)));
			if(i == 2)
			{
				pI2CHandle->pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
				I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
			}
			*pRxBuffer = pI2CHandle->pI2Cx->DR;
			pRxBuffer ++;
		}
	}

	if(pI2CHandle->I2C_Config.I2C_ACKControl == ENABLE)
	{
		pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
	}


}
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx,uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
	}
	else
	{
		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
	}
}

void I2C_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi)
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

void I2C_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = (8 * iprx_section) + (8 + NO_PR_NITS_IMPLEMENTED) ;
	*(NVIC_PR_BASE_ADDR + (iprx)) |= (IRQPriority<< shift_amount);
}
