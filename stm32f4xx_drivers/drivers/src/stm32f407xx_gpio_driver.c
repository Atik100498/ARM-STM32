/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: 07-Nov-2019
 *      Author: ATIQUE
 */

#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"

/*
 * info					- Initializing all the parameters.
 * 						  pin mode,output type, output port speed
 * @param[1]			- address of variable of the structure.
 * 						  @param[1] info - contains all the value of gpio initialization parameter.
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp=0;
	//1.PIN MODE
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)));
		pGPIOHandle->pGPIOx->MODER &= ~( 0x03 << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER |= temp;
	}else
	{
		/*
		 * 1. Interrupt mode
		 * 	  Select the respective EXTIx [x = 1,2,3,...,15] with RT,FT ot RFT
		 */
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			//. 1.config FTSR(Falling Trigger Selection Register)
			EXTI->FTSR |= ( 1 <<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//Clear RTSR BIT
			EXTI->RTSR &= ~( 1 <<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			//. 1.config RTSR
			EXTI->RTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//Clear FTSR BIT
			EXTI->FTSR &= ~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			//. 1.config FTSR
			EXTI->FTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//2. config RTSR BIT
			EXTI->RTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		/*
		 * configuring the SYSCFG_EXTIR[n]
		 * to set the output from the selected port pin to come as output from the EXTI[n] register or MUX
		 * select the respective port from the SYCFG_EXTICR (System Conficgutarion EXTI Control Register)
		 */

		/*
		 * SYSCFG_EXTICR[X] Have four register thus each register contain four EXTI line control
		 * SYSCFG_EXTICR[1] = | EXTI3 | EXTI2 | EXTI1 | EXTI0 | each of four bits.
		 * Eg., GPIO_PIN_15 interupt will be controlled by EXTI15 this 15/4(four register) = (int) 3.
		 * syscfg_EXTI[3] the position of this configuration bit starts at postion 4 as 15%4.
		 */
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portcode = GPIO_BASEADDER_TO_CODE(pGPIOHandle->pGPIOx);
		/* Enabling RCC peripheral clock for SYSCFG */
		SYSCFG_PCLK_EN();
		SYSCFG ->EXTICR[temp1] = (portcode << (4*temp2));

		/*
		 * enable the exti interrupt delivery using IMR
		 */
		EXTI->IMR = (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}
	temp=0;
	//2. SPEED
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); /* Each pin takes 2 bits in OPSPEEDR, thus, multiply by 2 */
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x03<<2 *pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	temp=0;
	//3. PULL UP/DOWM
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl <<(2 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x03<<2*(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	temp=0;
	//4. OUTPUT TYPE
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x02<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	temp=0;
	//5. ALT. FUNC
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber <= GPIO_PIN_7)
		{
			/*
			 * AFR[0] -> AFRL, contains pin configuration for GPIO_PIN_0 to GPIO_PIN_7
			 */
			temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
			pGPIOHandle->pGPIOx->AFR[0] &= ~(0x0F<< 4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			pGPIOHandle->pGPIOx->AFR[0] |= temp;
		}
		else
		{
			/*
			* AFR[1] -> AFRH, contains pin configuration for GPIO_PIN_8 to GPIO_PIN_15
			*/
			temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber - 8)));
			pGPIOHandle->pGPIOx->AFR[1] &= ~(0x0F<<4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			pGPIOHandle->pGPIOx->AFR[1] |= temp;
		}
	}

}
void GPIO_DeInit(GPIO_RegDef_t *pGPIO)//PERIPHERAL RESET IN RCC WILL RESET ALL THE REGISTER OF THAT PORT WILL BE RESET
{
	if(pGPIO == GPIOA)
			{
				GPIOA_REG_RESET();
			}
			if(pGPIO == GPIOB)
			{
				GPIOB_REG_RESET();
			}
			if(pGPIO == GPIOC)
			{
				GPIOC_REG_RESET();
			}
			if(pGPIO == GPIOD)
			{
				GPIOD_REG_RESET();
			}
			if(pGPIO == GPIOE)
			{
				GPIOE_REG_RESET();
			}
			if(pGPIO == GPIOF)
			{
				GPIOF_REG_RESET();
			}
			if(pGPIO == GPIOG)
			{
				GPIOG_REG_RESET();
			}
			if(pGPIO == GPIOH)
			{
				GPIOH_REG_RESET();
			}
			if(pGPIO == GPIOI)
			{
				GPIOI_REG_RESET();
			}
}


void GPIO_PClkControl(GPIO_RegDef_t *pGPIO, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pGPIO == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		if(pGPIO == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		if(pGPIO == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		if(pGPIO == GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		if(pGPIO == GPIOE)
		{
			GPIOE_PCLK_EN();
		}
		if(pGPIO == GPIOF)
		{
			GPIOF_PCLK_EN();
		}
		if(pGPIO == GPIOG)
		{
			GPIOG_PCLK_EN();
		}
		if(pGPIO == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
		if(pGPIO == GPIOI)
		{
			GPIOI_PCLK_EN();
		}
	}
	else
	{
		if(pGPIO == GPIOA)
		{
			GPIOA_PCLK_DI();
		}
		if(pGPIO == GPIOB)
		{
			GPIOB_PCLK_DI();
		}
		if(pGPIO == GPIOC)
		{
			GPIOC_PCLK_DI();
		}
		if(pGPIO == GPIOD)
		{
			GPIOD_PCLK_DI();
		}
		if(pGPIO == GPIOE)
		{
			GPIOE_PCLK_DI();
		}
		if(pGPIO == GPIOF)
		{
			GPIOF_PCLK_DI();
		}
		if(pGPIO == GPIOG)
		{
			GPIOG_PCLK_DI();
		}
		if(pGPIO == GPIOH)
		{
			GPIOH_PCLK_DI();
		}
		if(pGPIO == GPIOI)
		{
			GPIOI_PCLK_DI();
		}
	}
}
/*
 * @param[1]			- GPIO port name to access the respective config registers.
 * @param[2]			- GPIOx pin number from where you want to read the data.
 *
 * return				- Gives data read from the specific pin.
 */

uint8_t GPIO_ReadPin(GPIO_RegDef_t *pGPIO, uint8_t PinNumber)
{
	uint8_t data;
	data = (uint8_t)((pGPIO->IDR >> PinNumber) & 0x00000001);
	return data;
}

uint16_t GPIO_ReadPort(GPIO_RegDef_t *pGPIO)
{
	uint16_t data;
	data = (uint16_t)(pGPIO->IDR);
	return data;
}

void GPIO_WritePin(GPIO_RegDef_t *pGPIO, uint8_t PinNumber, uint8_t Value)
{
	if(Value == GPIO_PIN_SET)
	{
		pGPIO->ODR |= (1 << PinNumber);
	}
	else
	{
		pGPIO->ODR &= ~(1<<PinNumber);
	}
}

void GPIO_WritePort(GPIO_RegDef_t *pGPIO, uint16_t Value)
{
	pGPIO->ODR = Value;
}

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIO, uint8_t PinNumber)
{
	pGPIO->ODR ^= (1<<PinNumber);
}

void GPIO_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi)
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

void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = (8 * iprx_section) + (8 + NO_PR_NITS_IMPLEMENTED) ;
	*(NVIC_PR_BASE_ADDR + (iprx)) |= (IRQPriority<< shift_amount);
}

void GPIO_ReadHandling(uint8_t PinNumber)
{
	//clear the exti pending register bit corresponding to the pin number
	//this bit it cleared by setting it to HIGH ot 1
	if(EXTI->PR & (1<<PinNumber))
	{
		//clear bit
		EXTI->PR |= ( 1<< PinNumber );
	}

}
