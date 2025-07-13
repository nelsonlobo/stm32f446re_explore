/*
 * stm32f446xx_gpio_driver.c
 *
 *  Created on: Jul 5, 2025
 *      Author: Nelson Lobo
 */



#include "stm32f446xx.h"

/*
 * APIs supported by this driver
 */

//Peripheral clock setup
/**********************************************************
 * @fn		-	GPIO_PClk_Ctrl(
 *
 * @brief	- This function enables or disable peripheral clock for any given GPIO port
 *
 * @param	- Base address of the GPIO peripheral
 * @param	- ENABLE or DISABLE macros
 *
 * @return	- none
 *
 * @note	- none
 */
void GPIO_PClk_Ctrl(GPIO_RegDef_t *pGPIOx, uint8_t status)
{
	if(status == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}
		else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}
		else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
		else
		{
//			#error "GPIO Port Does not exist"
		}
	}
	else
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DIS();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DIS();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DIS();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DIS();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DIS();
		}
		else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DIS();
		}
		else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_DIS();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DIS();
		}
		else
		{
//			#error "GPIO Port Does not exist"
		}
	}
}

/**********************************************************
 * @fn		- GPIO_Init
 *
 * @brief	- This function is used to initialize any given GPIO port
 *
 * @param	- GPIO handle provides all the information related to any given GPIO port
 *
 * @return	- none
 *
 * @note	- none
 */

void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0;

	GPIO_PClk_Ctrl(pGPIOHandle->pGPIOx,ENABLE);

	//Configure the mode register
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANLG)
	{
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3U<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->MODER |= temp;
	}
	else
	{
		//Handles the interrupt section
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_IT_FT)
		{
			//1.Configure the falling edge trigger status register
			EXTI->FTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//2.Clear respective RTSR bit
			EXTI->RTSR &= ~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_IT_RT)
		{
			//1.Configure the rising edge trigger status register
			EXTI->RTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//2.Clear respective FTSR bit
			EXTI->FTSR &= ~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_IT_RFT)
		{
			//1.Configure the rising & falling edge trigger status register
			EXTI->RTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		//2. Configure the gpio port selection in sysconfig_extiCR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber /4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber %4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = portcode <<(temp2 * 4);

		//3. Enable the exti interrupt delivery using IMR (Interrupt mask register)
		EXTI->IMR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}
	temp = 0;

	//Configure the speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;
	temp = 0;

	//Configure the PU/PD settings
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR |= temp;
	temp = 0;

	//Configure the Output type
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER |= temp;
	temp = 0;

	//Configure the alternate functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		uint8_t temp1, temp2;

		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber%8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF<<(4*temp2));
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFnMode << (4*temp2));

	}
}


/**********************************************************
 * @fn		- GPIO_DeInit
 *
 * @brief	- This function is used to de-initialize any given GPIO port
 *
 * @param	- Base address of the GPIO peripheral
 *
 * @return	- none
 *
 * @note	- none
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}
	else if(pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}
	else if(pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}
	else if(pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}
	else if(pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}
	else if(pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
	}
	else if(pGPIOx == GPIOG)
	{
		GPIOG_REG_RESET();
	}
	else if(pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}
	else
	{
//			#error "GPIO Port Does not exist"
	}
}

/**********************************************************
 * @fn		- GPIO_ReadFromInputPin
 *
 * @brief	- This function is used to read status of any given GPIO port pin
 *
 * @param	- Base address of the GPIO peripheral
 *
 * @return	- none
 *
 * @note	- none
 */
bool GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber)
{
	return (bool)((pGPIOx->IDR >> PinNumber)&0x00000001);
}

/**********************************************************
 * @fn		- GPIO_ReadFromInputPin
 *
 * @brief	- This function is used to read status of any given GPIO port pin
 *
 * @param	- Base address of the GPIO peripheral
 *
 * @return	- none
 *
 * @note	- none
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	return (uint16_t)pGPIOx->IDR;
}

/**********************************************************
 * @fn		- GPIO_WriteToOutputPin
 *
 * @brief	- This function is used to read status of any given GPIO port pin
 *
 * @param1	- Base address of the GPIO peripheral
 * @param2	- Pass the pin number of the respective GPIO port
 * @param3	- Set or clear the state of the pin
 *
 * @return	- none
 *
 * @note	- none
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber,bool state)
{
	if(state)
	{
		pGPIOx->ODR |= (1<<PinNumber);
	}
	else
	{
		pGPIOx->ODR &= ~(1<<PinNumber);
	}
}

/**********************************************************
 * @fn		- GPIO_WriteToOutputPort
 *
 * @brief	- This function is used to read status of any given GPIO port
 *
 * @param1	- Base address of the GPIO peripheral
 * @param2	- Pass 16 bit value to set or clear individual bits on this port
 *
 * @return	- none
 *
 * @note	- none
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,uint16_t value)
{
	pGPIOx->ODR = value;
}

/**********************************************************
 * @fn		- GPIO_TogglePin
 *
 * @brief	- This function is used to toggle the output state of any given GPIO port pin
 *
 * @param1	- Base address of the GPIO peripheral
 * @param2	- Pass the pin number of the respective GPIO port
 *
 * @return	- none
 *
 * @note	- none
 */
void GPIO_TogglePin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1<<PinNumber);
}

/**********************************************************
 * @fn		- GPIO_IRQConfig
 *
 * @brief	- This function is used to Configure the interrupt for any given GPIO
 *
 * @param1	- IRQPosition
 * @param2	- IRQPriority is the priority set by the user
 * @param3	- Enable or Disable
 *
 * @return	- none
 *
 * @note	- none
 */
void GPIO_IRQConfig(uint8_t IRQPosition, bool status)
{
	if(status == ENABLE)
	{
		if(IRQPosition < 32)
		{
			//Program the ISER0 register
			*NVIC_ISER0 |= (1<<IRQPosition);
		}
		else if(IRQPosition>31 && IRQPosition<64)
		{
			//Program the ISER1 register
			*NVIC_ISER1 |= (1<<(IRQPosition%32));
		}
		else if(IRQPosition>63 && IRQPosition<96)
		{
			//Program the ISER2 register
			*NVIC_ISER2 |= (1<<(IRQPosition%64));
		}
	}
	else
	{
		if(IRQPosition < 32)
		{
			//Program the ICER0 register
			*NVIC_ICER0 |= (1<<IRQPosition);
		}
		else if(IRQPosition>31 && IRQPosition<64)
		{
			//Program the ICER1 register
			*NVIC_ICER1 |= (1<<(IRQPosition%32));
		}
		else if(IRQPosition>63 && IRQPosition<96)
		{
			//Program the ICER2 register
			*NVIC_ICER2 |= (1<<(IRQPosition%64));
		}
	}
}


void GPIO_IRQPriorityConfig(uint8_t IRQPosition, uint32_t IRQPriority)
{
	uint8_t iprx = IRQPosition/4;
	uint8_t iprx_section = IRQPosition%4;

	uint8_t shift_amount = (8*iprx_section) + (8-NO_PRIORITY_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}
/**********************************************************
 * @fn		- GPIO_IRQHandling
 *
 * @brief	- This function is used to handle the interrupt for any given GPIO
 *
 * @param1	- PinNumber
 *
 * @return	- none
 *
 * @note	- none
 */
void GPIO_IRQHandling(uint8_t pinNumber)
{
	//Clear the EXTI_PR register corresponding to the pin number
	if(EXTI->PR & (1<<pinNumber))
	{
		EXTI->PR |= (1<<pinNumber);
	}
}
