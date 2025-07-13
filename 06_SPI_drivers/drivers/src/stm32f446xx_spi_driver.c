/*
 * stm32f446xx_spi_driver.c
 *
 *  Created on: Jul 8, 2025
 *      Author: Nelson Lobo
 */


#include "stm32f446xx_spi_driver.h"

static void SPI_txeIntrHandle(SPI_Handle_t *pSPIHandle);
static void SPI_rxneIntrHandle(SPI_Handle_t *pSPIHandle);
static void SPI_ovrIntrHandle(SPI_Handle_t *pSPIHandle);
static void SPI_modfIntrHandle(SPI_Handle_t *pSPIHandle);


/**********************************************************
 * @fn		- SPI_PClk_Ctrl
 *
 * @brief	- Control SPI clock
 *
 * @param1	- SPIx peripheral to gain access of
 * @param2	- ENABLE or DISABLE
 *
 * @return	- none
 *
 * @note	- none
 */
void SPI_PClk_Ctrl(SPI_RegDef_t *pSPIx, uint8_t status)
{
	if(status == ENABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}
		else if(pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}
		else if(pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		}
		else if(pSPIx == SPI4)
		{
			SPI4_PCLK_EN();
		}
		else
		{

		}
	}
	else
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_DIS();
		}
		else if(pSPIx == SPI2)
		{
			SPI2_PCLK_DIS();
		}
		else if(pSPIx == SPI3)
		{
			SPI3_PCLK_DIS();
		}
		else if(pSPIx == SPI4)
		{
			SPI4_PCLK_DIS();
		}
		else
		{

		}
	}
}


/**********************************************************
 * @fn		- SPI_Init
 *
 * @brief	- Initialize the SPIx peripheral
 *
 * @param1	- Pass all the peripheral configurations to this parameter
 *
 * @return	- none
 *
 * @note	- none
 */
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	uint32_t tempReg = 0;

	SPI_PClk_Ctrl(pSPIHandle->pSPIx, ENABLE);

	//1.Configure the SPI mode as master or slave by setting or clearing this bit respectively
	tempReg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

	//2.Configure the bus
	if(pSPIHandle->SPIConfig.SPI_BusConfig==SPI_BUS_CONFIG_FULL_DUPLEX)
	{
		tempReg &= ~(1<<SPI_CR1_BIDIMODE);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig==SPI_BUS_CONFIG_HALF_DUPLEX)
	{
		tempReg |= (1<<SPI_CR1_BIDIMODE);
	}
	else
	{
		tempReg &= ~(1<<SPI_CR1_BIDIMODE);
		tempReg |= (1<<SPI_CR1_RXONLY);
	}

	//Configure the speed of the bus
	tempReg |= pSPIHandle->SPIConfig.SPI_SclkSpeed <<SPI_CR1_BR;
	//SPI frame format
	tempReg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;
	//Configure CPOL bit which manages the clock polarity
	tempReg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;
	//Configure CPHA bit which manages at which phase the sampling should occur
	tempReg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;
	//Configure SSM bit which manages Slave mode setting
	tempReg |= pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM;
	//Configure SSI bit which manages Slave mode setting
	tempReg |= pSPIHandle->SPIConfig.SPI_SSI << SPI_CR1_SSI;

	pSPIHandle->pSPIx->CR1 = tempReg;
}

/**********************************************************
 * @fn		- SPI_GetFlagStatus
 *
 * @brief	- Polls for status of SPI peripheral
 *
 * @param1	- Specify which SPI peripheral you want to gain access of (SPI1,SPI2,SPI3,SPI4)
 * @param2	- Pass the flag to be checked
 *
 * @return	- none
 *
 * @note	- Check for TXE, RXE or BSY flag
 */
bool SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t flagName)
{
	if(pSPIx->SR & flagName)	//Compare both bit-fields
	{
		return FLAG_SET;
	}

	return FLAG_CLEAR;
}

/**********************************************************
 * @fn		- SPI_SendData
 *
 * @brief	- Transmit data in polling mode
 *
 * @param1	- Specify which SPI peripheral you want to gain access of (SPI1,SPI2,SPI3,SPI4)
 * @param2	- Buffer to send messages to external devices
 * @param3	- Enable or Disable
 *
 * @return	- none
 *
 * @note	- Check for TXE flag
 */
void SPI_SendData(SPI_RegDef_t *pSPIx,uint8_t *pTXBuffer, uint32_t len)
{
	while(len >0)
	{
		//1. Wait until TXNE is set
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG)==FLAG_CLEAR);

		//2. Check for DFF bit whether set to 16-bits or 8-bits
		if((pSPIx->CR1 & (1<<SPI_CR1_DFF)))
		{
			//16bit data
			pSPIx->DR 	= *((uint16_t*)pTXBuffer);
			len	-=2;
			pTXBuffer	+=2;
		}
		else
		{
			//8bit data
			pSPIx->DR = *(pTXBuffer);
			len--;
			pTXBuffer++;
		}
	}
}

/**********************************************************
 * @fn		- SPI_ReceiveData
 *
 * @brief	- Receive data in polling mode
 *
 * @param1	- Specify which SPI peripheral you want to gain access of (SPI1,SPI2,SPI3,SPI4)
 * @param2	- Buffer to receive messages from external sources
 * @param3	- Enable or Disable
 *
 * @return	- none
 *
 * @note	- Check for RXNE
 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx,uint8_t *pRXBuffer, uint32_t len)
{
	while(len >0)
	{
		//1. Wait until RXNE is set
		while(SPI_GetFlagStatus(pSPIx, SPI_RXE_FLAG)==FLAG_CLEAR);

		//2. Check the DFF bit in CR1
		if((pSPIx->CR1 & (1<<SPI_CR1_DFF)))
		{
			//16bit data
			//1. Load the data from DR to RX buffer address
			*((uint16_t*)pRXBuffer) = pSPIx->DR;
			len	-=2;
			pRXBuffer	+=2;
		}
		else
		{
			//8bit data
			*(pRXBuffer) = pSPIx->DR;
			len--;
			pRXBuffer++;
		}
	}
}

/**********************************************************
 * @fn		- SPI_SSOEConfig
 *
 * @brief	- Controls the Slave select pin automatically when SPE bit is set and cleared
 *
 * @param1	- Specify which SPI peripheral you want to gain access of (SPI1,SPI2,SPI3,SPI4)
 * @param2	- Enable or Disable
 *
 * @return	- none
 *
 * @note	- Only used when we need the Slave select pin to be controlled
 * 			  automatically when SPI is configured in master mode
 */
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t status)
{
	if(status == ENABLE)
	{
		pSPIx->CR2 |= (1<<SPI_CR2_SSOE);
	}
	else
	{
		pSPIx->CR2 &= ~(1<<SPI_CR2_SSOE);
	}
}


/**********************************************************
 * @fn		- SPI_PeripheralControl
 *
 * @brief	- Controls the peripheral's status
 *
 * @param1	- Specify which SPI peripheral you want to gain access of (SPI1,SPI2,SPI3,SPI4)
 * @param2	- Enable or Disable
 *
 * @return	- none
 *
 * @note	- none
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t status)
{
	if(status == ENABLE)
	{
		pSPIx->CR1 |= (1<<SPI_CR1_SPE);
	}
	else
	{
		pSPIx->CR1 &= ~(1<<SPI_CR1_SPE);
	}
}


/**********************************************************
 * @fn		- SPI_IRQConfig
 *
 * @brief	- This function is used to Configure the interrupt for any given SPIx
 *
 * @param1	- IRQPosition
 * @param2	- IRQPriority is the priority set by the user
 * @param3	- Enable or Disable
 *
 * @return	- none
 *
 * @note	- none
 */
void SPI_IRQConfig(uint8_t IRQPosition, bool status)
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

/**********************************************************
 * @fn		- SPI_IRQPriorityConfig
 *
 * @brief	- This function is used to Configure the priorities for the interrupt for any given SPIx
 *
 * @param1	- IRQPosition
 * @param2	- IRQPriority is the priority set by the user
 * @param3	- Enable or Disable
 *
 * @return	- none
 *
 * @note	- none
 */
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	uint8_t iprx = IRQNumber/4;
	uint8_t iprx_section = IRQNumber%4;

	uint8_t shift_amount = (8*iprx_section) + (8-NO_PRIORITY_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}

/**********************************************************
 * @fn		- SPI_SendDataIntr
 *
 * @brief	- This function is used to send data in interrupt mode
 *
 * @param1	- Specify which SPI peripheral you want to gain access of (SPI1,SPI2,SPI3,SPI4)
 * @param2	- Buffer to send messages to external devices
 * @param3	- Length of data
 *
 * @return	- none
 *
 * @note	- none
 */
uint8_t SPI_SendDataIntr(SPI_Handle_t *pSPIHandle,volatile uint8_t *pTxBuffer, uint32_t len)
{
	uint8_t state = pSPIHandle->txState;

	if(state != SPI_TX_BSY)
	{
		//1. 	Save the tx buffer address and len information in global variables
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->txLen = len;
		//2. 	Mark the SPI state as busy in transmission so that
		//		no other code can take over the SPI peripheral
		//		until transmission is over
		pSPIHandle->txState = SPI_TX_BSY;

//		pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;

		//3. 	Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->CR2	|= (1<<SPI_CR2_TXEIE);

		//4. 	Data transmission will be handled by the ISR code
	}
	return state;
}

/**********************************************************
 * @fn		- SPI_IRQPriorityConfig
 *
 * @brief	- This function is used to Configure the priorities for the interrupt for any given SPIx
 *
 * @param1	- Specify which SPI peripheral you want to gain access of (SPI1,SPI2,SPI3,SPI4)
 * @param2	- Buffer to receive messages from external devices
 * @param3	- Length of data
 *
 * @return	- none
 *
 * @note	- none
 */
uint8_t SPI_ReceiveDataIntr(SPI_Handle_t *pSPIHandle,volatile uint8_t *pRxBuffer, uint32_t len)
{
	uint8_t state = pSPIHandle->rxState;

	if(state != SPI_RX_BSY)
	{
		//1. 	Save the rx buffer address and len information in global variables
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->rxLen = len;
		//2. 	Mark the SPI state as busy in transmission so that
		//		no other code can take over the SPI peripheral
		//		until reception is over
		pSPIHandle->rxState = SPI_RX_BSY;

		//3. 	Enable the RXNEIE control bit to get interrupt whenever RXE flag is set in SR
		pSPIHandle->pSPIx->CR2	|= (1<<SPI_CR2_RXNEIE);

		//4. 	Data transmission will be handled by the ISR code
	}

	return state;
}

static void SPI_txeIntrHandle(SPI_Handle_t *pSPIHandle)
{
	//2. Check for DFF bit whether set to 16-bits or 8-bits
	if((pSPIHandle->pSPIx->CR1 & (1<<SPI_CR1_DFF)))
	{
		//16bit data
		pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
		pSPIHandle->txLen -= 2;
		pSPIHandle->pTxBuffer += 2; // Increment by 2 bytes
	}
	else
	{
		//8bit data
		pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;
		pSPIHandle->txLen--;;
		pSPIHandle->pTxBuffer++;
	}

	if(!pSPIHandle->txLen)
	{
		SPI_closeTransmission(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_TX_COMPLETE);
	}
}

static void SPI_rxneIntrHandle(SPI_Handle_t *pSPIHandle)
{
	//2. Check the DFF bit in CR1
	if((pSPIHandle->pSPIx->CR1 & (1<<SPI_CR1_DFF)))
	{
		//16bit data
		//1. Load the data from DR to RX buffer address
		*((uint16_t*)pSPIHandle->pRxBuffer) = (uint16_t)pSPIHandle->pSPIx->DR;
		pSPIHandle->rxLen -= 2;
		pSPIHandle->pRxBuffer += 2; // Increment by 2 bytes
	}
	else
	{
		//8bit data
		*(pSPIHandle->pRxBuffer) = (uint8_t)pSPIHandle->pSPIx->DR;
		pSPIHandle->rxLen--;
		pSPIHandle->pRxBuffer++;
	}

	if(!pSPIHandle->rxLen)
	{
		SPI_closeReception(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_RX_COMPLETE);
	}
}

static void SPI_ovrIntrHandle(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp;

	if(pSPIHandle->txState != SPI_TX_BSY)
	{
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	(void)temp;
	SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_OVR_ERR);
}

static void SPI_modfIntrHandle(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp;

	if(pSPIHandle->pSPIx->SR & (1<<SPI_SR_MODF))
	{
		temp = pSPIHandle->pSPIx->SR;
		pSPIHandle->pSPIx->CR1 |= (1<<SPI_CR1_MSTR);
	}
	(void)temp;
	SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_MODF_ERR);
}

/**********************************************************
 * @fn		- SPI_IRQHandling
 *
 * @brief	- This function is used to handle the transactions that occur within SPI in interrupt mode
 *
 * @param1	- Specify which SPI peripheral you want to gain access of (SPI1,SPI2,SPI3,SPI4)
 *
 * @return	- none
 *
 * @note	- none
 */
void SPI_IRQHandling(SPI_Handle_t *pHandle)
{
	uint8_t temp1, temp2;
	//Check for TXE interrupt
	temp1 = pHandle->pSPIx->SR & (1<<SPI_SR_TXE);
	temp2 = pHandle->pSPIx->CR2 & (1<<SPI_CR2_TXEIE);

	if(temp1 && temp2)
	{
		//Handle TXE
		SPI_txeIntrHandle(pHandle);
	}

	//Check for RXNE interrupt
	temp1 = pHandle->pSPIx->SR & (1<<SPI_SR_RXNE);
	temp2 = pHandle->pSPIx->CR2 & (1<<SPI_CR2_RXNEIE);

	if(temp1 && temp2)
	{
		//Handle RXNE
		SPI_rxneIntrHandle(pHandle);
	}

	//Check for OVR interrupt
	temp1 = pHandle->pSPIx->SR & (1<<SPI_SR_OVR);
	temp2 = pHandle->pSPIx->CR2 & (1<<SPI_CR2_ERRIE);

	if(temp1 && temp2)
	{
		//Handle RXNE
		SPI_ovrIntrHandle(pHandle);
	}

	//Check for MODF interrupt
	temp1 = pHandle->pSPIx->SR & (1<<SPI_SR_MODF);
	temp2 = pHandle->pSPIx->CR2 & (1<<SPI_CR2_ERRIE);

	if(temp1 && temp2)
	{
		//Handle RXNE
		SPI_modfIntrHandle(pHandle);
	}
}

/**********************************************************
 * @fn		- SPI_clrOVRFlag
 *
 * @brief	- This function clears the overflow flag if set
 *
 * @param1	- Specify which SPI peripheral you want to gain access of (SPI1,SPI2,SPI3,SPI4)
 *
 * @return	- none
 *
 * @note	- none
 */
void SPI_clrOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->SR;
	temp = pSPIx->DR;
	(void)temp;
}

/**********************************************************
 * @fn		- SPI_closeTransmission
 *
 * @brief	- This function closes SPI transmit feature
 *
 * @param1	- Pass respective handle information to this function
 *
 * @return	- none
 *
 * @note	- none
 */
void SPI_closeTransmission(SPI_Handle_t *pSPIHandle)
{
	//Close SPI transmission
	//Inform application that SPI transmission is complete!
	pSPIHandle->pSPIx->CR2 &= ~(1<<SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->txLen = 0;
	pSPIHandle->txState = SPI_RDY;
}

/**********************************************************
 * @fn		- SPI_closeReception
 *
 * @brief	- This function closes SPI receive feature
 *
 * @param1	- Pass respective handle information to this function
 *
 * @return	- none
 *
 * @note	- none
 */
void SPI_closeReception(SPI_Handle_t *pSPIHandle)
{
	//Close SPI transmission
	//Inform application that SPI transmission is complete!
	pSPIHandle->pSPIx->CR2 &= ~(1<<SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->rxLen = 0;
	pSPIHandle->rxState = SPI_RDY;
}

/**********************************************************
 * @fn		- SPI_ApplicationEventCallback
 *
 * @brief	- This function handles what happens in the SPIx ISR routines
 *
 * @param1	- Pass respective handle information to this function
 *
 * @param2	- Shares status of the event that is currently being processed by SPI
 *
 * @return	- none
 *
 * @note	- none
 */
__attribute__((weak)) void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t evntVal)
{

}
