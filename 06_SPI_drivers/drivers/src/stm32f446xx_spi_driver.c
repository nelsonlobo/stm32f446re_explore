/*
 * stm32f446xx_spi_driver.c
 *
 *  Created on: Jul 8, 2025
 *      Author: Nelson Lobo
 */


#include "stm32f446xx_spi_driver.h"


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

bool SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t flagName)
{
	if(pSPIx->SR & flagName)	//Compare both bit-fields
	{
		return FLAG_SET;
	}

	return FLAG_CLEAR;
}

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
			pSPIx->DR = *((uint16_t*)pTXBuffer);
			len--;
			len--;
			(uint16_t*)pTXBuffer++;
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
			len--;
			len--;
			(uint16_t*)pRXBuffer++;
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
