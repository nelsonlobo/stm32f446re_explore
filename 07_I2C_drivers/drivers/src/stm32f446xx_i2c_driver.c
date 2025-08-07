/*
 * stm32f446_i2c_drivers.c
 *
 *  Created on: Jul 14, 2025
 *      Author: Nelson Lobo
 */


#include "stm32f446xx.h"

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2Cx,uint8_t slaveAddr,uint8_t activity);
static void I2C_ClearAddrFlag(I2C_Handle_t *pI2CHandle );
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);
static void I2C_MasterHandleRXNE_Interrupt(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleTXE_Interrupt(I2C_Handle_t *pI2CHandle);

/**********************************************************
 * @fn		- I2C_PClk_Ctrl
 *
 * @brief	- Control I2C clock
 *
 * @param1	- I2Cx peripheral to gain access of I2C1, I2C2, I2C3
 * @param2	- ENABLE or DISABLE
 *
 * @return	- none
 *
 * @note	- none
 */
void I2C_PClk_Ctrl(I2C_RegDef_t *pI2Cx, uint8_t status)
{
	if(status == ENABLE)
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_EN();
		}
		else if(pI2Cx == I2C2)
		{
			I2C2_PCLK_EN();
		}
		else if(pI2Cx == I2C3)
		{
			I2C3_PCLK_EN();
		}
		else
		{

		}
	}
	else
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_DIS();
		}
		else if(pI2Cx == I2C2)
		{
			I2C2_PCLK_DIS();
		}
		else if(pI2Cx == I2C3)
		{
			I2C3_PCLK_DIS();
		}
		else
		{

		}
	}
}

/*********************************************************************
 * @fn  	- I2C_PeripheralControl
 *
 * @brief  	- Enable or Disable I2C peripheral
 *
 * @param1	- I2Cx peripheral to gain access of I2C1, I2C2, I2C3
 * @param2	- ENABLE or DISABLE
 *
 * @return	- none
 *
 * @Note    -
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t status)
{
	if(status == ENABLE)
	{
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
		//pI2cBaseAddress->CR1 |= I2C_CR1_PE_Bit_Mask;
	}
	else
	{
		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
	}

}


/**********************************************************
 * @fn		- I2C_Init
 *
 * @brief	- Initialize the I2Cx peripheral
 *
 * @param1	- Pass all the peripheral configurations to this parameter
 *
 * @return	- none
 *
 * @note	- none
 */
void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	//Enable the I2C peripheral clock
	I2C_PClk_Ctrl(pI2CHandle->pI2Cx, ENABLE);


	/******************************************/
	//Setting up the CR1 Register
	/******************************************/
	//Set the ack control bit by default
	uint32_t tempReg = 0;
	tempReg |= pI2CHandle->I2C_Config.I2C_AckControl <<10;
	pI2CHandle->pI2Cx->CR1 = tempReg;

	/******************************************/
	//Setting up the CR2 Register
	/******************************************/
	//configure the freq field of CR2
	tempReg = 0;
	tempReg = RCC_GetPClk1Value()/1000000U;
	pI2CHandle->pI2Cx->CR2 = (tempReg & 0x3F);

	/******************************************/
	//Setting up the OAR1 Register
	/******************************************/
	//Configure the slave address
	tempReg = 0;

	if(pI2CHandle->I2C_Config.I2C_DeviceAddress > 0xFF)
	{
		tempReg |= (1 <<I2C_OAR1_ADDMODE);	//Set to 10-Bit address mode
		tempReg |= pI2CHandle->I2C_Config.I2C_DeviceAddress;	//Assign the 10-bit address
	}
	else
	{
		tempReg &= ~(1 <<I2C_OAR1_ADDMODE);	//Set to 7-Bit address mode
		tempReg |= (uint8_t)(pI2CHandle->I2C_Config.I2C_DeviceAddress <<I2C_OAR1_ADD7_1);	//Assign the 7-bit address
	}
	//Mandatory setting of 14-bit as specified by the Ref Man
	tempReg |= (1<<14);
	pI2CHandle->pI2Cx->OAR1 = tempReg;

	/******************************************/
	//Setting up the CCR Register
	/******************************************/
	uint16_t ccr_value = 0;
	tempReg = 0;
	if(pI2CHandle->I2C_Config.I2C_SclSpeed <= I2C_SCL_SPEED_SM100K)
	{
		//Configuration for standard mode
		ccr_value = (RCC_GetPClk1Value() / (2* pI2CHandle->I2C_Config.I2C_SclSpeed));
		tempReg |= (ccr_value & 0xFFF);
	}
	else
	{
		//Configuration for fast mode
		tempReg |= (1<<I2C_CCR_FS);

		tempReg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle<<I2C_CCR_DUTY);

		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			ccr_value = RCC_GetPClk1Value()/(3*pI2CHandle->I2C_Config.I2C_SclSpeed);
		}
		else
		{
			ccr_value = RCC_GetPClk1Value()/(25*pI2CHandle->I2C_Config.I2C_SclSpeed);
		}
		tempReg |= (ccr_value & 0xFFF);
	}
	pI2CHandle->pI2Cx->CCR = tempReg;

	/******************************************/
	//Setting up the TRISE Register
	/******************************************/
	if(pI2CHandle->I2C_Config.I2C_SclSpeed <= I2C_SCL_SPEED_SM100K)
	{
		//Configuration for standard mode
		tempReg = (RCC_GetPClk1Value() / 1000000U) +1;

	}
	else
	{
		//Configuration for fast mode
		tempReg = ((RCC_GetPClk1Value()*300) / 1000000000U ) +1;
	}
	pI2CHandle->pI2Cx->TRISE = (tempReg & 0x3F);
}

/**********************************************************
 * @fn		- I2C_DeInit
 *
 * @brief	- Initialize the I2Cx peripheral
 *
 * @param1	- Pass all the peripheral configurations to this parameter
 *
 * @return	- none
 *
 * @note	- none
 */
void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{
	if(pI2Cx == I2C1)
	{
		I2C1_REG_RESET();
	}
	else if(pI2Cx == I2C2)
	{
		I2C2_REG_RESET();
	}
	else if(pI2Cx == I2C3)
	{
		I2C3_REG_RESET();
	}
	else
	{
		//Error message
	}
}

/**********************************************************
 * @fn		- I2C _GetFlagStatus
 *
 * @brief	- Polls for status of I2C  peripheral
 *
 * @param1	- Specify which I2C  peripheral you want to gain access of (I2C 1,I2C 2,I2C 3,I2C 4)
 * @param2	- Pass the flag to be checked
 *
 * @return	- none
 *
 * @note	- Check for TXE, RXE or BSY flag
 */
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t flagName)
{
	if(pI2Cx->SR1 & flagName)	//Compare both bit-fields
	{
		return FLAG_SET;
	}

	return FLAG_CLEAR;
}

/**********************************************************
 * @fn		- I2C_ManageACK_bit
 *
 * @brief	- Polls for status of I2C  peripheral
 *
 * @param1	- Specify which I2C  peripheral you want to gain access of (I2C 1,I2C 2,I2C 3,I2C 4)
 * @param2	- Pass the flag to be checked
 *
 * @return	- none
 *
 * @note	- Check for TXE, RXE or BSY flag
 */
void I2C_ManageACK_bit(I2C_RegDef_t *pI2Cx,uint8_t status)
{
	if(status == I2C_ACK_ENABLE)
	{
		pI2Cx->CR1 |= (1<<I2C_CR1_ACK);
	}
	else
	{
		pI2Cx->CR1 &= ~(1<<I2C_CR1_ACK);
	}
}

/**********************************************************
 * @fn		- I2C_GenerateStartCondition
 *
 * @brief	- Function for sending data in master mode, polling method
 *
 * @param1	- Specify which I2C  peripheral you want to gain access of (I2C 1,I2C 2,I2C 3)
 * @param2	- Pass the flag to be checked
 *
 * @return	- none
 *
 * @note	- Check for TXE, RXE or BSY flag
 */
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1<<I2C_CR1_START);
}


static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1<<I2C_CR1_STOP);
}


/**********************************************************
 * @fn		- I2C_ExecuteAddressPhase
 *
 * @brief	- Function for sending data in master mode, polling method
 *
 * @param1	- Specify which I2C  peripheral you want to gain access of (I2C 1,I2C 2,I2C 3)
 * @param2	- Pass the flag to be checked
 *
 * @return	- none
 *
 * @note	- Check for TXE, RXE or BSY flag
 */
static void I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2Cx,uint8_t slaveAddr,uint8_t activity)
{
	slaveAddr = slaveAddr<<1;
	if(activity==I2C_WRITE)
		slaveAddr &= ~(1);	//slave address along with the r/nw bit=0
	else
		slaveAddr |= (1);	//slave address along with the r/nw bit=0
	pI2Cx->DR = slaveAddr;
}


static void I2C_ClearAddrFlag(I2C_Handle_t *pI2CHandle )
{
	uint32_t dummy_read;
	//check for device mode
	if(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_MSL))
	{
		//device is in master mode
		if(pI2CHandle->TxRxState == I2C_BUSY_RX)
		{
			if(pI2CHandle->RxSize  == 1)
			{
				//first disable the ack
				I2C_ManageACK_bit(pI2CHandle->pI2Cx,DISABLE);

				//clear the ADDR flag ( read SR1 , read SR2)
				dummy_read = pI2CHandle->pI2Cx->SR1;
				dummy_read = pI2CHandle->pI2Cx->SR2;
				(void)dummy_read;
			}

		}
		else
		{
			//clear the ADDR flag ( read SR1 , read SR2)
			dummy_read = pI2CHandle->pI2Cx->SR1;
			dummy_read = pI2CHandle->pI2Cx->SR2;
			(void)dummy_read;

		}

	}
	else
	{
		//device is in slave mode
		//clear the ADDR flag ( read SR1 , read SR2)
		dummy_read = pI2CHandle->pI2Cx->SR1;
		dummy_read = pI2CHandle->pI2Cx->SR2;
		(void)dummy_read;
	}
}


/**********************************************************
 * @fn		- I2C_MasterSendData
 *
 * @brief	- Function for sending data in master mode, polling method
 *
 * @param1	- Specify which I2C  peripheral you want to gain access of (I2C 1,I2C 2,I2C 3)
 * @param2	- Pass the flag to be checked
 *
 * @return	- none
 *
 * @note	- Check for TXE, RXE or BSY flag
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle,uint8_t *pTxBuffer, uint32_t len, uint8_t slaveAddr,uint8_t SR)
{
	//1. Generate the start condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. Confirm that start generation is completed by checking the SB flag in SR1
	//NOTE: Until SB1 is cleared SCL will be stretched (pulled low)
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_SB));

	//3. Send the address of the slave with r/nw bit set to w(0) (total 8-bits)
	I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx,slaveAddr,I2C_WRITE);

	//4. Confirm that the address phase is completed by checking the ADDR flag in the SR1
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

	//5. Clear the ADDR1 flag according to its software sequence
	//NOTE:Until ADDR1 is cleared SCL will be stretched (pulled low)
	I2C_ClearAddrFlag(pI2CHandle);

	//6. Send the data until len is 0
	while(len>0)
	{
		while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE)); //Wait till txe is set
		pI2CHandle->pI2Cx->DR = *pTxBuffer;
		pTxBuffer++;
		len--;
	}

	//7. When len becomes 0, wait for TXE=1 and BTF=1 before generating the STOP condition
	//NOTE: TXE=1, BTF=1, means that both SR and DR are empty and next transmission should begin
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));

	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF));

	//		when BTF=1, SCL will be stretched (pulled low)
	//8. Generate STOP condition and MASTER need not wait for the completion of STOP condition.
	//NOTE: Generating STOP condition automatically clears BTF flag.
	if(!SR)
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
}


/**********************************************************
 * @fn		- I2C_MasterReceiveData
 *
 * @brief	- Function for receiving data in master mode, polling method
 *
 * @param1	- Specify which I2C  peripheral you want to gain access of (I2C 1,I2C 2,I2C 3)
 * @param2	- Pass the flag to be checked
 *
 * @return	- none
 *
 * @note	- Check for TXE, RXE or BSY flag
 */
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint32_t len, uint8_t slaveAddr,uint8_t SR)
{
	//1. Generate the START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. Confirm that START state is completed by checking the SB flag in the SR1
	//	 NOTE: Until SB is cleared SCL will be stretched (pulled to LOW)
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_SB));

	//3. Send SLAVE address with r/nw bit set to R(1) (total 8-bits)
	I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, slaveAddr, I2C_READ);

	//4. Wait until address phase is completed by checking the ADDR in SR1 register
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));


	//Procedure to read only 1 Byte
	if(len ==1)
	{
		//Disable ACK
		I2C_ManageACK_bit(pI2CHandle->pI2Cx,DISABLE);

		//Clear ADDR flag
		I2C_ClearAddrFlag(pI2CHandle);

		//Wait until RXNE becomes 1
		while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));

		//Generate STOP condition
		if(!SR)
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

		//Read Data into buffer
		*pRxBuffer = pI2CHandle->pI2Cx->DR;
	}
	if(len > 1)
	{
		//Clear the ADDR flag
		I2C_ClearAddrFlag(pI2CHandle);

		for(uint32_t i = len; i>0; i--)
		{
			//wait until RXNE becomes 1
			while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));

			//Execute following procedure when last 2 bytes are to be processed
			if(i == 2)
			{
				//Clear the ACK bit
				I2C_ManageACK_bit(pI2CHandle->pI2Cx,I2C_ACK_DISABLE);

				//Generate STOP condition
				if(!SR)
					I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
			}

			//read data from the DATA REGISTER into the buffer
			*pRxBuffer = pI2CHandle->pI2Cx->DR;

			//increment the buffer address
			pRxBuffer++;
		}
	}

	//Re-enable ACK bit
	if(pI2CHandle->I2C_Config.I2C_AckControl == I2C_ACK_ENABLE)
		I2C_ManageACK_bit(pI2CHandle->pI2Cx,I2C_ACK_ENABLE);
}


/*********************************************************************
 * @fn      		  - I2C_IRQInterruptConfig
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void I2C_IRQConfig(uint8_t IRQPosition, bool status)
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

uint8_t I2C_MasterSendDataIntr(I2C_Handle_t *pI2CHandle,uint8_t *pTxBuffer, uint32_t len, uint8_t slaveAddr,uint8_t SR)
{
	uint8_t busyState = pI2CHandle->TxRxState;

	if((busyState != I2C_BUSY_TX) && (busyState != I2C_BUSY_RX))
	{
		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->txLen = len;
		pI2CHandle->TxRxState = I2C_BUSY_TX;
		pI2CHandle->DevAddr = slaveAddr;
		pI2CHandle->SR = SR;

		//Generate Start Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//Enable ITBUFEN control bit
		pI2CHandle->pI2Cx->CR2 |= (1<<I2C_CR2_ITBUFEN);

		//Enable ITEVFEN control bit
		pI2CHandle->pI2Cx->CR2 |= (1<<I2C_CR2_ITEVTEN);

		//Enable ITERREN control bit
		pI2CHandle->pI2Cx->CR2 |= (1<<I2C_CR2_ITERREN);
	}

	return busyState;
}

uint8_t I2C_MasterReceiveDataIntr(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint32_t len, uint8_t slaveAddr,uint8_t SR)
{
	uint8_t busyState = pI2CHandle->TxRxState;

	if((busyState != I2C_BUSY_TX) && (busyState != I2C_BUSY_RX))
	{
		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->rxLen = len;
		pI2CHandle->TxRxState = I2C_BUSY_RX;
		pI2CHandle->RxSize = len;
		pI2CHandle->DevAddr = slaveAddr;
		pI2CHandle->SR = SR;

		//Generate Start Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//Enable ITBUFEN control bit
		pI2CHandle->pI2Cx->CR2 |= (1<<I2C_CR2_ITBUFEN);

		//Enable ITEVFEN control bit
		pI2CHandle->pI2Cx->CR2 |= (1<<I2C_CR2_ITEVTEN);

		//Enable ITERREN control bit
		pI2CHandle->pI2Cx->CR2 |= (1<<I2C_CR2_ITERREN);
	}

	return busyState;
}


void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	//Interrupt handling for both master and slave mode of a device
	uint32_t temp1, temp2, temp3;
	temp1 = pI2CHandle->pI2Cx->CR2 & (1<<I2C_CR2_ITEVTEN);
	temp2 = pI2CHandle->pI2Cx->CR2 & (1<<I2C_CR2_ITBUFEN);

	//1. Handle for interrupt generated by SB event
	//NOTE: SB Flag is only applicable in Master mode
	temp3 = pI2CHandle->pI2Cx->SR1 & (1<<I2C_SR1_SB);
	if(temp1 && temp3)
	{
		//SB flag is set
		//The interrupt is generated because of SB event
		//This block will not be executed in slave mode because in slave config SB is always ZERO
		//In this block lets execute the address phase
		if(pI2CHandle->TxRxState == I2C_BUSY_TX)
		{
			I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, pI2CHandle->DevAddr, I2C_WRITE);
		}
		else if(pI2CHandle->TxRxState == I2C_BUSY_RX)
		{
			I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, pI2CHandle->DevAddr, I2C_READ);
		}
	}

	//2. Handle for interrupt generated by ADDR event
	//NOTE: when master mode: 	Address is sent
	//		when slave mode : 	Address is matched with own address
	temp3 = pI2CHandle->pI2Cx->SR1 & (1<<I2C_SR1_ADDR);
	if(temp1 && temp3)
	{
		//ADDR flag is set
		I2C_ClearAddrFlag(pI2CHandle);
	}

	//3. Handle for interrupt generated by BTF(byte transfer finished bit) event
	temp3 = pI2CHandle->pI2Cx->SR1 & (1<<I2C_SR1_BTF);
	if(temp1 && temp3)
	{
		//BTF flag is set
		if(pI2CHandle->TxRxState == I2C_BUSY_TX)
		{
			if(pI2CHandle->pI2Cx->SR1 & (1<<I2C_SR1_TXE))
			{
				//BTF,TXE = 1
				if(pI2CHandle->txLen == 0)
				{
					//1. Generate STOP condition
					if(pI2CHandle->SR == DISABLE)
						I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

					//2. Reset all the member elements of handle structure
					I2C_CloseSendData(pI2CHandle);
					//3. Notify the application about transmission complete
					I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_TX_CMPLT);
				}
			}
		}
		else if(pI2CHandle->TxRxState == I2C_BUSY_RX)
		{
			;
		}
	}

	//4. Handle for interrupt generated by STOPF event
	//NOTE: Stop detection flag is applicable only slave mode. For master this will be
	//The below code block will not be executed by the master since STOPF will not set in master mode
	temp3 = pI2CHandle->pI2Cx->SR1 & (1<<I2C_SR1_STOPF);
	if(temp1 && temp3)
	{
		//STOPF flag is set
		pI2CHandle->pI2Cx->CR1 |= 0x0000;

		//Notify the app that STOP is detected
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);
	}

	//5. Handle for interrupt generated by TXE event
	temp3 = pI2CHandle->pI2Cx->SR1 & (1<<I2C_SR1_TXE);
	if(temp1 && temp2 && temp3)
	{
		//TXE flag is set
		//We perform data transmission only if application is busy in TX
		if(pI2CHandle->pI2Cx->SR2 & (1<<I2C_SR2_MSL))
		{
			if(pI2CHandle->TxRxState == I2C_BUSY_TX)
			{
				I2C_MasterHandleTXE_Interrupt(pI2CHandle);
			}
		}
		else
		{
			//Slave mode
			if(pI2CHandle->pI2Cx->SR2 & (1<<I2C_SR2_TRA))
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_REQ);
		}
	}

	//6. Handle for interrupt generated by RXNE event
	temp3 = pI2CHandle->pI2Cx->SR1 & (1<<I2C_SR1_RXNE);
	if(temp1 && temp2 && temp3)
	{
		//RXNE flag is set
		if(pI2CHandle->pI2Cx->SR2 & (1<<I2C_SR2_MSL))	//Check if device is configured as MASTER
		{
			//The device is master

			//RXNE flag is set
			if(pI2CHandle->TxRxState == I2C_BUSY_RX)
			{
				I2C_MasterHandleRXNE_Interrupt(pI2CHandle);

			}
		}
		else
		{
			//Slave mode
			if(pI2CHandle->pI2Cx->SR2 & (1<<I2C_SR2_TRA))
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_RCV);
		}
	}
}


/*********************************************************************
 * @fn      		  - I2C_ER_IRQHandling
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 */

void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{

	uint32_t temp1,temp2;

    //Check the status of  ITERREN control bit in the CR2
	temp2 = (pI2CHandle->pI2Cx->CR2) & ( 1 << I2C_CR2_ITERREN);


	/***********************Check for Bus error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1<< I2C_SR1_BERR);
	if(temp1  && temp2 )
	{
		//This is Bus error

		//Clear the buss error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_BERR);

		//Notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_BERR);
	}

	/***********************Check for arbitration lost error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_ARLO );
	if(temp1  && temp2)
	{
		//This is arbitration lost error

		//Clear the arbitration lost error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_ARLO);

		//Notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_ARLO);

	}

/***********************Check for ACK failure  error************************************/

	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_AF);
	if(temp1  && temp2)
	{
		//This is ACK failure error

	    //Clear the ACK failure error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_AF);

		//Notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_AF);
	}

/***********************Check for Overrun/underrun error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_OVR);
	if(temp1  && temp2)
	{
		//This is Overrun/underrun

	    //Clear the Overrun/underrun error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_OVR);
		//Notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_OVR);
	}

/***********************Check for Time out error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_TIMEOUT);
	if(temp1  && temp2)
	{
		//This is Time out error

	    //Clear the Time out error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_TIMEOUT);
		//Notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_TIMEOUT);
	}

}


static void I2C_MasterHandleTXE_Interrupt(I2C_Handle_t *pI2CHandle)
{
	if(pI2CHandle->txLen > 0)
	{
		//1. Load the data into DR
		pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);

		//2. Decrement the txLen
		pI2CHandle->txLen--;

		//3. Increment the buffer address
		pI2CHandle->pTxBuffer++;
	}
}

static void I2C_MasterHandleRXNE_Interrupt(I2C_Handle_t *pI2CHandle)
{
//	if(pI2CHandle->TxRxState == I2C_BUSY_RX)
	{
		if(pI2CHandle->RxSize == 1)
		{
			*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
			pI2CHandle->rxLen--;
		}

		if(pI2CHandle->RxSize > 1)
		{
			if(pI2CHandle->rxLen ==2)
			{
				I2C_ManageACK_bit(pI2CHandle->pI2Cx, DISABLE);
			}

			*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
			pI2CHandle->pRxBuffer++;
			pI2CHandle->rxLen--;
		}
		if(pI2CHandle->rxLen ==0)
		{
			//Close I2C data reception and notify the applicaiton
			//1. Generate the STOP condition
			if(pI2CHandle->SR == DISABLE)
				I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

			//2. Close I2C RX
			I2C_CloseReceiveData(pI2CHandle);

			//3. Notify the application
			I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);
		}
	}
}


void I2C_CloseSendData(I2C_Handle_t *pI2CHandle)
{
	//Disable ITBUFEN control bit
	pI2CHandle->pI2Cx->CR2 &= ~(1<<I2C_CR2_ITBUFEN);

	//Disable ITEVTEN control bit
	pI2CHandle->pI2Cx->CR2 &= ~(1<<I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->txLen = 0;
}

void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle)
{
	//Disable ITBUFEN control bit
	pI2CHandle->pI2Cx->CR2 &= ~(1<<I2C_CR2_ITBUFEN);

	//Disable ITEVTEN control bit
	pI2CHandle->pI2Cx->CR2 &= ~(1<<I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->rxLen = 0;
	pI2CHandle->RxSize = 0;

	if(pI2CHandle->I2C_Config.I2C_AckControl == I2C_ACK_ENABLE)
		I2C_ManageACK_bit(pI2CHandle->pI2Cx, ENABLE);
}


void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t evntVal)
{
	uint8_t tx_buff[] = "STM32 greets you to C world!\nWe are learning how to write efficient code using the bare metal approach.\n";
	static uint8_t  slaveCmdCode = 0;
	static uint32_t slaveCnt = 0;
	static uint32_t w_ptr = 0;
	uint32_t data_len = strlen((char*)tx_buff);

	switch(evntVal)
	{
		case I2C_EV_TX_CMPLT:
		{
			printf("Tx is complete!\n");
			break;
		}

		case I2C_EV_RX_CMPLT:
		{
			printf("Rx is complete!\n");
			pI2CHandle->rxComplete = 1;
			break;
		}

		case I2C_ERROR_BERR:
		{
			printf("Bus Error occurred!\n");
			break;
		}

		case I2C_ERROR_ARLO:
		{
			printf("Arbitration Error occurred!\n");
			break;
		}

		case I2C_ERROR_OVR:
		{
			printf("Over-run Error occurred!\n");
			break;
		}

		case I2C_ERROR_AF:
		{
			if(pI2CHandle->pI2Cx->SR2 & (1<<I2C_SR2_MSL))
			{
				printf("Ack failure occurred!\n");
				I2C_CloseSendData(pI2CHandle);
				I2C_GenerateStopCondition(I2C1);
				while(1);
			}
			else
			{
				//In slave mode this event occurs only during Transmission.
				//When master sends NACK it indicates that the Master doesn't need any more information.
				if(slaveCmdCode!=0x52)
					slaveCmdCode = 0xFF;

				slaveCnt = 0;

				if(w_ptr >= data_len)
				{
					w_ptr = 0;
					slaveCmdCode = 0xFF;
				}
			}
			break;
		}
		case I2C_EV_DATA_REQ:
		{
			switch(slaveCmdCode)
			{
				case 0x51:
				{
					//Send length information to master
					I2C_SlaveSendData(pI2CHandle->pI2Cx, ((data_len >> ((slaveCnt%4) * 8)) & 0xFF));
					slaveCnt++;
					break;
				}
				case 0x52:
				{
					//Send the contents of tx_buff to master
					I2C_SlaveSendData(pI2CHandle->pI2Cx, tx_buff[w_ptr++]);
					break;
				}
			}
			break;
		}
		case I2C_EV_DATA_RCV:
		{
			//Data is waiting for the slave to read.
			slaveCmdCode = I2C_SlaveReceiveData(pI2CHandle->pI2Cx);
			break;
		}
		case I2C_EV_STOP:
		{
			//Occurs only during slave reception
			//when master terminates I2C communication with slave.
			slaveCnt = 0;
			break;
		}
	}
}

void I2C_SlaveEnableDisableCallbackEvents(I2C_RegDef_t *pI2Cx,uint8_t status)
{
	if(status == ENABLE)
	{
		pI2Cx->CR2 |= (1<<I2C_CR2_ITBUFEN);
		pI2Cx->CR2 |= (1<<I2C_CR2_ITERREN);
		pI2Cx->CR2 |= (1<<I2C_CR2_ITEVTEN);
	}
	else
	{
		pI2Cx->CR2 &= ~(1<<I2C_CR2_ITBUFEN);
		pI2Cx->CR2 &= ~(1<<I2C_CR2_ITERREN);
		pI2Cx->CR2 &= ~(1<<I2C_CR2_ITEVTEN);
	}
}

void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t data)
{
	pI2Cx->DR = data;
}

uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx)
{
	return (uint8_t)pI2Cx->DR;
}
