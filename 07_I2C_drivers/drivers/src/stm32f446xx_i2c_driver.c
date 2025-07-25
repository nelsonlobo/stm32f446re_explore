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
//	tempReg &= ~(pI2CHandle->I2C_Config.I2C_AddrMode <<I2C_OAR1_ADDMODE);	//Set to 7-Bit/10-Bit address mode
//	if(pI2CHandle->I2C_Config.I2C_AddrMode)
//		tempReg |= pI2CHandle->I2C_Config.I2C_DeviceAddress;
//	else
//		tempReg |= (uint8_t)(pI2CHandle->I2C_Config.I2C_DeviceAddress <<I2C_OAR1_ADD7_1);

	tempReg |= (uint8_t)(pI2CHandle->I2C_Config.I2C_DeviceAddress <<I2C_OAR1_ADD7_1);
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
//	if(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_MSL))
//	{
//		//device is in master mode
//		if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
//		{
//			if(pI2CHandle->RxSize  == 1)
//			{
//				//first disable the ack
//				I2C_ManageAcking(pI2CHandle->pI2Cx,DISABLE);
//
//				//clear the ADDR flag ( read SR1 , read SR2)
//				dummy_read = pI2CHandle->pI2Cx->SR1;
//				dummy_read = pI2CHandle->pI2Cx->SR2;
//				(void)dummy_read;
//			}
//
//		}
//		else
//		{
//			//clear the ADDR flag ( read SR1 , read SR2)
//			dummy_read = pI2CHandle->pI2Cx->SR1;
//			dummy_read = pI2CHandle->pI2Cx->SR2;
//			(void)dummy_read;
//
//		}
//
//	}
//	else
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
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle,uint8_t *pTxBuffer, uint32_t len, uint8_t slaveAddr)
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
	if(!pI2CHandle->I2C_Config.I2C_RepeatedStart)
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
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint32_t len, uint8_t slaveAddr)
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
		if(!pI2CHandle->I2C_Config.I2C_RepeatedStart)
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
				if(!pI2CHandle->I2C_Config.I2C_RepeatedStart)
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
