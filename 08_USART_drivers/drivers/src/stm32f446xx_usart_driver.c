/*
 * stm32f446xx_usart.c
 *
 *  Created on: Aug 4, 2025
 *      Author: Nelson Lobo
 */

#include "stm32f446xx.h"

/**********************************************************
 * @fn		- USART_PClk_Ctrl
 *
 * @brief	- Control USART clock
 *
 * @param1	- USARTx peripheral to gain access of USART1, USART2, USART3, UART4, UART5, USART6
 * @param2	- ENABLE or DISABLE
 *
 * @return	- none
 *
 * @note	- none
 */
void USART_PClk_Ctrl(USART_RegDef_t *pUSARTx, uint8_t status)
{
	if(status == ENABLE)
	{
		if(pUSARTx == USART1)
		{
			USART1_PCLK_EN();
		}
		else if(pUSARTx == USART2)
		{
			USART2_PCLK_EN();
		}
		else if(pUSARTx == USART3)
		{
			USART3_PCLK_EN();
		}
		else if(pUSARTx == UART4)
		{
			UART4_PCLK_EN();
		}
		else if(pUSARTx == UART5)
		{
			UART5_PCLK_EN();
		}
		else if(pUSARTx == USART6)
		{
			USART6_PCLK_EN();
		}
		else
		{

		}
	}
	else
	{
		if(pUSARTx == USART1)
		{
			USART1_PCLK_DIS();
		}
		else if(pUSARTx == USART2)
		{
			USART2_PCLK_DIS();
		}
		else if(pUSARTx == USART3)
		{
			USART3_PCLK_DIS();
		}
		else if(pUSARTx == UART4)
		{
			UART4_PCLK_EN();
		}
		else if(pUSARTx == UART5)
		{
			UART5_PCLK_EN();
		}
		else if(pUSARTx == USART6)
		{
			USART6_PCLK_EN();
		}
		else
		{

		}
	}
}

/*********************************************************************
 * @fn  	- USART_PeripheralControl
 *
 * @brief  	- Enable or Disable USART peripheral
 *
 * @param1	- USARTx peripheral to gain access of USART1, USART2, USART3, UART4, UART5, USART6
 * @param2	- ENABLE or DISABLE
 *
 * @return	- none
 *
 * @Note    -
 */
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t status)
{
	if(status == ENABLE)
	{
		pUSARTx->CR1 |= (1 << USART_CR1_UE);
	}
	else
	{
		pUSARTx->CR1 &= ~(1 << USART_CR1_UE);
	}

}


/*********************************************************************
 * @fn      		  - USART_Init
 *
 * @brief             -
 *
 * @param[in]         - USARTx Handle
 *
 * @return            -
 *
 * @Note              - Resolve all the TODOs

 */
void USART_Init(USART_Handle_t *pUSARTHandle)
{

	//Temporary variable
	uint32_t tempreg=0;

	//Enable the USART peripheral clock
	USART_PClk_Ctrl(pUSARTHandle->pUSARTx, ENABLE);

	/* Configuration of CR1 */
	//Enable USART Tx and Rx engines according to the USART_Mode configuration item
	if (pUSARTHandle->USART_Config.mode == USART_MODE_ONLY_RX)
	{
		//Implement the code to enable the Receiver bit field
		tempreg|= (1 << USART_CR1_RE);
	}
	else if (pUSARTHandle->USART_Config.mode == USART_MODE_ONLY_TX)
	{
		//Implement the code to enable the Transmitter bit field
		tempreg |= ( 1 << USART_CR1_TE);

	}
	else if (pUSARTHandle->USART_Config.mode == USART_MODE_TXRX)
	{
		//Implement the code to enable the both Transmitter and Receiver bit fields
		tempreg |= ( ( 1 << USART_CR1_TE) | ( 1 << USART_CR1_RE) );
	}

    //Implement the code to configure the Word length configuration item
	tempreg |= pUSARTHandle->USART_Config.wordLength << USART_CR1_M;


    //Configuration of parity control bit fields
	if ( pUSARTHandle->USART_Config.parityControl == USART_PARITY_EN_EVEN)
	{
		//Implement the code to enale the parity control
		tempreg |= ( 1 << USART_CR1_PCE);

		//Implement the code to enable EVEN parity
//		tempreg &= ~( 1 << USART_CR1_PS);
		//Not required because by default EVEN parity will be selected once you enable the parity control

	}
	else if (pUSARTHandle->USART_Config.parityControl == USART_PARITY_EN_ODD )
	{
		//Implement the code to enable the parity control
	    tempreg |= ( 1 << USART_CR1_PCE);

	    //Implement the code to enable ODD parity
	    tempreg |= ( 1 << USART_CR1_PS);

	}

   //Program the CR1 register
	pUSARTHandle->pUSARTx->CR1 = tempreg;

	/*** Configuration of CR2 ***/

	tempreg=0;

	//Implement the code to configure the number of stop bits inserted during USART frame transmission
	tempreg |= pUSARTHandle->USART_Config.numOfStopBits << USART_CR2_STOP;

	//Program the CR2 register
	pUSARTHandle->pUSARTx->CR2 = tempreg;

	/**** Configuration of CR3 ****/

	tempreg=0;

	//Configuration of USART hardware flow control
	if ( pUSARTHandle->USART_Config.hwFlowControl == USART_HW_FLOW_CTRL_CTS)
	{
		//Implement the code to enable CTS flow control
		tempreg |= ( 1 << USART_CR3_CTSE);
	}
	else if (pUSARTHandle->USART_Config.hwFlowControl == USART_HW_FLOW_CTRL_RTS)
	{
		//Implement the code to enable RTS flow control
		tempreg |= (1 << USART_CR3_RTSE);

	}
	else if (pUSARTHandle->USART_Config.hwFlowControl == USART_HW_FLOW_CTRL_CTS_RTS)
	{
		//Implement the code to enable both CTS and RTS Flow control
		tempreg |= (( 1 << USART_CR3_CTSE)|(1<<USART_CR3_RTSE));
	}


	pUSARTHandle->pUSARTx->CR3 = tempreg;

	/*** Configuration of BRR(Baudrate register) ***/

	//Implement the code to configure the baud rate
	//We will cover this in the lecture. No action required here
	USART_SetBaudRate(pUSARTHandle->pUSARTx,pUSARTHandle->USART_Config.baud);
}


/**********************************************************
 * @fn		- USART_DeInit
 *
 * @brief	- Initialize the USARTx peripheral
 *
 * @param1	- Pass all the peripheral configurations to this parameter
 *
 * @return	- none
 *
 * @note	- none
 */
void USART_DeInit(USART_RegDef_t *pUSARTx)
{
	if(pUSARTx == USART1)
	{
		USART1_REG_RESET();
	}
	else if(pUSARTx == USART2)
	{
		USART2_REG_RESET();
	}
	else if(pUSARTx == USART3)
	{
		USART3_REG_RESET();
	}
	else if(pUSARTx == UART4)
	{
		UART4_REG_RESET();
	}
	else if(pUSARTx == UART5)
	{
		UART5_REG_RESET();
	}
	else if(pUSARTx == USART6)
	{
		USART6_REG_RESET();
	}
	else
	{
		//Error message
	}
}

void USART_IRQConfig(uint8_t IRQPosition, bool status)
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

/*********************************************************************
 * @fn      		  - USART_IRQPriorityConfig
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void USART_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
{
	//1. first lets find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section  = IRQNumber %4 ;

	uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PRIORITY_BITS_IMPLEMENTED) ;
	*(NVIC_PR_BASE_ADDR + iprx ) |=  ( IRQPriority << shift_amount );
}


/**********************************************************
 * @fn		- USART _GetFlagStatus
 *
 * @brief	- Polls for status of USART  peripheral
 *
 * @param1	- Specify which USART  peripheral you want to gain access of (USART1,USART2,USART3,UART4,UART5,USART6)
 * @param2	- Pass the flag to be checked
 *
 * @return	- none
 *
 * @note	- Check for TXE, RXE or BSY flag
 */
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint8_t flagName)
{
	if(pUSARTx->SR & flagName)	//Compare both bit-fields
	{
		return FLAG_SET;
	}

	return FLAG_CLEAR;
}


/*********************************************************************
 * @fn      		  - USART_ClearFlag
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - Applicable to only USART_CTS_FLAG , USART_LBD_FLAG
 * USART_TC_FLAG,USART_TC_FLAG flags
 *

 */

void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t StatusFlagName)
{
	pUSARTx->SR &= ~( StatusFlagName);

}


/*********************************************************************
 * @fn      		  - USART_SendData
 *
 * @brief             - API to send data from USART to external world
 *
 * @param[in]         - USART handle
 * @param[in]         - USART TX buffer
 * @param[in]         - USART data len
 *
 * @return            - None
 *
 * @Note              - Resolve all the TODOs

 */
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t len)
{

	uint16_t *pdata;
   //Loop over until "Len" number of bytes are transferred
	for(uint32_t i = 0 ; i < len; i++)
	{
		//Implement the code to wait until TXE flag is set in the SR
		while(! USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_FLAG_TXE));

         //Check the USART_WordLength item for 9BIT or 8BIT in a frame
		if(pUSARTHandle->USART_Config.wordLength == USART_WORDLEN_9BITS)
		{
			//if 9BIT, load the DR with 2bytes masking the bits other than first 9 bits
			pdata = (uint16_t*) pTxBuffer;
			pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);

			//check for USART_ParityControl
			if(pUSARTHandle->USART_Config.parityControl == USART_PARITY_DISABLE)
			{
				//No parity is used in this transfer. so, 9bits of user data will be sent
				//Implement the code to increment pTxBuffer twice
				pTxBuffer++;
				pTxBuffer++;
			}
			else
			{
				//Parity bit is used in this transfer . so , 8bits of user data will be sent
				//The 9th bit will be replaced by parity bit by the hardware
				pTxBuffer++;
			}
		}
		else
		{
			//This is 8bit data transfer
			pUSARTHandle->pUSARTx->DR = (*pTxBuffer  & (uint8_t)0xFF);

			//Implement the code to increment the buffer address
			pTxBuffer++;
		}
	}

	//Implement the code to wait till TC flag is set in the SR
	while( ! USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_FLAG_TC));
}


/*********************************************************************
 * @fn      		  - USART_ReceiveData
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */

void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t len)
{
   //Loop over until "Len" number of bytes are transferred
	for(uint32_t i = 0 ; i < len; i++)
	{
		//Implement the code to wait until RXNE flag is set in the SR
		while( ! USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_FLAG_RXNE));

		//Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
		if(pUSARTHandle->USART_Config.wordLength == USART_WORDLEN_9BITS)
		{
			//We are going to receive 9bit data in a frame

			//check are we using USART_ParityControl control or not
			if(pUSARTHandle->USART_Config.parityControl == USART_CR1_PCE)
			{
				//No parity is used. so, all 9bits will be of user data

				//read only first 9 bits. so, mask the DR with 0x01FF
				*((uint16_t*) pRxBuffer) = (pUSARTHandle->pUSARTx->DR  & (uint16_t)0x01FF);

				//Now increment the pRxBuffer two times
				pRxBuffer++;
				pRxBuffer++;
			}
			else
			{
				//Parity is used, so, 8bits will be of user data and 1 bit is parity
				 *pRxBuffer = (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);

				 //Increment the pRxBuffer
				 pRxBuffer++;
			}
		}
		else
		{
			//We are going to receive 8bit data in a frame

			//check are we using USART_ParityControl control or not
			if(pUSARTHandle->USART_Config.parityControl == USART_PARITY_DISABLE)
			{
				//No parity is used , so all 8bits will be of user data

				//read 8 bits from DR
				*pRxBuffer = (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);
			}

			else
			{
				//Parity is used, so , 7 bits will be of user data and 1 bit is parity

				//read only 7 bits , hence mask the DR with 0X7F
				 *pRxBuffer = (pUSARTHandle->pUSARTx->DR  & (uint8_t)0x7F);

			}

			//increment the pRxBuffer
			// Filtering data to receive only ASCII value less than 128
			if(*pRxBuffer <128)
				pRxBuffer++;
			else
				i--;	//If ascii is >=128 then shift back index
		}
	}

}


/*********************************************************************
 * @fn      		  - USART_SetBaudRate
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 *
 * @return            - None
 *
 * @Note              -

 */
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t baudRate)
{
	uint32_t PCLKx;
	uint32_t usartDiv;
	uint32_t mantissa, fraction;
	uint32_t tempreg=0;

	//1. Identify the clock rate on APBx bus
	if(pUSARTx == USART1||pUSARTx == USART6)
	{
		PCLKx = RCC_GetPClk2Value();
	}
	else
	{
		PCLKx = RCC_GetPClk1Value();
	}

	//2. Check for OVER8 configuration bit
	if(pUSARTx->CR1 & (1 << USART_CR1_OVER8))
	{
	   //OVER8 = 1 , over sampling by 8
		usartDiv = ((25 * PCLKx) / (2 *baudRate));
	}
	else
	{
	   //over sampling by 16
		usartDiv = ((25 * PCLKx) / (4 *baudRate));
	}


	//3. Extract the Mantissa part
	mantissa = usartDiv/100;

	//Place the Mantissa part in appropriate bit position . refer USART_BRR
	tempreg |= mantissa << 4;

	//4. Extract the fraction part
	fraction = (usartDiv - (mantissa * 100));

	//5. Calculate the final fractional
	if(pUSARTx->CR1 & ( 1 << USART_CR1_OVER8))
	{
	  //OVER8 = 1 , over sampling by 8
	  fraction = ((( fraction * 8)+ 50) / 100)& ((uint8_t)0x07);

	}
	else
	{
	   //over sampling by 16
	   fraction = ((( fraction * 16)+ 50) / 100) & ((uint8_t)0x0F);

	}

	//6. Place the fractional part in appropriate bit position . refer USART_BRR
	tempreg |= fraction;

	//7. copy the value of tempreg in to BRR register
	pUSARTx->BRR = tempreg;
}

/*********************************************************************
 * @fn      		  - USART_SendDataWithIntr
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - Resolve all the TODOs

 */
uint8_t USART_SendDataIntr(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t len)
{
	uint8_t txstate = pUSARTHandle->txBusyState;

	if(txstate != USART_BUSY_TX)
	{
		pUSARTHandle->txLen = len;
		pUSARTHandle->pTxBuffer = pTxBuffer;
		pUSARTHandle->txBusyState = USART_BUSY_TX;

		//Implement the code to enable interrupt for TXE
		pUSARTHandle->pUSARTx->CR1 |= (1<<USART_CR1_TXEIE);


		//Implement the code to enable interrupt for TC
		pUSARTHandle->pUSARTx->CR1 |= (1<<USART_CR1_TCIE);
	}

	return txstate;

}


/*********************************************************************
 * @fn      		  - USART_ReceiveDataIntr
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - Resolve all the TODOs

 */
uint8_t USART_ReceiveDataIntr(USART_Handle_t *pUSARTHandle,uint8_t *pRxBuffer, uint32_t len)
{
	uint8_t rxstate = pUSARTHandle->rxBusyState;

	if(rxstate != USART_BUSY_RX)
	{
		pUSARTHandle->rxLen = len;
		pUSARTHandle->pRxBuffer = pRxBuffer;
		pUSARTHandle->rxBusyState = USART_BUSY_RX;

		(void)pUSARTHandle->pUSARTx->DR;

		//Implement the code to enable interrupt for RXNE
		pUSARTHandle->pUSARTx->CR1 |= (1<<USART_CR1_RXNEIE);

	}

	return rxstate;
}

/*********************************************************************
 * @fn      		  - USART_IRQHandler
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
void USART_IRQHandling(USART_Handle_t *pUSARTHandle)
{

	uint32_t temp1 , temp2, temp3;

	uint16_t *pdata;

	/*************************Check for TC flag ********************************************/

    //Implement the code to check the state of TC bit in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_TC);

	 //Implement the code to check the state of TCEIE bit
	temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_TCIE);

	if(temp1 && temp2 )
	{
		//this interrupt is because of TC

		//close transmission and call application callback if TxLen is zero
		if ( pUSARTHandle->txBusyState == USART_BUSY_TX)
		{
			//Check the TxLen . If it is zero then close the data transmission
			if(! pUSARTHandle->txLen )
			{
				//Implement the code to clear the TC flag
				pUSARTHandle->pUSARTx->SR &= ~( 1 << USART_SR_TC);

				//Implement the code to clear the TCIE control bit

				//Reset the application state
				pUSARTHandle->txBusyState = USART_READY;

				//Reset Buffer address to NULL
				pUSARTHandle->pTxBuffer = NULL;

				//Reset the length to zero
				pUSARTHandle->txLen = 0;

				//Call the application call back with event USART_EVENT_TX_CMPLT
				USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_TX_CMPLT);
			}
		}
	}

	/*************************Check for TXE flag ********************************************/

	//Check the state of TXE bit in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_TXE);

	//Check the state of TXEIE bit in CR1
	temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_TXEIE);


	if(temp1 && temp2 )
	{
		//this interrupt is because of TXE

		if(pUSARTHandle->txBusyState == USART_BUSY_TX)
		{
			//Keep sending data until Txlen reaches to zero
			if(pUSARTHandle->txLen > 0)
			{
				//Check the USART_WordLength item for 9BIT or 8BIT in a frame
				if(pUSARTHandle->USART_Config.wordLength == USART_WORDLEN_9BITS)
				{
					//if 9BIT load the DR with 2bytes masking  the bits other than first 9 bits
					pdata = (uint16_t*) pUSARTHandle->pTxBuffer;
					pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);

					//check for USART_ParityControl
					if(pUSARTHandle->USART_Config.parityControl == USART_PARITY_DISABLE)
					{
						//No parity is used in this transfer , so 9bits of user data will be sent
						//Implement the code to increment pTxBuffer twice
						pUSARTHandle->pTxBuffer++;
						pUSARTHandle->pTxBuffer++;
						pUSARTHandle->txLen-=2;
					}
					else
					{
						//Parity bit is used in this transfer . so 8bits of user data will be sent
						//The 9th bit will be replaced by parity bit by the hardware
						pUSARTHandle->pTxBuffer++;
						pUSARTHandle->txLen-=1;
					}
				}
				else
				{
					//This is 8bit data transfer
					pUSARTHandle->pUSARTx->DR = (*pUSARTHandle->pTxBuffer  & (uint8_t)0xFF);

					//Implement the code to increment the buffer address
					pUSARTHandle->pTxBuffer++;
					pUSARTHandle->txLen-=1;
				}

			}
			if (pUSARTHandle->txLen == 0 )
			{
				//TxLen is zero
				//Implement the code to clear the TXEIE bit (disable interrupt for TXE flag )
				pUSARTHandle->pUSARTx->CR1 &= ~( 1 << USART_CR1_TXEIE);
			}
		}
	}

	/*************************Check for RXNE flag ********************************************/

	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_RXNE);
	temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_RXNEIE);


	if(temp1 && temp2 )
	{
		//this interrupt is because of rxne
		if(pUSARTHandle->rxBusyState == USART_BUSY_RX)
		{
			if(pUSARTHandle->rxLen > 0)
			{
				//Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
				if(pUSARTHandle->USART_Config.wordLength == USART_WORDLEN_9BITS)
				{
					//We are going to receive 9bit data in a frame

					//Now, check are we using USART_ParityControl control or not
					if(pUSARTHandle->USART_Config.parityControl == USART_PARITY_DISABLE)
					{
						//No parity is used , so all 9bits will be of user data

						//read only first 9 bits so mask the DR with 0x01FF
						*((uint16_t*) pUSARTHandle->pRxBuffer) = (pUSARTHandle->pUSARTx->DR  & (uint16_t)0x01FF);

						//Now increment the pRxBuffer two times
						pUSARTHandle->pRxBuffer++;
						pUSARTHandle->pRxBuffer++;
						pUSARTHandle->rxLen-=2;
					}
					else
					{
						//Parity is used, so 8bits will be of user data and 1 bit is parity
						 *pUSARTHandle->pRxBuffer = (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);
						 pUSARTHandle->pRxBuffer++;
						 pUSARTHandle->rxLen-=1;
					}
				}
				else
				{
					//We are going to receive 8bit data in a frame

					//Now, check are we using USART_ParityControl control or not
					if(pUSARTHandle->USART_Config.parityControl == USART_PARITY_DISABLE)
					{
						//No parity is used , so all 8bits will be of user data

						//read 8 bits from DR
						 *pUSARTHandle->pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);

					}

					else
					{
						//Parity is used, so , 7 bits will be of user data and 1 bit is parity

						//read only 7 bits , hence mask the DR with 0X7F
						 *pUSARTHandle->pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR  & (uint8_t)0x7F);

					}

					//Now , increment the pRxBuffer
					pUSARTHandle->pRxBuffer++;
					pUSARTHandle->rxLen-=1;
				}


			}//if of >0

			if(pUSARTHandle->rxLen==1)
			{
				//disable the rxne
				pUSARTHandle->pUSARTx->CR1 &= ~( 1 << USART_CR1_RXNEIE );
				pUSARTHandle->rxBusyState = USART_READY;
				USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_RX_CMPLT);
			}
		}
	}


	/*************************Check for CTS flag ********************************************/
	//Note : CTS feature is not applicable for UART4 and UART5

	//Implement the code to check the status of CTS bit in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_CTS);

	//Implement the code to check the state of CTSE bit in CR1
	temp2 = pUSARTHandle->pUSARTx->CR3 & ( 1 << USART_CR3_CTSE);

	//Implement the code to check the state of CTSIE bit in CR3 (This bit is not available for UART4 & UART5.)
	temp3 = pUSARTHandle->pUSARTx->CR3 & ( 1 << USART_CR3_CTSIE);


	if(temp1  && temp2 && temp3)
	{
		//Implement the code to clear the CTS flag in SR
		pUSARTHandle->pUSARTx->SR &=  ~( 1 << USART_SR_CTS);

		//this interrupt is because of cts
		USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_CTS);
	}

/*************************Check for IDLE detection flag ********************************************/

	//Implement the code to check the status of IDLE flag bit in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_IDLE);

	//Implement the code to check the state of IDLEIE bit in CR1
	temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_IDLEIE);


	if(temp1 && temp2)
	{
		//Implement the code to clear the IDLE flag. Refer to the RM to understand the clear sequence
		temp1 = pUSARTHandle->pUSARTx->SR &= ~( 1 << USART_SR_IDLE);

		//this interrupt is because of idle
		USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_IDLE);
	}

/*************************Check for Overrun detection flag ********************************************/

	//Implement the code to check the status of ORE flag  in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & USART_SR_ORE;

	//Implement the code to check the status of RXNEIE  bit in the CR1
	temp2 = pUSARTHandle->pUSARTx->CR1 & USART_CR1_RXNEIE;


	if(temp1  && temp2 )
	{
		//Need not to clear the ORE flag here, instead give an api for the application to clear the ORE flag .

		//this interrupt is because of Overrun error
		USART_ApplicationEventCallback(pUSARTHandle,USART_ERR_ORE);
	}



/*************************Check for Error Flag ********************************************/

//Noise Flag, Overrun error and Framing Error in multibuffer communication
//We dont discuss multibuffer communication in this course. please refer to the RM
//The blow code will get executed in only if multibuffer mode is used.

	temp2 =  pUSARTHandle->pUSARTx->CR3 & ( 1 << USART_CR3_EIE) ;

	if(temp2 )
	{
		temp1 = pUSARTHandle->pUSARTx->SR;
		if(temp1 & ( 1 << USART_SR_FE))
		{
			/*
				This bit is set by hardware when a de-synchronization, excessive noise or a break character
				is detected. It is cleared by a software sequence (an read to the USART_SR register
				followed by a read to the USART_DR register).
			*/
			USART_ApplicationEventCallback(pUSARTHandle,USART_ERR_FE);
		}

		if(temp1 & ( 1 << USART_SR_NF) )
		{
			/*
				This bit is set by hardware when noise is detected on a received frame. It is cleared by a
				software sequence (an read to the USART_SR register followed by a read to the
				USART_DR register).
			*/
			USART_ApplicationEventCallback(pUSARTHandle,USART_ERR_NE);
		}

		if(temp1 & ( 1 << USART_SR_ORE) )
		{
			USART_ApplicationEventCallback(pUSARTHandle,USART_ERR_ORE);
		}
	}
}



/*********************************************************************
 * @fn      		  - USART_ApplicationEventCallback
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
__attribute__((weak)) void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle,uint8_t event)
{

}

