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
				 *pRxBuffer = (uint8_t)0x7F;

			}

			//increment the pRxBuffer
			pRxBuffer++;
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

///*********************************************************************
// * @fn      		  - USART_SendDataWithIntr
// *
// * @brief             -
// *
// * @param[in]         -
// * @param[in]         -
// * @param[in]         -
// *
// * @return            -
// *
// * @Note              - Resolve all the TODOs
//
// */
//uint8_t USART_SendDataIntr(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t len)
//{
//	uint8_t txstate = pUSARTHandle->txState;
//
//	if(txstate != USART_BUSY_IN_TX)
//	{
//		pUSARTHandle->TODO = len;
//		pUSARTHandle->pTxBuffer = TODO;
//		pUSARTHandle->TxBusyState = TODO;
//
//		//Implement the code to enable interrupt for TXE
//		TODO
//
//
//		//Implement the code to enable interrupt for TC
//		TODO
//
//
//	}
//
//	return txstate;
//
//}
//
//
///*********************************************************************
// * @fn      		  - USART_ReceiveDataIntr
// *
// * @brief             -
// *
// * @param[in]         -
// * @param[in]         -
// * @param[in]         -
// *
// * @return            -
// *
// * @Note              - Resolve all the TODOs
//
// */
//uint8_t USART_ReceiveDataIintr(USART_Handle_t *pUSARTHandle,uint8_t *pRxBuffer, uint32_t len)
//{
//	uint8_t rxstate = pUSARTHandle->rxState;
//
//	if(rxstate != USART_BUSY_RX)
//	{
//		pUSARTHandle->RxLen = len;
//		pUSARTHandle->pRxBuffer = TODO;
//		pUSARTHandle->RxBusyState = TODO;
//
//		//Implement the code to enable interrupt for RXNE
//		TODO
//
//	}
//
//	return rxstate;
//}
