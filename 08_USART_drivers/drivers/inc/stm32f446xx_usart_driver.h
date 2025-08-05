/*
 * stm32f446xx_usart.h
 *
 *  Created on: Aug 4, 2025
 *      Author: Nelson Lobo
 */

#ifndef INC_STM32F446XX_USART_DRIVER_H_
#define INC_STM32F446XX_USART_DRIVER_H_

//USART Baud rate calculations
// Case#1:
// Fck = 16MHz
// TxRx Baudrate = 9600
// OVER8 = 0
// USART_DIV = ?
// BAUD = Fck/(8*(2-OVER8)*USART_DIV)
// USART_DIV = 16M/(8*2*9600) = 104.1875
// USART_BRR consists of 4-bit fraction and 12bits Mantissa part
// In our case we need to write 104 into the Mantissa part and 1875 into the fraction part
// Fraction part: 0.1875 * 16(OVER8=0) = 0x3
// Mantissa part: 104 = 0x68
// USARTDIV = 0x683
//
// Case#2:
// Fck = 16MHz
// TxRx Baudrate = 115200
// OVER8 = 1
// USART_DIV = ?
// BAUD = Fck/(8*(2-OVER8)*USART_DIV)
// USART_DIV = 16M/(8*1*115200) = 17.361
// USART_BRR consists of 4-bit fraction and 12bits Mantissa part
// In our case we need to write 104 into the Mantissa part and 1875 into the fraction part
// Fraction part: 0.361 * 8(OVER8=0) = 2.88 = 0x3
// Mantissa part: 17 = 0x11
// USARTDIV = 0x113

/*
 * Configuration structure for USARTx peripheral
 */
typedef struct
{
	uint8_t 	mode;
	uint32_t 	baud;
	uint8_t 	numOfStopBits;
	uint8_t 	wordLength;
	uint8_t		parityControl;
	uint8_t 	hwFlowControl;
}USART_Config_t;

/*
 * Handle structure for USARTx peripheral
 */
typedef struct
{
	USART_RegDef_t 	*pUSARTx;
	USART_Config_t 	USART_Config;
	uint8_t 		txState;
	uint8_t			rxState;
}USART_Handle_t;


#define USART_READY							0
#define USART_BUSY_RX						1
#define USART_BUSY_TX						2

/*
 * @USART_Mode
 * Possible Options for USART_Mode
 */
#define USART_MODE_ONLY_TX					0
#define USART_MODE_ONLY_RX					1
#define USART_MODE_TXRX						2

/*
 *@USART_Baud
 *Possible options for USART_Baud
 */
#define USART_STD_BAUD_1200					1200
#define USART_STD_BAUD_2400					400
#define USART_STD_BAUD_9600					9600
#define USART_STD_BAUD_19200 				19200
#define USART_STD_BAUD_38400 				38400
#define USART_STD_BAUD_57600 				57600
#define USART_STD_BAUD_115200 				115200
#define USART_STD_BAUD_230400 				230400
#define USART_STD_BAUD_460800 				460800
#define USART_STD_BAUD_921600 				921600
#define USART_STD_BAUD_2M 					2000000
#define SUART_STD_BAUD_3M 					3000000

/*
 *@USART_ParityControl
 *Possible options for USART_ParityControl
 */
#define USART_PARITY_DISABLE   				0
#define USART_PARITY_EN_EVEN  				1
#define USART_PARITY_EN_ODD   				2


/*
 *@USART_WordLength
 *Possible options for USART_WordLength
 */
#define USART_WORDLEN_8BITS  				0
#define USART_WORDLEN_9BITS  				1

/*
 *@USART_NoOfStopBits
 *Possible options for USART_NoOfStopBits
 */
#define USART_STOPBITS_1     				0
#define USART_STOPBITS_0_5   				1
#define USART_STOPBITS_2     				2
#define USART_STOPBITS_1_5   				3

/*
 *@USART_HWFlowControl
 *Possible options for USART_HWFlowControl
 */
#define USART_HW_FLOW_CTRL_NONE    			0
#define USART_HW_FLOW_CTRL_CTS    			1
#define USART_HW_FLOW_CTRL_RTS    			2
#define USART_HW_FLOW_CTRL_CTS_RTS			3


/*
 *@USART_Flags
 *Possible options for USART_Flags
 */
#define USART_FLAG_CTS						(1<<USART_SR_CTS)
#define USART_FLAG_LBD						(1<<USART_SR_LBD)
#define USART_FLAG_TXE						(1<<USART_SR_TXE)
#define USART_FLAG_TC						(1<<USART_SR_TC)
#define USART_FLAG_RXNE						(1<<USART_SR_RXNE)
#define USART_FLAG_IDLE						(1<<USART_SR_IDLE)
#define USART_FLAG_ORE						(1<<USART_SR_ORE)
#define USART_FLAG_NF						(1<<USART_SR_NF)
#define USART_FLAG_FE						(1<<USART_SR_FE)
#define USART_FLAG_PE						(1<<USART_SR_PE)


/******************************************************************************************
 *								APIs supported by this driver
 *		 For more information about the APIs check the function definitions
 ******************************************************************************************/
/*
 * Peripheral Clock setup
 */
void USART_PClk_Ctrl(USART_RegDef_t *pUSARTx,uint8_t status);
void USART_PeripheralControl(USART_RegDef_t *pUSARTx,uint8_t status);

/*
 * Init and De-init
 */
void USART_Init(USART_Handle_t *pUSARTHandle);
void USART_DeInit(USART_RegDef_t *pUSARTx);

uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint8_t statusFlagName);
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t statusFlagName);

/*
 * USART Baudrate calculator
 */
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t baudRate);

/*
 * Data Send and Receive
 */
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t len);
void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t len);
uint8_t USART_SendDataIntr(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t len);
uint8_t USART_ReceiveDataIntr(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t len);


/*
 * IRQ Configuration and ISR handling
 */
void USART_IRQConfig(uint8_t IRQPosition, bool status);
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void USART_IRQHandling(USART_Handle_t *pHandle);

/*
 * Application callback
 */
void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle,uint8_t AppEv);

#endif /* INC_STM32F446XX_USART_DRIVER_H_ */
