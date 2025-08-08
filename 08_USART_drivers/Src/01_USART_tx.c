#include "stm32f446xx.h"
#include <string.h>

// Handle for the USART3 peripheral
USART_Handle_t usart3_handle;

// Message to be transmitted
char tx_msg[] = "Hello from STM32F446RE!\r\n";
// Buffer to store received data
char rx_buf[64],rxVal;


void transmitUart(uint8_t data);
void transmitStrUart(char* str);
uint8_t receiveUart(void);
uint32_t receiveStrUart(char* rxBuffer, uint32_t maxLength);
void delay(void);
void USART3_GPIOInit(void);
void USART3_Init(void);
void GPIO_ButtonInit(void);
void MCO_gpioInit(void);



int main(void)
{
	SystemClock_Config();
	MCO_gpioInit();

    // Initialize all peripherals
    GPIO_ButtonInit();
    USART3_GPIOInit();
    USART3_Init();

    // Enable the USART3 peripheral.
    USART_PeripheralControl(USART3, ENABLE);

    printf("USART_Polling\n");

    // Main polling loop
    while (1)
    {

#ifdef SINGLE_BYTE_RXTX
    	//receives byte values and loops them back via TX pin
    	rxVal = receiveUart();
		if(rxVal > 0)
		{
			//transmitStrUart("Character Received\n\r");

			// TRANSMIT AN ARBITRARY 'U' (ASCII 85) OVER UART
			transmitUart(rxVal);
		}
#endif
        // Wait for a button press to start the communication cycle.
        if (!GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NUM_13))
        {
            // Debounce the button press with a small delay.
            delay();

            // Transmit the message using the new function.
            transmitStrUart(tx_msg);
//            USART_SendData(&usart3_handle, (uint8_t *)tx_msg, strlen(tx_msg));
            printf("TxData:%s\n",tx_msg);
            //IMP: On the reception side of the controller you should
            // add a delay of atleast 5ms based on the baudrate so that
            // you are comfortably prepared to receive data to be sent
            // from the other end(in our case ARDUINO BR:115200, delay = 5ms)

            //receiveStrUart(rx_buf, strlen(tx_msg));
            USART_ReceiveData(&usart3_handle, (uint8_t *)rx_buf, strlen(tx_msg));
            printf("RxData:%s\n",rx_buf);
            memset(&rx_buf,0,sizeof(rx_buf));
        }
    }
}


/*****************************************************************
 transmitUart

 This function sends a byte of data over UART.
*****************************************************************/
void transmitUart(uint8_t data)     // BYTE OF DATA TO BE TRANSMITTED
{
    // WAIT UNTIL TRANSMIT DATA REGISTER IS READY TO TAKE DATA
    // USART_ISR_TXE EXPANDS TO (1 << 7);
    while(!(USART3->SR & (1<<USART_SR_TXE)));

    // LOAD DATA TO TRANSMIT REGISTER
    USART3->DR = data;
}

/*****************************************************************
 transmitStrUart

 This function transmits a string over UART.
*****************************************************************/
void transmitStrUart(char* str) // POINTER TO A STRING TO BE TRANSMITTED
{
    // WHILE NOT END OF STRING
    while(*str)
    {
        // TRANSMIT CHARACTER THEN INCREMENT POINTER TO NEXT CHARACTER
        transmitUart(*str++);
    }
}

/*****************************************************************
 receiveUart

 This function reads a byte of data from the receive data register
 if there is data to be read.

 Returns
 a byte of data read from the receive data register
*****************************************************************/
uint8_t receiveUart(void)
{
    uint8_t rxData = 0;

    // IF THERE IS DATA IN THE RECEIVE DATA REGISTER
    // USART_ISR_RXNE EXPANDS TO (1 << 5)
    if(USART3->SR & (1<<USART_SR_RXNE))
    {
        // READ DATA FROM THE REGISTER
        rxData = USART3->DR;
    }

    return rxData;
}

/*****************************************************************
 receiveStrUart

 This function polls for incoming characters and stores them
 in a buffer until a newline character ('\n') is received.

 @param   rxBuffer - Pointer to the buffer to store the received string.
 @param   maxLength - The maximum size of the receive buffer.

 Returns: The number of characters received.
*****************************************************************/
uint32_t receiveStrUart(char* rxBuffer, uint32_t maxLength)
{
    uint32_t count = 0;

    // Polling loop to check for received data
    while (1)
    {
        // Check if there is data in the receive data register
        if (USART3->SR & (1 << USART_SR_RXNE))
        {
            // Store the character in the buffer if there's space
        	// Filtering data to receive only ASCII value less than 128
        	if(USART3->DR < 128)
        		rxBuffer[count++] = (char)USART3->DR;

	        if(count == maxLength)
	        	return count;
        }
    }
}


// Simple delay function
void delay(void)
{
    for (uint32_t i = 0; i < 500000/2; i++);
}

// Function to initialize the GPIO pins for USART3 (PC10, PC11)
void USART3_GPIOInit(void)
{
    GPIO_Handle_t usart_gpios;

    usart_gpios.pGPIOx = GPIOC;
    usart_gpios.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
    usart_gpios.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    usart_gpios.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
    usart_gpios.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    usart_gpios.GPIO_PinConfig.GPIO_PinAltFnMode = 7;

    // USART3 TX pin (PC10)
    usart_gpios.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_10;
    GPIO_Init(&usart_gpios);

    // USART3 RX pin (PC11)
    usart_gpios.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_11;
    GPIO_Init(&usart_gpios);
}

// Function to initialize the USART3 peripheral
void USART3_Init(void)
{
    usart3_handle.pUSARTx = USART3;
    usart3_handle.USART_Config.baud = USART_STD_BAUD_115200;
    usart3_handle.USART_Config.hwFlowControl = USART_HW_FLOW_CTRL_NONE;
    usart3_handle.USART_Config.mode = USART_MODE_TXRX;
    usart3_handle.USART_Config.numOfStopBits = USART_STOPBITS_1;
    usart3_handle.USART_Config.wordLength = USART_WORDLEN_8BITS;
    usart3_handle.USART_Config.parityControl = USART_PARITY_DISABLE;

    // Initialize the USART peripheral with the defined configurations.
    USART_Init(&usart3_handle);
}

// Function to initialize the user button (PC13)
void GPIO_ButtonInit(void)
{
    GPIO_Handle_t gpioBtn;

    gpioBtn.pGPIOx = GPIOC;
    gpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_13;
    gpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_INPUT;
    gpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    gpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
    GPIO_Init(&gpioBtn);
}


// Function to initialize the MCO2 (PC9)
void MCO_gpioInit(void)
{
	uint32_t clkSrc = ((RCC->CFGR >> RCC_CFGR_SWS) & 0x3);

    GPIO_Handle_t mcoPin;

    mcoPin.pGPIOx = GPIOC;
    mcoPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_9;
    mcoPin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
    mcoPin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
    mcoPin.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    mcoPin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
    mcoPin.GPIO_PinConfig.GPIO_PinAltFnMode = 0;
    GPIO_Init(&mcoPin);

    // 4. Set MCO2 source to SYSCLK (0b00) and prescaler to divide by 4 (0b110)
    // MCO2 is on bits 31:30 of RCC->CFGR, MCO2PRE is on bits 29:27
    // MCO2 source selection (0b00=SYSCLK, 0b01=PLLI2S, 0b10=HSE, 0b11=PLL)
    // MCO2 prescaler (0b000=no div, 0b100=div2, 0b101=div3, 0b110=div4, 0b111=div5)
    RCC->CFGR &= ~(0x3U << RCC_CFGR_MCO2); // Clear MCO2 source selection (bits 31:30)
    if(clkSrc >1)	//For PLL
    	RCC->CFGR |= (0x3U << RCC_CFGR_MCO2);
    if(clkSrc ==1)	//For HSE
    	RCC->CFGR |= (0x2U << RCC_CFGR_MCO2);
    // RCC->CFGR |= (0x0U << 30); // Select SYSCLK as MCO2 source (no change needed)

    RCC->CFGR &= ~(0x7U << RCC_CFGR_MCO2PRE); // Clear MCO2 prescaler selection (bits 29:27)
    RCC->CFGR |= (0x6U << RCC_CFGR_MCO2PRE); // Set prescaler to divide by 4 (0b110)
}

