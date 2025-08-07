/*
 * 02_USART_interrupt.c
 *
 *  Created on: Aug 6, 2025
 *      Author: Nelson Lobo
 */
#include "stm32f446xx.h"

//we have 3 different messages that we transmit to arduino
//you can by all means add more messages
char *msg[3] = {"Ian is a small Baby!", "Hello, How are you ?" , "Today is a Beautiful day!"};
//reply from arduino will be stored here
char rx_buf[1024] ;
USART_Handle_t usart3_handle;
//This flag indicates reception completion
uint8_t msg_index = 0;
void start_next_transfer(void);

void USART3_Init(void)
{
	usart3_handle.pUSARTx = USART3;
	usart3_handle.USART_Config.baud = USART_STD_BAUD_115200;
	usart3_handle.USART_Config.hwFlowControl = USART_HW_FLOW_CTRL_NONE;
	usart3_handle.USART_Config.mode = USART_MODE_TXRX;
	usart3_handle.USART_Config.numOfStopBits = USART_STOPBITS_1;
	usart3_handle.USART_Config.wordLength = USART_WORDLEN_8BITS;
	usart3_handle.USART_Config.parityControl = USART_PARITY_DISABLE;
	USART_Init(&usart3_handle);
}

void USART3_GPIOInit(void)
{
	GPIO_Handle_t usart_gpios;

	usart_gpios.pGPIOx = GPIOC;
	usart_gpios.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	usart_gpios.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	usart_gpios.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	usart_gpios.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	usart_gpios.GPIO_PinConfig.GPIO_PinAltFnMode =7;

	usart_gpios.GPIO_PinConfig.GPIO_PinNumber  = GPIO_PIN_NUM_10;
	GPIO_Init(&usart_gpios);

	usart_gpios.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_11;
	GPIO_Init(&usart_gpios);

}

void GPIO_ButtonInit(void)
{
	GPIO_Handle_t GPIOBtn,GpioLed;

	//this is btn gpio configuration
	GPIOBtn.pGPIOx = GPIOC;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_13;
//	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_INPUT;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GPIOBtn);

	//this is led gpio configuration
	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_5;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GpioLed);

}

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}
int main(void)
{
//	initialise_monitor_handles();

    GPIO_ButtonInit();
    GPIO_IRQConfig(IRQN_EXIT15_10,ENABLE);

	USART3_GPIOInit();
    USART3_Init();

    USART_IRQConfig(IRQ_USART3,ENABLE);

    USART_PeripheralControl(USART3,ENABLE);

    printf("Application is running\n");

    //do forever
    while(1)
    {
    }
	return 0;
}

// Button interrupt handler (example)
void EXTI15_10_IRQHandler(void)
{
    // Clear the interrupt pending bit
	GPIO_IRQHandling(GPIO_PIN_NUM13);
    // Only start a new transfer if the previous one is fully complete.
	start_next_transfer();

}
void USART3_IRQHandler(void)
{
	USART_IRQHandling(&usart3_handle);
}


// Function to initiate a transfer
void start_next_transfer(void)
{
    // Ensure the message index doesn't go out of bounds
    uint32_t len = strlen(msg[msg_index]);

    // 1. Start the receive operation first
    // This primes the receiver to listen for incoming data.
	USART_ReceiveDataIntr(&usart3_handle, (uint8_t *)rx_buf, len);

    // 2. Then, start the transmit operation.
    // The transmit operation will run concurrently via interrupts.
	USART_SendDataIntr(&usart3_handle, (uint8_t *)msg[msg_index], len);
}

void USART_ApplicationEventCallback( USART_Handle_t *pUSARTHandle,uint8_t ApEv)
{
    if (ApEv == USART_EVENT_TX_CMPLT)
    {
    	printf("Transmitted : %s\n", msg[msg_index]);
        printf("Transmission complete.\n");
        msg_index = (msg_index + 1) % 3;
    }
    else if (ApEv == USART_EVENT_RX_CMPLT)
    {
    	rx_buf[strlen(msg[msg_index])] = '\0';
        printf("Received: %s\nReception complete.\n---\n", rx_buf);
        memset(&rx_buf,0,sizeof(rx_buf));
    }
}
