/**
 ******************************************************************************
 * @file           : 01_USART_tx.c
 * @author         : Nelson Lobo
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 *
 ******************************************************************************
 */

//#include <stdint.h>
//#include<stdio.h>
//#include<string.h>
#include "stm32f446xx.h"

char msg[1024] = "UART Tx testing...\n\r";
USART_Handle_t usart2_handle;

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

void USART2_Init(void);
void USART2_GPIOInit(void);
void GPIO_ButtonInit(void);
void delay(void);


int main(void)
{
//	SystemClock_Config();

	GPIO_ButtonInit();

	USART2_GPIOInit();

    USART2_Init();

    USART_PeripheralControl(USART2,ENABLE);

    USART2->DR = 'A';

    while(1)
    {
		//wait till button is pressed
		while(GPIO_ReadFromInputPin(GPIOC,GPIO_PIN_NUM13) );

		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		USART_SendData(&usart2_handle,(uint8_t*)msg,strlen(msg));
    }
}

void USART2_Init(void)
{
	usart2_handle.pUSARTx = USART2;
	usart2_handle.USART_Config.baud = USART_STD_BAUD_115200;
	usart2_handle.USART_Config.hwFlowControl = USART_HW_FLOW_CTRL_NONE;
	usart2_handle.USART_Config.mode = USART_MODE_ONLY_TX;
	usart2_handle.USART_Config.numOfStopBits = USART_STOPBITS_1;
	usart2_handle.USART_Config.wordLength = USART_WORDLEN_8BITS;
	usart2_handle.USART_Config.parityControl = USART_PARITY_DISABLE;
	USART_Init(&usart2_handle);
}

void USART2_GPIOInit(void)
{
	GPIO_Handle_t usart_gpios;

	usart_gpios.pGPIOx = GPIOA;
	usart_gpios.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	usart_gpios.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	usart_gpios.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	usart_gpios.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	usart_gpios.GPIO_PinConfig.GPIO_PinAltFnMode =7;

	//USART2 TX
	usart_gpios.GPIO_PinConfig.GPIO_PinNumber  = GPIO_PIN_NUM_2;
	GPIO_Init(&usart_gpios);

	//USART2 RX
	usart_gpios.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_3;
	GPIO_Init(&usart_gpios);


}

void GPIO_ButtonInit(void)
{
	GPIO_Handle_t gpioBtn,gpioLed;

	//this is btn gpio configuration
	gpioBtn.pGPIOx = GPIOC;
	gpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_13;
	gpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_INPUT;
	gpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&gpioBtn);

	//this is led gpio configuration
	gpioLed.pGPIOx = GPIOA;
	gpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM5;
	gpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;
	gpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	gpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PClk_Ctrl(GPIOD,ENABLE);

	GPIO_Init(&gpioLed);

}

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}

