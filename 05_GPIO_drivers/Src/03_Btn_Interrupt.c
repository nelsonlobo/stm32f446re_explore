/*
 * 03_Btn_Interrupt.c
 *
 *  Created on: Jun 26, 2025
 *      Author: Nelson Lobo
 */

#include "string.h"
#include "stm32f446xx.h"

void delay(void)
{
	for(uint32_t i = 0; i< 500000;i++);
}

int main (void)
{
	*NVIC_ISER0 |= (1<<4);
	*NVIC_ICER0 &= ~(1<<4);
	delay();
	*NVIC_ISER0 |= (1<<4);

	GPIO_Handle_t GpioLed,GpioBtn;
	memset(&GpioLed,0,sizeof(GpioLed));
	memset(&GpioBtn,0,sizeof(GpioBtn));

	//Configure the LED GPIO pin
	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM5;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PClk_Ctrl(GPIOA, ENABLE);
	GPIO_Init(&GpioLed);

	//Configure the User button GPIO pin
	GpioBtn.pGPIOx = GPIOC;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM13;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
//	GpioBtn.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;	//Not applicable in input mode
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	GPIO_PClk_Ctrl(GPIOC, ENABLE);
	GPIO_Init(&GpioBtn);

	/*
	 * Configure the priority of the interrupt to be generated on
	 * the respective pin, in this case we are configuring the
	 * interrupt to be generated on Port C pin 13 the user button.
	 * Since Pin 13 falls between 10 and 15 we set the IRQ_EXTI15_10
	 * register on the NVIC. We are setting a random value for priority
	 * which in this case is 7.
	 */
	GPIO_IRQPriorityConfig(IRQN_EXIT15_10,NVIC_IRQ_PRIO7);
	/*
	 * Once we are done configuring the priority. We should enable
	 * the respective ISERx register. In this case since we are
	 * setting PC13 for external interrupt, we have to set ISER1
	 * register since we are enabling the 40th byte in the EXTI register.
	 * There are all together 240 interrupts that can be handled
	 * by the NVIC which are organized in 4 bytes per IPRx register.
	 * Hence EXTI15_10 i.e. 40 is assigned to IPR10 register's first byte.
	 * Hence we select ISER1 which falls between 32-63 ISER value.
	 *
	 */
	GPIO_IRQConfig(IRQN_EXIT15_10,ENABLE);

	while(1)
	{
//		if(!GPIO_ReadFromInputPin(GPIOC,GPIO_PIN_NUM13))
//		{
//
//			delay();
//		}
	}
	return 0;
}


void EXTI15_10_IRQHandler(void)
{
	GPIO_IRQHandling(GPIO_PIN_NUM13);
	GPIO_TogglePin(GPIOA,GPIO_PIN_NUM5);
}
