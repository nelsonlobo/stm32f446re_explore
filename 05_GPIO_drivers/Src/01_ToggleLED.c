/*
 * 01_ToggleLED.c
 *
 *  Created on: Jun 22, 2025
 *      Author: lenovo
 */

#include "stm32f446xx.h"
#include "stm32f446xx_gpio_driver.h"

void delay(void)
{
	for(uint32_t i = 0; i< 500000;i++);
}

int main (void)
{
	GPIO_Handle_t GpioLed;

	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM5;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	GPIO_PClk_Ctrl(GPIOA, ENABLE);
	GPIO_Init(&GpioLed);
	while(1)
	{
		GPIO_TogglePin(GPIOA,GPIO_PIN_NUM5);
		delay();
	}
	return 0;
}
