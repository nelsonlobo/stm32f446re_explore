/*
 * main.c
 *
 *  Created on: Jun 26, 2025
 *      Author: lenovo
 */

#include "stm32f446xx.h"
#include "stm32f446xx_gpio_driver.h"

int main (void)
{
	return 0;
}

void EXTI0_IRQHandler(void)
{
	GPIO_IQRHandling(0);
}
