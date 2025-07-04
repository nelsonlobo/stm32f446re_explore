/*
 * stm32f446xx_gpio_driver.h
 *
 *  Created on: Jun 21, 2025
 *      Author: lenovo
 */

#ifndef INC_STM32F446XX_GPIO_DRIVER_H_
#define INC_STM32F446XX_GPIO_DRIVER_H_

#include "stdint.h"
#include "stdbool.h"
#include "stm32f446xx.h"

typedef struct
{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFnMode;
}GPIO_PinConfig_t;

typedef struct
{
	GPIO_RegDef_t *pGPIOx;
	GPIO_PinConfig_t GPIO_PinConfig;
}GPIO_Handle_t;


/*
 * APIs supported by this driver
 */
//Peripheral clock setup
void GPIO_PClk_Ctrl(GPIO_RegDef_t *pGPIOx, uint8_t status);

//GPIO init & deinit
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

//GPIO data read & write
bool GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber,bool state);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,uint16_t value);
void GPIO_TogglePin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber);

//IRQ and ISR related funcions
void GPIO_IRQConfig(uint8_t IRQPosition, bool status);
void GPIO_IRQPriorityConfig(uint8_t IRQPosition, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t pinNumber);

//Gpio Modes
#define GPIO_MODE_INPUT		0
#define GPIO_MODE_OUTPUT	1
#define GPIO_MODE_ALTFN		2
#define GPIO_MODE_ANLG		3
#define GPIO_MODE_IT_RT		4
#define GPIO_MODE_IT_FT		5
#define GPIO_MODE_IT_RFT	6

//GPIO Output types options
#define GPIO_OP_TYPE_PP		0
#define GPIO_OP_TYPE_OD		1

//GPIO Speed options
#define GPIO_SPEED_LOW		0
#define GPIO_SPEED_MED		1
#define GPIO_SPEED_FAST		2
#define GPIO_SPEED_HIGH		3

//GPIO PU/PD configurations
#define GPIO_NO_PUPD		0
#define GPIO_PIN_PU				1
#define GPIO_PIN_PD				2

//GPIO pin numbers
#define GPIO_PIN_NUM0		0
#define GPIO_PIN_NUM1		1
#define GPIO_PIN_NUM2		2
#define GPIO_PIN_NUM3		3
#define GPIO_PIN_NUM4		4
#define GPIO_PIN_NUM5		5
#define GPIO_PIN_NUM6		6
#define GPIO_PIN_NUM7		7
#define GPIO_PIN_NUM8		8
#define GPIO_PIN_NUM9		9
#define GPIO_PIN_NUM10		10
#define GPIO_PIN_NUM11		11
#define GPIO_PIN_NUM12		12
#define GPIO_PIN_NUM13		13
#define GPIO_PIN_NUM14		14
#define GPIO_PIN_NUM15		15


#endif /* INC_STM32F446XX_GPIO_DRIVER_H_ */
