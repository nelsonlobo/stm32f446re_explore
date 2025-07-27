/*
 * 05_I2C_slaveTxIntr_LongString.c
 *
 *  Created on: Jul 27, 2025
 *      Author: Nelson Lobo
 */
#include "stm32f446xx.h"


#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

I2C_Handle_t I2C1handle;

void delay(void);
void btn_GPIO_Init(void);
void I2C1_GPIOInits(void);
void I2C1_Inits(void);

uint8_t some_data[] = "Hello World! I2CMaster TX...\n";
uint8_t rx_buff[32];

int main(void)
{
	uint8_t cmd_code,len;

//	SystemClock_Config();

	printf("Hello World! This is an I2C Program...\n");
	btn_GPIO_Init();
	I2C1_GPIOInits();
	I2C1_Inits();

	I2C_IRQConfig(IRQ_I2CEV_1,ENABLE);
	I2C_IRQConfig(IRQ_I2CER_1,ENABLE);

	I2C_SlaveEnableDisableCallbackEvents(I2C1, ENABLE);

	I2C_PeripheralControl(I2C1, ENABLE);

	//Set ack bit after setting PE=1
	I2C_ManageACK_bit(I2C1,ENABLE);
	while(1)
	{
		while(GPIO_ReadFromInputPin(GPIOC,GPIO_PIN_NUM13));
		delay(); // Debounce delay

		cmd_code = 0x51;
		while(I2C_MasterSendDataIntr(&I2C1handle, &cmd_code, 1,ARDUINO_SLAVE_ADDR,ENABLE)!= I2C_READY);
		while(I2C_MasterReceiveDataIntr(&I2C1handle, &len, 1,ARDUINO_SLAVE_ADDR,ENABLE)!= I2C_READY);
		I2C1handle.rxComplete = 0;

		cmd_code = 0x52;
		while(I2C_MasterSendDataIntr(&I2C1handle, &cmd_code, 1,ARDUINO_SLAVE_ADDR,ENABLE)!= I2C_READY);
		while(I2C_MasterReceiveDataIntr(&I2C1handle,rx_buff, len,ARDUINO_SLAVE_ADDR,DISABLE)!= I2C_READY);
		I2C1handle.rxComplete = 0;
		while(!I2C1handle.rxComplete);
		rx_buff[len+1] = '\0';
		printf("Length of string: %d\n",len);
		printf("Data: %s\n",rx_buff);
	}
}


void I2C1_EV_IRQHandler (void)
{
	I2C_EV_IRQHandling(&I2C1handle);
}

void I2C1_ER_IRQHandler (void)
{
	I2C_ER_IRQHandling(&I2C1handle);
}

void btn_GPIO_Init(void)
{
	GPIO_Handle_t GpioBtn;

	//Configure the User button GPIO pin
	GpioBtn.pGPIOx = GPIOC;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM13;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_INPUT;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GpioBtn);
}

void I2C1_GPIOInits(void)
{
	GPIO_Handle_t I2CPins;

	I2CPins.pGPIOx = GPIOB;
	I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	I2CPins.GPIO_PinConfig.GPIO_PinAltFnMode = 4;
	I2CPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//I2C1 SCL pin
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM8;
	GPIO_Init(&I2CPins);

	//I2C SDA pin
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM9;
	GPIO_Init(&I2CPins);
}

void I2C1_Inits(void)
{
	I2C1handle.pI2Cx = I2C1;
	I2C1handle.I2C_Config.I2C_AckControl 	= I2C_ACK_ENABLE;
	I2C1handle.I2C_Config.I2C_DeviceAddress = MY_I2C_ADDR;
	I2C1handle.I2C_Config.I2C_FMDutyCycle 	= I2C_FM_DUTY_2;
	I2C1handle.I2C_Config.I2C_SclSpeed 		= I2C_SCL_SPEED_SM100K;

	I2C_Init(&I2C1handle);
}

void delay(void)
{
	for(uint32_t i = 0; i< 500000;i++);
}
