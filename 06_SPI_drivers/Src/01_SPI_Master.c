/*
 * 01_SPI_Master.c
 *
 *  Created on: Jul 5, 2025
 *      Author: Nelson Lobo
 * Description: Implementation of SPI master mode without using Slave select and MISO pin
 * 		  Note: Please review the way SSM and SSI bits are configured for this operation
 * 		  Task: If you are evaluating this mode, please tinker with the SPI:CR1 registers to explore the waveforms
 */


#include <stdint.h>
#include <string.h>
#include "stm32f446xx.h"

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

void SPI1_GPIO_Init(void);
void SPI1_Configurations(void);
void btn_GPIO_Init(void);
void delay(void);
uint8_t SPI_verifyResponse(uint8_t ackByte);

char user_data[] = "Hello World,its a beautiful day!";
char user_rxd[100]; // Buffer to store the received text message from ESP32 (e.g., "STM32 string len: 32")

int main(void)
{
	btn_GPIO_Init();
	SPI1_GPIO_Init();
	SPI1_Configurations();

	while(1)
	{
		// --- TRANSACTION 1: STM32 sends its original data to ESP32 ---
		while(GPIO_ReadFromInputPin(GPIOC,GPIO_PIN_NUM13)); // Wait for first button press
		delay(); // Debounce delay
		SPI_PeripheralControl(SPI1, ENABLE); // Enable SPI peripheral (assert NSS)
		SPI_SendData(SPI1, (uint8_t *)user_data, strlen(user_data));
        while(SPI_GetFLagStatus(SPI1, SPI_BSY_FLAG)); // Wait until SPI is not busy
        SPI_PeripheralControl(SPI1, DISABLE); // Disable SPI peripheral (de-assert NSS)
	}
	return 0;
}

// (The rest of your existing functions: btn_GPIO_Init, SPI_GPIO_Init, SPI1_Configurations, SPI_verifyResponse, delay remain unchanged)
void btn_GPIO_Init(void)
{
	GPIO_Handle_t GpioBtn;

	//Configure the User button GPIO pin
	GpioBtn.pGPIOx = GPIOC;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM13;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_INPUT;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	GPIO_Init(&GpioBtn);
}

void SPI1_GPIO_Init(void)
{
	GPIO_Handle_t SPI1Pins;

	SPI1Pins.pGPIOx = GPIOA;
	SPI1Pins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPI1Pins.GPIO_PinConfig.GPIO_PinAltFnMode = 5;
	SPI1Pins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPI1Pins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPI1Pins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

//	//SPI_NSS
//	SPIPins.GPIO_PinConfig.GPIO_PinNumber = 4;
//	GPIO_Init(&SPIPins);

	//SPI_SCK
	SPI1Pins.GPIO_PinConfig.GPIO_PinNumber = 5;
	GPIO_Init(&SPI1Pins);

//	//SPI_MISO
//	SPIPins.GPIO_PinConfig.GPIO_PinNumber = 6;
//	GPIO_Init(&SPIPins);

	//SPI_MOSI
	SPI1Pins.GPIO_PinConfig.GPIO_PinNumber = 7;
	GPIO_Init(&SPI1Pins);

}


void SPI1_Configurations(void)
{
	SPI_Handle_t SPI1Handle;

	SPI1Handle.pSPIx = SPI1;
	SPI1Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FULL_DUPLEX;
	SPI1Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI1Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8;

	//Try tinkering with the following bits to see the waveform variations
	SPI1Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI1Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI1Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI1Handle.SPIConfig.SPI_SSM = SPI_SSM_EN;		//Since we are testing this code without a slave peripheral we ignore the Slave Select Pin by enabling this bit
	SPI1Handle.SPIConfig.SPI_SSI = SPI_SSI_EN;		//Enable this bit only if you are not configuring the Slave select pin

	SPI_Init(&SPI1Handle);
}


void delay(void)
{
	for(uint32_t i = 0; i< 500000;i++);
}
