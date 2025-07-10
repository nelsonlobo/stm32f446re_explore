/*
 * 02_SPI_TXRX_Polling.c
 *
 *  Created on: Jul 5, 2025
 *      Author: Nelson Lobo
 */

/*
 * Using Arm's Semihosting feature
 * 1. Linker Argument settings:
 * -specs=rdimon.specs -lc -lrdimon
 *
 * 2. Debug configurations of your application
 * monitor arm semihosting enable
 *
 * 3. in main.c use below codes
 * extern void initialise_monitor_handles();
 * initialise_monitor_handles();
 *
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "stm32f446xx.h"

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

extern void initialise_monitor_handles();


#define NACK 					0xA5
#define ACK 					0xF5

//command codes
#define COMMAND_LED_CTRL        0x50
#define COMMAND_SENSOR_READ     0x51
#define COMMAND_LED_READ        0x52
#define COMMAND_PRINT			0x53 // Use this to request the print/length message
#define COMMAND_ID_READ 	    0x54
#define COMMAND_RELAY_TEXT      0x55 // New command for relaying text back

#define LED_ON     				1
#define LED_OFF    				0

#define LED_PIN					9

//arduino analog pins
#define ANALOG_PIN0   0
#define ANALOG_PIN1   1
#define ANALOG_PIN2   2
#define ANALOG_PIN3   3
#define ANALOG_PIN4   4
void SPI_GPIO_Init(void);
void SPI1_Configurations(void);
void btn_GPIO_Init(void);
void delay(void);
uint8_t SPI_verifyResponse(uint8_t ackByte);

char user_data[] = "Hello World,its a beautiful day!";
char user_rxd[100]; // Buffer to store the received text message from ESP32 (e.g., "STM32 string len: 32")

int main(void)
{
	initialise_monitor_handles();
	printf("Hello World!\nLets Begin with SPI testing\n");

	uint8_t dummy_write = 0xff;
	uint8_t dummy_read;

	btn_GPIO_Init();
	SPI_GPIO_Init();
	SPI1_Configurations();
	SPI_SSOEConfig(SPI1,ENABLE);

	while(1)
	{
		// --- TRANSACTION 1: STM32 sends its original data to ESP32 ---
		while(GPIO_ReadFromInputPin(GPIOC,GPIO_PIN_NUM13)); // Wait for first button press
		delay(); // Debounce delay
		SPI_PeripheralControl(SPI1, ENABLE); // Enable SPI peripheral (assert NSS)

	    //1. CMD_LED_CTRL  	<pin no(1)>     <value(1)>

		uint8_t commandcode = COMMAND_LED_CTRL;
		uint8_t ackbyte;
		uint8_t args[2];

		//send command
		SPI_SendData(SPI1,&commandcode,1);

		//do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI1,&dummy_read,1);

		//Send some dummy bits (1 byte) fetch the response from the slave
		SPI_SendData(SPI1,&dummy_write,1);

		//read the ack byte received
		SPI_ReceiveData(SPI1,&ackbyte,1);

		if( SPI_verifyResponse(ackbyte))
		{
			args[0] = LED_PIN;
			args[1] = LED_ON;

			//send arguments
			SPI_SendData(SPI1,args,2);
			// dummy read
			SPI_ReceiveData(SPI1,args,2);
			printf("COMMAND_LED_CTRL Executed\n");
		}
		//end of COMMAND_LED_CTRL


        // --- TRANSACTION 2: STM32 requests sensor information
		while(GPIO_ReadFromInputPin(GPIOC,GPIO_PIN_NUM13)); // Wait for second button press
		delay(); // Debounce delay

		commandcode = COMMAND_SENSOR_READ;

		//send command
		SPI_SendData(SPI1,&commandcode,1);

		//do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI1,&dummy_read,1);


		//Send some dummy byte to fetch the response from the slave
		SPI_SendData(SPI1,&dummy_write,1);

		//read the ack byte received
		SPI_ReceiveData(SPI1,&ackbyte,1);

		if( SPI_verifyResponse(ackbyte))
		{
			args[0] = ANALOG_PIN0;

			//send arguments
			SPI_SendData(SPI1,args,1); //sending one byte of

			//do dummy read to clear off the RXNE
			SPI_ReceiveData(SPI1,&dummy_read,1);

			//insert some delay so that slave can ready with the data
			delay();

			//Send some dummy bits (1 byte) fetch the response from the slave
			SPI_SendData(SPI1,&dummy_write,1);

			uint8_t analog_read;
			SPI_ReceiveData(SPI1,&analog_read,1);
			printf("COMMAND_SENSOR_READ %d\n",analog_read);
		}
		//End of Sensor read command


        // --- TRANSACTION 3: STM32 commands to control LEDs on arduino
		while(GPIO_ReadFromInputPin(GPIOC,GPIO_PIN_NUM13)); // Wait for third button press
		delay(); // De-bounce delay

		commandcode = COMMAND_LED_READ;

		//send command
		SPI_SendData(SPI1,&commandcode,1);

		//do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI1,&dummy_read,1);

		//Send some dummy byte to fetch the response from the slave
		SPI_SendData(SPI1,&dummy_write,1);

		//read the ack byte received
		SPI_ReceiveData(SPI1,&ackbyte,1);

		if( SPI_verifyResponse(ackbyte))
		{
			args[0] = LED_PIN;

			//send arguments
			SPI_SendData(SPI1,args,1); //sending one byte of

			//do dummy read to clear off the RXNE
			SPI_ReceiveData(SPI1,&dummy_read,1);

			//insert some delay so that slave can ready with the data
			delay();

			//Send some dummy bits (1 byte) fetch the response from the slave
			SPI_SendData(SPI1,&dummy_write,1);

			uint8_t led_status;
			SPI_ReceiveData(SPI1,&led_status,1);
			printf("COMMAND_READ_LED %d\n",led_status);

		}
		//End of LED read comm/and


		//4. CMD_PRINT 		<len(2)>  <message(len) >

		//wait till button is pressed
		while( ! GPIO_ReadFromInputPin(GPIOC,GPIO_PIN_NUM13) );

		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		commandcode = COMMAND_PRINT;

		//send command
		SPI_SendData(SPI1,&commandcode,1);

		//do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI1,&dummy_read,1);

		//Send some dummy byte to fetch the response from the slave
		SPI_SendData(SPI1,&dummy_write,1);

		//read the ack byte received
		SPI_ReceiveData(SPI1,&ackbyte,1);

		uint8_t message[] = "Hello ! How are you ??";
		if( SPI_verifyResponse(ackbyte))
		{
			args[0] = strlen((char*)message);

			//send arguments
			SPI_SendData(SPI1,args,1); //sending length

			//do dummy read to clear off the RXNE
			SPI_ReceiveData(SPI1,&dummy_read,1);

			delay();

			//send message
			for(int i = 0 ; i < args[0] ; i++){
				SPI_SendData(SPI1,&message[i],1);
				SPI_ReceiveData(SPI1,&dummy_read,1);
			}

			printf("COMMAND_PRINT Executed \n");

		}

		//5. CMD_ID_READ
		//wait till button is pressed
		while( ! GPIO_ReadFromInputPin(GPIOC,GPIO_PIN_NUM13) );

		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		commandcode = COMMAND_ID_READ;

		//send command
		SPI_SendData(SPI1,&commandcode,1);

		//do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI1,&dummy_read,1);

		//Send some dummy byte to fetch the response from the slave
		SPI_SendData(SPI1,&dummy_write,1);

		//read the ack byte received
		SPI_ReceiveData(SPI1,&ackbyte,1);

		uint8_t id[11];
		uint32_t i=0;
		if( SPI_verifyResponse(ackbyte))
		{
			//read 10 bytes id from the slave
			for(  i = 0 ; i < 10 ; i++)
			{
				//send dummy byte to fetch data from slave
				SPI_SendData(SPI1,&dummy_write,1);
				SPI_ReceiveData(SPI1,&id[i],1);
			}

			id[10] = '\0';

			printf("COMMAND_ID : %s \n",id);

		}

		//lets confirm SPI is not busy
		while( SPI_GetFlagStatus(SPI1, SPI_BSY_FLAG));

		//Disable the SPI1 peripheral
		SPI_PeripheralControl(SPI1,DISABLE);

		printf("SPI Communication Closed\n");


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

	GPIO_PClk_Ctrl(GPIOC, ENABLE);
	GPIO_Init(&GpioBtn);
}

void SPI_GPIO_Init(void)
{
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOA;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFnMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//SPI_NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = 4;
	GPIO_Init(&SPIPins);

	//SPI_SCK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = 5;
	GPIO_Init(&SPIPins);

	//SPI_MISO
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = 6;
	GPIO_Init(&SPIPins);

	//SPI_MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = 7;
	GPIO_Init(&SPIPins);

}


void SPI1_Configurations(void)
{
	SPI_Handle_t SPI1Handle;

	SPI1Handle.pSPIx = SPI1;
	SPI1Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FULL_DUPLEX;
	SPI1Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI1Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV32;

	SPI1Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI1Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI1Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;

	SPI1Handle.SPIConfig.SPI_SSM = SPI_SSM_DIS;
	SPI1Handle.SPIConfig.SPI_SSI = SPI_SSI_DIS;

	SPI_Init(&SPI1Handle);
}

uint8_t SPI_verifyResponse(uint8_t ackByte)
{
	if(ackByte == 0xF5)
	{
		return 1;
	}
	return 0;
}

void delay(void)
{
	for(uint32_t i = 0; i< 250000;i++);
}
