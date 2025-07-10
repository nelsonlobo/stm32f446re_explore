/*
 * 04_SPI_TX_test.c
 *
 * Created on: Jul 1, 2025
 * Author: Nelson Lobo
 */

/*
 * PA4	--> SPI1_NSS
 * PA5	--> SPI1_SCK
 * PA6	--> SPI1_MISO
 * PA7	--> SPI1_MOSI
 */
#include "string.h"
#include "stm32f446xx.h"

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

#define LED_PIN					13

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
	uint8_t dummy_write = 0xff;
	uint8_t dummy_read;

	btn_GPIO_Init();
	SPI_GPIO_Init();
	SPI1_Configurations();
	SPI_SSOEConfig(SPI1,ENABLE);

	uint8_t cmdCode;
//	uint8_t ackByte;
//	uint8_t args[2] = {0};

	while(1)
	{
		// --- TRANSACTION 1: STM32 sends its original data to ESP32 ---
		while(GPIO_ReadFromInputPin(GPIOC,GPIO_PIN_NUM13)); // Wait for first button press
		delay(); // Debounce delay
		SPI_PeripheralControl(SPI1, ENABLE); // Enable SPI peripheral (assert NSS)

        uint8_t data_len = strlen(user_data); // Get the length of the string to send

        // Send the length of the string as the first byte
        SPI_SendData(SPI1, (uint8_t *)&data_len, 1);
        // Send the string data
        SPI_SendData(SPI1, (uint8_t *)user_data, data_len);


        while(SPI_GetFLagStatus(SPI1, SPI_BSY_FLAG));

        // Wait until SPI is not busy after TX
		SPI_PeripheralControl(SPI1, DISABLE);
		// Disable SPI peripheral (de-assert NSS)

        // --- TRANSACTION 2: STM32 requests and receives reply from ESP32 ---
		while(GPIO_ReadFromInputPin(GPIOC,GPIO_PIN_NUM13)); // Wait for second button press
		delay(); // Debounce delay

		SPI_PeripheralControl(SPI1, ENABLE); // Re-enable SPI peripheral (assert NSS)

        // Send a command to signal ESP32 to send its prepared reply.
        cmdCode = COMMAND_PRINT;
        SPI_SendData(SPI1, &cmdCode, 1); // Send the command (e.g., "give me the reply")

        // Dummy read to clear RXNE before receiving actual data.
        SPI_ReceiveData(SPI1, &dummy_read, 1);

        memset(&user_rxd, 0, sizeof(user_rxd)); // Clear receive buffer before reception

        // Receive the text message from ESP32 (e.g., "STM32 string len: 32")
        for (int i = 0; i < (sizeof(user_rxd) -1); i++) // -1 for null termination
        {
            SPI_SendData(SPI1, &dummy_write, 1); // Send a dummy byte to generate clock pulses
            SPI_ReceiveData(SPI1, (uint8_t *)&user_rxd[i], 1); // Receive one byte

            if (user_rxd[i] == '\0' || user_rxd[i] == '\n') { // Check for null terminator or newline
                break;
            }
        }
        user_rxd[sizeof(user_rxd) -1] = '\0'; // Ensure null-termination

		while(SPI_GetFLagStatus(SPI1, SPI_BSY_FLAG)); // Wait until SPI is not busy after RX
		SPI_PeripheralControl(SPI1, DISABLE); // Disable SPI peripheral (de-assert NSS)

        // At this point, user_rxd should contain the reply "STM32 string len: 32"
        // You would typically output this to a UART console or debugger for verification.
        // Example: printf("Received from ESP32: %s\r\n", user_rxd);


        // --- TRANSACTION 3: STM32 relays the received text back to ESP32 ---
        while(GPIO_ReadFromInputPin(GPIOC,GPIO_PIN_NUM13)); // Wait for third button press
		delay(); // Debounce delay

		SPI_PeripheralControl(SPI1, ENABLE); // Re-enable SPI peripheral (assert NSS)

        // Send a command to signal ESP32 that this is the relayed text
        cmdCode = COMMAND_RELAY_TEXT;
        SPI_SendData(SPI1, &cmdCode, 1);

        // Send the content of user_rxd (the message received from ESP32) back to ESP32
        SPI_SendData(SPI1, (uint8_t *)user_rxd, strlen(user_rxd) + 1); // +1 to send the null terminator

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
	for(uint32_t i = 0; i< 500000;i++);
}
