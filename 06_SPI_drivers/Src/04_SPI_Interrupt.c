/*
 * 04_SPI_Interrupt.c
 *
 *  Created on: Jul 10, 2025
 *      Author: Nelson Lobo
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "stm32f446xx.h"

void delay(void);
void SystemClock_Config(void);
void SPI1_GPIO_Init(void);
void SPI1_Configurations(void);
void Slave_gpioIntrPinInit(void);
//extern void initialise_monitor_handles();
static inline void enable_global_interrupts(void);
static inline void disable_global_interrupts(void);


#define MAX_LEN	500
char RcvBuff[MAX_LEN];
volatile char ReadByte;
volatile uint8_t rcvStop = 0;
volatile uint8_t dataAvailable = 0;

SPI_Handle_t SPI1handle;
int main(void)
{
//	initialise_monitor_handles();
	SystemClock_Config();

	uint8_t dummy = 0xff;
	printf("Hello World!\n");


	SPI1_GPIO_Init();
	Slave_gpioIntrPinInit();

	SPI1_Configurations();
	SPI_SSOEConfig(SPI1,ENABLE);
	SPI_IRQPriorityConfig(IRQN_SPI1, NVIC_IRQ_PRIO2);
	SPI_IRQConfig(IRQN_SPI1, ENABLE);

	enable_global_interrupts();

	while(1)
	{
		rcvStop = 0;

		while(!dataAvailable);

		GPIO_IRQConfig(IRQN_EXIT9_5, DISABLE);

		SPI_PeripheralControl(SPI1,ENABLE);

		while(!rcvStop)
		{
			while(SPI_SendDataIntr(&SPI1handle, &dummy, 1) == SPI_TX_BSY);
			while(SPI_ReceiveDataIntr(&SPI1handle,(uint8_t *)&ReadByte, 1) == SPI_RX_BSY);
		}

		while(SPI_GetFlagStatus(SPI1, SPI_BSY_FLAG));

		SPI_PeripheralControl(SPI1, DISABLE);

		printf("Rcvd data:%s\n",RcvBuff);

		dataAvailable = 0;

		GPIO_IRQConfig(IRQN_EXIT9_5, ENABLE);
	}

	return 0;
}

// Function to enable global interrupts
static inline void enable_global_interrupts(void) {
    __asm volatile ("CPSIE i"); // Enable interrupts
}

// Function to disable global interrupts (for completeness)
static inline void disable_global_interrupts(void) {
    __asm volatile ("CPSID i"); // Disable interrupts
}

/**********************************************************
 * @fn		- SPI1_IRQHandler
 *
 * @brief	- ISR to handle SPI1 interrupt routines
 *
 * @return	- none
 *
 * @note	- none
 */
void SPI1_IRQHandler(void)
{
	SPI_IRQHandling(&SPI1handle);
}

/**********************************************************
 * @fn		- 	EXTI9_5_IRQHandler
 *
 * @brief	- 	External Interrupt for GPIO ISR routine
 * 				to handle GPIO5 - GPIO9 related interrupt routines
 *
 * @return	- 	none
 *
 * @note	- 	none
 */
void EXTI9_5_IRQHandler(void)
{
	GPIO_IRQHandling(GPIO_PIN_NUM7);
	dataAvailable = 1;
}


/**********************************************************
 * @fn		- External Interrupt for GPIO ISR routine
 *
 * @brief	- ISR to handle GPIO5 - GPIO9 related interrupt routines
 *
 * @return	- none
 *
 * @note	- none
 */
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t evntVal)
{
	static uint32_t i = 0;

	if(evntVal == SPI_EVENT_RX_COMPLETE)
	{
		RcvBuff[i++] = ReadByte;
		if(ReadByte == '\0'|| i==MAX_LEN)
		{
			rcvStop = 1;
			RcvBuff[i-1] = '\0';
			i = 0;
		}
	}
}

/**********************************************************
 * @fn		- Slave_gpioIntrPinInit
 *
 * @brief	- Initialization routines for Slave pin to receive an interrupt
 *
 * @return	- none
 *
 * @note	- none
 */
void Slave_gpioIntrPinInit(void)
{
	GPIO_Handle_t spiIntPin;
	memset(&spiIntPin,0,sizeof(spiIntPin));
	//Configure the User button GPIO pin
	spiIntPin.pGPIOx = GPIOC;
	spiIntPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM7;
	spiIntPin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	spiIntPin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
	spiIntPin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&spiIntPin);
	GPIO_IRQPriorityConfig(IRQN_EXIT9_5, NVIC_IRQ_PRIO3);
	GPIO_IRQConfig(IRQN_EXIT9_5, ENABLE);
}

/**********************************************************
 * @fn		- Slave_gpioIntrPinInit
 *
 * @brief	- Initialization routines for Slave pin to receive an interrupt
 *
 * @return	- none
 *
 * @note	- none
 */
void SPI1_GPIO_Init(void)
{
	GPIO_Handle_t SPI1Pins;

	SPI1Pins.pGPIOx = GPIOA;
	SPI1Pins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPI1Pins.GPIO_PinConfig.GPIO_PinAltFnMode = 5;
	SPI1Pins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPI1Pins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPI1Pins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//SPI_NSS
	SPI1Pins.GPIO_PinConfig.GPIO_PinNumber = 4;
	GPIO_Init(&SPI1Pins);

	//SPI_SCK
	SPI1Pins.GPIO_PinConfig.GPIO_PinNumber = 5;
	GPIO_Init(&SPI1Pins);

	//SPI_MISO
	SPI1Pins.GPIO_PinConfig.GPIO_PinNumber = 6;
	GPIO_Init(&SPI1Pins);

	//SPI_MOSI
	SPI1Pins.GPIO_PinConfig.GPIO_PinNumber = 7;
	GPIO_Init(&SPI1Pins);

}

/**********************************************************
 * @fn		- SPI1_Configurations
 *
 * @brief	- SPI1 configuration routines
 *
 * @return	- none
 *
 * @note	- Make sure to use globally declared handles
 */
void SPI1_Configurations(void)
{
	SPI1handle.pSPIx = SPI1;
	SPI1handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FULL_DUPLEX;
	SPI1handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI1handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV32;

	SPI1handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI1handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI1handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;

	SPI1handle.SPIConfig.SPI_SSM = SPI_SSM_DIS;
	SPI1handle.SPIConfig.SPI_SSI = SPI_SSI_DIS;

	SPI_Init(&SPI1handle);
}


/**********************************************************
 * @fn		- Delays
 *
 * @brief	- A dummy implementation of delays
 *
 * @return	- none
 *
 * @note	- Make sure to use globally declared handles
 */
void delay(void)
{
	for(uint32_t i = 0; i< 250000;i++);
}


/*
 * SystemClock_Config(): Configures the microcontroller's system clock.
 * This function is designed to be placed in a .c file (e.g., 04_SPI_Interrupt.c).
 * It will use the SYSCLK_FREQ_... macros defined above to select the clock source.
 *
 * It's crucial that this function is called once at the very beginning of your main() function.
 *
 * For higher frequencies (e.g., >24MHz HCLK), ensure voltage scaling (PWR->CR) and
 * Flash latency (FLASH->ACR) are correctly set PRIOR to switching the system clock.
 * The commented-out PLL_48MHZ section shows an example for this.
 */

void SystemClock_Config(void)
{
    // 1. Enable Power Control clock (always good to do early)
    RCC->APB1ENR |= (1U << 28); // PWR_CLK_ENABLE() (Bit 28 for PWR in APB1ENR)
    // 2. Configure the main internal regulator output voltage (essential for higher frequencies)
    PWR->CR |= PWR_REGULATOR_VOLTAGE_SCALE3; // Setting VOS[1:0] to 01 for Scale 3 (1.2V)

#if defined(SYSCLK_FREQ_HSI_8MHZ) || defined(SYSCLK_FREQ_HSI_16MHZ)

    // --- Option 1: Internal Oscillator (HSI) at 8MHz ---
    // HSI = 16MHz. HCLK = 8MHz. PCLK1/PCLK2 = 8MHz.
    // This is a very safe and simple configuration.

    // Enable HSI (Internal High-Speed oscillator)
    RCC->CR |= (1U << 0); // HSION bit (Bit 0)
    while (!(RCC->CR & (1U << 1))); // Wait for HSIRDY bit (Bit 1)

    // Configure Flash Latency (for 8MHz HCLK, 0 wait states is correct)
    FLASH->ACR &= ~(0xF << 0); // Clear LATENCY bits (Bits 3:0)
    FLASH->ACR |= (0x0 << 0);  // Set 0 wait states

    // Configure AHB and APB bus prescalers
    RCC->CFGR = 0x00000000U; // Clear CFGR register

    // Set AHB prescaler (HPRE) to divide by 2 (HCLK = SYSCLK / 2) -> 16MHz / 2 = 8MHz
    RCC->CFGR |= (0x8U << 4); // HPRE = /2 (binary 1000)

    // Set APB1 prescaler (PPRE1) to divide by 1 (PCLK1 = HCLK / 1 = 8MHz)
    RCC->CFGR |= (0x0U << 10); // PPRE1 = /1

    // Set APB2 prescaler (PPRE2) to divide by 1 (PCLK2 = HCLK / 1 = 8MHz)
    RCC->CFGR |= (0x0U << 13); // PPRE2 = /1

    // Select HSI as the system clock source (SW[1:0] bits 0-1)
    RCC->CFGR &= ~(0x3U << 0); // Clear SW bits
    RCC->CFGR |= (0x0U << 0);  // Select HSI as SYSCLK source (binary 00)

    // Wait for SYSCLK to switch to HSI (SWS[1:0] bits 2-3)
    while (!((RCC->CFGR & (0x3U << 2)) == (0x0U << 2))); // Wait for SWS bits to show HSI
#endif

#ifdef SYSCLK_FREQ_HSI_8MHZ
    // HCLK = 16MHz / 2 = 8MHz
    RCC->CFGR |= (0x8U << 4); // HPRE = /2 (binary 1000)
#elif defined(SYSCLK_FREQ_HSI_16MHZ)
    // HCLK = 16MHz / 1 = 16MHz
    RCC->CFGR |= (0x0U << 4); // HPRE = /1 (binary 0000)
#elif defined(SYSCLK_FREQ_HSE_16MHZ)
    // --- Option 2: External Oscillator (HSE) at 16MHz ---
    // HCLK = 16MHz. PCLK1/PCLK2 = 16MHz. Requires a 16MHz crystal.

    // 1. Enable HSE (High-Speed External) oscillator
    RCC->CR &= ~(1U << 16); // Clear HSEON bit
#ifdef HSE_BYPASS_MODE_ENABLE
    // If using external clock signal (not crystal), set bypass mode
    RCC->CR |= (1U << 18); // HSEBYP bit (Bit 18)
#else
    // If using a crystal, ensure bypass is cleared
    RCC->CR &= ~(1U << 18); // HSEBYP bit (Bit 18)
#endif
    RCC->CR |= (1U << 16); // HSEON bit (Enable HSE)
    while (!(RCC->CR & (1U << 17))); // Wait for HSERDY bit (Bit 17)

    // 2. Configure Flash Latency (for 16MHz HCLK, 0 wait states is correct)
    FLASH->ACR &= ~(0xF << 0); // Clear LATENCY bits
    FLASH->ACR |= (0x0 << 0);  // Set 0 wait states

    // 3. Configure AHB and APB bus prescalers
    RCC->CFGR = 0x00000000U; // Clear CFGR register

    // Set AHB prescaler (HPRE) to divide by 1 (HCLK = SYSCLK / 1 = 16MHz)
    RCC->CFGR |= (0x0U << 4); // HPRE = /1

    // Set APB1 prescaler (PPRE1) to divide by 1 (PCLK1 = HCLK / 1 = 16MHz)
    RCC->CFGR |= (0x0U << 10); // PPRE1 = /1

    // Set APB2 prescaler (PPRE2) to divide by 1 (PCLK2 = HCLK / 1 = 16MHz)
    RCC->CFGR |= (0x0U << 13); // PPRE2 = /1

    // 4. Select HSE as the system clock source (SW[1:0] bits 0-1)
    RCC->CFGR &= ~(0x3U << 0); // Clear SW bits
    RCC->CFGR |= (0x1U << 0);  // Select HSE as SYSCLK source (binary 01)

    // 5. Wait for SYSCLK to switch to HSE (SWS[1:0] bits 2-3)
    while (!((RCC->CFGR & (0x3U << 2)) == (0x1U << 2))); // Wait for SWS bits to show HSE

#elif defined(SYSCLK_FREQ_HSE_PLL_48MHZ)
    // --- Option 3: External Oscillator (HSE) with PLL at 48MHz ---
    // HCLK = 48MHz. PCLK1 = 24MHz. PCLK2 = 48MHz. Requires a 16MHz crystal.
    // For higher frequencies, voltage scaling and flash latency are crucial.

    // 1. Start HSE (High-Speed External) oscillator
    RCC->CR &= ~(1U << 16); // Clear HSEON bit
#ifdef HSE_BYPASS_MODE_ENABLE
    RCC->CR |= (1U << 18); // HSEBYP bit
#else
    RCC->CR &= ~(1U << 18); // HSEBYP bit
#endif
    RCC->CR |= (1U << 16); // HSEON bit (Enable HSE)
    while (!(RCC->CR & (1U << 17))); // Wait for HSERDY bit

    // 2. Configure PLL: Ensure PLL is OFF before reconfiguring
    RCC->CR &= ~(1U << 24); // Clear PLLON bit
    // Optional: Add a small delay to ensure PLL is fully off if it was previously on
    // for(volatile uint32_t i=0; i<100; i++);

    // 3. Clear and configure PLLCFGR with new values
    // HSE = 16MHz, target SYSCLK = 48MHz
    // PLLM = 8 (16MHz / 8 = 2MHz PLL input, within 1-2MHz range)
    // PLLN = 192 (2MHz * 192 = 384MHz VCO output, within 100-432MHz range)
    // PLLP = DIV8 (384MHz / 8 = 48MHz SYSCLK)
    // PLLQ = 2 (common for USB)
    // PLLR = 2 (common for audio/SPDIF)

    RCC->PLLCFGR = 0x00000000U; // Clear for fresh configuration
    RCC->PLLCFGR |= (1U << 22); // PLLSRC bit (bit 22) set for HSE source
    RCC->PLLCFGR |= (8U << 0);         // PLLM = 8 (bits 5:0)
    RCC->PLLCFGR |= (192U << 6);       // PLLN = 192 (bits 14:6)
    RCC->PLLCFGR |= (3U << 16);        // PLLP = DIV8 (bits 17:16 set to 11 for /8)
                                        // RCC_PLLP_DIV8 is (3U << 16)
    RCC->PLLCFGR |= (2U << 24);        // PLLQ = 2 (bits 27:24)
    RCC->PLLCFGR |= (2U << 28);        // PLLR = 2 (bits 31:28)

    // 4. Enable PLL
    RCC->CR |= (1U << 24); // PLLON bit (Enable PLL)

    // 5. Wait for PLL to be ready
    while (!(RCC->CR & (1U << 25))); // PLLRDY bit (Bit 25)

    // 6. Set Flash Latency for 48MHz HCLK
    // For 48MHz HCLK, 1 wait state (0x1) is typically required for VCORE Scale 3.
    FLASH->ACR &= ~(0xF << 0); // Clear LATENCY bits
    FLASH->ACR |= (0x1 << 0);  // Set 1 wait state

    // 7. Configure AHB and APB bus prescalers for 48MHz HCLK
    RCC->CFGR &= ~(0xF << 4); // Clear HPRE bits
    RCC->CFGR |= (0x0U << 4); // HPRE = /1 (HCLK = SYSCLK / 1 = 48MHz)

    RCC->CFGR &= ~(0x7 << 10); // Clear PPRE1 bits
    RCC->CFGR |= (0x4U << 10); // PPRE1 = /2 (PCLK1 = HCLK / 2 = 24MHz)

    RCC->CFGR &= ~(0x7 << 13); // Clear PPRE2 bits
    RCC->CFGR |= (0x0U << 13); // PPRE2 = /1 (PCLK2 = HCLK / 1 = 48MHz)

    // 8. Select PLLCLK as SYSCLK source
    RCC->CFGR &= ~(0x3U << 0); // Clear SW bits
    RCC->CFGR |= (0x2U << 0);  // Select PLLCLK as SYSCLK source (binary 10)

    // 9. Wait for SYSCLK to switch to PLLCLK
    while (!((RCC->CFGR & (0x3U << 2)) == (0x2U << 2))); // Wait for SWS bits to show PLLCLK

#else
    // Default or Error Handling (if no clock option is selected)
    // This part runs if none of the SYSCLK_FREQ_... macros are defined.
    // It's good to have a basic HSI clock or call Error_Handler here.
    // For example, falling back to HSI 16MHz default:
    RCC->CR |= (1U << 0); // HSION
    while (!(RCC->CR & (1U << 1))); // HSIRDY
    RCC->CFGR = 0x00000000U; // HPRE=/1, PPRE1=/1, PPRE2=/1, SW=HSI (default after reset)
    FLASH->ACR &= ~(0xF << 0); // Clear LATENCY bits
    FLASH->ACR |= (0x0 << 0);  // 0 wait states for 16MHz

#endif /* SYSCLK_FREQ_... */
}
