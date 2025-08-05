/*
 * stm32f446xx_rcc_driver.c
 *
 *  Created on: Jul 24, 2025
 *      Author: Nelson Lobo
 */

#include "stm32f446xx.h"

uint16_t AHBPrescalar[8] = {2,4,8,16,64,128,256,512};
uint16_t APB1Prescalar[4] = {2,4,8,16};
uint16_t APB2Prescalar[4] = {2,4,8,16};

uint32_t RCC_GetPClk1Value(void)
{
	uint32_t pclk1,SystemClk;
	uint8_t clkSrc,temp,ahbp,apb1p;
	clkSrc = ((RCC->CFGR>>RCC_CFGR_SWS_BIT) & 0x3);

	if(clkSrc==0)
	{
		SystemClk = 16000000;
	}
	else if(clkSrc==1)
	{
		SystemClk = 8000000;
	}
	else if(clkSrc ==2)
	{
//		SystemClk = RCC_GetPllOutputClock();
	}


	/*
	 * Fetch the value for HPRE in RCC's CFGR register
	 * which indicates the prescalar used for AHB
	 * where bits 4-7 are configured
	 */
	temp = ((RCC->CFGR >>RCC_CFGR_HPRE_BIT)&0xF);
	if(temp<8)
	{
		ahbp = 1;
	}
	else
	{
		ahbp = AHBPrescalar[temp-8];
	}

	/*
	 * Fetch the value for HPRE in RCC's CFGR register
	 * which indicates the prescalar used for APB1 Low speed prescalar
	 * where bits 10-12 are configured
	 */
	temp = ((RCC->CFGR >>RCC_CFGR_PPRE1_BIT)&0x7);
	if(temp<4)
	{
		apb1p = 1;
	}
	else
	{
		ahbp = APB1Prescalar[temp-4];
	}

	pclk1 = (SystemClk/ahbp)/apb1p;

	return pclk1;
}


uint32_t RCC_GetPClk2Value(void)
{
	uint32_t pclk2,SystemClk;
	uint8_t clkSrc,temp,ahbp,apb2p;

	//1. Identify the clock source
	clkSrc = ((RCC->CFGR>>RCC_CFGR_SWS_BIT) & 0x3);

	if(clkSrc==0)
	{
		SystemClk = 16000000;
	}
	else if(clkSrc==1)
	{
		SystemClk = 8000000;
	}
	else if(clkSrc ==2)
	{
//		SystemClk = RCC_GetPllOutputClock();
	}


	/*
	 * Fetch the value for HPRE in RCC's CFGR register
	 * which indicates the prescalar used for AHB
	 * where bits 4-7 are configured
	 */
	temp = ((RCC->CFGR >>RCC_CFGR_HPRE_BIT)&0xF);
	if(temp<8)
	{
		ahbp = 1;
	}
	else
	{
		ahbp = AHBPrescalar[temp-8];
	}

	/*
	 * Fetch the value for HPRE in RCC's CFGR register
	 * which indicates the prescalar used for APB1 Low speed prescalar
	 * where bits 10-12 are configured
	 */
	temp = ((RCC->CFGR >>RCC_CFGR_PPRE2_BIT)&0x7);
	if(temp<4)
	{
		apb2p = 1;
	}
	else
	{
		ahbp = APB2Prescalar[temp-4];
	}

	pclk2 = (SystemClk/ahbp)/apb2p;

	return pclk2;
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

