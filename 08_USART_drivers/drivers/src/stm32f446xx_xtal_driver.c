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
    uint32_t pclk1, SystemClk;
    uint8_t clkSrc, temp, ahbp;
    uint8_t apb1p;

    clkSrc = ((RCC->CFGR >> RCC_CFGR_SWS) & 0x3);

    if (clkSrc == 0) {
        // HSI is SYSCLK source. HSI is 16MHz.
        // We must check HPRE to find the HCLK value.
        SystemClk = HSI_VALUE;
    }
    else if (clkSrc == 1)
    {
        // HSE is SYSCLK source. Assuming 8MHz HSE.
        SystemClk = HSE_VALUE;
    }
    else if (clkSrc == 2)
    {
        // PLL is SYSCLK source.
        SystemClk = RCC_GetPllOutputClock();
    }

    // Read AHB prescaler (HPRE bits 4-7)
    temp = ((RCC->CFGR >> RCC_CFGR_HPRE) & 0xF);
    if (temp < 8)
    {
        ahbp = 1; // Division by 1
    }
    else
    {
        ahbp = AHBPrescalar[temp - 8]; // Division by 2, 4, 8, etc.
    }

    // Divide SystemClk by AHB prescaler to get HCLK
    SystemClk /= ahbp;

    // Read APB1 prescaler (PPRE1 bits 10-12)
    temp = ((RCC->CFGR >> RCC_CFGR_PPRE1) & 0x7);
    if (temp < 4)
    {
        apb1p = 1;
    }
    else
    {
        apb1p = APB1Prescalar[temp - 4];
    }

    // Divide HCLK by APB1 prescaler to get PCLK1
    pclk1 = SystemClk / apb1p;

    //Divide by 2 since PCLK1 is interfaced with APB1 which will always
    //Operate at half the system clock
    return pclk1;
}


uint32_t RCC_GetPClk2Value(void)
{
	uint32_t pclk2,SystemClk;
	uint8_t clkSrc,temp,ahbp,apb2p;

	//1. Identify the clock source
	clkSrc = ((RCC->CFGR>>RCC_CFGR_SWS) & 0x3);

    if (clkSrc == 0)
    {
        // HSI is SYSCLK source. HSI is 16MHz.
        // We must check HPRE to find the HCLK value.
        SystemClk = HSI_VALUE;
    }
    else if (clkSrc == 1)
    {
        // HSE is SYSCLK source. Assuming 8MHz HSE.
        SystemClk = HSE_VALUE;
    }
    else if (clkSrc == 2)
    {
        // PLL is SYSCLK source.
        SystemClk = RCC_GetPllOutputClock();
    }


	/*
	 * Fetch the value for HPRE in RCC's CFGR register
	 * which indicates the prescalar used for AHB
	 * where bits 4-7 are configured
	 */
	temp = ((RCC->CFGR >>RCC_CFGR_HPRE)&0xF);
	if(temp<8)
	{
		ahbp = 1;
	}
	else
	{
		ahbp = AHBPrescalar[temp-8];
	}

    // Divide SystemClk by AHB prescaler to get HCLK
    SystemClk /= ahbp;

	/*
	 * Fetch the value for HPRE in RCC's CFGR register
	 * which indicates the prescalar used for APB1 Low speed prescalar
	 * where bits 10-12 are configured
	 */
	temp = ((RCC->CFGR >>RCC_CFGR_PPRE2)&0x7);
	if(temp<4)
	{
		apb2p = 1;
	}
	else
	{
		ahbp = APB2Prescalar[temp-4];
	}

	pclk2 = SystemClk / apb2p;

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
    // 1. Enable Power Control clock
    RCC->APB1ENR |= (1U << RCC_APB1ENR_PWREN);
    // 2. Configure the main internal regulator output voltage
//    PWR->CR |= PWR_REGULATOR_VOLTAGE_SCALE3;


    RCC->CR &= ~(1<<RCC_CR_HSEBYP);
#ifdef HSE_BYPASS_MODE_ENABLE
    RCC->CR |= (1<<RCC_CR_HSEBYP);
#endif

#if defined(SYSCLK_FREQ_HSI_8MHZ)
    // --- Internal Oscillator (HSI) at 8MHz ---
    // HSI = 16MHz. HCLK = 8MHz. PCLK1/PCLK2 = 8MHz.
    RCC->CR |= (1<<RCC_CR_HSION); // Enable HSI
    while (!(RCC->CR & (1U << RCC_CR_HSIRDY))){} // Wait for HSI to be ready

	// Configure Flash Latency for 8MHz HCLK
	FLASH->ACR &= ~(0xF << FLASH_ACR_LATENCY);

    RCC->CR &= ~(1U << RCC_CR_PLLON);
    while ((RCC->CR & (1U << RCC_CR_PLLRDY)));

    // Configure AHB and APB bus prescalers
    RCC->CFGR = 0x00000000U; // Clear CFGR register
    RCC->CFGR |= (0x8U << RCC_CFGR_HPRE); 	// HPRE = /2 -> 16MHz / 2 = 8MHz
    RCC->CFGR |= (0x0U << RCC_CFGR_PPRE1); 	// PPRE1 = /1
    RCC->CFGR |= (0x0U << RCC_CFGR_PPRE2); 	// PPRE2 = /1

    // Select HSI as the system clock source
    RCC->CFGR &= ~(0x3U << RCC_CFGR_SW); // Clear SW bits
    RCC->CFGR |= (0x0U << RCC_CFGR_SW);  // Select HSI (binary 00)
    while (!((RCC->CFGR & (0x3U << RCC_CFGR_SWS)) == (0))); // Wait for SWS bits to show HSI

#elif defined(SYSCLK_FREQ_HSE_8MHZ)
    // --- External Oscillator (HSE) at 8MHz ---
    // HCLK = 8MHz. PCLK1/PCLK2 = 8MHz.

    RCC->CR &= ~(1U << RCC_CR_HSEON);
    RCC->CR |= (1U << RCC_CR_HSEON); // HSEON bit
    while (!(RCC->CR & (1U << RCC_CR_HSERDY))); // Wait for HSERDY

    // Configure Flash Latency for 8MHz HCLK
    FLASH->ACR &= ~(0xF << FLASH_ACR_LATENCY);
//    FLASH->ACR |= FLASH_ACR_LATENCY_0WS; // 0 wait states for 8MHz

    // Configure AHB and APB bus prescalers
    RCC->CFGR = 0x00000000U; // Clear CFGR register
    RCC->CFGR |= (0x0U << RCC_CFGR_HPRE); // HPRE = /1 -> 8MHz / 1 = 8MHz
    RCC->CFGR |= (0x0U << RCC_CFGR_PPRE1); // PPRE1 = /1
    RCC->CFGR |= (0x0U << RCC_CFGR_PPRE2); // PPRE2 = /1

    // Select HSE as the system clock source
    RCC->CFGR &= ~(0x3U << RCC_CFGR_SW);
    RCC->CFGR |= (0x1U << RCC_CFGR_SW);  // Select HSE (binary 01)
    while (!((RCC->CFGR & (0x3U << RCC_CFGR_SWS)) == (0x1U << 2))); // Wait for SWS bits to show HSE

#elif defined(SYSCLK_FREQ_HSE_PLL_48MHZ)
    // --- External Oscillator (HSE) with PLL at 48MHz ---
    // HCLK = 48MHz. PCLK1 = 24MHz. PCLK2 = 48MHz. Requires an 8MHz crystal.

    // 1. Set Flash Latency FIRST (crucial for stability)
    FLASH->ACR &= ~(0xF << FLASH_ACR_LATENCY);
    FLASH->ACR |= (0x1 << FLASH_ACR_LATENCY);

    // 2. Start HSE
    RCC->CR &= ~(1U << RCC_CR_HSEON);
    RCC->CR |= (1U << RCC_CR_HSEON);
    while (!(RCC->CR & (1U << RCC_CR_HSERDY)));

    // 3. Configure PLL: Ensure PLL is OFF
    RCC->CR &= ~(1U << RCC_CR_PLLON);
    while ((RCC->CR & (1U << RCC_CR_PLLRDY)));

    // 4. Configure PLLCFGR for 8MHz HSE
    // HSE = 8MHz, target SYSCLK = 48MHz
    // PLLM = 4 (8MHz / 4 = 2MHz PLL input)
    // PLLN = 96 (2MHz * 96 = 192MHz VCO output)
    // PLLP = DIV4 (192MHz / 4 = 48MHz SYSCLK)
    RCC->PLLCFGR = 0x00000000U;
    RCC->PLLCFGR |= (1U  << RCC_PLLCFGR_PLLSRC); // PLLSRC bit set for HSE source
    RCC->PLLCFGR |= (4U  << RCC_PLLCFGR_PLLM);  // PLLM = 4
    RCC->PLLCFGR |= (96U << RCC_PLLCFGR_PLLN); // PLLN = 96
    RCC->PLLCFGR |= (1U  << RCC_PLLCFGR_PLLP); // PLLP = DIV4 (0b01)

    // 5. Enable PLL
    RCC->CR |= (1U << RCC_CR_PLLON);
    while (!(RCC->CR & (1U << RCC_CR_PLLRDY)));

    // 6. Configure AHB and APB bus prescalers for 48MHz HCLK
    RCC->CFGR &= ~(0xF << RCC_CFGR_HPRE);
    RCC->CFGR |= (0x0U << RCC_CFGR_HPRE); // HPRE = /1
    RCC->CFGR &= ~(0x7 << RCC_CFGR_PPRE1);
    RCC->CFGR |= (0x4U << RCC_CFGR_PPRE1); // PPRE1 = /2 (PCLK1 = 24MHz)
    RCC->CFGR &= ~(0x7 << RCC_CFGR_PPRE2);
    RCC->CFGR |= (0x0U << RCC_CFGR_PPRE2); // PPRE2 = /1 (PCLK2 = 48MHz)

    // 7. Select PLLCLK as SYSCLK source
    RCC->CFGR &= ~(0x3U << RCC_CFGR_SW);
    RCC->CFGR |= (0x2U <<  RCC_CFGR_SW);
    while (!((RCC->CFGR & (0x3U << RCC_CFGR_SWS)) == (0x2U << 2)));

#else
    // Default fallback to HSI 16MHz if no macro is defined.
    RCC->CR |= (1<<RCC_CR_HSION);
    while (!(RCC->CR & (1U << RCC_CR_HSIRDY)));
    RCC->CFGR = 0x00000000U;
    FLASH->ACR &= ~(0xF << FLASH_ACR_LATENCY);
#endif
}


uint32_t RCC_GetPllOutputClock(void)
{
    uint32_t pllinputclock = 0;
    uint32_t pll_m, pll_n, pll_p;
    uint32_t pll_source;

    // 1. Read the PLL source from PLLCFGR register (bit 22)
    pll_source = (RCC->PLLCFGR >> RCC_PLLCFGR_PLLSRC) & 0x1;

    // 2. Determine the input clock frequency
    if(pll_source == 0) // HSI selected as PLL source
    {
        pllinputclock = HSI_VALUE; // HSI_VALUE is defined as 16MHz
    }
    else // HSE selected as PLL source
    {
        // For simplicity, assuming a fixed 16MHz HSE value
        // In a real application, you'd need to know the HSE crystal frequency
        pllinputclock = HSE_VALUE;
    }

    // 3. Read the PLL configuration factors from PLLCFGR
    pll_m = RCC->PLLCFGR & 0x3F; // Bits 5:0
    pll_n = (RCC->PLLCFGR >> 6) & 0x1FF; // Bits 14:6
    pll_p = (RCC->PLLCFGR >> 16) & 0x3; // Bits 17:16

    // 4. Convert PLLP bits to division factor
    if(pll_p == 0) pll_p = 2;
    else if(pll_p == 1) pll_p = 4;
    else if(pll_p == 2) pll_p = 6;
    else if(pll_p == 3) pll_p = 8;

    // 5. Calculate the PLL output frequency
    // PLL_VCO = pll_source_clock / PLLM * PLLN
    // PLL_P_output_clock = PLL_VCO / PLLP
    uint32_t pll_vco_freq = (pllinputclock / pll_m) * pll_n;
    uint32_t pll_p_output_freq = pll_vco_freq / pll_p;

    return pll_p_output_freq;
}
