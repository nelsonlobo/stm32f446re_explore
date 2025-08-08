/*
 * stm32f446xx_rcc_driver.h
 *
 *  Created on: Jul 24, 2025
 *      Author: Nelson Lobo
 */

#ifndef INC_STM32F446XX_XTAL_DRIVER_H_
#define INC_STM32F446XX_XTAL_DRIVER_H_


// Define HSI_VALUE if not already in stm32f446xx.h
#define HSI_VALUE				((uint32_t)16000000) // Default HSI frequency for STM32F4
#define HSE_VALUE				((uint32_t)8000000) //Nucleo-F446RE has 8MHz by default via bypass


// --- Clock Configuration Selection Macros ---
// Define ONE of these macros to select your desired system clock
// If none are defined, a default basic configuration (e.g., HSI 16MHz) might be used or result in no clock setup.

#define SYSCLK_FREQ_HSI_8MHZ      	// Option 1: Internal Oscillator (HSI) at 8MHz
//#define SYSCLK_FREQ_HSI_16MHZ     	// Option 1: Internal Oscillator (HSI) at 16MHz
//#define SYSCLK_FREQ_HSE_8MHZ   		// Option 2: External Oscillator (HSE) at 8MHz
//#define SYSCLK_FREQ_HSE_PLL_48MHZ 	// Option 3: External Oscillator (HSE) with PLL at 48MHz

// --- HSE Bypass Mode Macro ---
// Define this macro to enable HSE Bypass mode if you are providing an external clock signal
// (e.g., from a signal generator) directly to the OSC_IN pin, instead of using a crystal.
// If using a crystal, DO NOT define this macro.
//#define HSE_BYPASS_MODE_ENABLE // Option 4: Enable HSE Bypass mode


// --- Forward declaration for SystemClock_Config ---
void SystemClock_Config(void);

// Note: Ensure PWR_RegDef_t, FLASH_TypeDef, PWR, and FLASH macros are correctly defined in this header.
// (You should have these from previous steps).

uint32_t RCC_GetPClk1Value(void);
uint32_t RCC_GetPClk2Value(void);


uint32_t RCC_GetPllOutputClock(void);
#endif /* INC_STM32F446XX_XTAL_DRIVER_H_ */
