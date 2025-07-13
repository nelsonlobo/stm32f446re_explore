/*
 * stm32f446xx.h
 *
 *  Created on: Jul 8, 2025
 *      Author: Nelson Lobo
 */

#ifndef INC_STM32F446XX_H_
#define INC_STM32F446XX_H_



#include <stdint.h>


// --- Clock Configuration Selection Macros ---
// Define ONE of these macros to select your desired system clock
// If none are defined, a default basic configuration (e.g., HSI 16MHz) might be used or result in no clock setup.

//#define SYSCLK_FREQ_HSI_8MHZ      // Option 1: Internal Oscillator (HSI) at 8MHz
#define SYSCLK_FREQ_HSI_16MHZ      // Option 1: Internal Oscillator (HSI) at 16MHz
//#define SYSCLK_FREQ_HSE_16MHZ   // Option 2: External Oscillator (HSE) at 16MHz
//#define SYSCLK_FREQ_HSE_PLL_48MHZ // Option 3: External Oscillator (HSE) with PLL at 48MHz

// --- HSE Bypass Mode Macro ---
// Define this macro to enable HSE Bypass mode if you are providing an external clock signal
// (e.g., from a signal generator) directly to the OSC_IN pin, instead of using a crystal.
// If using a crystal, DO NOT define this macro.
//#define HSE_BYPASS_MODE_ENABLE // Option 4: Enable HSE Bypass mode

// --- Forward declaration for SystemClock_Config ---
void SystemClock_Config(void);

// Note: Ensure PWR_RegDef_t, FLASH_TypeDef, PWR, and FLASH macros are correctly defined in this header.
// (You should have these from previous steps).


/*******************Start Processor specific details*************************/
/*
 * ARM cortex MX processors NVIC ISERx register address
 *
 */
#define NVIC_ISER0				((volatile uint32_t*)0xE000E100)
#define NVIC_ISER1				((volatile uint32_t*)0xE000E104)
#define NVIC_ISER2				((volatile uint32_t*)0xE000E108)
#define NVIC_ISER3				((volatile uint32_t*)0xE000E10C)


#define NVIC_ICER0				((volatile uint32_t*)0xE000E180)
#define NVIC_ICER1				((volatile uint32_t*)0xE000E184)
#define NVIC_ICER2				((volatile uint32_t*)0xE000E188)
#define NVIC_ICER3				((volatile uint32_t*)0xE000E18C)


#define NVIC_PR_BASE_ADDR		((volatile uint32_t*)0xE000E400)

#define NO_PRIORITY_BITS_IMPLEMENTED	4

#define FLASH_BASEADDR 			0x08000000U
#define SRAM1_BASEADDR			0x20000000U
#define SRAM2_BASEADDR			SRAM1_BASEADDR + 0x1C000
#define SRAM					SRAM1_BASEADDR
#define ROM_SYS					0x1FFF0000U


/*
 * AHBx and APBx Bus addresses
 * AHB: Advanced High-performance bus
 * APB: Advanced Peripheral bus
 */
#define PERIPHERAL_BASEADDR		0x40000000U
#define APB1PERIPH_BASEADDR		PERIPHERAL_BASEADDR
#define APB2PERIPH_BASEADDR		0x40010000U
#define AHB1PERIPH_BASEADDR		0x40020000U
#define AHB2PERIPH_BASEADDR		0x50000000U


/*
 * Base addresses of peripherals which are interfaced with the AHB1 bus
 * AHB1: Advanced High-performance bus
 */

#define GPIOA_BASEADDR			(AHB1PERIPH_BASEADDR+0x0000)
#define GPIOB_BASEADDR			(AHB1PERIPH_BASEADDR+0x0400)
#define GPIOC_BASEADDR			(AHB1PERIPH_BASEADDR+0x0800)
#define GPIOD_BASEADDR			(AHB1PERIPH_BASEADDR+0x0C00)
#define GPIOE_BASEADDR			(AHB1PERIPH_BASEADDR+0x1000)
#define GPIOF_BASEADDR			(AHB1PERIPH_BASEADDR+0x1400)
#define GPIOG_BASEADDR			(AHB1PERIPH_BASEADDR+0x1800)
#define GPIOH_BASEADDR			(AHB1PERIPH_BASEADDR+0x1C00)
#define CRC_BASEADDR			(AHB1PERIPH_BASEADDR+0x3000)
#define RCC_BASEADDR			(AHB1PERIPH_BASEADDR+0x3800)
#define FLASH_IF_BASEADDR		(AHB1PERIPH_BASEADDR+0x3C00)
#define BKPSRAM_BASEADDR		(AHB1PERIPH_BASEADDR+0x4000)
#define DMA1_BASEADDR			(AHB1PERIPH_BASEADDR+0x6000)
#define DMA2_BASEADDR			(AHB1PERIPH_BASEADDR+0x6400)
#define USB_OTGHS_BASEADDR		(AHB1PERIPH_BASEADDR+0x20000)

/*
 * Base addresses of peripherals which are interfaced with the AHB2 bus
 * AHB2: Advanced High-performance bus
 */
#define USB_OTGFS_BASEADDR		(AHB2PERIPH_BASEADDR+0x0000)
#define DCMI_BASEADDR			(AHB2PERIPH_BASEADDR+0x50000)
#define DCMI_BASEADDR			(AHB2PERIPH_BASEADDR+0x50000)

/*
 * Base addresses of peripherals which are interfaced with the APB1 bus
 * APB1: Advanced peripheral bus
 */
#define TIM2_BASEADDR			(APB1PERIPH_BASEADDR+0x0000)
#define TIM3_BASEADDR			(APB1PERIPH_BASEADDR+0x0400)
#define TIM4_BASEADDR			(APB1PERIPH_BASEADDR+0x0800)
#define TIM5_BASEADDR			(APB1PERIPH_BASEADDR+0x0C00)
#define TIM6_BASEADDR			(APB1PERIPH_BASEADDR+0x1000)
#define TIM7_BASEADDR			(APB1PERIPH_BASEADDR+0x1400)
#define TIM12_BASEADDR			(APB1PERIPH_BASEADDR+0x1800)
#define TIM13_BASEADDR			(APB1PERIPH_BASEADDR+0x1C00)
#define TIM14_BASEADDR			(APB1PERIPH_BASEADDR+0x2000)
#define RTCBKP_BASEADDR			(APB1PERIPH_BASEADDR+0x2800)
#define WWDG_BASEADDR			(APB1PERIPH_BASEADDR+0x2C00)
#define IWDG_BASEADDR			(APB1PERIPH_BASEADDR+0x3000)
#define SPI2_BASEADDR			(APB1PERIPH_BASEADDR+0x3800)
#define SPI3_BASEADDR			(APB1PERIPH_BASEADDR+0x3C00)
#define SPDIFRX_BASEADDR		(APB1PERIPH_BASEADDR+0x4000)
#define USART2_BASEADDR			(APB1PERIPH_BASEADDR+0x4400)
#define USART3_BASEADDR			(APB1PERIPH_BASEADDR+0x4800)
#define UART4_BASEADDR			(APB1PERIPH_BASEADDR+0x4C00)
#define UART5_BASEADDR			(APB1PERIPH_BASEADDR+0x5000)
#define I2C1_BASEADDR			(APB1PERIPH_BASEADDR+0x5400)
#define I2C2_BASEADDR			(APB1PERIPH_BASEADDR+0x5800)
#define I2C3_BASEADDR			(APB1PERIPH_BASEADDR+0x5C00)
#define CAN1_BASEADDR			(APB1PERIPH_BASEADDR+0x6400)
#define CAN2_BASEADDR			(APB1PERIPH_BASEADDR+0x6800)
#define HDMICEC_BASEADDR		(APB1PERIPH_BASEADDR+0x6C00)
#define PWR_BASEADDR			(APB1PERIPH_BASEADDR+0x7000)
#define DAC_BASEADDR			(APB1PERIPH_BASEADDR+0x7400)


/*
 * Base addresses of peripherals which are interfaced with the APB2 bus
 * APB2: Advanced peripheral bus
 */
#define TIM1_BASEADDR			(APB2PERIPH_BASEADDR+0x0000)
#define TIM8_BASEADDR			(APB2PERIPH_BASEADDR+0x0400)
#define USART1_BASEADDR			(APB2PERIPH_BASEADDR+0x1000)
#define USART6_BASEADDR			(APB2PERIPH_BASEADDR+0x1400)
#define ADC123_BASEADDR			(APB2PERIPH_BASEADDR+0x2000)
#define SDMMC_BASEADDR			(APB2PERIPH_BASEADDR+0x2C00)
#define SPI1_BASEADDR			(APB2PERIPH_BASEADDR+0x3000)
#define SPI4_BASEADDR			(APB2PERIPH_BASEADDR+0x3400)
#define SYSCFG_BASEADDR			(APB2PERIPH_BASEADDR+0x3800)
#define EXTI_BASEADDR			(APB2PERIPH_BASEADDR+0x3C00)
#define TIM9_BASEADDR			(APB2PERIPH_BASEADDR+0x4000)
#define TIM10_BASEADDR			(APB2PERIPH_BASEADDR+0x4400)
#define TIM11_BASEADDR			(APB2PERIPH_BASEADDR+0x4800)
#define SAI1_BASEADDR			(APB2PERIPH_BASEADDR+0x5800)
#define SAI2_BASEADDR			(APB2PERIPH_BASEADDR+0x5C00)



/*
 * Structure to manage the RCC related registers
 * Reset & Clock Control
 *
 */

typedef struct
{
	volatile uint32_t CR;				//0x00
	volatile uint32_t PLLCFGR;			//0x04
	volatile uint32_t CFGR;				//0x08
	volatile uint32_t CIR;				//0x0C
	volatile uint32_t AHB1RSTR;			//0x10
	volatile uint32_t AHB2RSTR;			//0x14
	volatile uint32_t AHB3RSTR;			//0x18
	volatile uint32_t reserved0;		//0x1C
	volatile uint32_t APB1RSTR;			//0x20
	volatile uint32_t APB2RSTR;			//0x24
	volatile uint32_t reserved1[2];		//0x28
	volatile uint32_t AHB1ENR;			//0x30
	volatile uint32_t AHB2ENR;			//0x34
	volatile uint32_t AHB3ENR;			//0x38
	volatile uint32_t reserved2;		//0x3C
	volatile uint32_t APB1ENR;			//0x40
	volatile uint32_t APB2ENR;			//0x44
	volatile uint32_t reserved3[2];		//0x48
	volatile uint32_t AHB1LPENR;		//0x50
	volatile uint32_t AHB2LPENR;		//0x54
	volatile uint32_t AHB3LPENR;		//0x58
	volatile uint32_t reserved4;		//0x5C
	volatile uint32_t APB1LPENR;		//0x60
	volatile uint32_t APB2LPENR;		//0x64
	volatile uint32_t reserved5[2];		//0x68
	volatile uint32_t BDCR;				//0x70
	volatile uint32_t CSR;				//0x74
	volatile uint32_t reserved6[2];		//0x78
	volatile uint32_t SSCGR;			//0x80
	volatile uint32_t PLLI2SCFGR;		//0x84
	volatile uint32_t PLLSAICFGR;		//0x88
	volatile uint32_t DCKCFGR;			//0x8C
	volatile uint32_t CKGATENR;			//0x90
	volatile uint32_t DCKCFGR2;			//0x94
}RCC_RegDef_t;

#define RCC_CR_REG_OFFSET		0x00
#define RCC_CR_REG_ADDR			(RCC_BASEADDR+RCC_CR_REG_OFFSET)

#define RCC_CFG_REG_OFFSET		0x08
#define RCC_CFG_REG_ADDR		(RCC_BASEADDR + RCC_CFG_REG_OFFSET)

#define RCC_GPIOC_REG_OFFSET	0x30
#define RCC_GPIOC_REG_ADDR		(RCC_BASEADDR + RCC_GPIOC_REG_OFFSET)

#define RCC						((RCC_RegDef_t*)RCC_BASEADDR)

// Define HSI_VALUE if not already in stm32f446xx.h
#ifndef HSI_VALUE
#define HSI_VALUE    ((uint32_t)16000000) // Default HSI frequency for STM32F4
#endif

// Define constants for clock configuration based on CubeMX output
// These would typically be part of a HAL-like structure or global defines if not using HAL
#define PWR_REGULATOR_VOLTAGE_SCALE3 (0x00004000U) // From stm32f4xx_hal_rcc.h / stm32f4xx_hal_pwr.h (PWR_CR_VOS)
#define RCC_OSCILLATORTYPE_HSI       (0x00000001U) // From stm32f4xx_hal_rcc.h (RCC_CR_HSION)
#define RCC_HSI_ON                   (0x00000001U) // From stm32f4xx_hal_rcc.h (RCC_CR_HSION)
#define RCC_HSICALIBRATION_DEFAULT   (0x00000010U) // From stm32f4xx_hal_rcc.h (RCC_CR_HSICAL_Msk >> RCC_CR_HSICAL_Pos, default 16)
#define RCC_PLL_ON                   (0x00000002U) // From stm32f4xx_hal_rcc.h (RCC_CR_PLLON)
#define RCC_PLLSOURCE_HSI            (0x00000000U) // From stm32f4xx_hal_rcc.h (RCC_PLLCFGR_PLLSRC_HSI)
#define RCC_PLLP_DIV2                (0x00000000U) // From stm32f4xx_hal_rcc.h (RCC_PLLCFGR_PLLP_DIV2)
#define RCC_CLOCKTYPE_HCLK           (0x00000001U) // From stm32f4xx_hal_rcc.h (RCC_CFGR_HPRE_Pos)
#define RCC_CLOCKTYPE_SYSCLK         (0x00000002U) // From stm32f4xx_hal_rcc.h (RCC_CFGR_SW_Pos)
#define RCC_CLOCKTYPE_PCLK1          (0x00000004U) // From stm32f4xx_hal_rcc.h (RCC_CFGR_PPRE1_Pos)
#define RCC_CLOCKTYPE_PCLK2          (0x00000008U) // From stm32f4xx_hal_rcc.h (RCC_CFGR_PPRE2_Pos)
#define RCC_SYSCLKSOURCE_PLLCLK      (0x00000008U) // From stm32f4xx_hal_rcc.h (RCC_CFGR_SW_PLL)
#define RCC_SYSCLK_DIV4              (0x00000080U) // From stm32f4xx_hal_rcc.h (RCC_CFGR_HPRE_DIV4)
#define RCC_HCLK_DIV2                (0x00000100U) // From stm32f4xx_hal_rcc.h (RCC_CFGR_PPRE1_DIV2)
#define RCC_HCLK_DIV1                (0x00000000U) // From stm32f4xx_hal_rcc.h (RCC_CFGR_PPRE2_DIV1)

// Define Flash Latency (for example, FLASH_LATENCY_0 from stm32f4xx_hal_flash.h)
#define FLASH_ACR_LATENCY_0WS        (0x00000000U) // For 0 wait states


/*
 * Structure to manage the EXTI related registers
 * External Interrupts
 *
 */
typedef struct
{
	volatile uint32_t IMR;
	volatile uint32_t EMR;
	volatile uint32_t RTSR;
	volatile uint32_t FTSR;
	volatile uint32_t SWIER;
	volatile uint32_t PR;
}EXTI_RegDef_t;

#define EXTI					((EXTI_RegDef_t*)EXTI_BASEADDR)


/*
 * Base addresses of peripherals which are interfaced with the APB1 bus
 * APB1: Advanced peripheral bus
 */
// ... (your existing APB1 definitions) ...
#define PWR_BASEADDR            (APB1PERIPH_BASEADDR+0x7000) // Your existing definition
// ...

/*
 * FLASH Access Control Register (ACR) base address.
 * FLASH is a special peripheral, its registers aren't part of a common struct in the same way.
 * The ACR is directly accessed for latency settings.
 */
#define FLASH_R_BASE            0x40023C00U     // Base address of FLASH registers
#define FLASH_ACR_OFFSET        0x00U           // ACR is at offset 0 from FLASH_R_BASE

/*
 * peripheral register definition structure for PWR (Power Control)
 */
typedef struct
{
    volatile uint32_t CR;     /*!< PWR power control register,                    Address offset: 0x00 */
    volatile uint32_t CSR;    /*!< PWR power control/status register,             Address offset: 0x04 */
} PWR_RegDef_t;

// ... (after other peripheral definitions like RCC, EXTI, SYSCFG, SPI) ...

#define PWR                     ((PWR_RegDef_t*)PWR_BASEADDR)
//#define FLASH_ACR_LATENCY_0WS   (0x00000000U) // For 0 wait states


// Add this structure definition near your other peripheral register definitions (e.g., after SPI_RegDef_t)
typedef struct
{
    volatile uint32_t ACR;        /*!< FLASH access control register, Address offset: 0x00 */
    volatile uint32_t KEYR;       /*!< FLASH key register,           Address offset: 0x04 */
    volatile uint32_t OPTKEYR;    /*!< FLASH option key register,    Address offset: 0x08 */
    volatile uint32_t SR;         /*!< FLASH status register,        Address offset: 0x0C */
    volatile uint32_t CR;         /*!< FLASH control register,       Address offset: 0x10 */
    volatile uint32_t OPTCR;      /*!< FLASH option control register, Address offset: 0x14 */
    volatile uint32_t OPTCR1;     /*!< FLASH option control register 1, Address offset: 0x18 */
} FLASH_TypeDef; // Using TypeDef as typically done in HAL/CMSIS

// Add this peripheral definition along with GPIOA, GPIOB, RCC, EXTI, etc.
#define FLASH                   ((FLASH_TypeDef *)FLASH_R_BASE)

/*
 * Peripheral Clock Enable macros for SYSCFG registers
 * Peripheral Clock Enable macros for SYSCFG registers
 */
#define SYSCFG_PCLK_EN()		(RCC->APB2ENR|=(1<<14))
#define SYSCFG_PCLK_DIS()		(RCC->APB2ENR &=~(1<<14))

typedef struct
{
	volatile uint32_t MEMRMP;
	volatile uint32_t PMC;
	volatile uint32_t EXTICR[4];
	volatile uint32_t reserved1[2];
	volatile uint32_t CMPCR;
	volatile uint32_t reserved2[2];
	volatile uint32_t CFGR;
}SYSCFG_RegDef_t;

#define SYSCFG					((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)



/*
 * Peripheral Clock Enable macros for GPIOx registers
 */

#define GPIOA_PCLK_EN()			(RCC->AHB1ENR|=(1<<0))
#define GPIOB_PCLK_EN()			(RCC->AHB1ENR|=(1<<1))
#define GPIOC_PCLK_EN()			(RCC->AHB1ENR|=(1<<2))
#define GPIOD_PCLK_EN()			(RCC->AHB1ENR|=(1<<3))
#define GPIOE_PCLK_EN()			(RCC->AHB1ENR|=(1<<4))
#define GPIOF_PCLK_EN()			(RCC->AHB1ENR|=(1<<5))
#define GPIOG_PCLK_EN()			(RCC->AHB1ENR|=(1<<6))
#define GPIOH_PCLK_EN()			(RCC->AHB1ENR|=(1<<7))

/*
 * Peripheral Clock Enable macros for GPIOx registers
 */

#define GPIOA_PCLK_DIS()		(RCC->AHB1ENR &=~(1<<0))
#define GPIOB_PCLK_DIS()		(RCC->AHB1ENR &=~(1<<1))
#define GPIOC_PCLK_DIS()		(RCC->AHB1ENR &=~(1<<2))
#define GPIOD_PCLK_DIS()		(RCC->AHB1ENR &=~(1<<3))
#define GPIOE_PCLK_DIS()		(RCC->AHB1ENR &=~(1<<4))
#define GPIOF_PCLK_DIS()		(RCC->AHB1ENR &=~(1<<5))
#define GPIOG_PCLK_DIS()		(RCC->AHB1ENR &=~(1<<6))
#define GPIOH_PCLK_DIS()		(RCC->AHB1ENR &=~(1<<7))

//GPIO register RESET
#define GPIOA_REG_RESET()		do{RCC->AHB1RSTR|=(1<<0);RCC->AHB1RSTR&=~(1<<0);}while(0)
#define GPIOB_REG_RESET()		do{RCC->AHB1RSTR|=(1<<1);RCC->AHB1RSTR&=~(1<<1);}while(0)
#define GPIOC_REG_RESET()		do{RCC->AHB1RSTR|=(1<<2);RCC->AHB1RSTR&=~(1<<2);}while(0)
#define GPIOD_REG_RESET()		do{RCC->AHB1RSTR|=(1<<3);RCC->AHB1RSTR&=~(1<<3);}while(0)
#define GPIOE_REG_RESET()		do{RCC->AHB1RSTR|=(1<<4);RCC->AHB1RSTR&=~(1<<4);}while(0)
#define GPIOF_REG_RESET()		do{RCC->AHB1RSTR|=(1<<5);RCC->AHB1RSTR&=~(1<<5);}while(0)
#define GPIOG_REG_RESET()		do{RCC->AHB1RSTR|=(1<<6);RCC->AHB1RSTR&=~(1<<6);}while(0)
#define GPIOH_REG_RESET()		do{RCC->AHB1RSTR|=(1<<7);RCC->AHB1RSTR&=~(1<<7);}while(0)


/*
 * Structure to manage all the GPIO related registers
 * GPIO: General purpose Input Output
 */

typedef struct
{
	volatile uint32_t MODER;
	volatile uint32_t OTYPER;
	volatile uint32_t OSPEEDR;
	volatile uint32_t PUPDR;
	volatile uint32_t IDR;
	volatile uint32_t ODR;
	volatile uint16_t BSRRL;
	volatile uint16_t BSRRH;
	volatile uint32_t LCKR;
	volatile uint32_t AFR[2];

}GPIO_RegDef_t;

#define GPIO_PIN_NUM_0			0
#define GPIO_PIN_NUM_1			1
#define GPIO_PIN_NUM_2			2
#define GPIO_PIN_NUM_3			3
#define GPIO_PIN_NUM_4			4
#define GPIO_PIN_NUM_5			5
#define GPIO_PIN_NUM_6			6
#define GPIO_PIN_NUM_7			7
#define GPIO_PIN_NUM_8			8
#define GPIO_PIN_NUM_9			9
#define GPIO_PIN_NUM_10			10
#define GPIO_PIN_NUM_11			11
#define GPIO_PIN_NUM_12			12
#define GPIO_PIN_NUM_13			13
#define GPIO_PIN_NUM_14 		14
#define GPIO_PIN_NUM_15			15

#define GPIOA 					((GPIO_RegDef_t *)GPIOA_BASEADDR)
#define GPIOB 					((GPIO_RegDef_t *)GPIOB_BASEADDR)
#define GPIOC 					((GPIO_RegDef_t *)GPIOC_BASEADDR)
#define GPIOD 					((GPIO_RegDef_t *)GPIOD_BASEADDR)
#define GPIOE 					((GPIO_RegDef_t *)GPIOE_BASEADDR)
#define GPIOF 					((GPIO_RegDef_t *)GPIOF_BASEADDR)
#define GPIOG 					((GPIO_RegDef_t *)GPIOG_BASEADDR)
#define GPIOH 					((GPIO_RegDef_t *)GPIOH_BASEADDR)

#define GPIO_BASEADDR_TO_CODE(x) 	(	(x == GPIOA)? 0:\
										(x == GPIOB)? 1:\
										(x == GPIOC)? 2:\
										(x == GPIOD)? 3:\
										(x == GPIOE)? 4:\
										(x == GPIOF)? 5:\
										(x == GPIOG)? 6:\
										(x == GPIOH)? 7:0)


/*
 * Peripheral Clock Enable macros for SPIx registers
 */
#define SPI1_PCLK_EN()			(RCC->APB2ENR|=(1<<12))
#define SPI2_PCLK_EN()			(RCC->APB1ENR|=(1<<14))
#define SPI3_PCLK_EN()			(RCC->APB1ENR|=(1<<15))
#define SPI4_PCLK_EN()			(RCC->APB2ENR|=(1<<13))

/*
 * Peripheral Clock Enable macros for SPIx registers
 */
#define SPI1_PCLK_DIS()			(RCC->APB2ENR &=~(1<<12))
#define SPI2_PCLK_DIS()			(RCC->APB1ENR &=~(1<<14))
#define SPI3_PCLK_DIS()			(RCC->APB1ENR &=~(1<<15))
#define SPI4_PCLK_DIS()			(RCC->APB2ENR &=~(1<<13))

/*
 * Register Reset macros for SPIx registers
 */
#define SPI1_REG_RESET()		do{RCC->APB2RSTR|=(1<<12);RCC->APB2RSTR&=~(1<<12);}while(0)
#define SPI2_REG_RESET()		do{RCC->APB1RSTR|=(1<<14);RCC->APB1RSTR&=~(1<<14);}while(0)
#define SPI3_REG_RESET()		do{RCC->APB1RSTR|=(1<<15);RCC->APB1RSTR&=~(1<<15);}while(0)
#define SPI4_REG_RESET()		do{RCC->APB2RSTR|=(1<<13);RCC->APB2RSTR&=~(1<<13);}while(0)


/*
 * SPI Register definition
 */
typedef struct
{
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t SR;
	volatile uint32_t DR;
	volatile uint32_t CRCPR;
	volatile uint32_t RXCRCR;
	volatile uint32_t TXCRCR;
	volatile uint32_t I2SCFGR;
	volatile uint32_t I2SPR;
}SPI_RegDef_t;

#define SPI1					((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2					((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3					((SPI_RegDef_t*)SPI3_BASEADDR)
#define SPI4					((SPI_RegDef_t*)SPI4_BASEADDR)


/*
 * Peripheral Clock Enable macros for I2Cx registers
 */
#define I2C1_PCLK_EN()			(RCC->APB1ENR|=(1<<21))
#define I2C2_PCLK_EN()			(RCC->APB1ENR|=(1<<22))
#define I2C3_PCLK_EN()			(RCC->APB1ENR|=(1<<23))

/*
 * Peripheral Clock Enable macros for I2Cx registers
 */
#define I2C1_PCLK_DIS()			(RCC->APB1ENR &=~(1<<21))
#define I2C2_PCLK_DIS()			(RCC->APB1ENR &=~(1<<22))
#define I2C3_PCLK_DIS()			(RCC->APB1ENR &=~(1<<23))


/*
 * Peripheral Clock Enable macros for USART registers
 */
#define USART1_PCLK_EN()		(RCC->APB2ENR|=(1<<4))
#define USART2_PCLK_EN()		(RCC->APB2ENR|=(1<<17))
#define USART3_PCLK_EN()		(RCC->APB2ENR|=(1<<18))
#define UART4_PCLK_EN()			(RCC->APB2ENR|=(1<<19))
#define UART5_PCLK_EN()			(RCC->APB2ENR|=(1<<20))
#define USART6_PCLK_EN()		(RCC->APB2ENR|=(1<<5))

/*
 * Peripheral Clock Enable macros for USART registers
 */
#define USART1_PCLK_DIS()		(RCC->APB2ENR &=~(1<<4))
#define USART2_PCLK_DIS()		(RCC->APB2ENR &=~(1<<17))
#define USART3_PCLK_DIS()		(RCC->APB2ENR &=~(1<<18))
#define UART4_PCLK_DIS()		(RCC->APB2ENR &=~(1<<19))
#define UART5_PCLK_DIS()		(RCC->APB2ENR &=~(1<<20))
#define USART6_PCLK_DIS()		(RCC->APB2ENR &=~(1<<5))



/*
 * IRQ(Interrupt Request) Numbers of STM32F446xx MCU
 * NOTE:Adjust these macros with valid values according to your MCU
 *
 */
#define IRQN_EXIT0				6
#define IRQN_EXIT1				7
#define IRQN_EXIT2				8
#define IRQN_EXIT3				9
#define IRQN_EXIT4				10
#define IRQN_EXIT9_5			23
#define IRQN_EXIT15_10			40

//SPI Related IRQ positions
#define IRQN_SPI1				35
#define IRQN_SPI2				36
#define IRQN_SPI3				51
#define IRQN_SPI4				84

#define NVIC_IRQ_PRIO0			0
#define NVIC_IRQ_PRIO1			1
#define NVIC_IRQ_PRIO2			2
#define NVIC_IRQ_PRIO3			3
#define NVIC_IRQ_PRIO4			4
#define NVIC_IRQ_PRIO5			5
#define NVIC_IRQ_PRIO6			6
#define NVIC_IRQ_PRIO7			7
#define NVIC_IRQ_PRIO8			8
#define NVIC_IRQ_PRIO9			9
#define NVIC_IRQ_PRIO10			10
#define NVIC_IRQ_PRIO11			11
#define NVIC_IRQ_PRIO12			12
#define NVIC_IRQ_PRIO13			13
#define NVIC_IRQ_PRIO14			14
#define NVIC_IRQ_PRIO15			15



#define ENABLE					1
#define DISABLE					0
#define SET						ENABLE
#define CLEAR					DISABLE
#define GPIO_PIN_SET			SET
#define GPIO_PIN_CLEAR			CLEAR
#define FLAG_CLEAR				CLEAR
#define FLAG_SET				SET


#include "stm32f446xx_gpio_driver.h"
#include "stm32f446xx_spi_driver.h"

#endif /* INC_STM32F446XX_H_ */

