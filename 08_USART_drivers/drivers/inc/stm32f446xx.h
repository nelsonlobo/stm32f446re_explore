/*
 * stm32f446xx.h
 *
 *  Created on: Jul 8, 2025
 *      Author: Nelson Lobo
 */

#ifndef INC_STM32F446XX_H_
#define INC_STM32F446XX_H_


#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stddef.h>

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
 * peripheral register definition structure for PWR (Power Control)
 */
typedef struct
{
    volatile uint32_t CR;     /*!< PWR power control register,                    Address offset: 0x00 */
    volatile uint32_t CSR;    /*!< PWR power control/status register,             Address offset: 0x04 */
} PWR_RegDef_t;

#define PWR                     	((PWR_RegDef_t*)PWR_BASEADDR)

#define PWR_CR_FISSR			21
#define PWR_CR_FMSSR			20
#define PWR_CR_UDEN				18
#define PWR_CR_ODSWEN			17
#define PWR_CR_ODEN				16
#define PWR_CR_VOS				14
#define PWR_CR_ADCDC1			13
#define PWR_CR_MRUDS			11
#define PWR_CR_LPUDS			10
#define PWR_CR_FPDS				9
#define PWR_CR_DBP				8
#define PWR_CR_PLS				5
#define PWR_CR_PVDE				4
#define PWR_CR_CSBF				3
#define PWR_CR_CWUF				2
#define PWR_CR_PDDS				1
#define PWR_CR_LPUS				0

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

#define RCC							((RCC_RegDef_t*)RCC_BASEADDR)

//Bit values for RCC_CR
#define RCC_CR_PLLSAIRDY		29
#define RCC_CR_PLLSAION			28
#define RCC_CR_PLLI2SRDY		27
#define RCC_CR_PLLI2SON			26
#define RCC_CR_PLLRDY			25
#define RCC_CR_PLLON			24
#define RCC_CR_CSSON			19
#define RCC_CR_HSEBYP			18
#define RCC_CR_HSERDY			17
#define RCC_CR_HSEON			16
#define RCC_CR_HSICAL			8
#define RCC_CR_HSITRIM			3
#define RCC_CR_HSIRDY			1
#define RCC_CR_HSION			0

//Bit values for RCC_PLLCFGR
#define RCC_PLLCFGR_PLLR		28	//3-bits
#define RCC_PLLCFGR_PLLQ		24	//4-bits
#define RCC_PLLCFGR_PLLSRC		22	//1-bit
#define RCC_PLLCFGR_PLLP		16	//2-bits
#define RCC_PLLCFGR_PLLN		6	//9-bits
#define RCC_PLLCFGR_PLLM		0	//6-bits

//Bit values for RCC_CFGR
#define RCC_CFGR_MCO2			30
#define RCC_CFGR_MCO2PRE		27
#define RCC_CFGR_MCO1PRE		24
#define RCC_CFGR_MCO1			21
#define RCC_CFGR_RTCPRE			16
#define RCC_CFGR_PPRE2			13
#define RCC_CFGR_PPRE1			10
#define RCC_CFGR_HPRE			4
#define RCC_CFGR_SWS			2
#define RCC_CFGR_SW				0

//Bit value for RCC_APB1ENR
#define RCC_APB1ENR_DACEN				29
#define RCC_APB1ENR_PWREN				28
#define RCC_APB1ENR_CECEN				27
#define RCC_APB1ENR_CAN2EN				26
#define RCC_APB1ENR_CAN1EN				25
#define RCC_APB1ENR_FMPI2C1EN			24
#define RCC_APB1ENR_I2C3EN				23
#define RCC_APB1ENR_I2C2EN				22
#define RCC_APB1ENR_I2C1EN				21
#define RCC_APB1ENR_UART5EN				20
#define RCC_APB1ENR_UART4EN				19
#define RCC_APB1ENR_USART3EN			18
#define RCC_APB1ENR_USART2EN			17
#define RCC_APB1ENR_SPDIFRXEN			16
#define RCC_APB1ENR_SPI3EN				15
#define RCC_APB1ENR_SPI2EN				14
#define RCC_APB1ENR_WWDGEN				11
#define RCC_APB1ENR_TIM14EN				8
#define RCC_APB1ENR_TIM13EN				7
#define RCC_APB1ENR_TIM12EN				6
#define RCC_APB1ENR_TIM7EN				5
#define RCC_APB1ENR_TIM6EN				4
#define RCC_APB1ENR_TIM5EN				3
#define RCC_APB1ENR_TIM4EN				2
#define RCC_APB1ENR_TIM3EN				1
#define RCC_APB1ENR_TIM2EN				0

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
 * FLASH Access Control Register (ACR) base address.
 * FLASH is a special peripheral, its registers aren't part of a common struct in the same way.
 * The ACR is directly accessed for latency settings.
 */
#define FLASH_BASE            	0x40023C00U     // Base address of FLASH registers
#define FLASH_ACR_OFFSET        0x00U           // ACR is at offset 0 from FLASH_R_BASE


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
#define FLASH                   ((FLASH_TypeDef *)FLASH_BASE)

#define FLASH_ACR_DCRST			12
#define FLASH_ACR_ICRST			11
#define FLASH_ACR_DCEN			10
#define FLASH_ACR_ICEN			9
#define FLASH_ACR_PRFTEN		8
#define FLASH_ACR_LATENCY		0

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
 * Bit position definition for AHB1ENR to Enable the respective GPIOx peripheral
 */
#define GPIOA_EN			0
#define GPIOB_EN			1
#define GPIOC_EN			2
#define GPIOD_EN			3
#define GPIOE_EN			4
#define GPIOF_EN			5
#define GPIOG_EN			6
#define GPIOH_EN			7


/*
 * Peripheral Clock Enable macros for GPIOx registers
 */

#define GPIOA_PCLK_EN()			(RCC->AHB1ENR|=(1<<GPIOA_EN))
#define GPIOB_PCLK_EN()			(RCC->AHB1ENR|=(1<<GPIOB_EN))
#define GPIOC_PCLK_EN()			(RCC->AHB1ENR|=(1<<GPIOC_EN))
#define GPIOD_PCLK_EN()			(RCC->AHB1ENR|=(1<<GPIOD_EN))
#define GPIOE_PCLK_EN()			(RCC->AHB1ENR|=(1<<GPIOE_EN))
#define GPIOF_PCLK_EN()			(RCC->AHB1ENR|=(1<<GPIOF_EN))
#define GPIOG_PCLK_EN()			(RCC->AHB1ENR|=(1<<GPIOG_EN))
#define GPIOH_PCLK_EN()			(RCC->AHB1ENR|=(1<<GPIOH_EN))

/*
 * Peripheral Clock Enable macros for GPIOx registers
 */

#define GPIOA_PCLK_DIS()		(RCC->AHB1ENR &=~(1<<GPIOA_EN))
#define GPIOB_PCLK_DIS()		(RCC->AHB1ENR &=~(1<<GPIOB_EN))
#define GPIOC_PCLK_DIS()		(RCC->AHB1ENR &=~(1<<GPIOC_EN))
#define GPIOD_PCLK_DIS()		(RCC->AHB1ENR &=~(1<<GPIOD_EN))
#define GPIOE_PCLK_DIS()		(RCC->AHB1ENR &=~(1<<GPIOE_EN))
#define GPIOF_PCLK_DIS()		(RCC->AHB1ENR &=~(1<<GPIOF_EN))
#define GPIOG_PCLK_DIS()		(RCC->AHB1ENR &=~(1<<GPIOG_EN))
#define GPIOH_PCLK_DIS()		(RCC->AHB1ENR &=~(1<<GPIOH_EN))

/*
 * Bit position definition for AHB1ENR to Enable the respective GPIOx peripheral
 */
#define GPIOA_RST			0
#define GPIOB_RST			1
#define GPIOC_RST			2
#define GPIOD_RST			3
#define GPIOE_RST			4
#define GPIOF_RST			5
#define GPIOG_RST			6
#define GPIOH_RST			7

//GPIO register RESET
#define GPIOA_REG_RESET()		do{RCC->AHB1RSTR|=(1<<GPIOA_RST);RCC->AHB1RSTR&=~(1<<GPIOA_RST);}while(0)
#define GPIOB_REG_RESET()		do{RCC->AHB1RSTR|=(1<<GPIOB_RST);RCC->AHB1RSTR&=~(1<<GPIOB_RST);}while(0)
#define GPIOC_REG_RESET()		do{RCC->AHB1RSTR|=(1<<GPIOC_RST);RCC->AHB1RSTR&=~(1<<GPIOC_RST);}while(0)
#define GPIOD_REG_RESET()		do{RCC->AHB1RSTR|=(1<<GPIOD_RST);RCC->AHB1RSTR&=~(1<<GPIOD_RST);}while(0)
#define GPIOE_REG_RESET()		do{RCC->AHB1RSTR|=(1<<GPIOE_RST);RCC->AHB1RSTR&=~(1<<GPIOE_RST);}while(0)
#define GPIOF_REG_RESET()		do{RCC->AHB1RSTR|=(1<<GPIOF_RST);RCC->AHB1RSTR&=~(1<<GPIOF_RST);}while(0)
#define GPIOG_REG_RESET()		do{RCC->AHB1RSTR|=(1<<GPIOG_RST);RCC->AHB1RSTR&=~(1<<GPIOG_RST);}while(0)
#define GPIOH_REG_RESET()		do{RCC->AHB1RSTR|=(1<<GPIOH_RST);RCC->AHB1RSTR&=~(1<<GPIOH_RST);}while(0)


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
 * Bit position definitions of SPI CR1 register
 */
#define SPI_CR1_CPHA			0
#define SPI_CR1_CPOL			1
#define SPI_CR1_MSTR			2
#define SPI_CR1_BR				3
#define SPI_CR1_SPE				6
#define SPI_CR1_LSB_FIRST		7
#define SPI_CR1_SSI				8
#define SPI_CR1_SSM				9
#define SPI_CR1_RXONLY			10
#define SPI_CR1_DFF				11
#define SPI_CR1_CRCNEXT			12
#define SPI_CR1_CRCEN			13
#define SPI_CR1_BIDIOE			14
#define SPI_CR1_BIDIMODE		15


/*
 * Bit position definitions of SPI CR2 register
 */
#define SPI_CR2_RXDMAEN			0
#define SPI_CR2_TXDMAEN			1
#define SPI_CR2_SSOE			2
#define SPI_CR2_FRF				4
#define SPI_CR2_ERRIE			5
#define SPI_CR2_RXNEIE			6
#define SPI_CR2_TXEIE			7


/*
 * Bit position definitions for SPI_SR register
 */
#define SPI_SR_RXNE				0
#define SPI_SR_TXE				1
#define SPI_SR_CHSIDE			2
#define SPI_SR_UDR				3
#define SPI_SR_CRCERR			4
#define SPI_SR_MODF				5
#define SPI_SR_OVR				6
#define SPI_SR_BSY				7
#define SPI_SR_FRE				8

/*
 * Bit position definition for APBxENR to Enable the respective I2Cx peripheral
 */
#define SPI1_EN				12
#define SPI2_EN				14
#define SPI3_EN				15
#define SPI4_EN				13

/*
 * Peripheral Clock Enable macros for SPIx registers
 */
#define SPI1_PCLK_EN()			(RCC->APB2ENR|=(1<<SPI1_EN))
#define SPI2_PCLK_EN()			(RCC->APB1ENR|=(1<<SPI2_EN))
#define SPI3_PCLK_EN()			(RCC->APB1ENR|=(1<<SPI3_EN))
#define SPI4_PCLK_EN()			(RCC->APB2ENR|=(1<<SPI4_EN))

/*
 * Peripheral Clock Enable macros for SPIx registers
 */
#define SPI1_PCLK_DIS()			(RCC->APB2ENR &=~(1<<SPI1_EN))
#define SPI2_PCLK_DIS()			(RCC->APB1ENR &=~(1<<SPI2_EN))
#define SPI3_PCLK_DIS()			(RCC->APB1ENR &=~(1<<SPI3_EN))
#define SPI4_PCLK_DIS()			(RCC->APB2ENR &=~(1<<SPI4_EN))

/*
 * Bit position definition for APBxRSTR to Reset the respective I2Cx peripheral
 */
#define SPI1_RST			12
#define SPI2_RST			14
#define SPI3_RST			15
#define SPI4_RST			13

/*
 * Register Reset macros for SPIx registers
 */
#define SPI1_REG_RESET()		do{RCC->APB2RSTR|=(1<<SPI1_RST);RCC->APB2RSTR&=~(1<<SPI1_RST);}while(0)
#define SPI2_REG_RESET()		do{RCC->APB1RSTR|=(1<<SPI2_RST);RCC->APB1RSTR&=~(1<<SPI2_RST);}while(0)
#define SPI3_REG_RESET()		do{RCC->APB1RSTR|=(1<<SPI3_RST);RCC->APB1RSTR&=~(1<<SPI3_RST);}while(0)
#define SPI4_REG_RESET()		do{RCC->APB2RSTR|=(1<<SPI4_RST);RCC->APB2RSTR&=~(1<<SPI4_RST);}while(0)


/*
 * I2C Register definition
 */
typedef struct
{
	volatile uint32_t CR1;		//0x00
	volatile uint32_t CR2;		//0x04
	volatile uint32_t OAR1;		//0x08
	volatile uint32_t OAR2;		//0x0C
	volatile uint32_t DR;		//0x10
	volatile uint32_t SR1;		//0x14
	volatile uint32_t SR2;		//0x18
	volatile uint32_t CCR;		//0x1C
	volatile uint32_t TRISE;	//0x20
	volatile uint32_t FLTR;		//0x24
}I2C_RegDef_t;

#define I2C1					((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2					((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3					((I2C_RegDef_t*)I2C3_BASEADDR)

/*
 * Bit position definition for I2C_CR1
 */
#define I2C_CR1_PE				0
#define I2C_CR1_SMBUS			1
//Reserved
#define I2C_CR1_SMBTYPE			3
#define I2C_CR1_ENARP			4
#define I2C_CR1_ENPEC			5
#define I2C_CR1_ENGC			6
#define I2C_CR1_NOSTRETCH		7
#define I2C_CR1_START			8
#define I2C_CR1_STOP			9
#define I2C_CR1_ACK				10
#define I2C_CR1_POS				11
#define I2C_CR1_PEC				12
#define I2C_CR1_ALERT			13
//Reserved
#define I2C_CR1_SWRST			15


/*
 * Bit position definition for I2C_CR2
 */
#define I2C_CR2_FREQ			0
//Reserved
//Reserved
#define I2C_CR2_ITERREN			8
#define I2C_CR2_ITEVTEN			9
#define I2C_CR2_ITBUFEN			10
#define I2C_CR2_DMAEN			11
#define I2C_CR2_LAST			12

/*
 * Bit position definition for I2C_OAR1
 */
#define I2C_OAR1_ADD0			0
#define I2C_OAR1_ADD7_1			1
#define I2C_OAR1_ADD9_8			8
#define I2C_OAR1_ADDMODE		15


/*
 * Bit position definition for I2C_OAR2
 */
#define I2C_OAR2_ENDUAL			0
#define I2C_OAR2_ADD2_7_1		1


/*
 * Bit position definition for I2C_SR1
 */
#define I2C_SR1_SB				0
#define I2C_SR1_ADDR			1
#define I2C_SR1_BTF				2
#define I2C_SR1_ADD10			3
#define	I2C_SR1_STOPF			4
//reserved
#define I2C_SR1_RXNE			6
#define I2C_SR1_TXE				7
#define I2C_SR1_BERR			8
#define I2C_SR1_ARLO			9
#define I2C_SR1_AF				10
#define I2C_SR1_OVR				11
#define I2C_SR1_PECERR			12
//reserved
#define I2C_SR1_TIMEOUT			14
#define I2C_SR1_SMBALERT		15


/*
 * Bit position definition for I2C_SR2
 */
#define I2C_SR2_MSL				0
#define I2C_SR2_BUSY			1
#define I2C_SR2_TRA				2
//Reserved
#define	I2C_SR2_GENCALL			4
#define I2C_SR2_SMBDEFAULT		5
#define	I2C_SR2_SMBHOST			6
#define I2C_SR2_DUALF			7
#define I2C_PEC_7_0				8


/*
 * Bit position definition for I2C_CCR
 */
#define I2C_CCR_11_0			0
#define I2C_CCR_DUTY			14
#define I2C_CCR_FS				15


/*
 * Bit position definition for ABP1ENR to Enable the respective I2Cx peripheral
 */
#define I2C1_EN				21
#define I2C2_EN				22
#define I2C3_EN				23

/*
 * Peripheral Clock Enable macros for I2Cx registers
 */
#define I2C1_PCLK_EN()			(RCC->APB1ENR|=(1<<I2C1_EN))
#define I2C2_PCLK_EN()			(RCC->APB1ENR|=(1<<I2C2_EN))
#define I2C3_PCLK_EN()			(RCC->APB1ENR|=(1<<I2C3_EN))

/*
 * Peripheral Clock Enable macros for I2Cx registers
 */
#define I2C1_PCLK_DIS()			(RCC->APB1ENR &=~(1<<I2C1_EN))
#define I2C2_PCLK_DIS()			(RCC->APB1ENR &=~(1<<I2C2_EN))
#define I2C3_PCLK_DIS()			(RCC->APB1ENR &=~(1<<I2C3_EN))

/*
 * Bit position definition for AHB1RSTR to Reset the respective I2Cx peripheral
 */
#define I2C1_RST			21
#define I2C2_RST			22
#define I2C3_RST			23

#define I2C1_REG_RESET()		do{RCC->AHB1RSTR|=(1<<I2C1_RST);RCC->AHB1RSTR&=~(1<<I2C1_RST);}while(0)
#define I2C2_REG_RESET()		do{RCC->AHB1RSTR|=(1<<I2C2_RST);RCC->AHB1RSTR&=~(1<<I2C2_RST);}while(0)
#define I2C3_REG_RESET()		do{RCC->AHB1RSTR|=(1<<I2C3_RST);RCC->AHB1RSTR&=~(1<<I2C3_RST);}while(0)


/*
 * USART Register definition
 */
typedef struct
{
	volatile uint32_t SR;		//0x00
	volatile uint32_t DR;		//0x04
	volatile uint32_t BRR;		//0x08
	volatile uint32_t CR1;		//0x0C
	volatile uint32_t CR2;		//0x10
	volatile uint32_t CR3;		//0x14
}USART_RegDef_t;

#define USART1					((USART_RegDef_t *)USART1_BASEADDR)
#define USART2					((USART_RegDef_t *)USART2_BASEADDR)
#define USART3					((USART_RegDef_t *)USART3_BASEADDR)
#define UART4					((USART_RegDef_t *)UART4_BASEADDR)
#define UART5					((USART_RegDef_t *)UART5_BASEADDR)
#define USART6					((USART_RegDef_t *)USART6_BASEADDR)

/*
 * Bit position definition for USART_SR register
 * Reset Value: 0x00C0 0000
 */
//Reserved bits:31-10 must be kept at RESET value
#define USART_SR_CTS			9
#define USART_SR_LBD			8
#define USART_SR_TXE			7
#define USART_SR_TC				6
#define USART_SR_RXNE			5
#define USART_SR_IDLE			4
#define USART_SR_ORE			3
#define USART_SR_NF				2
#define USART_SR_FE				1
#define USART_SR_PE				0

/*
 * Bit position definition for USART_DR register
 * Reset Value: 0x0000 0000
 */
//Reserved bits:31-9 must be kept at RESET value
#define USART_DR_8_0			0


/*
 * Bit position definition for USART_BRR register
 * Reset Value: 0x0000 0000
 */
//Reserved bits:31-14 must be kept at RESET value
#define USART_BRR_DIV_MANTISSA_15_4		4	//12-bits
#define USART_BRR_DIV_FRACTION_3_0		0	//4-bits

/*
 * Bit position definition for USART_CR1 register
 * Reset Value: 0x0000 0000
 */
#define USART_CR1_OVER8			15
//Reserved						Bit 14 must be kept at RESET value
#define USART_CR1_UE			13
#define USART_CR1_M				12
#define USART_CR1_WAKE			11
#define USART_CR1_PCE			10
#define USART_CR1_PS			9
#define USART_CR1_PEIE			8
#define USART_CR1_TXEIE			7
#define USART_CR1_TCIE			6
#define USART_CR1_RXNEIE		5
#define USART_CR1_IDLEIE		4
#define USART_CR1_TE			3
#define	USART_CR1_RE			2
#define USART_CR1_RWU			1
#define USART_CR1_SBK			0


/*
 * Bit position definition for USART_CR2 register
 * Reset Value: 0x0000 0000
 */
//Reserved bits:31-13 must be kept at RESET value
#define USART_CR2_LINEN			14
#define USART_CR2_STOP			12
#define USART_CR2_CLKEN			11
#define USART_CR2_CPOL			10
#define USART_CR2_CPHA			9
#define USART_CR2_LBCL			8
//Reserved bit:7 must be kept at RESET value
#define USART_CR2_LBDIE			6
#define USART_CR2_LBDL			5
//Reserved bit:4 must be kept at RESET value
#define USART_CR2_ADD_3_0		0

/*
 * Bit position definition for USART_CR1 register
 * Reset Value: 0x0000 0000
 */
//Reserved bits:31-12 must be kept at RESET value
#define USART_CR3_ONEBIT		11
#define USART_CR3_CTSIE			10
#define USART_CR3_CTSE			9
#define USART_CR3_RTSE			8
#define USART_CR3_DMAT			7
#define USART_CR3_DMAR			6
#define USART_CR3_SCEN			5
#define USART_CR3_NACK			4
#define USART_CR3_HDSEL			3
#define USART_CR3_IRLP			2
#define USART_CR3_IREN			1
#define USART_CR3_EIE			0

/*
 * Bit position definition for USART_GTPR register
 * Reset Value: 0x0000 0000
 */
//Reserved bits:31-16 must be kept at RESET value
#define	USART_GTPR_GT_15_8		8
#define USART_GTPR_PSC_7_0		0


/*
 * Bit position definition for AHB1RSTR to Reset the respective USARTx peripheral
 */
#define USART1_APB			4
#define USART2_APB			17
#define USART3_APB			18
#define UART4_APB			19
#define UART5_APB			20
#define USART6_APB			5

/*
 * Peripheral Clock Enable macros for USART registers
 */
#define USART1_PCLK_EN()		(RCC->APB2ENR|=(1<<USART1_APB))
#define USART2_PCLK_EN()		(RCC->APB1ENR|=(1<<USART2_APB))
#define USART3_PCLK_EN()		(RCC->APB1ENR|=(1<<USART3_APB))
#define UART4_PCLK_EN()			(RCC->APB1ENR|=(1<<UART4_APB))
#define UART5_PCLK_EN()			(RCC->APB1ENR|=(1<<UART5_APB))
#define USART6_PCLK_EN()		(RCC->APB2ENR|=(1<<USART6_APB))

/*
 * Peripheral Clock Enable macros for USART registers
 */
#define USART1_PCLK_DIS()		(RCC->APB2ENR &=~(1<<USART1_APB))
#define USART2_PCLK_DIS()		(RCC->APB1ENR &=~(1<<USART2_APB))
#define USART3_PCLK_DIS()		(RCC->APB1ENR &=~(1<<USART3_APB))
#define UART4_PCLK_DIS()		(RCC->APB1ENR &=~(1<<USART4_APB))
#define UART5_PCLK_DIS()		(RCC->APB1ENR &=~(1<<USART5_APB))
#define USART6_PCLK_DIS()		(RCC->APB2ENR &=~(1<<USART6_APB))

#define USART1_REG_RESET()		do{RCC->APB2RSTR|=(1<<USART1_APB);RCC->APB2RSTR&=~(1<<USART1_APB);}while(0)
#define USART2_REG_RESET()		do{RCC->APB1RSTR|=(1<<USART2_APB);RCC->APB1RSTR&=~(1<<USART2_APB);}while(0)
#define USART3_REG_RESET()		do{RCC->APB1RSTR|=(1<<USART3_APB);RCC->APB1RSTR&=~(1<<USART3_APB);}while(0)
#define UART4_REG_RESET()		do{RCC->APB1RSTR|=(1<<UART4_APB); RCC->APB1RSTR&=~(1<<UART4_APB);} while(0)
#define UART5_REG_RESET()		do{RCC->APB1RSTR|=(1<<UART5_APB); RCC->APB1RSTR&=~(1<<UART5_APB);} while(0)
#define USART6_REG_RESET()		do{RCC->APB2RSTR|=(1<<USART6_APB);RCC->APB2RSTR&=~(1<<USART6_APB);}while(0)

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

//I2C related IRQ positions
#define	IRQ_I2CEV_1				31
#define	IRQ_I2CER_1				32
#define	IRQ_I2CEV_2				33
#define	IRQ_I2CER_2				34
#define	IRQ_I2CEV_3				72
#define	IRQ_I2CER_3				73

//USART related IRQ positions
#define IRQ_USART1				37
#define IRQ_USART2				38
#define IRQ_USART3				39
#define IRQ_UART4				52
#define IRQ_UART5				53
#define IRQ_USART6				71

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

#define FLAG_SET				SET
#define FLAG_CLEAR				CLEAR


#include "stm32f446xx_gpio_driver.h"
#include "stm32f446xx_i2c_driver.h"
#include "stm32f446xx_spi_driver.h"
#include "stm32f446xx_usart_driver.h"
#include <stm32f446xx_xtal_driver.h>

#endif /* INC_STM32F446XX_H_ */

