# STM32F4xxyy Exploration Journal

## MCU & Dev boards used:
1. NUCLEO-F446RE
2. STM32F407
3. STM32F411

## Programs executed
### 1. Hello World
- Introduction ot SWV (Serial Wire Viewer), also known to use SWO line in the ITM hardware.
- To use the SWV we need to use the following piece of code in syscall.c
```c
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//					Implementation of printf like feature using ARM Cortex M3/M4/ ITM functionality
//					This function will not work for ARM Cortex M0/M0+
//					If you are using Cortex M0, then you can use semihosting feature of openOCD
/////////////////////////////////////////////////////////////////////////////////////////////////////////


//Debug Exception and Monitor Control Register base address
#define DEMCR        			*((volatile uint32_t*) 0xE000EDFCU )

/* ITM register addresses */
#define ITM_STIMULUS_PORT0   	*((volatile uint32_t*) 0xE0000000 )
#define ITM_TRACE_EN          	*((volatile uint32_t*) 0xE0000E00 )

void ITM_SendChar(uint8_t ch)
{

	//Enable TRCENA
	DEMCR |= ( 1 << 24);

	//enable stimulus port 0
	ITM_TRACE_EN |= ( 1 << 0);

	// read FIFO status in bit [0]:
	while(!(ITM_STIMULUS_PORT0 & 1));

	//Write to ITM stimulus port0
	ITM_STIMULUS_PORT0 = ch;
}
```
- Modify the write function to use ITM_SendChar()
```c
    //__io_putchar(*ptr++);
	  ITM_SendChar(*ptr++);
```
- In Debugger setting ENABLE Serial Wire Viewer option
-  Set the Core Clock Speed to half the frequency set for the CPU.
- Apply and close. 
- If you are planning to use many printf statements, it would be best to increase the following value from 0x400 to 0x1000 or higher based on your requirement:
```c
_Min_Stack_Size = 0x400; /* required amount of stack */
```
- Now you can start using printf statements in your code.
- Try not using them in ISRs.
---
### 2. MCO
---
### 3. HSE Measurements
---
### 4. volatile usage
---
### 5. GPIO drivers
1. Polling
2. Interrupt driven
--- 
### 6. SPI drivers
1. Polling
   - Master
   - Slave 
2. Interrupt driven
   - Master
   - Slave 
---