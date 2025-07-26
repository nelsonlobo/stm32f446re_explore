/*
 * stm32f446xx_i2c_drivers.h
 *
 *  Created on: Jul 14, 2025
 *      Author: Nelson Lobo
 */

#ifndef INC_STM32F446XX_I2C_DRIVER_H_
#define INC_STM32F446XX_I2C_DRIVER_H_

/*
 * I2C Clock Frequency Calculation
 * STEP#1:
 * Identify the APB clock frequency
 * In our case the APB freq = 16MHz i.e. Time period 62.5ns = 62.5 x 10^(-9)
 * STEP#2:
 * Select the Mode: Standard Mode(upto 100KHz) or Fast Mode(upto 400KHz)
 * STEP#3:
 * Configure CR2 the 5-bit Freq field as 16, since our APB1 clock is set at 16MHz we need to,
 * STEP#4:
 * (https://www.unitjuggler.com/convert-frequency-from-Hz-to-%C2%B5s(p).html?val=100000)
 * Now we need to calculate the 12-bit CCR filed
 ********************************************
 ********************************************
 ********************************************
 * Case#1: Configure for 100KHz i.e. 10uS
 ********************************************
 * Select Standard Mode
 * i.e. time period = 10uS, Thigh = 5uS, Tlow = 5uS
 * Thigh(scl) 	= CCR * Tpclk
 * Tlow(scl)	= CCR * Tpclk
 * i.e. Thigh(scl)+Tlow(scl) = 2*(CCR * Tpclk)
 * i.e. 10*10^(-6) = 2*(CCR * 62*10^(-9))
 * i.e.  5*10^(-6) = CCR * 62.5*10^(-9)
 * CCR 	= (5*10^(-6))/(62.5 * 10^(-9))
 * 		= (5*10^3)/62.5 = 80 i.e. 0x50
 * 	Tlow 	= 5uS
 * 	Thigh 	= 5uS
 ********************************************
 ********************************************
 ********************************************
 * Case#2: Configure for 200KHz i.e. 5uS
 ********************************************
 ********************************************
 * Select Fast Mode in CR2 register
 * NOTE: When we select Fast mode we need to take care of the Duty Cycle bit in the CCR register.
 * For speeds less than 400KHz we can select Tlow = 2*Thigh
 * else select 9*Tlow = 16*Thigh i.e. approx Tlow = 1.8*Thigh
 ********************************************
 ********************************************
 * i.e. time period = 5uS, Thigh = 2.5uS, Tlow = 2.5uS
 * Thigh(scl) 	= 1 * CCR * Tpclk
 * Tlow(scl)	= 2 * CCR * Tpclk
 * i.e. Thigh(scl)+Tlow(scl) = 3*(CCR * Tpclk)
 * i.e. 5*10^(-6) 	= CCR*(3*62.5*10^(-9))
 * 		CCR 		= (5*10^(-6))/(3*62.5*10^(-9))
 * 					= (5*10^3)/(3*62.5)
 * 					= 26.67 ~27 i.e 0x1B
 ********************************************
 * 	Tlow 	= 3.375uS
 * 	Thigh 	= 1.687uS
 * 	Period  = 5.062uS i.e. 197.55kHz
 ********************************************
 ********************************************
 ********************************************
 * Case#3: Configure for 400KHz i.e. 2.5uS
 ********************************************
 * i.e. time period = 2.5uS, Thigh = 1.25uS, Tlow = 1.25uS
 * Thigh(scl) 	= 9 * CCR * Tpclk
 * Tlow(scl)	= 16* CCR * Tpclk
 * i.e. Thigh(scl)+Tlow(scl) = 25*(CCR * Tpclk)
 * i.e. 2.5*10^(-6) = CCR*(25*62.5*10^(-9))
 * 		CCR 		= (2.5*10^(-6))/(25*62.5*10^(-9))
 * 					= (2.5*10^3)/(25*62.5)
 * 					= 1.6 ~2 i.e 0x02
 ********************************************
 * 	Tlow 	= 2uS
 * 	Thigh 	= 1.125uS i.e. 320KHz
 ********************************************
 ********************************************
 * Minimum Requirements for Clock signals:
 * Standard Mode:
 * Tlow		= 4.7uS
 * Thigh 	= 4.0uS
 ********************************************
 * Fast Mode:
 * Tlow 	= 1.3uS
 * Thigh	= 0.6uS
 ********************************************
 * Fast Mode Plus:
 * Tlow		= 0.5uS
 * Thigh	= 0.26uS
 ********************************************
 ********************************************
 ********************************************
 * How to Calculate the Resistor values
 ********************************************
 ********************************************
 Look up Page.43 in the I2C bus specification datasheet.
 For Fast Mode I2C communication with following parameters:
 Vcc = 3.3V
 Bus capacitance Cb = 110pF, (assumption where pin capacitance = 10pF(DS3231), bus capacitance ~100pF)
 Iol = 3mA, Vol = 0.4V (Pg.43 of I2C bus specs)
 Rp(min) = (3.3-0.4)/(3*10^(-3)) = 966ohms
 Trise = 300ns, Cb = 110pF, (t2-t1) = 0.8473(pg.50 of I2C bus specs)
 Rp(max) = (300*10^(-9))/(0.8473*110*10^(-12)) = 3218 = 3.2kOhms
 ********************************************
 ********************************************
 Look up Page.43 in the I2C bus specification datasheet.
 For Standard Mode I2C communication with following parameters:
 Vcc = 3.3V
 Bus capacitance Cb = 110pF, (assumption where pin capacitance = 10pF(DS3231), bus capacitance ~100pF)
 Iol = 3mA, Vol = 0.4V (Pg.43 of I2C bus specs)
 Rp(min) = (3.3-0.4)/(3*10^(-3)) = 966ohms
 Trise = 300ns, Cb = 110pF, (t2-t1) = 0.8473(pg.50 of I2C bus specs)
 Rp(max) = (1000*10^(-9))/(0.8473*110*10^(-12)) = 10729 = 10.73kOhms
 ********************************************
 */

#define MY_I2C_ADDR			0x61
#define ARDUINO_SLAVE_ADDR	0x68

/*
 * Configuration structure for I2Cx peripheral
 */
typedef struct
{
	uint32_t 	I2C_SclSpeed;
	uint8_t 	I2C_AddrMode;
	uint16_t 	I2C_DeviceAddress;
	uint8_t 	I2C_AckControl;
	uint16_t	I2C_FMDutyCycle;
}I2C_Config_t;

/*
 * Handle structure for I2Cx peripheral
 */
typedef struct
{
	I2C_RegDef_t	*pI2Cx;
	I2C_Config_t	I2C_Config;

	uint8_t 		*pTxBuffer;
	uint8_t			*pRxBuffer;
	uint32_t 		txLen;
	uint32_t		rxLen;
	uint8_t			TxRxState;	//I2C_READY/BUSY_RX/BUSY_TX
	uint8_t			DevAddr;
	uint32_t		RxSize;
	uint8_t			SR;
	uint8_t 		rxComplete;
}I2C_Handle_t;


#define I2C_READY						0
#define I2C_BUSY_RX						1
#define I2C_BUSY_TX						2

/*
 * I2C read/n-Write
 */
#define I2C_READ						0x01
#define I2C_WRITE						0x00


/*
 * @I2C_SclSpeed
 */
#define I2C_SCL_SPEED_SM100K			100000
#define I2C_SCL_SPEED_FM400K			400000
#define I2C_SCL_SPEED_FM200K			200000

/*
 * @I2C_AckControl
 */
#define I2C_ACK_ENABLE					1
#define I2C_ACK_DISABLE					0

/*
 * @I2C_FMDutyCycle
 */
#define I2C_FM_DUTY_2					0
#define I2C_FM_DUTY_16_9				1

/*
 * I2C related status flag definitions
 */
#define	I2C_FLAG_SB						(1<<I2C_SR1_SB)
#define	I2C_FLAG_ADDR					(1<<I2C_SR1_ADDR)
#define I2C_FLAG_BTF					(1<<I2C_SR1_BTF)
#define I2C_FLAG_ADD10					(1<<I2C_SR1_ADD10)
#define I2C_FLAG_STOP					(1<<I2C_SR1_STOPF)
#define I2C_FLAG_RXNE					(1<<I2C_SR1_RXNE)
#define I2C_FLAG_TXE					(1<<I2C_SR1_TXE)
#define I2C_FLAG_BERR					(1<<I2C_SR1_BERR)
#define I2C_FLAG_ARLO					(1<<I2C_SR1_ARLO)
#define I2C_FLAG_AF						(1<<I2C_SR1_AF)
#define I2C_FLAG_OVR					(1<<I2C_SR1_OVR)
#define I2C_FLAG_PECERR					(1<<I2C_SR1_PECERR)
#define I2C_FLAG_TIMEOUT				(1<<I2C_SR1_TIMEOUT)
#define I2C_FLAG_SMBALERT				(1<<I2C_SR1_SMBALERT)

//I2C Events macros
#define I2C_EV_TX_CMPLT					0
#define I2C_EV_RX_CMPLT					1
#define I2C_EV_STOP						2

#define I2C_ERROR_BERR  				3
#define I2C_ERROR_ARLO  				4
#define I2C_ERROR_AF    				5
#define I2C_ERROR_OVR   				6
#define I2C_ERROR_TIMEOUT 				7

//Peripheral clock setup
void I2C_PClk_Ctrl(I2C_RegDef_t *pGPIOx, uint8_t status);

//I2C init & deinit
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);


/*
 * Data Send and Receive Polling based API
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle,uint8_t *pTxBuffer, uint32_t len, uint8_t slaveAddr,uint8_t SR);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint32_t len, uint8_t slaveAddr,uint8_t SR);

/*
 * Data Send and Receive Interrupt based API
 */
uint8_t I2C_MasterSendDataIntr(I2C_Handle_t *pI2CHandle,uint8_t *pTxBuffer, uint32_t len, uint8_t slaveAddr,uint8_t SR);
uint8_t I2C_MasterReceiveDataIntr(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint32_t len, uint8_t slaveAddr,uint8_t SR);

/*
 * IRQ configuration and ISR handling
 */
void I2C_IRQConfig(uint8_t IRQPosition, bool status);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t status);
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t flagName);
void I2C_ManageACK_bit(I2C_RegDef_t *pI2Cx,uint8_t status);

void I2C_CloseSendData(I2C_Handle_t *pI2CHandle);
void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle);
/*
 * I2C application callback
 */
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t evntVal);

#endif /* INC_STM32F446XX_I2C_DRIVER_H_ */
