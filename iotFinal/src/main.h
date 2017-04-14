/*
 * main.h
 *
 *  Created on: Apr 3, 2017
 *      Author: richa
 */

#ifndef SRC_MAIN_H_
#define SRC_MAIN_H_


#include <stdint.h>
#include <stdbool.h>
#include <stdint.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_int.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_letimer.h"
#include "em_gpio.h"
#include "em_acmp.h"
#include "em_timer.h"
#include "em_adc.h"
#include "dmactrl.h"
#include "em_dma.h"
#include "em_i2c.h"
#include "em_leuart.h"
#include "em_lcd.h"
#include "bme280.h"
#include "bme280_support.h"
#include "em_prs.h"
#include "em_system.h"

#define ON							1
#define OFF							0

#define LOWEST_ENERGY_MODE 			EM0			//Set the default minimum Energy mode here
#define LED0_PORT					gpioPortE
#define LED1_PORT					gpioPortE
#define LED0_PIN					2
#define LED1_PIN					3



#define I2C1_SDA_PORT				gpioPortC
#define I2C1_SCL_PORT				gpioPortC
#define I2C1_SDA_PIN				4
#define I2C1_SCL_PIN				5

#define I2C1_POWER_PORT				gpioPortD
#define I2C1_POWER_PIN				0
#define I2C1_INTERRUPT_PORT			gpioPortD
#define I2C1_INTERRUPT_PIN			1

#define I2C_MIN_SLEEP_BLOCK			EM1
#define I2C_SPEED_MODE				i2cClockHLRFast					//i2cClockHLRStandard
#define I2C_FREQ_MAX				I2C_FREQ_STANDARD_MAX			//I2C_FREQ_FAST_MAX

#define BME280_I2C_ADDRESS			0x76							//0x77 if SDO is connected to high
#define WRITE_TO_I2C				0
#define READ_FROM_I2C				1
typedef enum
{
	EM0=0, EM1, EM2, EM3
}sleepstate_t;

uint8_t sleep_block_counter[4];
/*
 * Lowest Energy modes
 */
#define ACMP_LOWEST_ENERGY_MODE		EM3

/*
 * Modules Power Control
 */
#define SYSTEM_TEST					OFF
#define CAPACITIVE_SENSOR			OFF
#define I2C_FOR_BME280				OFF			//only turn this on to test the BME280
#define BME280						OFF
#define PWM_TEST					OFF
#define LEUART_TEST					ON

/*##################################################################*/
#define LEUART0_TX_PORT				gpioPortD
#define LEUART0_TX_PIN				4
#define LEUART0_RX_PORT				gpioPortD
#define LEUART0_RX_PIN				5

//LEUART0 Initialization values
#define	LEU0_PARITY_VALUE				leuartNoParity
#define LEU0_BAUD						9600
#define	LEU0_STOPB						leuartStopbits1
#define LEU0_DATAB						leuartDatabits8
#define LEU0_REF_FREQ					0
#define LEAURT_LOOPBACK					ON
#define LEUART_INTERRUPTS				ON
#define LEUART_TX_INTERRUPT				ON




#endif /* SRC_MAIN_H_ */
