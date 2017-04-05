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

#define ON							1
#define OFF							0

#define LOWEST_ENERGY_MODE 			EM0			//Set the default minimum Energy mode here
#define LED0_PORT					gpioPortE
#define LED1_PORT					gpioPortE
#define LED0_PIN					2
#define LED1_PIN					3

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
#define CAPACITIVE_SENSOR			ON

#endif /* SRC_MAIN_H_ */
