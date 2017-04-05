#include "main.h"

/*****************************************************************************************/
/*
 *  Sleep functions
 */
/***************************************************************************
 * This is the function which blocks the sleep mode to the minmode set
 * Eg. If minmode is EM2, the device cannot go below EM2
 */
/**************************************************************************************/
/*
 * This function is used for unblocking the minimum sleep level that a peripheral needs.
 * This is an atomic function as it should not be interrupted when being executed
 * Input variables: minimumMode
 * Global Variables: sleep_block_counter
 * Output Variables: None required
 ********************************************************************************
 * @section License
 * <b>Copyright 2016 Silicon Laboratories, Inc. http://www.silabs.com</b>
 *******************************************************************************
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Silicon Labs has no
 * obligation to support this Software. Silicon Labs is providing the
 * Software "AS IS", with no express or implied warranties of any kind,
 * including, but not limited to, any implied warranties of merchantability
 * or fitness for any particular purpose or warranties against infringement
 * of any proprietary rights of a third party.
 *
 * Silicon Labs will not be liable for any consequential, incidental, or
 * special damages, or any other relief, or for any claim by any third party,
 * arising from your use of this Software.
 *
 ******************************************************************************/
void blockSleepMode(sleepstate_t minimumMode)
{
	INT_Disable();
	sleep_block_counter[minimumMode]++;
	INT_Enable();
}
/*
 * This is the function which unblocks the sleep mode to the minmode set
 * Eg. If minmode is EM2, the device cannot go below EM2
 */
/**************************************************************************************/
/*
 * This function is used for unblocking the minimum sleep level that a peripheral needs
 * Input variables: minimumMode
 * Global Variables: sleep_block_counter
 * Output Variables: None required
 ********************************************************************************
 * @section License
 * <b>Copyright 2016 Silicon Laboratories, Inc. http://www.silabs.com</b>
 *******************************************************************************
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Silicon Labs has no
 * obligation to support this Software. Silicon Labs is providing the
 * Software "AS IS", with no express or implied warranties of any kind,
 * including, but not limited to, any implied warranties of merchantability
 * or fitness for any particular purpose or warranties against infringement
 * of any proprietary rights of a third party.
 *
 * Silicon Labs will not be liable for any consequential, incidental, or
 * special damages, or any other relief, or for any claim by any third party,
 * arising from your use of this Software.
 *
 ******************************************************************************/
void unblockSleepMode(sleepstate_t minimumMode)
{
	INT_Disable();
	if(sleep_block_counter[minimumMode] > 0)
	{
		sleep_block_counter[minimumMode]--;
	}
	INT_Enable();
}
/***************************************************************************/
/**************************************************************************************/
/*
 * This function contains the sleep routine to put the board to sleep . This routine
 * takes the board to the minimum sleep mode possible
 * Input variables: None required
 * Global Variables: sleep_block_counter
 * Output Variables: None required
 ********************************************************************************
 * @section License
 * <b>Copyright 2016 Silicon Laboratories, Inc. http://www.silabs.com</b>
 *******************************************************************************
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Silicon Labs has no
 * obligation to support this Software. Silicon Labs is providing the
 * Software "AS IS", with no express or implied warranties of any kind,
 * including, but not limited to, any implied warranties of merchantability
 * or fitness for any particular purpose or warranties against infringement
 * of any proprietary rights of a third party.
 *
 * Silicon Labs will not be liable for any consequential, incidental, or
 * special damages, or any other relief, or for any claim by any third party,
 * arising from your use of this Software.
 *
 ******************************************************************************/
void sleep(void)
{
	if (sleep_block_counter[0] > 0)
	{
		return; 						// BlockedeveryingbelowEM0
	}
	else if(sleep_block_counter[1] > 0)
	{
		EMU_EnterEM1();					// BlockedeveryithingbelowEM1, enterEM1
	}
	else if(sleep_block_counter[2] > 0)
	{
		EMU_EnterEM2(true);				// BlockedeverythingbelowEM2, enterEM2
	}
	else if(sleep_block_counter[3] > 0)
	{
		EMU_EnterEM3(true);				// Block every thing below EM3, enterEM3
	}
	else
	{
		EMU_EnterEM3(true);				// Nothingisblocked, enterEM3  as EM4 is dangerous
	}
	return;

}
/***************************************************************************/
/*
 * Peripheral function
 */

/**************************************************************************************/
/*
 * This function initializes the GPIO LEDS
 * Input variables: None required
 * Global Variables: None required
 * Output Variables: None required
 ********************************************************************************
 */
void GPIO_LedsInit(void)
{
  CMU_ClockEnable(cmuClock_HFPER, true);
  CMU_ClockEnable(cmuClock_GPIO, true);

  GPIO_PinModeSet(LED0_PORT,LED0_PIN, gpioModePushPull, 0);			//Initialize the LED0
  GPIO_PinModeSet(LED1_PORT,LED1_PIN, gpioModePushPull, 0);			//Initialize the LED1

}
/************************************************************************************/
/**************************************************************************************/
/*
 * This function turns on the LED that is specified
 * Input variables: LED_no
 * Global Variables: None required
 * Output Variables: None required
 ********************************************************************************
  ******************************************************************************/
void Turn_on_LED(uint8_t LED_no)
{
	if(LED_no==0)
		GPIO_PinOutSet(LED0_PORT,LED0_PIN);
	else
		GPIO_PinOutSet(LED1_PORT,LED1_PIN);
}

/**************************************************************************************/
/*
 * This function turns off the LED that is specified
 * Input variables: LED_no
 * Global Variables: None required
 * Output Variables: None required
 ********************************************************************************
 ******************************************************************************/
void Turn_off_LED(uint8_t LED_no)
{
	if(LED_no==0)
		GPIO_PinOutClear(LED0_PORT,LED0_PIN);		//LED0 is on portE pin2
	else
		GPIO_PinOutClear(LED1_PORT,LED1_PIN);		//LED1 is on portE pin3
}
/**************************************************************************************/
/*
 * This function toggles the LED that is specified
 * Input variables: LED_no
 * Global Variables: None required
 * Output Variables: None required
 ********************************************************************************
 ******************************************************************************/
void Toggle_LED(uint8_t LED_no)
{
	if(LED_no==0)
		GPIO_PinOutToggle(LED0_PORT,LED0_PIN);
	else
		GPIO_PinOutToggle(LED1_PORT,LED1_PIN);
}

/********************************************************************************/
/**************************************************************************************/
/* * This function initializes the Analog comparator
 * Input variables: None required
 * Global Variables: sleep_block_counter
 * Output Variables: None required
 ********************************************************************************
 ********************************************************************************
* Light sense: Output from the photodiode is connected to PC6 and ACMP0 Channel 6
* * The excite pin is input to the photodiode and is connected to the PD6 Something LES_ALTEX0*/

void Capacitive_Sensor_Init(void)
{
	CMU_ClockEnable(cmuClock_ACMP1,true);
#if 0
	ACMP_Init_TypeDef my_acmp0;				//might be a global variable

	my_acmp0.biasProg=0x7;									/* biasProg */
	my_acmp0.enable=false;									//Enable after init
	my_acmp0.fullBias=false;								//default is false higher the bias the faster the comparison
	my_acmp0.halfBias=false;								//Lower bias better for power
	my_acmp0.hysteresisLevel=acmpHysteresisLevel7;			//Higher the better..default is 5
	my_acmp0.inactiveValue=false;
	my_acmp0.interruptOnFallingEdge=false;
	my_acmp0.interruptOnRisingEdge=false;					//Default is false
	my_acmp0.lowPowerReferenceEnabled=false;					/* Disabled emitting inactive value during warmup. */
	my_acmp0.vddLevel=0x3D;						/* VDD level */
	my_acmp0.warmTime=acmpWarmTime512;						/* 512 cycle warmup to be safe */


	ACMP_Init(ACMP0, &my_acmp0);

	ACMP0->INPUTSEL=0X02<<_ACMP_INPUTSEL_VDDLEVEL_SHIFT;

	ACMP_ChannelSet(ACMP0,acmpChannelVDD,acmpChannel6);
#endif
/************************************************************************/
	ACMP_CapsenseInit_TypeDef my_acmp_capsense=
		{
		   .fullBias                 = false,
		   .halfBias                 = false,
		   .biasProg                 = 0x7,
		   .warmTime                 = acmpWarmTime512,
		   .hysteresisLevel          = acmpHysteresisLevel7,
		   .resistor                 = acmpResistor0,
		   .lowPowerReferenceEnabled = false,
		   .vddLevel                 = 0x3D,
		   .enable                   = false
		  };

	ACMP_CapsenseInit(ACMP1,&my_acmp_capsense);
	/*// if gpio is to be disabled
	// Configure ACMP locations, ACMP output to pin disabled.
	  ACMP_GPIOSetup(ACMP0, 0, false, false);
	  ACMP_GPIOSetup(ACMP1, 0, false, false);
*/
	  // Initialize ACMPs in capacitive sense mode.
	  /*ACMP_CapsenseInit(ACMP0, &initACMP);
	  ACMP_CapsenseInit(ACMP1, &initACMP);*/

	ACMP_Channel_TypeDef my_acmp_channel=acmpChannelCapSense;
	ACMP_CapsenseChannelSet(ACMP1,my_acmp_channel);

	  //ACMP_ChannelSet(ACMP0,acmpChannelVDD,acmpChannel6);

	//Interrupts part
	ACMP0->IFC=0xFFFF ;									//Clear all interrupts
	blockSleepMode(ACMP_LOWEST_ENERGY_MODE);					//Minimum is EM3 possible
	ACMP_Enable(ACMP1);
}
/***********************************************************************************/
int main(void)
{
  /* Chip errata */
  CHIP_Init();
  GPIO_LedsInit();
#if SYSTEM_TEST==ON
  while(1)
  {
  Turn_on_LED(1);
  for(int i=0;i<1000000;i++);
  Turn_off_LED(1);
  for(int i=0;i<1000000;i++);
  }
#endif
#if CAPACITIVE_SENSOR==ON
  Capacitive_Sensor_Init();
#endif
  /* Infinite loop */
  while (1) {
	  //sleep();
  }
}
