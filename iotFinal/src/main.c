#include "main.h"

int Desired_Period;
int LETIMER0_prescaler;
int Prescaled_two_power;

#define LETIMER0_Max_Count 0xFFFF
#define LETIMER0_period 0.004  // the ambient light sensor should be excited every 4 mili seconds
#define OnTime 5.5  // the peiod wanterd in the question
#define LETIMER0_LFXO_count 32768
#define PortD gpioPortD
#define Pin1 1
#define PWM_FREQ 10000

typedef enum
{
	pwmA=350,pwmB=460,pwmC=700,pwmD=1300
}pwmState_t;
pwmState_t pwmCapValue=pwmA;
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



void CLOCK(void)
 {

	 		CMU_OscillatorEnable(cmuOsc_LFXO, true, true);  		//Enabling LFXO
	 	 	CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);		//Selecting  LFA clock

	 //	CMU_ClockEnable(cmuClock_LETIMER0, true);

	 	CMU_ClockEnable(cmuClock_TIMER0, true);
	 	//Enabling the clock for the LETIMER
	 	CMU_ClockEnable(cmuClock_CORELE, true);					//Enabling Low frequency Clock tree
 }


void prescaler()
{
	Desired_Period = OnTime * LETIMER0_LFXO_count;
	LETIMER0_prescaler = 0;

	int temp = Desired_Period / 1;
	Prescaled_two_power = 1;
	while (temp > LETIMER0_Max_Count)
	{
	LETIMER0_prescaler ++;
	Prescaled_two_power = Prescaled_two_power * 2;
	temp =  Desired_Period / Prescaled_two_power;
	}
	Desired_Period = temp;
}

void LETIMER0_Setup(){

	CLOCK(); // starting the clocks

			CMU->LFAPRESC0= LETIMER0_prescaler <<8;
			LETIMER_CompareSet(LETIMER0, 0, Desired_Period );
		    LETIMER_CompareSet(LETIMER0, 1, LETIMER0_period*(LETIMER0_LFXO_count/Prescaled_two_power));

	    // Defining the initialization values of the LETIMER

	           const LETIMER_Init_TypeDef letimerInitvalues =
	             {
	             .enable         = true,                   // Start counting when init is completed.
	             .debugRun       = false,                  // Counter will stop during debug halt.
	             .rtcComp0Enable = false,                  // Stop counting when RTC COMP0 match.
	             .rtcComp1Enable = false,                  // Stop counting when RTC COMP1 match.
	             .comp0Top       = true,                   // COMP0 is used as TOP. Hence, load COMP0 into CNT when counter underflows.
	             .bufTop         = false,                  // Don't load COMP1 into COMP0 when REP0 reaches 0.
	             .out0Pol        = 0,                      // Idle value for output 0.
	             .out1Pol        = 0,                      // Idle value for output 1.
	             .ufoa0          = letimerUFOANone,
	             .ufoa1          = letimerUFOANone,
	             .repMode        = letimerRepeatFree       // Count in a loop.
	             };
		// Clears all interrupts LETIMER0_IFC_UF
		 LETIMER_IntClear(LETIMER0, 0x01);
		 // Enabling the LETIMER
		 LETIMER_IntEnable(LETIMER0, LETIMER_IEN_COMP0|LETIMER_IEN_COMP1);
		 // Initialize LETIMER
		 LETIMER_Init(LETIMER0, &letimerInitvalues);
		 // blocking sleep mode
		 	 	 	 // blockSleepMode(ENERGY_MODE);
		 // Enable interrupt in the processor
		 NVIC_EnableIRQ(LETIMER0_IRQn);
}

void LETIMER0_IRQHandler(){
	int flag;
	 INT_Disable();

	  flag = LETIMER_IntGet(LETIMER0);
	  LETIMER_IntClear(LETIMER0, LETIMER_IFC_COMP0 | LETIMER_IFC_COMP1);

	  // the PWM is here. Depending on that, the interrupt will be handled
	 if(flag & LETIMER_IF_COMP0)
	 {

		  Turn_on_LED(1);
		 ACMP0->CTRL &= ~ACMP_CTRL_EN;
		 LETIMER0 ->IFC = flag;
	 }
		 else
		 {
			 ACMP0->CTRL |=ACMP_CTRL_EN;
			 Turn_off_LED(1);
		 LETIMER0->IFC = flag;
		 while(!(ACMP0->STATUS & ACMP_STATUS_ACMPACT));
		 }
	 INT_Enable();

 }



//void GPIO_Setup(void)
//{
//
//		GPIO_PinModeSet(PortD , Pin1, gpioModePushPull, 1);
//		GPIO_IntConfig(PortD , Pin1,0, true, true);
//		GPIO->IEN = 1<< Pin1;
//  //NVIC_EnableIRQ(GPIO_ODD_IRQn);
//}




void TimerSetup()
{

	GPIO_PinModeSet(gpioPortD, 1, gpioModePushPull, 0);
	// Setting CC channel parameters
	  TIMER_InitCC_TypeDef timerCCInit =
	  {
	    .eventCtrl  = timerEventEveryEdge,
	    .edge       = timerEdgeBoth,
	    .prsSel     = timerPRSSELCh0,
	    .cufoa      = timerOutputActionNone,
	    .cofoa      = timerOutputActionNone,
	    .cmoa       = timerOutputActionToggle,
	    .mode       = timerCCModePWM,
	    .filter     = false,
	    .prsInput   = false,
	    .coist      = false,
	    .outInvert  = false,
	  };

	  /* Configure CC channel 0 */
	    TIMER_InitCC(TIMER0, 0, &timerCCInit);

	    /* Route CC0 to location 3 (PD1) and enable pin */
	    TIMER0->ROUTE |= (TIMER_ROUTE_CC0PEN | TIMER_ROUTE_LOCATION_LOC3);

	    /* Set Top Value */
	    TIMER_TopSet(TIMER0, CMU_ClockFreqGet(cmuClock_HFPER)/PWM_FREQ);

	    /* Set compare value starting at 0 */
	     TIMER_CompareBufSet(TIMER0, 0, 0);


	// setting timer 0 parameters
	 TIMER_Init_TypeDef timer0Init =
	   {
		 .enable     = true,
		 .debugRun   = true,
		 .prescale   = timerPrescale64,
		 .clkSel     = timerClkSelHFPerClk,  // Using the high frequency clock
		 .fallAction = timerInputActionNone,
		 .riseAction = timerInputActionNone,
		 .mode       = timerModeUp,  // counting up
		 .dmaClrAct  = false,
		 .quadModeX4 = false,
		 .oneShot    = false,
		 .sync       = false,
	   };

		 TIMER_IntEnable(TIMER0, TIMER_IF_OF);
	 	 NVIC_EnableIRQ(TIMER0_IRQn);
		 TIMER_Init(TIMER0, &timer0Init);
}

void TIMER0_IRQHandler(void)
{
	/*PWM signal of approximately 150Hz, with increasing
	 * duty cycle. */



	  uint32_t compareValue;

	  /* Clear flag for TIMER0 overflow interrupt */
	  TIMER_IntClear(TIMER0, TIMER_IF_OF);

	  compareValue = TIMER_CaptureGet(TIMER0, 0);
	  /* increment duty-cycle or reset if reached TOP value */

	  //compareValue=700;

	  if( compareValue == TIMER_TopGet(TIMER0))  // Reset
		{TIMER_CompareBufSet(TIMER0, 0, 0);
		}

	  else
		TIMER_CompareBufSet(TIMER0, 0, pwmCapValue); // Increment
		//GPIO_Setup();
}



int main(void)
{
  /* Chip errata */
  CHIP_Init();
#if PWM_TEST==ON
  CLOCK();
  GPIO_LedsInit();
  //LETIMER0_Setup();
  //Turn_on_LED(1);
  TimerSetup();
#endif
  /* Infinite loop */
  while (1) {
	  //sleep();
  }
}
