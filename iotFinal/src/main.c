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
/********************************************************************************/
/**************************************************************************************/
/* * This function initializes the Analog comparator
 * Input variables: None required
 * Global Variables: sleep_block_counter
 * Output Variables: None required
 ********************************************************************************
 ********************************************************************************/

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
void I2C_setup(void)
{
	CMU_ClockEnable(cmuClock_I2C1,true);			//Select the clock
	I2C1->ROUTE=I2C_ROUTE_LOCATION_LOC0;			//Select the location 0 of the sda and scl pins
	I2C1->ROUTE|=I2C_ROUTE_SDAPEN;					//Enable the sda
	I2C1->ROUTE|=I2C_ROUTE_SCLPEN;					//Enable the Scl

	I2C_Init_TypeDef my_i2c1=
	{
		.clhr=I2C_SPEED_MODE,						//Set high speed mode
		.enable=false,								//Dont enable immediately
		.freq=I2C_FREQ_MAX,							//Run at maximum frequency
		.master=true,								//Leopard Gecko is in the master mode
		.refFreq=0
	};

	I2C_Init(I2C1,&my_i2c1);

	//Interrupts part. interrupts are being used only to check status
	I2C1->IFC=0xFFFF ;								//Clear all interrupt flags

	//blockSleepMode(EM1);
	I2C_Enable(I2C1,true);
	if (I2C1->STATE & I2C_STATE_BUSY)			//Exit the busy state.  The I2Cn will be in this state out of RESET
	{
		I2C1->CMD = I2C_CMD_ABORT;
	}
}
/**************************************************************************//**
 *This function is the read driver of the I2C bus based on the requirements for the
 *active light sensor TSL2561
 *
 * Input variables: none
 * Global Variables: none
 * Output Variables: none

 *****************************************************************************/
uint8_t I2C1_read_single_register(uint8_t device_register_address)
{
	//blockSleepMode(I2C_MIN_SLEEP_BLOCK);
	uint8_t data=0;
	I2C1->TXDATA=(BME280_I2C_ADDRESS<<1)|WRITE_TO_I2C;			//Send slave address and the write bit
	I2C1->CMD=I2C_CMD_START;										//Send start command
	while((I2C1->IF & I2C_IF_ACK) ==0);								//Wait for acknowlegement
	I2C1->IFC=I2C_IFC_ACK;											//Clear ack flag

	I2C1->TXDATA=device_register_address;							//send command code and register address
	while((I2C1->IF & I2C_IF_ACK) ==0);								//Wait for ack
	I2C1->IFC=I2C_IFC_ACK;											//clear ack flag

	I2C1->CMD=I2C_CMD_START;										//Resend start command
	I2C1->TXDATA=(BME280_I2C_ADDRESS<<1)|READ_FROM_I2C;			//Send slave address and the read bit
	while((I2C1->IF & I2C_IF_ACK) ==0);								//Wait for ack
	I2C1->IFC=I2C_IFC_ACK;											//Clear ack flag

	while ((I2C1->IF & I2C_IF_RXDATAV) == 0);						//Wait till data is ready in the receive register
	data=I2C1->RXDATA;												//Store data in a variable

	//Single byte read
	I2C1->CMD=I2C_CMD_NACK;											//Send NACK from the master
	I2C1->CMD=I2C_CMD_STOP;											//Send stop bit

	//unblockSleepMode(I2C_MIN_SLEEP_BLOCK);
	return data;
}
/**************************************************************************//**
 *This function is the write driver of the I2C bus based on the requirements for the
 *active light sensor TSL2561
 *
 * Input variables: none
 * Global Variables: none
 * Output Variables: none

 *****************************************************************************/

void I2C1_write_single_register(uint8_t device_register_address,uint8_t data)
{
	//blockSleepMode(I2C_MIN_SLEEP_BLOCK);
	I2C1->TXDATA=(BME280_I2C_ADDRESS<<1)|WRITE_TO_I2C;		//Send slave address and write bit
	I2C1->CMD=I2C_CMD_START;									//Send start command
	while((I2C1->IF & I2C_IF_ACK) ==0);							//Wait for acknowledgement
	I2C1->IFC=I2C_IFC_ACK;										//Clear Acknowlegment flag

	I2C1->TXDATA=device_register_address;						//Write value to the bus
	while((I2C1->IF & I2C_IF_ACK) ==0);							//Wait for acknowledgment
	I2C1->IFC=I2C_IFC_ACK;										//Clear acknowledgement flag

	I2C1->TXDATA=data;											//Transmit data to be written to the device
	while((I2C1->IF & I2C_IF_ACK) ==0);							//Wait for acknowledgement
	I2C1->IFC=I2C_IFC_ACK;										//Clear acknowlegement
	I2C1->CMD=I2C_CMD_STOP;										//Send stop bit
	//unblockSleepMode(I2C_MIN_SLEEP_BLOCK);
}
#if 0
void BME280_I2C_bus_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt)
 /*	\Brief: The function is used as I2C bus write
 *	\Return : Status of the I2C write
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register, will data is going to be written
 *	\param reg_data : It is a value hold in the array,
 *		will be used for write the value into the register
 *	\param cnt : The no of byte of data to be write
 */
{
	//blockSleepMode(I2C_MIN_SLEEP_BLOCK);
		//uint8_t data=0;
		I2C1->TXDATA=(dev_addr<<1)|WRITE_TO_I2C;			//Send slave address and the write bit
		I2C1->CMD=I2C_CMD_START;										//Send start command
		while((I2C1->IF & I2C_IF_ACK) ==0);								//Wait for acknowlegement
		I2C1->IFC=I2C_IFC_ACK;											//Clear ack flag

		I2C1->TXDATA=reg_addr;							//send command code and register address
		while((I2C1->IF & I2C_IF_ACK) ==0);								//Wait for ack
		I2C1->IFC=I2C_IFC_ACK;											//clear ack flag

		I2C1->CMD=I2C_CMD_START;										//Resend start command
		I2C1->TXDATA=(BME280_I2C_ADDRESS<<1)|READ_FROM_I2C;			//Send slave address and the read bit
		while((I2C1->IF & I2C_IF_ACK) ==0);								//Wait for ack
		I2C1->IFC=I2C_IFC_ACK;											//Clear ack flag

		while(cnt)
		{
			while ((I2C1->IF & I2C_IF_RXDATAV) == 0);						//Wait till data is ready in the receive register
			*reg_data=I2C1->RXDATA;												//Store data in a variable
			reg_data++;
			I2C1->CMD=I2C_CMD_ACK;
			cnt--;
		}
		//Single byte read
		I2C1->CMD=I2C_CMD_NACK;											//Send NACK from the master
		I2C1->CMD=I2C_CMD_STOP;											//Send stop bit

		//unblockSleepMode(I2C_MIN_SLEEP_BLOCK);
		//return data;
}
void BME280_I2C_bus_write(uint8_t dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
/*	\Brief: The function is used as SPI bus write
 *	\Return : Status of the SPI write
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register, will data is going to be written
 *	\param reg_data : It is a value hold in the array,
 *		will be used for write the value into the register
 *	\param cnt : The no of byte of data to be write
 */
{
	//blockSleepMode(I2C_MIN_SLEEP_BLOCK);
		I2C1->TXDATA=(dev_addr<<1)|WRITE_TO_I2C;		//Send slave address and write bit
		I2C1->CMD=I2C_CMD_START;									//Send start command
		while((I2C1->IF & I2C_IF_ACK) ==0);							//Wait for acknowledgement
		I2C1->IFC=I2C_IFC_ACK;										//Clear Acknowlegment flag

		I2C1->TXDATA=reg_addr;						//Write value to the bus
		while((I2C1->IF & I2C_IF_ACK) ==0);							//Wait for acknowledgment
		I2C1->IFC=I2C_IFC_ACK;										//Clear acknowledgement flag

		while(cnt)
		{
			I2C1->TXDATA=*reg_data;											//Transmit data to be written to the device
			reg_data++;
			while((I2C1->IF & I2C_IF_ACK) ==0);							//Wait for acknowledgement
			I2C1->IFC=I2C_IFC_ACK;										//Clear acknowlegement
			cnt--;
		}
		I2C1->CMD=I2C_CMD_STOP;										//Send stop bit
		//unblockSleepMode(I2C_MIN_SLEEP_BLOCK);

}
#endif

/***********************************************************************************/
/**************************************************************************************/
/*
 * This function initializes the clock based on sleep mode
 * Input variables: None required
 * Global Variables: sleep_block_counter
 * Output Variables: None required
 ********************************************************************************
  ******************************************************************************/
void Clock_setup(sleepstate_t current_energy_mode)
{
	if(current_energy_mode==EM3)
			{
				CMU_OscillatorEnable(cmuOsc_ULFRCO,true, true);		//use ULFRCO for EM3
				CMU_ClockSelectSet(cmuClock_LFA,cmuSelect_ULFRCO);
				CMU_ClockEnable(cmuClock_CORELE, true);		//Supposed to be CORELE. Do a search in the enum part to know

				blockSleepMode(EM3);
			}
		else
			{
				CMU_OscillatorEnable(cmuOsc_LFXO,true, true);		//use ULFRCO for EM3
				CMU_ClockEnable(cmuClock_HFPER,true);
				CMU_OscillatorEnable(cmuOsc_HFRCO,true, true);		//use ULFRCO for EM3
				CMU_ClockSelectSet(cmuClock_LFA,cmuSelect_LFXO);
				CMU_ClockEnable(cmuClock_CORELE, true);

				//blockSleepMode(EM2);
			}

}


void LEUART0_Setup(void)
{
	  LEUART_Init_TypeDef my_leuart=
	  {
			  .baudrate=LEU0_BAUD,
			  .databits=LEU0_DATAB,
			  .enable=leuartDisable,
			  .parity=LEU0_PARITY_VALUE,
			  .refFreq=LEU0_REF_FREQ,
			  .stopbits=LEU0_STOPB
	  };
	  CMU_ClockSelectSet(cmuClock_LFB,cmuSelect_LFXO);
	  CMU_ClockEnable(cmuClock_LFB,true);										//Select the clock
	  CMU_ClockEnable(cmuClock_LEUART0,true);										//Select the clock



	GPIO_PinModeSet(LEUART0_TX_PORT,LEUART0_TX_PIN, gpioModePushPull, 1);			//Initialize the LED0
	GPIO_PinModeSet(LEUART0_RX_PORT,LEUART0_RX_PIN, gpioModeInputPull, 1);			//Initialize the LED0


	LEUART_Init(LEUART0,&my_leuart);
	LEUART0->ROUTE=LEUART_ROUTE_RXPEN|LEUART_ROUTE_TXPEN |LEUART_ROUTE_LOCATION_DEFAULT;
#if LEAURT_LOOPBACK==ON
	LEUART0->CTRL|=LEUART_CTRL_LOOPBK;
#endif
	//Interrupts part
	LEUART0->IFC =0xFFFF ;					//Clear all interrupts
#if LEUART_INTERRUPTS==ON
	LEUART0->IEN =LEUART_IEN_RXDATAV;
#if LEUART_TX_INTERRUPT==ON
	LEUART0->IEN |= LEUART_IEN_TXC ;
#endif

	NVIC_EnableIRQ(LEUART0_IRQn);
#endif

	LEUART_Enable(LEUART0,leuartEnable);
	LEUART0->CMD=LEUART_CMD_TXEN;

	//Disable part
	/*GPIO_PinModeSet(LEUART0_TX_PORT,LEUART0_TX_PIN, gpioModeDisabled, 0);			//Initialize the LED0
	GPIO_PinModeSet(LEUART0_RX_PORT,LEUART0_RX_PIN, gpioModeDisabled, 0);			//Initialize the LED0
	LEUART_Enable(LEUART0,leuartDisable);
	CMU_ClockEnable(cmuClock_LEUART0,false);*/


}

/*******************************************************************************************/
  /*
   * Interrupt handler routines
   */
  /**************************************************************************************/
  /*
   * This is the LEUART0 interrupt handler routine
   * This is an atomic function as it should not be interrupted when being executed
   * Input variables: None required
   * Global Variables: my_buffer, circ_buff_rd
   * Output Variables: None required
   ********************************************************************************/
 void LEUART0_IRQHandler(void)
 {
	//static char i=42;
	static uint8_t leuart_recvd;
	int intFlags=0;
	INT_Disable();								//Make routine atomic
	intFlags=LEUART_IntGet(LEUART0);
	LEUART_IntClear(LEUART0 ,intFlags);			//Clear the interrupts

	INT_Enable();

 }

int main(void)
{
  /* Chip errata */
  CHIP_Init();
  GPIO_LedsInit();
#if PWM_TEST==ON
  CLOCK();
  //LETIMER0_Setup();
  //Turn_on_LED(1);
  TimerSetup();
#endif

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

  #if I2C_FOR_BME280==ON
    s32 output;
    //Check clock if it is capacitive
    Clock_setup(EM0);
    I2C_setup();
    //GPIO_PinModeSet(I2C1_POWER_PORT,I2C1_POWER_PIN, gpioModePushPull, 1);			//Power to the active light sensor
  	//for(int i=0;i<500;i++);		//3ms													//Wait for the warm up
  	//GPIO_PinModeSet(I2C1_INTERRUPT_PORT,I2C1_INTERRUPT_PIN, gpioModeInput, 0);			//Initialize the interrupt pin
  	//TSL2561_Setup();
    GPIO_PinModeSet(I2C1_SCL_PORT,I2C1_SCL_PIN, gpioModeWiredAndPullUp, 1);			//Initialize the LED0
    GPIO_PinModeSet(I2C1_SDA_PORT,I2C1_SDA_PIN, gpioModeWiredAndPullUp, 1);			//Initialize the LED0

  	/*
  	GPIO_PinModeSet(I2C1_SCL_PORT,I2C1_SCL_PIN, gpioModeDisabled, 0);			//Initialize the LED0
  	GPIO_PinModeSet(I2C1_SDA_PORT,I2C1_SDA_PIN, gpioModeDisabled, 0);			//Initialize the LED0
  */

    output= bme280_data_readout_template();
  #endif

#if LEUART_TEST==ON
    Clock_setup(EM0);
    LEUART0_Setup();
    LEUART0->TXDATA='A';
#endif
  /* Infinite loop */
  while (1) {
	  //sleep();
  }
}
