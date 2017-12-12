/*
 * UltrasonicSensor.c
 *
 *  Created on: 8 sep 2013
 *      Author: zsz
 *
 * Driver for the ultrasonic sensor (Ultrasonic Ranging Module HC - SR04).
 * The sensor works by sending out a burst of 8 sonic signals and wait
 * for its reflection. The reflected signals are spread out in function of
 * the distance to the object in front of the sensor. This speed is then
 * used to calculate the distance to the object.
 *
 * The whole chain of events is activated by a short trigger signal
 * (around 10 microseconds) on the sensors incoming pin connection.
 *
 */

#include "bcm283x.h"
#include "Sensors.h"

/* Records whether this module has already been initialized */
static boolean ultra_init;

/**
 * Initialize the GPIO pins connected to the ultrasonic sensor
 */
void UltrasonicSensor_Init(void)
{
	/* Select functionality for the pins connected to the ultrasonic sensor */
	bcm2835_GpioFnSel(GPIO_ULTRA_TRIG, GPFN_OUT);  		// output pin, for triggering the sensor
	bcm2835_GpioFnSel(GPIO_ULTRA_ECHO, GPFN_IN);   		// input pin, for listening to echo signals from the sensor

	bcm2835_ClearGpioPin(GPIO_ULTRA_TRIG);			// Don't trigger just yet (set trigger output to low)

	bcm2835_Sleep(300);
	ultra_init = true;
}

/*
 * Measure the distance to an object in front of the ultrasonic sensor
 *
 * Send out a trigger signal for a short period of time. Then listen for the
 * echo, and depending on the duration of echo period, calculate the distance
 * to an object in front of the sensor.
 *
 * @return distance			----- distance to the nearest object in front of the sensor (in cm)
 */
uint32 UltrasonicSensor_Read(void)
{
	uint64 startTime;
	uint32 travelTime, distance;

	/* Initialize, if not already done */
	if (ultra_init == false){
		UltrasonicSensor_Init();
	}

	/* Send out a trigger signal and let it sound for around 10 microseconds */
	bcm2835_SetGpioPin(GPIO_ULTRA_TRIG);
	bcm2835_Sleep(10);
	bcm2835_ClearGpioPin(GPIO_ULTRA_TRIG);

	/* Wait for the echo start and record its time */
	int count1 = 0;
	/* Very occasionally, it doesn't become HIGH, maybe because we
	   were interrupted and missed it. Make sure we don't get stuck in
	   here. */
	while (bcm2835_ReadGpioPin(GPLEV, GPIO_ULTRA_ECHO) == LOW) {
	  count1++;
	  if (count1 > 10000)
	    return 0;
	}
	startTime = CURRENT_TIME;

	/* Wait for the echo end and record the time of the echo period */
	int count = 0;
	while (bcm2835_ReadGpioPin(GPLEV, GPIO_ULTRA_ECHO) == HIGH) {
		count++;
		// about 3000 us/m, so the maximum range 4m = 12000 us, so let's
		// break when > 20000 us.
		// One loop here takes about 1 us.
		if (count > 20000) {
			break;
		}
	}
	if (count > 20000) {
		travelTime = 200000;
	} else {
		travelTime = (uint32)(CURRENT_TIME - startTime);
	}
	//printf("ultra count = %d time = %d\r\n", count, travelTime);

	/* Calc the distance in cm (according to sensors datasheet) and return */
	distance = travelTime * 0.017;
	return distance;
}
