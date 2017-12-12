/*
 * sys_tick.h
 *
 *  Created on:  Mar 4, 2013
 *      Author:  Zhang Shuzhou
 *  Reviewed on:
 *     Reviewer:
 *
 */

#include "Os.h"
#include "internal.h"
#include "isr.h"
#include "arc.h"
#include "counter_i.h"
#include "bcm283x.h"
#include "irq_types.h"
#include "led.h"
#include "Uart.h"

boolean led_flag = true;
uint32 led_count = 0;

uint32 led_tick_period = 500;

uint32 led_pattern0 = 0x55;
uint32 led_pattern = 0;

// If RPI_B_PLUS is 0, we have an old RPi B

#undef RPI_B_PLUS
#define RPI_B_PLUS 2

#if RPI_B_PLUS==1
#define LED_BIT 47
#define GPIO_SEL_OUTPUT 1
#define LEDBIT1 0x00008000
#define LEDPORT IOPORT1
#define LIGHT_LED true

#elif RPI_B_PLUS==2
#define LED_BIT 12
#define GPIO_SEL_OUTPUT 1
#define LEDBIT1 0x00001000
#define LEDPORT IOPORT0
#define LIGHT_LED true

#elif defined(RPI_B_PLUS) && RPI_B_PLUS==0
#define LED_BIT 16
#define GPIO_SEL_OUTPUT 0
#define LEDBIT1 0x00010000
#define LEDPORT IOPORT0
#define LIGHT_LED false
#else
#error "RPI_B_PLUS must be 0 or 1 or 2"
#endif

uint32 act_led_gpio = LED_BIT;

/*
 *to proof the code is running by using the
 *led blink 500ms/once
 */
static void led_proof(void)
{
	if (led_pattern == 0) {
		led_pattern = led_pattern0;
	}

	led_tick_period = 1000/16*(led_pattern&0xf);

	led_count++;

	if (led_count / led_tick_period == 1) {
		if ((led_flag == LIGHT_LED)) {// turn on
			*((&LEDPORT)->gpset) = LEDBIT1;
			led_flag = !LIGHT_LED;
			led_count = 0;
		} else { // turn off
			*((&LEDPORT)->gpclr) = LEDBIT1;
			led_flag = LIGHT_LED;
			led_count = 0;
		}
		led_pattern >>= 4;
	}
}

static uint32 tick;
static void Bcm2835OsTick(void)
{
	OsTick();
	led_proof();

	if ((tick % 10000) == 0) {
		pi_print(tick, 1);
	}
	tick++;
}

/**
 * Init of free running timer.
 */
void Os_SysTickInit( void )
{
	/* necessary when LED_BIT = 47, but wrong when LED_BIT = 16
	   bcm2835_GpioFnSel(DIO_BCM2835_LED_CHANNEL, GPIO_SEL_OUTPUT); */
#if (RPI_B_PLUS==1) || (RPI_B_PLUS==2)
	bcm2835_GpioFnSel(LED_BIT, GPIO_SEL_OUTPUT);
#endif
	ISR_INSTALL_ISR2("OsTick", Bcm2835OsTick,
			 BCM2835_IRQ_ID_TIMER_0/*BCM2835_IRQ_ID_SYSTEM_TIMER3*/,
			 6,0);
}

/**
 * Start the Sys Tick timer
 *
 * @param period_ticks How long the period in timer ticks should be.
 *
 */

void Os_SysTickStart(uint32_t period_ticks)
{
	volatile struct bcm283x_irq_reg *irq = RPI_ARM_IRQ_BASE;

	ARM_TIMER_CTL = ARM_TIMER_DISABLE;
	ARM_TIMER_LOD = 1000-1;
	ARM_TIMER_RLD = 1000-1;     //should be 1000, then get 1us per once, the system tick is 1ms
	ARM_TIMER_DIV = 0x000000F9;  //source 250Mhz, divide 250, get 1Mhz
	ARM_TIMER_CLI = 0;
	ARM_TIMER_CTL = ARM_TIMER_ENABLE;  //23-bit counter, enable

	irq->IRQ_ENABLE_BASIC = ARM_TIMER_IRQ;
//	    IRQ_ENABLE1 = SYSTIMER_IRQEN3;
//	    clkupdate(SYSTIMER_CLOCK_FREQ / CLKTICKS_PER_SEC);
}


TickType Os_SysTickGetValue( void )
{
	uint32 SysTick;

	SysTick = ARM_TIMER_LOD - ARM_TIMER_VAL;

	return SysTick;

}


TickType Os_SysTickGetElapsedValue(uint32 preValue )
{
	uint32 curr;
	uint32 max;

	curr = ARM_TIMER_VAL;
	max  = ARM_TIMER_LOD;
	return Os_CounterDiff((max - curr),preValue,max);
}
