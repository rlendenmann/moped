/*
 * irq.c
 *
 *  Created on:  Mar 4, 2013
 *      Author:  Zhang Shuzhou
 *  Reviewed on:
 *     Reviewer:
 *
 */

#include "internal.h"
#include "task_i.h"
#include "isr.h"
#include "irq_types.h"
#include "led.h"
#include "bcm283x.h"
#include "Uart.h"
#include "arch.h"

static uint32 irq_mask;

#define clz(a) \
 ({ unsigned long __value, __arg = (a); \
     asm ("clz\t%0, %1": "=r" (__value): "r" (__arg)); \
     __value; })

void Irq_SOI( void ) {
	//start of interrupt
	//turns the selected interrupt off
	if (irq_mask == BCM2835_IRQ_ID_TIMER_0) {
		ARM_TIMER_CTL = ARM_TIMER_DISABLE;
	}
}

void Irq_EOI( void ) {
	//clear the flag of the timer interrupt
	if (irq_mask == BCM2835_IRQ_ID_TIMER_0) {
		irq_mask = 0;
		ARM_TIMER_CLI = 0;
		ARM_TIMER_CTL = ARM_TIMER_ENABLE;
	} else if (irq_mask == BCM2835_IRQ_ID_USB){
		irq_mask = 0;
		//IRQ_ENABLE1 = 1;
	}

}


//int DisableInterrupts(void) {
//	irqDisable();
//	return 0;
//}

void Irq_Init(void)
{
#if (OS_NUM_CORES > 1)
	volatile struct bcm2836_arm_ctrl_reg *arm_ctrl = RPI_ARM_CTRL_BASE;
	CoreIDType core_id = Os_ArchCoreId();

	/* enable default IOC mbox IRQ to current core */
	///arm_ctrl->CORE_MBOX_IRQ_CTRL[core_id] = (0x1 << RPI_ARM_IOC_MBOX);
#endif
}

void *Irq_Entry(void *stack_p)
{
	uint32 *stack;
#ifdef CFG_ARM_V7 // RPI 2/3
	uint32 stat;
	volatile struct bcm2836_arm_ctrl_reg *arm_ctrl = RPI_ARM_CTRL_BASE;
#endif
	volatile struct bcm283x_irq_reg *irq = RPI_ARM_IRQ_BASE;

	stack = stack_p;

#ifdef CFG_ARM_V7
	stat = arm_ctrl->CORE_IRQ_PENDING[0];

	if (stat & RPI_ARM_CORE_MBOX0_IRQ) {
		uint32_t rw_clr;

		/* clear ioc mailbox */
		rw_clr = arm_ctrl->CORE_MBOX_READ_WRITE_CLR[0][0];
		arm_ctrl->CORE_MBOX_READ_WRITE_CLR[0][0] = rw_clr;
	} else if (stat & RPI_ARM_CORE_GPU_IRQ) {
		/* peripheral (==gpu) interrupt */
		/* pi_print(stat, 0);
		pi_print(irq->IRQ_BASIC_PEND, 1); */
#endif
	/*if (irq->IRQ_BASIC_PEND & BIT(9)) {
		pi_print(irq->IRQ_PEND[1], 1);
	} */

	if (irq->IRQ_BASIC_PEND & ARM_TIMER_IRQ) { // Note how we mask out the GPU interrupt Aliases. IRQ_BASIC & ARM_TIMER_IRQ
		irq_mask = BCM2835_IRQ_ID_TIMER_0;
		stack = Os_Isr(stack, BCM2835_IRQ_ID_TIMER_0);

		/* TODO: continue handling of the pending 1 & 2 interrupts */
	} else if (irq->IRQ_PEND[0] & USB_IRQ) {
		irq_mask = BCM2835_IRQ_ID_USB;
		stack = Os_Isr(stack, BCM2835_IRQ_ID_USB);
	} else if (irq->IRQ_PEND[0] & AUX_IRQ) {
		stack = Os_Isr(stack, BCM2835_IRQ_ID_AUX_UART);
	} else if (irq->IRQ_PEND[1] & I2C_IRQ) {
		irq_mask = BCM2835_IRQ_ID_I2C;
		stack = Os_Isr(stack, BCM2835_IRQ_ID_I2C);
	} else if (irq->IRQ_PEND[1] & GPIO_IRQ0) {
		stack = Os_Isr(stack, BCM2835_IRQ_ID_GPIO_0);
	} else if (irq->IRQ_PEND[1] & SPI_IRQ) {
		stack = Os_Isr(stack, BCM2835_IRQ_ID_SPI);
	}

#ifdef CFG_ARM_V7
	}
#endif
	return stack;
}

#ifdef MULTI_CPU
void *Irq_Entry_c1(void *stack_p)
{
	uint32 *stack;
	uint32 stat;
	volatile struct bcm2836_arm_ctrl_reg *arm_ctrl = RPI_ARM_CTRL_BASE;

	stack = stack_p;

	stat = arm_ctrl->CORE_IRQ_PENDING[1];

	if (stat & RPI_ARM_CORE_MBOX0_IRQ) {
		uint32_t rw_clr;

		/* clear ioc mailbox */
		rw_clr = arm_ctrl->CORE_MBOX_READ_WRITE_CLR[1][0];
		arm_ctrl->CORE_MBOX_READ_WRITE_CLR[1][0] = rw_clr;
	} else {
		assert(0);
	}
	return stack;
}

void *Irq_Entry_c2(void *stack_p)
{
	uint32 *stack;
	uint32 stat;
	volatile struct bcm2836_arm_ctrl_reg *arm_ctrl = RPI_ARM_CTRL_BASE;

	stack = stack_p;

	stat = arm_ctrl->CORE_IRQ_PENDING[2];

	if (stat & RPI_ARM_CORE_MBOX0_IRQ) {
		uint32_t rw_clr;

#if 0
		Os_ArchGetSpinlock(&qLock[core_id]);
		wr = MsgBoxRw[coreId].wr;
		/* simple full check */
		if (OsMessageBoxQ[coreId][wr].result != E_OK) {
			Os_ArchReleaseSpinlock(&qLock[coreId]);
			return E_NOT_OK;
		}
		if ((wr + 1) < MBOX_Q_LEN) {
			MsgBoxRw[coreId].wr = wr + 1;
		} else {
			MsgBoxRw[coreId].wr = 0;
		}
		OsMessageBoxQ[coreId][wr].op = op;
		OsMessageBoxQ[coreId][wr].arg1 = arg1;
		OsMessageBoxQ[coreId][wr].arg2 = arg2;
		OsMessageBoxQ[coreId][wr].arg3 = arg3;
		OsMessageBoxQ[coreId][wr].opFinished = false;
		OsMessageBoxQ[coreId][wr].result = E_NOT_OK;
		Os_ArchReleaseSpinlock(&qLock[coreId]);
#endif
		/* clear ioc mailbox */
		rw_clr = arm_ctrl->CORE_MBOX_READ_WRITE_CLR[2][0];
		arm_ctrl->CORE_MBOX_READ_WRITE_CLR[2][0] = rw_clr;
	} else {
		assert(0);
	}
	return stack;
}

void *Irq_Entry_c3(void *stack_p)
{
	uint32 *stack;
	uint32 stat;
	volatile struct bcm2836_arm_ctrl_reg *arm_ctrl = RPI_ARM_CTRL_BASE;

	stack = stack_p;

	stat = arm_ctrl->CORE_IRQ_PENDING[3];

	if (stat & RPI_ARM_CORE_MBOX0_IRQ) {
		uint32_t rw_clr;

		/* clear ioc mailbox */
		rw_clr = arm_ctrl->CORE_MBOX_READ_WRITE_CLR[3][0];
		arm_ctrl->CORE_MBOX_READ_WRITE_CLR[3][0] = rw_clr;
	} else {
		assert(0);
	}
	return stack;
}
#endif

void Irq_EnableVector( int16_t vector, int priority, int core ) {
}

/**
 * NVIC prio have priority 0-31, 0-highest priority.
 * Autosar does it the other way around, 0-Lowest priority
 * NOTE: prio 255 is reserved for SVC and PendSV
 *
 * Autosar    NVIC
 *   31        0
 *   30        1
 *   ..
 *   0         31
 * @param prio
 * @return
 */
static inline int osPrioToCpuPio( uint8_t prio ) {
	assert(prio<32);
	prio = 31 - prio;
	return 0;
}

/**
 * Generates a soft interrupt, ie sets pending bit.
 * This could also be implemented using ISPR regs.
 *
 * @param vector
 */
void Irq_GenerateSoftInt( IrqType vector ) {


}

/**
 * Get the current priority from the interrupt controller.
 * @param cpu
 * @return
 */
uint8_t Irq_GetCurrentPriority( Cpu_t cpu) {

	 //uint8_t prio = 0;

	// SCB_ICSR contains the active vector
	return 0;
}




