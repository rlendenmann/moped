/*
 * arch.c
 *
 *  Created on:  Mar 4, 2013
 *      Author:  Zhang Shuzhou
 *  Reviewed on:
 *     Reviewer:
 *
 */
#include "internal.h"
#include "Cpu.h"
#include "sys.h"
#include "arch_stack.h"
#include "bcm283x.h"
#include "stdio.h"

volatile uint8_t numCoresRdy = 0;

void Os_ArchFirstCall( void )
{
	Irq_Enable();
	Os_Sys[GetCoreID()].currTaskPtr->constPtr->entry();
}

void *Os_ArchGetStackPtr(void)
{

	void *x;

	asm volatile ("mov  %0, sp": "=r" (x));

	return x;
}

unsigned int Os_ArchGetScSize( void ) {

	return SC_SIZE;
}

void Os_ArchSetTaskEntry(OsTaskVarType *pcbPtr ) {
	uint32_t *context = (uint32_t *)pcbPtr->stack.curr;

	context[C_CONTEXT_OFFS/4] = SC_PATTERN;

		/* Set LR to start function */
	if( pcbPtr->constPtr->proc_type == PROC_EXTENDED ) {
			context[VGPR_LR_OFF/4] = (uint32_t)Os_TaskStartExtended;
	} else if( pcbPtr->constPtr->proc_type == PROC_BASIC ) {
			context[VGPR_LR_OFF/4] = (uint32_t)Os_TaskStartBasic;
	}
}

void Os_ArchSetupContext( OsTaskVarType *pcb ) {


}

// only called 1 time for first core
void Os_ArchInit( void )
{
	numCoresRdy = 1;	// we have at least 1 core
}

void Os_ArchPanic(void)
{
	while(1) {
		printf("Os_ArchPanic(arm_v7)\n");
	};
}

void Os_ArchTest(void *stack_p){
//	uint32 *stack;
//	stack = (uint32 *)stack_p;
}

/* for debugging */
void *Os_ArchGetCurrentPC(void)
{
	void *p;

	__asm__ __volatile__(
"	mov	%[result], lr\n"
"	sub	%[result], #8"
	: [result] "=r" (p)
	:
	: "cc");

	return p;
}


#if OS_SPINLOCK_CNT != 0
void Os_ArchGetSpinlock(volatile unsigned int *lock)
{
	unsigned long tmp;

	// "	wfene\n"
	__asm__ __volatile__(
"1:	ldrex	%[result], [%[value]]\n"
"	teq	%[result], #0\n"

"	strexeq	%[result], %[flag], [%[value]]\n"
"	teqeq	%[result], #0\n"
"	bne	1b\n"
	: [result] "=&r" (tmp)
	: [value] "r" (lock), [flag] "r" (1)
	: "cc");

	smp_mb();
}

TryToGetSpinlockType Os_ArchTryToGetSpinlock(volatile unsigned int *lock)
{
	unsigned long tmp;

	__asm__ __volatile__(
"	ldrex	%[result], [%[value]]\n"
	: [result] "=&r" (tmp)
	: [value] "r" (lock)
	: "cc");

	return tmp;
}

void Os_ArchReleaseSpinlock(volatile unsigned int *lock)
{
	smp_mb();

	__asm__ __volatile__(
"	str	%[flag], [%[value]]\n"
	:
	: [value] "r" (lock), [flag] "r" (0)
	: "cc");

	dsb_sev();
}
#endif

#if (OS_NUM_CORES > 1)
CoreIDType Os_ArchCoreId(void)
{
	int id;

	__asm__ (
"	mrc p15, 0, %[result], c0, c0, 5\n"
"	and %[result], %[result], #0x3\n"
	: [result] "=&r" (id)
	:
	: "cc");

	return id;
}

boolean Os_StartCore(CoreIDType id)
{
	struct bcm2836_arm_ctrl_reg * arm_ctrl = RPI_ARM_CTRL_BASE;
	CoreIDType core_id = Os_ArchCoreId();

	if ((id >= OS_NUM_CORES) || (id == OS_CORE_ID_MASTER)) {
		return false;
	}

	numCoresRdy++;

	/* use mailbox 3 for start */
	//arm_ctrl->CORE_MBOX_WRITE_SET[id][3] = 0x1 << core_id;

	//sev();

	return true;
}

StatusType Os_ArchNotifyCore(CoreIDType coreId)
{
	volatile struct bcm2836_arm_ctrl_reg *arm_ctrl = RPI_ARM_CTRL_BASE;
	CoreIDType core_id = Os_ArchCoreId();

	if (unlikely(coreId >= OS_NUM_CORES)) {
		return E_NOT_OK;
	}

	arm_ctrl->CORE_MBOX_WRITE_SET[coreId][RPI_ARM_IOC_MBOX] = (0x1 << core_id);

	return E_OK;
}

void Os_ArchCoreNotificationInit(void)
{
	volatile struct bcm2836_arm_ctrl_reg *arm_ctrl = RPI_ARM_CTRL_BASE;
	CoreIDType core_id = Os_ArchCoreId();
	uint32_t rw_clr;

	/* clear ioc mailbox */
	rw_clr = arm_ctrl->CORE_MBOX_READ_WRITE_CLR[core_id][RPI_ARM_IOC_MBOX];
	arm_ctrl->CORE_MBOX_READ_WRITE_CLR[core_id][RPI_ARM_IOC_MBOX] = rw_clr;
}
#endif
