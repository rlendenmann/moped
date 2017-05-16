	.syntax unified
	.extern _bss_start
	.extern _bss_end
	.extern IrqHandler
	.extern IrqHandler_c1
	.extern IrqHandler_c2
	.extern IrqHandler_c3
	.extern main
	.extern mini_uart_send

	.set    MODE_USR, 0x10
	.set    MODE_FIQ, 0x11
	.set    MODE_IRQ, 0x12
	.set    MODE_SVC, 0x13
	.set    MODE_ABT, 0x17
	.set    MODE_HYP, 0x1A
	.set    MODE_UND, 0x1B
	.set    MODE_SYS, 0x1F
	.set    MODE_MASK, 0x1F
	.set    I_BIT, 0x80
	.set    F_BIT, 0x40

	/* MMU definitions */
	.equ    __tt_section_l1_size__, 0x100000
	.equ   TTE_L1_SECTION, 0x2
	.equ   PA_SHIFT_MASK, 20
	.equ   TTE_PA_SHIFT, 2
	.equ   TTE_STRICTLY_ORDERED, 0x00000
	.equ   TTE_BUFFERED, 0x00004
	.equ   TTE_CACHEBLE, 0x00008
	.equ   __gpu_ram_size, 0x04000000
.if (BCM2835 == 1)
	.equ   __ram_size__, 0x20000000
.else
	.equ   __ram_size__, 0x40000000
.endif

	/* stack definitions. TODO: add labels for stack overflow */
	.section .stack
	.align 4
	.globl __stack_top
	.globl __stack_end

	.equ    __und_stack_size__, 0x0010;
	.equ    __abt_stack_size__, 0x0010;
	.equ    __fiq_stack_size__, 0x0100;
	.equ    __irq_stack_size__, 0x2000;
	.equ    __svc_stack_size__, 0x0010;
	.equ    __sys_stack_size__, 0xA000;

	.equ    __arm_ctrl_base__, 0x40000000
	.equ    __core_0_mbox_3_rw__, 0x400000cc

__stack_end:
	.if MULTI_CPU == 1
	/* core 3 */
	.space __und_stack_size__ + __abt_stack_size__ + __fiq_stack_size__ +\
	       __irq_stack_size__ + __svc_stack_size__ + __sys_stack_size__
stack_3_top:

	/* core 2 */
	.space __und_stack_size__ + __abt_stack_size__ + __fiq_stack_size__ +\
	       __irq_stack_size__ + __svc_stack_size__ + __sys_stack_size__
stack_2_top:

	/* core 1 */
	.space __und_stack_size__ + __abt_stack_size__ + __fiq_stack_size__ +\
	       __irq_stack_size__ + __svc_stack_size__ + __sys_stack_size__
stack_1_top:
	.endif

	/* core 0 */
	.space __und_stack_size__ + __abt_stack_size__ + __fiq_stack_size__ +\
	       __irq_stack_size__ + __svc_stack_size__ + __sys_stack_size__
stack_0_top:

	.size __stack_end, . - __stack_end
__stack_top:
	.size __stack_top, . - __stack_top

	/* heap (used by squawk only */
	.section .heap
	.align 4
	.globl __heap_base__
	.globl __heap_end__
	.equ    __heap_size__, 0x10000;

__heap_base__:
	.if __heap_size__
	.space __heap_size__
	.endif
	.size __heap_base__, . - __heap_base__
__heap_end__:
	.size __heap_end__, . - __heap_end__

	/* TODO: For secure world :
    LDR r0, =_start //_start - base address of secure vector table
    MCR p15, 0, r0, c12, c0, 0
For Non-secure World :
    LDR r0, =_ns_start //ns_start - base address of non-secure vector table
    MCR p15, 0, r0, c12, c0, 0 */
	.section .init
	.code 32
	.align 5
	.globl _start
_start:
	/* All the following instruction should be read as: Load the address at symbol into the program counter.*/
	ldr  pc, reset_handler		/* Processor Reset handler -- we will have to force this on the raspi! Because this is the first instruction executed, of cause it causes an immediate branch into reset!*/
	ldr  pc, undefined_handler	/* Undefined instruction handler -- processors that dont have thumb can emulate thumb!*/
	ldr  pc, svc_handler		/* Software interrupt / TRAP (SVC) -- system SVC handler for switching to kernel mode.*/
	ldr  pc, prefetch_handler	/* Prefetch/abort handler.*/
	ldr  pc, data_handler		/* Data abort handler/*/
	nop
	ldr  pc, irq_handler		/* IRQ handler*/
	ldr  pc, fiq_handler		/* Fast interrupt handler.*/

/* Here we create an exception address table! This means that reset/hang/irq can be absolute addresses*/
reset_handler:
	.word ResetHandler
undefined_handler:
	.word UndHandler
svc_handler:
	.word SwiHandler
prefetch_handler:
	.word PrefetchHandler
data_handler:
	.word AbortHandler
	.word 0
irq_handler:
	.word IrqHandler
fiq_handler:
	.word FiqHandler

	.globl io_base_addr
io_base_addr:	.word 0
	.globl cpu_id
cpu_id:	.word 0

	/* hack */
	.globl core_dbg
core_dbg:
	.word 0, 0, 0, 0


	/* TODO: this gives full access to all registers in ns mode! */
nsacr_value:
	.word 0x63fff
	/* arm V7, A53 control register base for irq, mailboxes: 0x40000000 */
osc_value:
	.word 19200000

	.globl arm_ctrl
arm_ctrl:
	.word __arm_ctrl_base__
tt_section_sz:
	.word __tt_section_l1_size__
arm_ram_size:
	.word (__ram_size__ - __gpu_ram_size)
inv_tt_entry:
	.word 0xdeadbeac	// bits 0,1 must be 0

	.code   32
	.balign 4
	.global ResetHandler
ResetHandler:
	/* enable jtag TODO make it cpu id dependent*
	ldr r10, =#0x3f200000
	ldr r11, =#0x006db0c0
	str r11, [r10, #8]

	/* are we in hyp mode? */
	mrs r10, CPSR_all
	and r10, r10, #MODE_MASK
	cmp r10, #MODE_HYP
	bne svc_reset

	/* hyp -> svc mode switch */
	ldr r10, =svc_reset
	ldr r11, =MODE_SVC | I_BIT | F_BIT
	msr elr_hyp, r10
	msr spsr_hyp, r11
	eret

svc_reset:
.if (BCM2836 == 1 || BCM2837 == 1)
	/* only core 0 shall execute detection! */
	mrc  p15, 0, r3, c0, c0, 5
	ands r3, r3, #0x3
	bne  .skip_detect
.endif
	/*=================================================================
	  Auto-Detect Raspberry PI IO base address => sets io_base_addr
	  =================================================================*/
	ldr r4, =#0x61757830	// reset value of miniuart
	ldr r7, =#0x20215010
	ldr r3, [r7]		// Fetch value at 0x20215010 being uart0

	cmp r3, r4		// Check if we have the value the uart0 will be at reset
	bne .not_pi_1

	/* **** Detected Pi at Pi1 address being 0x20000000 ******/
	ldr r7, =io_base_addr
	mov r3, #0x20000000
	str r3, [r7]
	b .autodetect_done;

.not_pi_1:
	ldr r7, =#0x3f215010
	ldr r3, [r7]		// Fetch value at 0x3f215010

	cmp r3, r4		// Check if we have the value the uart0 will be at reset
	beq .pi_found

	/***** Auto-Detected failed it is not safe to do anything but deadloop  ******/
.pi_detect_fail:
	b .pi_detect_fail

	/*@ **** Detected Pi at Pi2/3 address being 0x3f000000 ******/
.pi_found:
	/*; Pi2/3 address 0x3f000000 */
	ldr r7, =io_base_addr
	mov r3, #0x3f000000
	str r3, [r7]

.autodetect_done:
	/*=================================================================
	  Read the Arm Main CPUID register => sets cpu_id
	  =================================================================*/
	ldr r7, =cpu_id
	mrc p15,0, r0,c0,c0,0	// Read Main ID Register
	str r0, [r7]

	mov  r3, #0		// restore cpu id 0
.skip_detect:

.if (BCM2836 == 1 || BCM2837 == 1)
.if (BCM2836 == 1)
	/* ACTLR, set coherent mode in secure state before enabling caches! */
	mrc p15, 0, r0, c1, c0, 1
	orr r0, r0, #(0x1<<6)		// SMP enable
	mcr p15, 0, r0, c1, c0, 1
.else
	mrrc p15, 1, r0, r1, c15	// CPU Extended Control Register
	orr r0, r0, #(1<<6)		// SMP
	mcrr p15, 1, r0, r1, c15	// CPU Extended Control Register
.endif
.endif
	/*====================================================================
	  Setup of table lookup table
	  simple VA == PA mapping
	  single level translation ttbr0 only
	  set tt entries up to max ram size - gpu ram allocated size

	  r4 end address (VA/PA)
	  r5 current VA/PA
	  r6 section size (1MiB)
	  r2 ttb 
	  ====================================================================*/
	// mcr p15, 0, r12, c8, 0, c7, 0 invalidate TLB on rpi 1/arm11
	// tlbiall  r12 a7 and a53 no hypervisor invaldiate tlb
	// tlbiallh  r12 a7, a53 with hypervisor extension
	mov  r5, #0
	mov  r7, #(TTE_BUFFERED, 0x00004 | TTE_CACHEBLE, 0x00008)
	ldr  r6, tt_section_sz
	ldr  r4, arm_ram_size		// ram end
.ram_loop:
	mov  r0, r5
	mov  r1, r7
	cmp  r0, r4
	addlo r5, r5, r6
	bllo mmu_section	// r0 VA, r1 tt section entry flags
	blo  .ram_loop

	/* non accessible ram address space */
	ldr  r1, =inv_tt_entry
	ldr  r4, =io_base_addr
	ldr  r2, =ttb0_all
	lsr  r4, r4, #PA_SHIFT_MASK // shift PA/VA to clear lower PA_SHIFT_MASK bits 
	orr  r4, r2, r4 lsl #TTE_PA_SHIFT
	lsr  r2, r2, r5, #(PA_SHIFT_MASK - TTE_PA_SHIFT) // current mmu l1 table address

.no_acc_loop:
	cmp  r2, r4
	strlo  r1, [r2], #4
	blo  .no_acc_loop

	/*=================================================================
	  Enable L1 Cache
	  TLB ,bit 0, for ARM11 (RPI 1/Zero), bit 23 XP should be set to 1
	  for single level translation only. A7/A53 this a no care.
	  =================================================================*/
	mrc p15, 0, r0, c1, c0, 0	;@ r0 = System Control Register
	orr r0, #0x0004			;@ Data Cache (Bit 2)
	orr r0, #0x0800			;@ Branch Prediction (Bit 11)
	orr r0, #0x1000			;@ Instruction Caches (Bit 12)
	mcr p15, 0, r0, c1, c0, 0	;@ System Control Register = R0

.if BCM2837 == 1
	/* invalidate caching and branch predictions */
	mov r12, #0
	mcr p15, 0, r12, c7, c10, 1
	dsb
	mov r12, #0
	mcr p15, 0, r12, c7, c5, 0
	mov r12, #0
	mcr p15, 0, r12, c7, c5, 6
	dsb
	isb
.endif
	/*=================================================================
	  Bring fpu online
	  =================================================================*/
	/* global acces to fpu for everyone */
	//mrc p15, 0, r0, c1, c1, 2
	//orr r0, r0, #3<<10
	ldr  r0, nsacr_value		// this gives access to evertying in NS mode
	mcr  p15, 0, r0, c1, c1, 2

	/* enable fpu */
	mrc  p15,0,r0,c1,c0, #2			// R0 = Access Control Register
	orr  r0, #(0x300000 + 0xC00000)	// Enable Single & Double Precision
	mcr  p15,0,r0,c1,c0, #2			// Access Control Register = R0
	mov  r0, #0x40000000				// R0 = Enable VFP
	vmsr fpexc,r0			// FPEXC = R0

	/* enable core timer */
	//ldr  r1, osc_value		// osc = 19200000
	//mcr  p15, 0, r1, c14, c0, 0	// write CNTFRQ

.if (BCM2836 == 1 || BCM2837 == 1)
	/* non-secure cpu mode*/
	//mrc p15, 0, r0, c1, c1, 0
	//orr r0, r0, #0x1
	//mcr p15, 0, r0, c1, c1, 0
.endif

.if MULTI_CPU ==1
	// hack
	ldr  r7, =core_dbg
	//mcr  p15, 0, r6, c1, c0, 1
	//mov  r6, 0x1<<31
	//str  r6, [r7, r3, lsl #2]
	// hack end

	/* core 1-3 */
	ldr  r4, =wait_start_cn

	cmp  r3, #1
	ldreq r0, =stack_1_top
	ldreq r5, =_start_c1
	beq  mode_n_init

	cmp  r3, #2
	ldreq r0, =stack_2_top
	ldreq r5, =_start_c2
	beq mode_n_init

	cmp  r3, #3
	ldreq r0, =stack_3_top
	ldreq r5, =_start_c3
	beq mode_n_init
.endif

	/*=================================================================
	  Clear bss
	  =================================================================*/
	mov r0, #0
	ldr r1, =_bss_start
	ldr r2, =_bss_end
bssloop:
	cmp r1, r2
	strlo r0, [r1], #4
	blo bssloop

	/* Setup stacks for core 0 */
	ldr  r0, =stack_0_top
	ldr  r4, =main
	ldr  r5, =_start		// vector table base address

mode_n_init:
	/* Undefined */
	msr cpsr_c, #MODE_UND | I_BIT | F_BIT
	mov sp, r0
	ldr r1, =__und_stack_size__
	sub r0, r0, r1
	/* Abort */
	msr cpsr_c, #MODE_ABT | I_BIT | F_BIT
	mov sp, r0
	ldr r1, =__abt_stack_size__
	sub r0, r0, r1
	/* FIQ */
	msr cpsr_c, #MODE_FIQ | I_BIT | F_BIT
	mov sp, r0
	ldr r1, =__fiq_stack_size__
	sub r0, r0, r1
	/* IRQ */
	msr cpsr_c, #MODE_IRQ | I_BIT | F_BIT
	mov sp, r0
	ldr r1, =__irq_stack_size__
	sub r0, r0, r1
	/* Supervisor */
	msr cpsr_c, #MODE_SVC | I_BIT | F_BIT
	mov sp, r0
	ldr r1, =__svc_stack_size__
	sub r0, r0, r1
	/* System */
	msr cpsr_c, #MODE_SYS | I_BIT | F_BIT
	mov sp, r0

.if BCM2835 == 1
	/* alignement exception behaviour. only readable on v7 or newer architecturs */
	mrc p15, 0, r0, c1, c0, 0
	orr r0, #1 << 22
	mcr p15, 0, r0, c1, c0, 0
.endif

.if (BCM2836 == 1 || BCM2837 == 1)
	/* set exception vector table base address as set in r5 */
	mcr  p15, 0, r5, c12, c0, 0 	// non-secure VBAR
	mcr  p15, 0, r5, c12, c0, 1	// secure MVBAR
.endif
	// hack
	ldr  r6, [r7, r3, lsl #2]
	mrc  p15, 0, r5, c12, c0, 1
	orr  r6, r6, r5
	str  r6, [r7, r3, lsl #2]
	// hack end
	bx  r4
	/* Call application entry point
	mov     r2, #0xDF
	msr     cpsr_c, r2 */

.if DEBUG == 1
	// --> debug toggle gpio 26
	ldr r7, =io_base_addr
	add r7, r7, 0x200000 // gpio device offset
	ldr r2, [r7, #8] // GPFSEL2
	orr r2, r2, #0x01 << 18 // output
	str r2, [r7, #0x8]

	mov r2, #0x1<<26
	str r2, [r7, #0x28] //GPFCLR0, pin 26

toggle_loop:
	str r2, [r7, #0x1c] // GPFSET0 pin 26
	ldr r0, =#0x000fffff
	bl delay_loop
	str r2, [r7, #0x28] //GPFCLR0, pin 26
	ldr r0, =#0x000fffff
	bl delay_loop
	b toggle_loop
	// --> end debug
.endif

	b       main

	.code 32
	.balign 4

	.weak UndHandler
UndHandler:
	mov  r2, #1
	b  _loop

	.weak SwiHandler
SwiHandler:
	mov  r2, #2
	b  _loop

	.weak PrefetchHandler
PrefetchHandler:
	mov  r2, #3
	b  _loop

	.weak AbortHandler
AbortHandler:
	mov  r2, #4
	b  _loop

	.weak FiqHandler
FiqHandler:
	mov  r2, #5
	b  _loop

	.global _unhandled_exception
_unhandled_exception:
	mov  r2, #0
_loop:
	mrc  p15, 0, r3, c0, c0, 5
	ands r3, r3, #0x3
	add  r0, r2, r3
	add  r0, r0, 0x30
	bl  mini_uart_send
	b  _loop

.if (BCM2836 == 1 || BCM2837 == 1)
	/* vector table 32 bit aligned for core 1*/
	.code 32
	.align 5
_start_c1:
	ldr  pc, reset_handler_c1
	ldr  pc, undefined_handler_c1
	ldr  pc, svc_handler_c1
	ldr  pc, prefetch_handler_c1
	ldr  pc, data_handler_c1
	nop
	ldr  pc, irq_handler_c1
	ldr  pc, fiq_handler_c1

reset_handler_c1:
	.word ResetHandler
undefined_handler_c1:
	.word UndHandler
svc_handler_c1:
	.word SwiHandler
prefetch_handler_c1:
	.word PrefetchHandler
data_handler_c1:
	.word AbortHandler
	.word 0
irq_handler_c1:
	.word IrqHandler_c1
fiq_handler_c1:
	.word FiqHandler

	/* vector table 32 bit aligned for core 1*/
	.code 32
	.align 5
_start_c2:
	ldr  pc, reset_handler_c2
	ldr  pc, undefined_handler_c2
	ldr  pc, svc_handler_c2
	ldr  pc, prefetch_handler_c2
	ldr  pc, data_handler_c2
	nop
	ldr  pc, irq_handler_c2
	ldr  pc, fiq_handler_c2

reset_handler_c2:
	.word ResetHandler
undefined_handler_c2:
	.word UndHandler
svc_handler_c2:
	.word SwiHandler
prefetch_handler_c2:
	.word PrefetchHandler
data_handler_c2:
	.word AbortHandler
	.word 0
irq_handler_c2:
	.word IrqHandler_c2
fiq_handler_c2:
	.word FiqHandler

	/* vector table 32 bit aligned for core 1*/
	.code 32
	.align 5
_start_c3:
	ldr  pc, reset_handler_c3
	ldr  pc, undefined_handler_c3
	ldr  pc, svc_handler_c3
	ldr  pc, prefetch_handler_c3
	ldr  pc, data_handler_c3
	nop
	ldr  pc, irq_handler_c3
	ldr  pc, fiq_handler_c3

reset_handler_c3:
	.word ResetHandler
undefined_handler_c3:
	.word UndHandler
svc_handler_c3:
	.word SwiHandler
prefetch_handler_c3:
	.word PrefetchHandler
data_handler_c3:
	.word AbortHandler
	.word 0
irq_handler_c3:
	.word IrqHandler_c3
fiq_handler_c3:
	.word FiqHandler

wait_start_cn:

	// hack
	ldr  r7, =core_dbg
	ldr  r6, [r7, r3, lsl #2]
	//add  r6, r6, #1
	//str  r6, [r7, r3, lsl #2]
	// hack end

	/* read from core n mailbox 3 */
	ldr  r12, =arm_ctrl
	add  r12, #(__core_0_mbox_3_rw__ - __arm_ctrl_base__)
	mov  r11, #0

reset_single_cn:
	wfe
	ldr  r10, [r12, r3, lsl #4]
	// hack
	add  r6, r6, #1
	str  r6, [r7, r3, lsl #2]
	// hack end
	cmp  r10, r11
	beq  reset_single_cn

	/* clear mailbox */
	str  r10, [r12, r3, lsl #4]
	bl main
	b _unhandled_exception
.endif

.if DEBUG == 1
	.code 32
	.type delay_loop, %function
delay_loop:
	subs r0, r0, #1
	isb
	nop
	nop
	nop
	bne delay_loop
	mov pc, lr // return
.endif

	/* set an mmu table entry as section. no address translation simply
	   VA == PA
	   r0: VA address to map
	   r1: flags */
mmu_section:
	/* calculate TTB0 entry address (VA). ARM doc for address table L1 walk */
	ldr  r2, =ttb0_all
	lsr  r0, r0, #PA_SHIFT_MASK // shift PA/VA to clear lower PA_SHIFT_MASK bits 
	orr  r2, r2, r0, lsl #TTE_PA_SHIFT

	/* TTB entry: PA[31:20] | flags | __ttb_section__ */
	orr  r1, r1, #TTE_L1_SECTION
	orr  r0, r1, r0, lsl #PA_SHIFT_MASK
	str  r0, [r2]

	bx  lr

	/* cache line sizes
	   arm11 (PI 1/Zero): I-cache:32 D-cache: 32
	   armV7 A7 (PI 2): i-cache:32 d-cache: 64
	   armV8 A53 (PI 3): i-cache:64 d-cache: 64
	   
	   align mmu ttb0 64 byte for all cpus. as per ARM spec this 16kbytes
	   TODO: better placement, maybe a table per CPU */
	.data
	.align 5
ttb0_all:
	.space __ttb_l1_size__
ttb0_all_end:
