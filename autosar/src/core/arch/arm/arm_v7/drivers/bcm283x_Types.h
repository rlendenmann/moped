/*
 * bcm283x_Types.h
 *
 * Created on: Mar 31, 2013
 * Author: Zhang Shuzhou
 */

#ifndef BCM283X_TYPES_H_
#define BCM283X_TYPES_H_

#include "Std_Types.h"

typedef struct {
	/**
	 * @brief   GPIO_LATCH register.
	 * @details This register represents the output latch of the GPIO port.
	 */
	uint32_t          latch;
	volatile uint32_t *gpset;
	volatile uint32_t *gpclr;
	volatile uint32_t *gplev;
	volatile uint32_t *gppudclk;
	unsigned int      pin_base;
} gpio_port_t;

/**
 * @brief   First I/O port identifier.
 * @details Low level drivers can define multiple ports, it is suggested to
 *          use this naming convention.
 */
extern gpio_port_t IOPORT0;
extern gpio_port_t IOPORT1;

typedef uint32_t ioportmask_t;
typedef gpio_port_t *ioportid_t;

/* crt0.S auto-decect values */
extern const uint32_t *io_base_addr;
extern const uint32_t cpu_id;

#ifndef __IO
#define __IO volatile
#endif
#ifndef __I
#define __I volatile const
#endif
#ifndef __O
#define __O volatile
#endif

/* GPIO registers at 0x7e200000 (physical arm address rpi1:0x20nnnnnn or rpi2/3:0x3fnnnnnn) */
struct bcm283x_gpio_reg {
	__IO uint32_t GPFSEL[6]; // GPIO Function Select 0-5 32 R/W
	__IO uint32_t reserved0;
	__IO uint32_t GPSET[2]; // GPIO Pin Output Set 0-1 32 W
	__IO uint32_t reserved1;
	__IO uint32_t GPCLR[2]; // GPIO Pin Output Clear 0-1 32 W
	__IO uint32_t reserved2;
	__IO uint32_t GPLEV[2]; // GPIO Pin Level 0-1 32 R
	__IO uint32_t reserved3;
	__IO uint32_t GPEDS[2]; // GPIO Pin Event Detect Status 0-1 32 R/W
	__IO uint32_t reserved4;
	__IO uint32_t GPREN[2]; // GPIO Pin Rising Edge Detect Enable 0-1 32 R/W
	__IO uint32_t reserved5;
	__IO uint32_t GPFEN[2]; // GPIO Pin Falling Edge Detect Enable 0-1 32 R/W
	__IO uint32_t reserved6;
	__IO uint32_t GPHEN[2]; // GPIO Pin High Detect Enable 0-1 32 R/W
	__IO uint32_t reserved7;
	__IO uint32_t GPLEN[2]; // GPIO Pin Low Detect Enable 0-1 32 R/W
	__IO uint32_t reserved8;
	__IO uint32_t GPAREN[2]; // GPIO Pin Async. Rising Edge Detect 0-1 32 R/W
	__IO uint32_t reserved9;
	__IO uint32_t GPAFEN[2]; // GPIO Pin Async. Falling Edge Detect 0-1 32 R/W
	__IO uint32_t reserved10;
	__IO uint32_t GPPUD; // GPIO Pin Pull-up/down Enable 32 R/W
	__IO uint32_t GPPUDCLK[2]; // GPIO Pin Pull-up/down Enable Clock 0-1 32 R/W
	__IO uint32_t reserved11;
	__IO uint32_t test; // Test 4 R/W
};

/* AUX registers at 0x7e215000 (physical arm address rpi1:0x202150nn or rpi2/3:0x3f2150nn) */
struct bcm283x_aux_reg {
	__I uint32_t AUX_IRQ; // Auxiliary Interrupt status 3
	__IO uint32_t AUX_ENABLES; // Auxiliary enables 3
	__I uint32_t AUX_PADDING0[14]; // 56 bytes
	__IO uint32_t AUX_MU_IO_REG; // Mini Uart I/O Data 8 0x7e215040
	__IO uint32_t AUX_MU_IER_REG; // Mini Uart Interrupt Enable 8
	__IO uint32_t AUX_MU_IIR_REG; // Mini Uart Interrupt Identify 8
	__IO uint32_t AUX_MU_LCR_REG; // Mini Uart Line Control 8
	__IO uint32_t AUX_MU_MCR_REG; // Mini Uart Modem Control 8
	__IO uint32_t AUX_MU_LSR_REG; // Mini Uart Line Status 8
	__IO uint32_t AUX_MU_MSR_REG; // Mini Uart Modem Status 8
	__IO uint32_t AUX_MU_SCRATCH; // Mini Uart Scratch 8
	__IO uint32_t AUX_MU_CNTL_REG; // Mini Uart Extra Control 8
	__IO uint32_t AUX_MU_STAT_REG; // Mini Uart Extra Status 32
	__IO uint32_t AUX_MU_BAUD_REG; // Mini Uart Baudrate 16
	__I uint32_t AUX_PADDING1[5]; // 20 bytes
	__IO uint32_t AUX_SPI0_CNTL0_REG; // SPI 1 Control register 0 32 0x7e21 5080
	__IO uint32_t AUX_SPI0_CNTL1_REG; // SPI 1 Control register 1 8
	__IO uint32_t AUX_SPI0_STAT_REG; // SPI 1 Status 32
	__I uint32_t AUX_PADDING2;
	__IO uint32_t AUX_SPI0_IO_REG; // SPI 1 Data 32
	__IO uint32_t AUX_SPI0_PEEK_REG; // SPI 1 Peek 16
	__I uint32_t AUX_PADDING3[10]; // 40 bytes
	__IO uint32_t AUX_SPI1_CNTL0_REG; // SPI 2 Control register 0 32 0x7e21 50c0
	__IO uint32_t AUX_SPI1_CNTL1_REG; // SPI 2 Control register 1 8
	__IO uint32_t AUX_SPI1_STAT_REG; // SPI 2 Status 32
	__I uint32_t AUX_PADDING4;
	__IO uint32_t AUX_SPI1_IO_REG; // SPI 2 Data 32 0x7e21 50d0
	__IO uint32_t AUX_SPI1_PEEK_REG; // SPI 2 Peek 16
};

struct bcm283x_irq_reg {
	__I uint32_t IRQ_BASIC_PEND;
	__I uint32_t IRQ_PEND[2];
	__IO uint32_t IRQ_FIQ_CONTROL;
	__IO uint32_t IRQ_ENABLE[2];
	__IO uint32_t IRQ_ENABLE_BASIC;
	__IO uint32_t IRQ_DISABLE[2];
	__IO uint32_t IRQ_DISABLE_BASIC;
};

struct bcm2836_arm_ctrl_reg {
	__IO uint32_t CONTROL; // Control register
	__I uint32_t UNUSED0; // <unused>
	__IO uint32_t CORE_TIMER_PRESCALER; // Core timer prescaler
	__IO uint32_t GPU_IRQ_ROUTING; // GPU interrupts routing (peripheral irq controller)
	__IO uint32_t PMU_IRQ_ROUTING_SET; // Performance Monitor Interrupts routing-set
	__IO uint32_t PMU_IRQ_ROUTING_CLR; // Performance Monitor Interrupts routing-clear
	__I uint32_t UNUSED1; // <unused>
	__I uint32_t CORE_TIMER_LS; // Core timer access LS 32 bits
	__I uint32_t CORE_TIMER_MS; // Core timer access MS 32 bits
	__IO uint32_t LOCAL_IRQ[2]; // Local Interrupt 0 [1-7] routing, Local Interrupts 8-15 routing UNUSED!!
	__I uint32_t AXI_OUTSTANDING_COUNTERS; // Axi outstanding counters
	__IO uint32_t AXI_OUTSTANDING_IRQ; // Axi outstanding IRQ
	__IO uint32_t LOCAL_TIMER_CTRL_STATUS; // Local timer control & status
	__O uint32_t LOCAL_TIMER_IRQ_CLR_RELOAD; // Local timer write flags
	__I uint32_t UNUSED2; // <unused>
	__IO uint32_t CORE_TIMER_IRQ_CTRL[4]; // Core[0-3] timers Interrupt control
	__IO uint32_t CORE_MBOX_IRQ_CTRL[4]; // Core[0-3] Mailboxes Interrupt control
	__I uint32_t CORE_IRQ_PENDING[4]; // Core[0-3] IRQ Source
	__I uint32_t CORE_FIQ_PENDING[4]; // Core[0-3] FIQ Source
	__O uint32_t CORE_MBOX_WRITE_SET[4][4]; // Core [0-3] Mailbox [0-3] write-set (WO)
	__IO uint32_t CORE_MBOX_READ_WRITE_CLR[4][4]; // Core [0-3] Mailbox [0-3] read & write-high-to-clear
};

#endif /* BCM283X_TYPES_H_ */
