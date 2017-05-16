/*
 * bcm283x_Types.h
 *
 *  Created on: Mar 31, 2013
 *      Author: Zhang Shuzhou
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
	unsigned int pin_base;
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

/* crt0.S auto-detect values */
extern volatile uint32_t *io_base_addr;
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

#endif /* BCM283X_TYPES_H_ */
