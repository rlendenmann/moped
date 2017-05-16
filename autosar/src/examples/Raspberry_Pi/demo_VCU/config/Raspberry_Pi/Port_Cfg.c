/*
* Configuration of module: Port (Port_Cfg.c)
*
* Created by:              
* Copyright:               
*
* Configured for (MCU):    bcm2835
*
* Module vendor:           SICS
* Generator version:       null
*
*/

	
#include "Port.h"
#include "bcm283x.h"

#define GPIO_REG(reg) BCM283X_PERIPH_ARM_BASE(struct bcm283x_gpio_reg, RPI_ARM_GPIO_OFFSET)->reg

const Port_ConfigType PortConfigData = {
	.gipo_select0 = &(GPIO_REG(GPSET[0])),
	.gipo_clear0  = &(GPIO_REG(GPCLR[0])),
	.gipo_level0  = &(GPIO_REG(GPLEV[0])),
	.gipo_pudclk0 = &(GPIO_REG(GPPUDCLK[0])),

	.gipo_select1 = &(GPIO_REG(GPSET[1])),
	.gipo_clear1  = &(GPIO_REG(GPCLR[1])),
	.gipo_level1  = &(GPIO_REG(GPLEV[1])),
	.gipo_pudclk1 = &(GPIO_REG(GPPUDCLK[1])),
	.gipo_pin_base = 32,
};
