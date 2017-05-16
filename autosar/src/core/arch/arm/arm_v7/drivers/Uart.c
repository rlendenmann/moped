/*
 * Uart.c
 *
 *  Created on: May 11, 2013
 *      Author: Zhang Shuzhou
 */
#include <stdio.h>
#include <stdlib.h>
#include "led.h"
#include "Std_Types.h"
#include "bcm283x.h"
#include "Uart.h"
#include "isr.h"
#include "irq_types.h"
#include "Os.h"
#ifdef CFG_ARM_V7 // RPI 2/3
#include "spinlock_i.h"
#endif

#ifndef MU_BAUD_RATE
#define MU_BAUD_RATE 921600
#endif

#ifdef CFG_ARM_V7 // RPI 2/3
static volatile int ulock;  	// by default 0, unlocked 
extern volatile uint8_t numCoresRdy; // ignore spinlock in only 1 core
#endif

//if define the interrupt of uart, should be
static boolean UART_FLAG = false;
boolean SPEEDEVENT = false;
boolean SERVOEVENT = false;

int result_channel = 2;
int result_value = 0;

char* s = NULL;
char channel[4];
char value[4];
int isLegalReceive = false;
int totalValidDataSize = 4 * 2;
int index = 0;
int channel_index = 0;
int value_index = 0;
char tmp = '0';

//static void Uart_Data_Handler(uint8 data){
//
//    pi_printf("uart data handler"); mini_uart_sendDec(data);pi_printf("\r\n");
//	tmp = data;
//	if (isLegalReceive == false) {
//		if (tmp == '^') {
//			isLegalReceive = true;
//			index = 0;
//			channel_index = 0;
//			value_index = 0;
//			pi_printf("input ^");
//		}
//	} else {
//
//		if (index < sizeof(int)) {
//			channel[channel_index] = tmp;
//			channel_index++;
//			pi_printf("channel_index\r\n");
//		} else if (index < totalValidDataSize && index >= sizeof(int)) {
//			value[value_index] = tmp;
//			value_index++;
//			pi_printf("value_index\r\n");
//		} else {
//			if (tmp == '$') {
//				tmp = '0';
//				isLegalReceive = false;
//
//				result_channel = Chars_To_Int(channel);
//				pi_printf("result_channel"); mini_uart_sendDec(result_channel);pi_printf("\r\n");
//				result_value = Chars_To_Int(value);
//				pi_printf("result_value"); mini_uart_sendDec(result_value);pi_printf("\r\n");
//				if (result_channel == SPEEDCH) {
//					SPEEDEVENT = true;
//					SERVOEVENT = false;
//					pi_printf("Activate speed\r\n");
//					ActivateTask(TASK_ID_PirteTask);       // ^^00001234$
//				}
//				if (result_channel == SERVOCH) {
//					SERVOEVENT = true;
//					SPEEDEVENT = false;
//					pi_printf("Activate servo\r\n");
//					ActivateTask(TASK_ID_PirteTask);
//				}
//			}
//		}
//		index++;
//	} // end if
//}

//static void Uart_Isr(void) {
////	if (AUX_MU_IIR_RX_IRQ) {
////		while (!AUX_MU_LSR_RX_RDY);
////		do {
////			Uart_Data_Handler(AUX_MU_IO_REG & 0xFF);
////		} while (AUX_MU_LSR_RX_RDY);
////
////	}
////	if (AUX_MU_IIR_TX_IRQ) {
////		while (!AUX_MU_LSR_TX_RDY)
////			;
////		uint32_t data = UART_TX[0];
////		if (data < 1) {
////			/* Disable tx interrupts.*/
////			AUX_MU_IER_REG &= ~AUX_MU_IER_TX_IRQEN;
////		} else {
////			mini_uart_send((uint32_t) data);
////			UART_TX[0] = 0;
////		}
////	}
//}

void Uart_Init(void)
{
	volatile struct bcm283x_irq_reg *irq = RPI_ARM_IRQ_BASE;
	volatile struct bcm283x_aux_reg *aux = RPI_ARM_AUX_BASE;

	/* disable aux interrupt line */
	irq->IRQ_DISABLE[BCM2835_IRQ_ID_AUX_UART /32] = BIT(BCM2835_IRQ_ID_AUX_UART % 32);
	dmb();

	/* gpio 14 & 15 */
	bcm2835_GpioFnSel(14, GPFN_ALT5);
	bcm2835_GpioFnSel(15, GPFN_ALT5);

	aux->AUX_ENABLES = 1;

	aux->AUX_MU_IER_REG  = 0x00; // disable rx & tx irq
	aux->AUX_MU_CNTL_REG = 0x00; // disable rx & tx receiver and any flow control
	aux->AUX_MU_LCR_REG  = 0x03; // Bit 1 must be set, 8 data bits
	aux->AUX_MU_MCR_REG  = 0x00; // de-assert RTS
#ifdef MU_USE_IRQ
	aux->AUX_MU_IER_REG  = 0x06; // clear rx & tx fifo
#endif
	aux->AUX_MU_IIR_REG  = 0xc6; // enable FIFO & clear rx/tx fifo
	/* write baud rate into 16 bit register */
	aux->AUX_MU_BAUD_REG = ((BCM283X_CLOCK_FREQ / (8 * (MU_BAUD_RATE))) - 1);
	
	aux->AUX_MU_CNTL_REG = 0x03; // enable rx & tx operation

#if 0 // not required as nothing has any pull on boot
	gpio->GPPUD = 0;
	bcm2835_Sleep(50);
	gpio->GPPUDCLK[0] = (1<<14)|(1<<15);
	bcm2835_Sleep(50);
	gpio->GPPUDCLK[0] = 0;
#endif
	//  ISR_INSTALL_ISR2("UART", Uart_Isr, BCM2835_IRQ_ID_AUX_UART, 12, 0);

	//  IRQ_ENABLE1 = BIT(29);

}

void mini_uart_send(uint32 c)
{
	volatile struct bcm283x_aux_reg *aux = RPI_ARM_AUX_BASE;

	if (unlikely(UART_FLAG == false )) {
		UART_FLAG = true;
		Uart_Init();
	}

	while ((aux->AUX_MU_LSR_REG & 0x20) == 0) {
		;
	}
	aux->AUX_MU_IO_REG = c;
}

static void mini_uart_sendstr(char *s)
{
	unsigned char c;
	int freeSlock = 0;

#ifdef CFG_ARM_V7 // RPI 2/3
	if (numCoresRdy > 1) {
		freeSlock = 1;
		Os_ArchGetSpinlock(&ulock);
	}
#endif
	while (c = *s++) {
		mini_uart_send(c);
	}
#ifdef CFG_ARM_V7 // RPI 2/3
	if (freeSlock > 0)
		Os_ArchReleaseSpinlock(&ulock);
#endif
}

static void mini_uart_sendstr2(char *s, int length)
{
	unsigned char c;
	int i, freeSlock = 0;

#ifdef CFG_ARM_V7 // RPI 2/3
	if (numCoresRdy > 1) {
		freeSlock = 1;
		Os_ArchGetSpinlock(&ulock);
	}
#endif
	for (i = 0; i < length; i++) {
		c = s[i];
		mini_uart_send(c);
	}
#ifdef CFG_ARM_V7 // RPI 2/3
	if (freeSlock > 0)
		Os_ArchReleaseSpinlock(&ulock);
#endif
}

static void mini_uart_sendhex(uint32 d, boolean newline)
{
	uint32 rb;
	uint32 rc;
	int freeSlock = 0;

	rb=32;
#ifdef CFG_ARM_V7 // RPI 2/3
	if (numCoresRdy > 1) {
		freeSlock = 1;
		Os_ArchGetSpinlock(&ulock);
	}
#endif
	while(1)
	{
		rb-=4;
		rc=(d>>rb)&0xF;
		if(rc>9) {
			rc+=0x37;
		} else{
			rc+=0x30;
		}
		mini_uart_send(rc);

		if(rb==0) {
			break;
		}
	}

	mini_uart_send(0x20);

	if (newline) {
		mini_uart_send(0x0A);
	}
#ifdef CFG_ARM_V7 // RPI 2/3
	if (freeSlock > 0)
		Os_ArchReleaseSpinlock(&ulock);
#endif
}

uint32 mini_uart_recv(void)
{
	volatile struct bcm283x_aux_reg *aux = RPI_ARM_AUX_BASE;

	while ((aux->AUX_MU_LSR_REG & 0x01) == 0){
		;
	}

	return aux->AUX_MU_IO_REG;
}

uint32 mini_uart_lcr(void)
{
	volatile struct bcm283x_aux_reg *aux = RPI_ARM_AUX_BASE;
	uint32 result;

	result = aux->AUX_MU_LSR_REG;

	return result;
}

#if 0 // fix functions
/*Int to chars
 *
 */
char* Int_To_Chars(int i)
{
	char *result = NULL;
	char str[4] = { 0, 0, 0, 0 };
	str[0] = (char) i;
	str[1] = (char) (i >> 8);
	str[2] = (char) (i >> 16);
	str[3] = (char) (i >> 24);
	result = str;
	return result;
}

/*
 * Send Int
 */
void Uart_Send_Int(int i)
{
	char *data = Int_To_Chars(i);
	int index;

	for (index = 0; index < 4; index++)
	{
		mini_uart_send(*data);
		data++;
	}
	data = NULL;
}
#endif

int Chars_To_Int(char header[4])
{
	unsigned int result = 0;

	result = header[0];
	result = result + (header[1] << 8);
	result = result + (header[2] << 16);
	result = result + (header[3] << 24);

	return result;
}

void mini_uart_sendDec(uint32 n)
{
	if (n >= 10){
		mini_uart_sendDec(n/10);
		n = n%10;
	}
	mini_uart_send(n+'0'); /* n is between 0 and 9 */
}

void pi_printf(char* s){

	if (UART_FLAG == false) {
		UART_FLAG = true;
		Uart_Init();
	}

	mini_uart_sendstr(s);
}

void pi_printf2(char* s, int length)
{
	if (UART_FLAG == false ) {
		UART_FLAG = true;
		Uart_Init();
	}

	mini_uart_sendstr2(s, length);
}

void pi_print(uint32 s, uint32 n)
{
	if (UART_FLAG == false ) {
		UART_FLAG = true;
		Uart_Init();
	}

	mini_uart_sendhex(s, n);
}
