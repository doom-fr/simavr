/*
	avr_uart.h

	Copyright 2008, 2009 Michel Pollet <buserror@gmail.com>

 	This file is part of simavr.

	simavr is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	simavr is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with simavr.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __AVR_UART_H__
#define __AVR_UART_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "sim_avr.h"

#include "fifo_declare.h"

DECLARE_FIFO(uint16_t, uart_fifo, 64);

/*
 * The method of "connecting" the the UART from external code is to use 4 IRQS.
 * The easy one is UART->YOU, where you will be called with the byte every time
 * the AVR firmware sends one. Do whatever you like with it.
 *
 * The slightly more tricky one is the INPUT part. Since the AVR is quite a bit
 * slower than your code most likely, there is a way for the AVR UART to tell
 * you to "pause" sending it bytes when its own input buffer is full.
 * So, the UART will send XON to you when its fifo is empty, XON means you can
 * send as many bytes as you have until XOFF is sent. Note that these are two
 * IRQs because you /will/ be called with XOFF when sending a byte in INPUT...
 * So it's a reentrant process.
 *
 * When XOFF has been called, do not send any new bytes, they would be dropped.
 * Instead wait for XON again and continue.
 * See examples/parts/uart_udp.c for a full implementation
 *
 * Pseudo code:
 *
 * volatile int off = 0;
 * void irq_xon()
 * {
 *     off = 0;
 *     while (!off && bytes_left)
 *     avr_raise_irq(UART_IRQ_INPUT, a_byte);
 * }
 * void irq_xoff()
 * {
 *     off = 1;
 * }
 *
 */
enum {
	UART_IRQ_INPUT = 0,
	UART_IRQ_OUTPUT,
	UART_IRQ_OUT_XON,		// signaled (continuously) when input fifo is not full
	UART_IRQ_OUT_XOFF,		// signaled when input fifo IS full
	UART_IRQ_COUNT
};

enum {
	UART_INPUT_FE = 0x8000		// framing error
};

// add port number to get the real IRQ
#define AVR_IOCTL_UART_GETIRQ(_name) AVR_IOCTL_DEF('u','a','r',(_name))

enum {
	// the uart code monitors for firmware that poll on
	// reception registers, and can do an atomic usleep()
	// if it's detected, this helps regulating CPU
	AVR_UART_FLAG_POOL_SLEEP = (1 << 0),
	AVR_UART_FLAG_POLL_SLEEP = (1 << 0),		// to replace pool_sleep
	AVR_UART_FLAG_STDIO = (1 << 1),				// print lines on the console
};

typedef struct avr_uart_t {
	avr_io_t	io;
	char name;
	avr_regbit_t	disabled;	// bit in the PRR
	
	avr_io_addr_t r_udr;
	avr_io_addr_t r_ucsra;
	avr_io_addr_t r_ucsrb;
	avr_io_addr_t r_ucsrc;

	avr_regbit_t	rxen;		// receive enabled
	avr_regbit_t	txen;		// transmit enable
	avr_regbit_t	u2x;		// double UART speed
	avr_regbit_t	usbs;		// stop bits
	avr_regbit_t	ucsz;		// data bits
	avr_regbit_t	ucsz2;		// data bits, continued

	// read-only bits (just to mask it out)
	avr_regbit_t	fe;			// frame error bit
	avr_regbit_t	dor;		// data overrun bit
	avr_regbit_t	upe;		// parity error bit
	avr_regbit_t	rxb8;		// receive data bit 8

	avr_regbit_t	ubrrl;
	avr_regbit_t	ubrrh;

	avr_int_vector_t rxc;
	avr_int_vector_t txc;
	avr_int_vector_t udrc;	

	uart_fifo_t	input;
	uint8_t		tx_cnt;			// number of unsent characters in the output buffer
	uint32_t	rx_cnt;			// number of characters read by app since rxc_raise_time

	uint32_t		flags;
	avr_cycle_count_t cycles_per_byte;
	avr_cycle_count_t rxc_raise_time; // the cpu cycle when rxc flag was raised last time

	uint8_t *		stdio_out;
	int				stdio_len;	// current size in the stdio output
} avr_uart_t;

/* takes a uint32_t* as parameter */
#define AVR_IOCTL_UART_SET_FLAGS(_name)	AVR_IOCTL_DEF('u','a','s',(_name))
#define AVR_IOCTL_UART_GET_FLAGS(_name)	AVR_IOCTL_DEF('u','a','g',(_name))

void avr_uart_init(avr_t * avr, avr_uart_t * port);


#define AVR_UARTRB_DECLARE(_name_reg, _name_bit, _prr, _prusart) \
	.uart ## _name_reg = { \
		.name = '0' + _name_reg, \
		.disabled = AVR_IO_REGBIT(_prr, _prusart), \
	\
		.r_udr = UDR ## _name_reg, \
	\
		.fe = AVR_IO_REGBIT(UCSR ## _name_reg ## A, FE ## _name_bit), \
		.dor = AVR_IO_REGBIT(UCSR ## _name_reg ## A, DOR ## _name_bit), \
		.upe = AVR_IO_REGBIT(UCSR ## _name_reg ## A, UPE ## _name_bit), \
		.u2x = AVR_IO_REGBIT(UCSR ## _name_reg ## A, U2X ## _name_bit), \
		.txen = AVR_IO_REGBIT(UCSR ## _name_reg ## B, TXEN ## _name_bit), \
		.rxen = AVR_IO_REGBIT(UCSR ## _name_reg ## B, RXEN ## _name_bit), \
		.rxb8 = AVR_IO_REGBIT(UCSR ## _name_reg ## B, RXB8 ## _name_bit), \
		.usbs = AVR_IO_REGBIT(UCSR ## _name_reg ## C, USBS ## _name_bit), \
		.ucsz = AVR_IO_REGBITS(UCSR ## _name_reg ## C, UCSZ ## _name_bit ## 0, 0x3), \
		.ucsz2 = AVR_IO_REGBIT(UCSR ## _name_reg ## B, UCSZ ## _name_bit ## 2), \
		.ubrrl = AVR_IO_REGBITS(UBRR ## _name_reg ## L, 0,0xFF), \
		.ubrrh = AVR_IO_REGBITS(UBRR ## _name_reg ## H, 0,0xF), \
		\
		.r_ucsra = UCSR ## _name_reg ## A, \
		.r_ucsrb = UCSR ## _name_reg ## B, \
		.r_ucsrc = UCSR ## _name_reg ## C, \
	\
		.rxc = { \
			.enable = AVR_IO_REGBIT(UCSR ## _name_reg ## B, RXCIE ## _name_bit), \
			.raised = AVR_IO_REGBIT(UCSR ## _name_reg ## A, RXC ## _name_bit), \
			.vector = USART ## _name_reg ## _RX_vect, \
			.raise_sticky = 1, \
		}, \
		.txc = { \
			.enable = AVR_IO_REGBIT(UCSR ## _name_reg ## B, TXCIE ## _name_bit), \
			.raised = AVR_IO_REGBIT(UCSR ## _name_reg ## A, TXC ## _name_bit), \
			.vector = USART ## _name_reg ## _TX_vect, \
		}, \
		.udrc = { \
			.enable = AVR_IO_REGBIT(UCSR ## _name_reg ## B, UDRIE ## _name_bit), \
			.raised = AVR_IO_REGBIT(UCSR ## _name_reg ## A, UDRE ## _name_bit), \
			.vector = USART ## _name_reg ## _UDRE_vect, \
			.raise_sticky = 1, \
		}, \
	}


#define AVR_UARTX_DECLARE(_name, _prr, _prusart)	AVR_UARTRB_DECLARE(_name, _name, _prr, _prusart)


// This macro is for older single-interface devices where variable names are bit divergent
#define AVR_UART_DECLARE(_prr, _prusart, _upe_name, _rname_ix, _intr_c) \
	.uart = { \
		.name = '0', \
		.disabled = AVR_IO_REGBIT(_prr, _prusart), \
		.r_udr = UDR ## _rname_ix, \
	\
		.fe = AVR_IO_REGBIT(UCSR ## _rname_ix ## A, FE ## _rname_ix), \
		.dor = AVR_IO_REGBIT(UCSR ## _rname_ix ## A, DOR ## _rname_ix), \
		.upe = AVR_IO_REGBIT(UCSR ## _rname_ix ## A, _upe_name ## _rname_ix), \
		.u2x = AVR_IO_REGBIT(UCSR ## _rname_ix ## A, U2X ## _rname_ix), \
		.txen = AVR_IO_REGBIT(UCSR ## _rname_ix ## B, TXEN ## _rname_ix), \
		.rxen = AVR_IO_REGBIT(UCSR ## _rname_ix ## B, RXEN ## _rname_ix), \
		.rxb8 = AVR_IO_REGBIT(UCSR ## _rname_ix ## B, RXB8 ## _rname_ix), \
		.usbs = AVR_IO_REGBIT(UCSR ## _rname_ix ## C, USBS ## _rname_ix), \
		.ucsz = AVR_IO_REGBITS(UCSR ## _rname_ix ## C, UCSZ ## _rname_ix ## 0, 0x3), \
		.ucsz2 = AVR_IO_REGBIT(UCSR ## _rname_ix ## B, UCSZ ## _rname_ix ## 2), \
		.ubrrl = AVR_IO_REGBITS(UBRR ## _rname_ix ## L, 0,0xFF), \
		.ubrrh = AVR_IO_REGBITS(UBRR ## _rname_ix ## H, 0,0xF), \
	\
		.r_ucsra = UCSR ## _rname_ix ## A, \
		.r_ucsrb = UCSR ## _rname_ix ## B, \
		.r_ucsrc = UCSR ## _rname_ix ## C, \
	\
		.rxc = { \
			.enable = AVR_IO_REGBIT(UCSR ## _rname_ix ## B, RXCIE ## _rname_ix), \
			.raised = AVR_IO_REGBIT(UCSR ## _rname_ix ## A, RXC ## _rname_ix), \
			.vector = USART_RX ## _intr_c ## _vect, \
			.raise_sticky = 1, \
		}, \
		.txc = { \
			.enable = AVR_IO_REGBIT(UCSR ## _rname_ix ## B, TXCIE ## _rname_ix), \
			.raised = AVR_IO_REGBIT(UCSR ## _rname_ix ## A, TXC ## _rname_ix), \
			.vector = USART_TX ## _intr_c ## _vect, \
		}, \
		.udrc = { \
			.enable = AVR_IO_REGBIT(UCSR ## _rname_ix ## B, UDRIE ## _rname_ix), \
			.raised = AVR_IO_REGBIT(UCSR ## _rname_ix ## A, UDRE ## _rname_ix), \
			.vector = USART_UDRE_vect, \
			.raise_sticky = 1, \
		}, \
	}

#ifdef __cplusplus
};
#endif

#endif /*__AVR_UART_H__*/
