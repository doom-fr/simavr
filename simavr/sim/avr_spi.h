/*
	avr_spi.h

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

#ifndef __AVR_SPI_H__
#define __AVR_SPI_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "sim_avr.h"

enum {
	SPI_IRQ_INPUT = 0,
	SPI_IRQ_OUTPUT,
	SPI_IRQ_COUNT
};

// add port number to get the real IRQ
#define AVR_IOCTL_SPI_GETIRQ(_name) AVR_IOCTL_DEF('s','p','i',(_name))

typedef struct avr_spi_t {
	avr_io_t	io;
	char name;
	avr_regbit_t	disabled;	// bit in the PRR

	avr_io_addr_t	r_spdr;			// data register
	avr_io_addr_t	r_spcr;			// control register
	avr_io_addr_t	r_spsr;			// status register
	
	avr_regbit_t spe;		// spi enable
	avr_regbit_t mstr;		// master/slave
	avr_regbit_t spr[4];	// clock divider
	
	avr_int_vector_t spi;	// spi interrupt

	uint8_t		input_data_register;
} avr_spi_t;

void avr_spi_init(avr_t * avr, avr_spi_t * port);

#define AVR_SPIRBV_DECLARE(_name_reg, _name_bit, _name_vect, _prr, _prspi) \
	.spi ## _name_reg = { \
		.name = '0' + _name_reg,\
		.disabled = AVR_IO_REGBIT(_prr, _prspi), \
	\
		.r_spdr = SPDR ## _name_reg, \
		.r_spcr = SPCR ## _name_reg, \
		.r_spsr = SPSR ## _name_reg, \
	\
		.spe = AVR_IO_REGBIT(SPCR ## _name_reg, SPE ## _name_bit), \
		.mstr = AVR_IO_REGBIT(SPCR ## _name_reg, MSTR ## _name_bit), \
	\
		.spr = { AVR_IO_REGBIT(SPCR ## _name_reg, SPR0 ## _name_bit), \
					AVR_IO_REGBIT(SPCR ## _name_reg, SPR1 ## _name_bit), \
					AVR_IO_REGBIT(SPSR ## _name_reg, SPI2X ## _name_bit) }, \
		.spi = { \
			.enable = AVR_IO_REGBIT(SPCR ## _name_reg, SPIE ## _name_bit), \
			.raised = AVR_IO_REGBIT(SPSR ## _name_reg, SPIF ## _name_bit), \
			.vector = SPI  ## _name_vect ## _STC_vect, \
		}, \
	}
	
#define AVR_SPIX_DECLARE(_name, _prr, _prspi)	AVR_SPIRBV_DECLARE(_name, _name , , _prr, _prspi)


#define AVR_SPI_DECLARE(_prr, _prspi) \
	.spi = { \
		.disabled = AVR_IO_REGBIT(_prr, _prspi), \
	\
		.r_spdr = SPDR, \
		.r_spcr = SPCR, \
		.r_spsr = SPSR, \
	\
		.spe = AVR_IO_REGBIT(SPCR, SPE), \
		.mstr = AVR_IO_REGBIT(SPCR, MSTR), \
	\
		.spr = { AVR_IO_REGBIT(SPCR, SPR0), AVR_IO_REGBIT(SPCR, SPR1), AVR_IO_REGBIT(SPSR, SPI2X) }, \
		.spi = { \
			.enable = AVR_IO_REGBIT(SPCR, SPIE), \
			.raised = AVR_IO_REGBIT(SPSR, SPIF), \
			.vector = SPI_STC_vect, \
		}, \
	}

#ifdef __cplusplus
};
#endif

#endif /*__AVR_SPI_H__*/
