#include "SPI_lib.h" 

void SPI_init()
{
	// set PB3 as output => MISO
	DDRB |= (1 << DDRB3);
	//enable SPI, MSB is transmitted first
	SPCR = (1 << SPIE) | (1 << SPE);
	SPCR &= ~(1 << DORD);
	SPDR = CONNECTION_CODE_SLAVE;
}