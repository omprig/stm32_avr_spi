#define F_CPU 7372800UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include "LCD_lib.h"
#include "SPI_lib.h"
#include <stdbool.h>

#define  NUMBER_OF_TIME_DIGITS 6

volatile unsigned char time_array[] = {'0', '0', '0', '0', '0', '0'};
volatile unsigned char time_index = 0;
volatile bool update_time	= false;
volatile bool set_connection = false;


ISR(SPI_STC_vect)
{
	uint8_t spi_data = SPDR;
	if(set_connection)
	{
		time_array[time_index] = (spi_data >> 4) + 0x30;
		time_array[time_index + 1] = (spi_data & 0x0F) + 0x30;
		time_index+=2;
	
		if(time_index == NUMBER_OF_TIME_DIGITS)
		{
			time_index = 0;
			update_time = true;
		}
	}
	else if(spi_data == CONNECTION_CODE_MASTER) //trying to set connection with master
	{
		set_connection = true;
	}
	else 
	{
		set_connection = false;
		SPDR = CONNECTION_CODE_SLAVE;
	}
}


int main(void)
{
	init_LCD();
	
	// init SPI in slave mode
	SPI_init();
	
	sei();
	
	//_delay_ms(4000);
	print_time(time_array);
		
	
    while (1) 
    {
		if(update_time)
		{
			print_time(time_array);
			update_time = false;
		}
    }
}

