#include "LCD_lib.h"

void data_LCD(unsigned char data){
	// set PA0 and PA1 as output
	DDRA |= 0x03;
	// E=1, RS=1
	//PORTA &= 0xFC;
	PORTA |= 0x03;

	// set PC0-PC7 as lines for command
	DDRC = 0xFF;
	PORTC = data;
	_delay_ms(10);

	// down E
	PORTA &= 0xFE;
	// wait
	_delay_ms(10);

}

void command_LCD(unsigned char command){
	// set PA0 and PA1 as output
	DDRA |= 0x03;
	// E=1, RS=0
	//PORTA &= 0xFC;
	PORTA |= 0x01;
	PORTA &= 0xFD; 
	
	// set PC0-PC7 as lines for command
	DDRC = 0xFF;
	PORTC = command;
	_delay_ms(10);
	
	// down E
	PORTA &= 0xFC;
	// wait
	_delay_ms(10);
}

void init_LCD(void){
	
	_delay_ms(2000);
	// Function set
	command_LCD(FUNC_SET);
	_delay_ms(10);

	// Entry mode set
	command_LCD(ENTRY_MODE_SET);
	_delay_ms(10);
	
	// cursor and display shift disabled
	command_LCD(CURS_DISP_SHIFT);
	_delay_ms(10);
	
	command_LCD(SET_DDRAM_ADDR);
	_delay_ms(10);
	
char init_LCD_data[] = {' ', ' ', ':', ' ', ' ', ':', ' ', ' '};
							
	
	unsigned char i;
	for(i = 0; i < 8; i++)
	{
		data_LCD(init_LCD_data[i]);
		_delay_ms(10);
	}
		
	// Display ON/OFF control
	command_LCD(DISP_ON_OFF);
	_delay_ms(10);
	
}


void print_time(const unsigned char *pTime){
	command_LCD(SET_DDRAM_ADDR);
	_delay_ms(10);
	
	// hours
	data_LCD(pTime[0]);
	_delay_ms(10);
	data_LCD(pTime[1]);
	_delay_ms(10);
	data_LCD(':');
	_delay_ms(10);	
	
	//minutes
	data_LCD(pTime[2]);
	_delay_ms(10);
	data_LCD(pTime[3]);
	_delay_ms(10);
	data_LCD(':');
	_delay_ms(10);
	
	//seconds
	data_LCD(pTime[4]);
	_delay_ms(10);
	data_LCD(pTime[5]);
	_delay_ms(10);
}
