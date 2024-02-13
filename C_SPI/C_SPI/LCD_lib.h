#ifndef LCD_LIB_H
#define LCD_LIB_H

#ifndef F_CPU
#define F_CPU 7372800UL
#endif

#include <avr/io.h>
#include <util/delay.h>


#define CURS_DISP_SHIFT 0x00 // cursor or display shift
#define ENTRY_MODE_SET 0x06 // increment cursor position, without display shift
#define FUNC_SET 0x34 // 8-bit mode, one line, 5x11 character size
#define DISP_ON_OFF 0x0C // display on, cursor off, cursor position off
#define SET_DDRAM_ADDR 0x80 // set address
#define SECOND_LINE_ADDRESS 0x40 // address for the second line

void data_LCD(unsigned char data);
void command_LCD(unsigned char command);
void init_LCD(void);
void print_time(const unsigned char*);
#endif