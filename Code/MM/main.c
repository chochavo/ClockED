#define F_CPU 8000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "rtc.h"

#define ROW_PORT PORTC
#define ROW_DDR DDRC
#define COL_PORT_0 PORTD
#define COL_PORT_1 PORTB
#define COL_DDR_0 DDRD
#define COL_DDR_1 DDRB

#define ROW_0 (1 << PORTC0)
#define ROW_1 (1 << PORTC1)
#define ROW_2 (1 << PORTC2)
#define ROW_3 (1 << PORTC3)
#define ROW_4 (1 << PORTC4)

#define COL_9 (1 << PORTD0)
#define COL_8 (1 << PORTD1)
#define COL_7 (1 << PORTD2)
#define COL_6 (1 << PORTD3)
#define COL_5 (1 << PORTD4)
#define COL_4 (1 << PORTD5)
#define COL_3 (1 << PORTD6)
#define COL_2 (1 << PORTD7)
#define COL_1 (1 << PORTB2)
#define COL_0 (1 << PORTB1)

#define NUMBER_OF_ROWS 9
#define NUMBER_OF_LAYERS 3

const unsigned char H[] = {0b00011111, 0b00000100,0b00011111};
const unsigned char E[] = {0b00011111, 0b00010101,0b00010101};
const unsigned char L[] = {0b00011111, 0b00010000, 0b00010000};
const unsigned char O[] = {0b00001110, 0b00010001,0b00001110};
const unsigned char A[] = {0b00011110,0b00000101,0b00011110};
const unsigned char D[] = {0b00011111,0b00010001,0b00001110};
const unsigned char R[] = {0b00011111,0b00000101,0b00011011};
const unsigned char ONE[] = {0b00010010, 0b00011111,0b00010000};
const unsigned char TWO[] = {0b00011001, 0b00010101,0b00010011};
const unsigned char THREE[] = {0b00010101,0b00010101,0b00011111};
const unsigned char FOUR[] = {0b00000111,0b00000100,0b00011111};
const unsigned char FIVE[] = {0b00010111,0b00010101,0b00011101};
const unsigned char SIX[] = {0b00011111,0b00010101,0b00011101};
const unsigned char SEVEN[] = {0b00011001,0b00000101,0b00000011};
const unsigned char EIGHT[] = {0b00011111,0b00010101,0b00011111};
const unsigned char NINE[] = {0b00010111,0b00010101,0b00011111};
const unsigned char ZERO[] = {0b00011111,0b00010001,0b00011111};
const unsigned char DOTS[] = {0b00000000,0b00001010,0b00000000};
const unsigned char RMARK[] = {0b00000000,0b00010111,0b00000000};
const unsigned char K[] = {0b00011111,0b00000100,0b00001010};
const unsigned char PE[] = {0b00011111,0b00000001,0b00011111};
const unsigned char ER[] = {0b00011111,0b00000101,0b00000111};
const unsigned char IE[] = {0b00001111,0b00010000,0b00011111};
const unsigned char VE[] = {0b00011111,0b00010101,0b00001110};
const unsigned char T[] =  {0b00000001,0b00011111,0b00000001};
const unsigned char ZX[] = {0b00000000,0b00001100,0b00011100};
const unsigned char GE[] = {0b00011111,0b00000001,0b00000001};
const unsigned char CR[] = {0b00010001,0b00010001,0b00001110};
const unsigned char DEF[] = {0b00000100,0b00000100,0b00000100};
const unsigned char Y[] = {0b00000011,0b00011100,0b00000011};


unsigned char temp[100];
volatile unsigned char buf[2] = {0,0};
char time_temp[12];

void Init_Ports(void);
void reset_leds(void);
void matrix_show(uint8_t column_in, uint8_t row_word);
void int_to_ascii(uint8_t xxx);
void print_time_string(uint8_t minute_in, uint8_t hour_in);
void write_string(char *string_x);
	
void Init_Ports(void)
{
	ROW_DDR	  |= ROW_0 | ROW_1 | ROW_2 | ROW_3 | ROW_4;
	COL_DDR_0 |= COL_0 | COL_1 | COL_2 | COL_3 | COL_4 | COL_5 | COL_6 | COL_7;
	COL_DDR_1 |= COL_8 | COL_9;
}

void reset_leds(void)
{
	ROW_PORT   &= ~(ROW_0) & ~(ROW_1) & ~(ROW_2) & ~(ROW_3) & ~(ROW_4);
	COL_PORT_0 &= 0;
	COL_PORT_1 &= ~COL_8 & ~COL_9; 
}

void matrix_show(uint8_t column_in, uint8_t row_word)
{
	reset_leds();
	volatile uint8_t pc0 = 0;
	volatile uint8_t pc1 = 0;
	switch(column_in)
	{
		case 9: pc0 = COL_9; pc1 &= 0b11111001; break;
		case 8: pc0 = COL_8; pc1 &= 0b11111001; break;
		case 7: pc0 = COL_7; pc1 &= 0b11111001; break;
		case 6: pc0 = COL_6; pc1 &= 0b11111001; break;
		case 5: pc0 = COL_5; pc1 &= 0b11111001; break;
		case 4: pc0 = COL_4; pc1 &= 0b11111001; break;
		case 3: pc0 = COL_3; pc1 &= 0b11111001; break;
		case 2: pc0 = COL_2; pc1 &= 0b11111001;break;
		case 1: pc1 &= 0b11111001; pc1 |= COL_1; pc0 = 0; break;
		case 0: pc1 &= 0b11111001; pc1 |= COL_0; pc0 = 0; break;
																			
	}
	COL_PORT_0 = pc0;
	COL_PORT_1 = pc1;
	ROW_PORT = row_word;
}

/* Integer to ASCII array conversion, only two digits are used, since there is no need for more */
void int_to_ascii(uint8_t xxx)
{
	if (xxx <= 9)
	{
		buf[0] = 0x30;
		buf[1] = xxx + 0x30;
	}
	else
	{
		buf[1] = xxx % 10 + 0x30;
		xxx /= 10;
		buf[0] = xxx % 10 + 0x30;
	}
}

/* Real-time clock string generation */
void print_time_string(uint8_t minute_in, uint8_t hour_in)
{
	int_to_ascii(hour_in);
	time_temp[0] = buf[0]; time_temp[1] = buf[1]; time_temp[2] = ':';
	int_to_ascii(minute_in); time_temp[3] = buf[0]; time_temp[4] = buf[1]; time_temp[5] = '\n';
	write_string(time_temp);
}

void print_date_string(uint8_t date_in, uint8_t month_in, uint8_t year_in)
{
	memset(time_temp,0,10);
	time_temp[0] = 'd'; time_temp[1] = 'a'; time_temp[2] = 't'; time_temp[3] = 'e'; time_temp[4] = ':';
	int_to_ascii(date_in);
	time_temp[5] = buf[0]; time_temp[6] = buf[1]; time_temp[7] = '-';
	int_to_ascii(month_in);
	time_temp[8] = buf[0]; time_temp[9] = buf[1]; time_temp[10] = '\n';
	write_string(time_temp);

}

void write_string(char *string_x)
{
	uint8_t ix = 0;
	volatile uint8_t chars_stored = 0;
	memset(temp,0,100);
for (ix = 0; string_x[ix] != '\n'; ix++)
{
	switch (string_x[ix])
	{
		case 'y': memcpy(temp + 4*chars_stored,Y,3); break;
		case 'r': memcpy(temp + 4*chars_stored,R,3); break;
		case 'u': memcpy(temp + 4*chars_stored,IE,3); break;
		case 't': memcpy(temp + 4*chars_stored,T,3); break;
		case 'a': memcpy(temp + 4*chars_stored,A,3); break;
		case 'd': memcpy(temp + 4*chars_stored,D,3); break;
		case 'h': memcpy(temp + 4*chars_stored,H,3); break;
		case 'e': memcpy(temp + 4*chars_stored,E,3); break;
		case 'l': memcpy(temp + 4*chars_stored,L,3); break;
		case 'o':memcpy(temp + 4*chars_stored,O,3); break;
		case '0':memcpy(temp + 4*chars_stored,ZERO,3); break;
		case '1':memcpy(temp + 4*chars_stored,ONE,3); break;
		case '2':memcpy(temp + 4*chars_stored,TWO,3); break;
		case '3':memcpy(temp + 4*chars_stored,THREE,3); break;
		case '4':memcpy(temp + 4*chars_stored,FOUR,3); break;
		case '5':memcpy(temp + 4*chars_stored,FIVE,3); break;
		case '6':memcpy(temp + 4*chars_stored,SIX,3); break;
		case '7':memcpy(temp + 4*chars_stored,SEVEN,3); break;
		case '8':memcpy(temp + 4*chars_stored,EIGHT,3); break;
		case '9':memcpy(temp + 4*chars_stored,NINE,3); break;
		case '!':memcpy(temp + 4*chars_stored,RMARK,3); break;
		case ':':memcpy(temp + 4*chars_stored,DOTS,3); break;
		case 'Ï':memcpy(temp + 4*chars_stored,PE,3); break;
		case 'Ð':memcpy(temp + 4*chars_stored,ER,3); break;
		case 'È':memcpy(temp + 4*chars_stored,IE,3); break;
		case 'Â':memcpy(temp + 4*chars_stored,VE,3); break;
		case 'Å':memcpy(temp + 4*chars_stored,E,3); break;
		case 'Ò':memcpy(temp + 4*chars_stored,T,3); break;
		case ',':memcpy(temp + 4*chars_stored,ZX,3); break;
		case 'Ã':memcpy(temp + 4*chars_stored,GE,3); break;
		case 'Í':memcpy(temp + 4*chars_stored,H,3); break;
		case ')':memcpy(temp + 4*chars_stored,CR,3); break;
		case '-':memcpy(temp + 4*chars_stored,DEF,3); break;
		case 'k':memcpy(temp + 4*chars_stored,K,3); break;
		default: break;
	}
	chars_stored++;
}
	bool OutX = true;
	volatile int i = 0;
	volatile int adjust = -10;
	while (OutX)
	{
		for (uint8_t col = 0; col < 10; col++)
		{
			if (col + adjust < 0) matrix_show(col,0);
			else matrix_show(col,temp[col + adjust]);
			_delay_us(200);
			if (i == 300) {
				i = 0;
				if (adjust == (ix)*4 + 10 ) {
					OutX = false;
					adjust = -10;
				}
				else adjust++;
			}
			else i++;
		}
		chars_stored = 0;
	}
}

uint8_t update_min(uint8_t minute_in)
{
	memset(time_temp,0,10);
	bool outcon = false;
	while(!outcon)
	{
		time_temp[0] = 'e'; time_temp[1] = ':';
		int_to_ascii(minute_in);
		time_temp[2] = buf[0]; time_temp[3] = buf[1]; time_temp[4] = '\n';
		write_string(time_temp);
		if ((PINB & (1 << PINB3)) != (1 << PINB3))
		{
			while((PINB & (1 << PINB3)) != (1 << PINB3));
			if (minute_in > 58) minute_in = 0;
			else if (minute_in < 0) minute_in = 59;
			else minute_in++;
		}
		if ( (PINB & (1 << PINB4)) != (1 << PINB4))
		{
			while((PINB & (1 << PINB4)) != (1 << PINB4));
			outcon = true;
			write_string("ok!");
		}
	}
	return minute_in;
}

uint8_t update_hour(uint8_t hour_in)
{
	memset(time_temp,0,10);
	bool outcon = false;
	while(!outcon)
	{
		time_temp[0] = 'h'; time_temp[1] = ':';
		int_to_ascii(hour_in);
		time_temp[2] = buf[0]; time_temp[3] = buf[1]; time_temp[4] = '\n';
		write_string(time_temp);
		if ((PINB & (1 << PINB4)) != (1 << PINB4))
		{
			while((PINB & (1 << PINB4)) != (1 << PINB4));
			if (hour_in > 22) hour_in = 0;
			else if (hour_in < 0) hour_in = 23;
			else hour_in++;
		}
		if ((PINB & (1 << PINB3)) != (1 << PINB3))
		{
			while((PINB & (1 << PINB3)) != (1 << PINB3));
			outcon = true;
			write_string("ok!");
		}
	}
	return hour_in;
}

uint8_t update_month(uint8_t month_in)
{
	memset(time_temp,0,10);
	bool outcon = false;
	while(!outcon)
	{
		time_temp[0] = 'y'; time_temp[1] = ':';
		int_to_ascii(month_in);
		time_temp[2] = buf[0]; time_temp[3] = buf[1]; time_temp[4] = '\n';
		write_string(time_temp);
		if ((PINB & (1 << PINB4)) != (1 << PINB4))
		{
			while((PINB & (1 << PINB4)) != (1 << PINB4));
			if (month_in > 11) month_in = 1;
			else if (month_in < 1) month_in = 11;
			else month_in++;
		}
		if ((PINB & (1 << PINB3)) != (1 << PINB3))
		{
			while((PINB & (1 << PINB3)) != (1 << PINB3));
			outcon = true;
			write_string("ok!");
		}
	}
	return month_in;
}

uint8_t update_day(uint8_t day_in)
{
	memset(time_temp,0,10);
	bool outcon = false;
	while(!outcon)
	{
		time_temp[0] = 'd'; time_temp[1] = ':';
		int_to_ascii(day_in);
		time_temp[2] = buf[0]; time_temp[3] = buf[1]; time_temp[4] = '\n';
		write_string(time_temp);
		if ((PINB & (1 << PINB3)) != (1 << PINB3))
		{
			while((PINB & (1 << PINB3)) != (1 << PINB3));
			if (day_in > 30) day_in = 1;
			else if (day_in < 1) day_in = 30;
			else day_in++;
		}
		if ((PINB & (1 << PINB4)) != (1 << PINB4))
		{
			while((PINB & (1 << PINB4)) != (1 << PINB4));
			outcon = true;
			write_string("ok!");
		}
	}
	return day_in;
}

int main(void)
{
	Init_Ports();
	struct rtc_time ds1302;				// DS1302 semi-"class" Definition
	struct rtc_time *rtc;
	rtc = &ds1302;
	DDRB |= (1 << 0) | (1 << 7) | (1 << 6);
	PORTB &= ~(1 << 0) & ~(1 << 7) & ~(1 << 6);
	ds1302_init();
	//ds1302_update(rtc);					// update all fields in the struct
	ds1302_write_byte(0x8E,0x00);
	//_delay_ms(2);
	//ds1302_set_time(rtc, MIN, 44);
	//_delay_us(200);
	//ds1302_set_time(rtc, HOUR, 17);
	//_delay_us(200);
	//ds1302_set_time(rtc, DATE, 1);
	//_delay_us(200);
	//ds1302_set_time(rtc, MONTH, 10);
	//_delay_us(200);
	write_string("ÏÐÈÂÅÒ ÅÂÃÅÍ!:)\n");
    while (1) 
    {
		ds1302_update_time(rtc,MIN);
		ds1302_update_time(rtc,HOUR);
		ds1302_update_time(rtc,DATE);
		ds1302_update_time(rtc,MONTH);
		print_time_string(rtc->minute,rtc->hour);
		print_date_string(rtc->date,rtc->month,rtc->year);
		
		if ((PINB & (1 << PINB3)) != (1 << PINB3)) 
		{
			while((PINB & (1 << PINB3)) != (1 << PINB3));
			if ((PINB & (1 << PINB4)) != (1 << PINB4))
			{
				while((PINB & (1 << PINB4)) != (1 << PINB4));
				ds1302_update_time(rtc,DATE);
				ds1302_set_time(rtc, DATE, update_day(rtc->date));	
			}
			else 
			{
			ds1302_update_time(rtc,MIN);
			ds1302_set_time(rtc, MIN, update_min(rtc->minute));
			}
		}
		if ((PINB & (1 << PINB4)) != (1 << PINB4))
		{
			while((PINB & (1 << PINB4)) != (1 << PINB4));
			if ((PINB & (1 << PINB4)) != (1 << PINB4))
			{
				while((PINB & (1 << PINB3)) != (1 << PINB3));
				ds1302_update_time(rtc,MONTH);
				ds1302_set_time(rtc, MONTH, update_month(rtc->month));
			}
			else 
			{
			ds1302_update_time(rtc,HOUR);
			ds1302_set_time(rtc, HOUR, update_hour(rtc->hour));	
			}
		}
    }
}


