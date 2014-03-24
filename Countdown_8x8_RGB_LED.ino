/*
  Colorduino - Colorduino DEMO for Arduino.
  Copyright (c) 2010 zzy@IteadStudio.  All right reserved.

  This DEMO is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This DEMO is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/
#include <Wire.h>
#include <avr/pgmspace.h> 

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

/*****************************
define the operate commands
*****************************/

/*****************************
define the status
*****************************/

/*****************************
define the IO
*****************************/
#define RST_BIT 0x04
#define LAT_BIT 0x02
#define SLB_BIT 0x01
#define SCL_BIT 0x40
#define SDA_BIT 0x80

#define RST PORTC
#define LAT PORTC
#define SLB PORTC
#define SDA PORTD
#define SCL PORTD

#define open_line0	{PORTB=0x01;}
#define open_line1	{PORTB=0x02;}
#define open_line2	{PORTB=0x04;}
#define open_line3	{PORTB=0x08;}
#define open_line4	{PORTB=0x10;}
#define open_line5	{PORTB=0x20;}
#define open_line6	{PORTD=0x08;}
#define open_line7	{PORTD=0x10;}
#define close_all_line	{PORTD=0x00;PORTB=0x00;}

/*******************************************
define the data zone
*******************************************/

unsigned char dots[2][8][8][3] = {0};
//dots matrix
//[2]:Page:one for display, one for receive data
//[8]:Row:8 row in LED plane
//[8]:Column:8 column in one row
//[3]:Color:RGB data: 0 for Red; 1 for green, 2 for Blue
unsigned char Gamma_Value[3] = {10,63,63};
//Gamma correctly value, every LED plane is different.value range is 0~63
//[3]:RGB data, 0 for Red; 1 for green, 2 for Blue
unsigned char Page_Index = 0; // the index of buffer
unsigned char row = 0;//the value of row in LED plane, from 0~7
unsigned char column = 0;//the value of every row, from 0~7
unsigned char color = 0;//the value of every dots, 0 is Red, 1 is Green, 2 is Blue

unsigned char line = 0;

/**************************************************
define the extern data zone
**************************************************/
extern unsigned char font8_3[10][3];
/***************************************************
all parts inition functions zone
***************************************************/
void _IO_Init()
{
  DDRD = 0xff; // set all pins direction of PortD
  DDRC = 0xff; // set all pins direction of PortC
  DDRB = 0xff; // set all pins direction of PortB
  
  PORTD = 0x00; // set all pins output is low of PortD
  PORTC = 0x00; // set all pins output is low of PortC
  PORTB = 0x00; // set all pins output is low of PortB
}
void _LED_Init()
{
  LED_RST(1);
  LED_Delay(1);
  LED_RST(0);
  LED_Delay(1);
  LED_RST(1);
  LED_Delay(1);
  
  SetGamma();
  line = 0;
  
}
void _TC2_Init()
{
  TCCR2A |= (1 << WGM21) | (1 << WGM20);   
  TCCR2B |= ((1<<CS22)|(1<<CS20));   // by clk/64
  TCCR2B &= ~((1<<CS21));   // by clk/64
  TCCR2A &= ~((1<<WGM21) | (1<<WGM20));   // Use normal mode
  ASSR |= (1<<AS2);       // Use internal clock - external clock not used in Arduino
  TIMSK2 |= (1<<TOIE2) | (0<<OCIE2B);   //Timer2 Overflow Interrupt Enable
  TCNT2 = 0xff;
  sei();
}


/****************************************************
the timer2 operate functions zone
****************************************************/
ISR(TIMER2_OVF_vect)          //Timer2  Service 
{ 
  cli();  
  TCNT2 = 0x64;      //flash a led matrix frequency is 100.3Hz,period is 9.97ms
  //TCNT2 = 0x63;      //flash a led matrix frequency is 99.66Hz,period is 10.034ms   
    if(line > 7) line = 0;    
    close_all_line;  
    run(line);
    open_line(line);
    line++;
    sei();
}
/****************************************************
the LED Hardware operate functions zone
****************************************************/
void LED_SDA(unsigned char temp)
{
  if (temp) 
    SDA|=SDA_BIT;
  else
    SDA&=~SDA_BIT;
}

void LED_SCL(unsigned char temp)
{
  if (temp) 
    SCL|=SCL_BIT;
  else
    SCL&=~SCL_BIT;
}

void LED_RST(unsigned char temp)
{
  if (temp) 
    RST|=RST_BIT;
  else
    RST&=~RST_BIT;
}

void LED_LAT(unsigned char temp)
{
  if (temp) 
    LAT|=LAT_BIT;
  else
    LAT&=~LAT_BIT;
}

void LED_SLB(unsigned char temp)
{
  if (temp) 
    SLB|=SLB_BIT;
  else
    SLB&=~SLB_BIT;
}
/***************************************************
the LED datas operate functions zone
***************************************************/
void SetGamma()
{
  unsigned char i = 0;
  unsigned char j = 0;
  unsigned char k = 0;
  unsigned char temp = 0;
  LED_LAT(0);
  LED_SLB(0);
  for(k=0;k<8;k++)
    for(i = 3;i > 0 ;i--)
    {
      temp = Gamma_Value[i-1]<<2;
      for(j = 0;j<6;j++)
      {
        if(temp &0x80)
          LED_SDA(1);
        else
          LED_SDA(0);
        
        temp =temp << 1;
        LED_SCL(0);
        LED_SCL(1);
    }
  }
  LED_SLB(1);
}
void run(unsigned char k)
{
  unsigned char i = 0;
  unsigned char j = 0;
  unsigned char p = 0;
  unsigned char temp = 0;
  LED_SLB(1);
  LED_LAT(0);
  for(i = 0;i<8;i++)
  {
    
    for(j=0;j<3;j++)
    {
      temp = dots[Page_Index][k][i][2-j];
      for(p=0;p<8;p++)
      {
         if(temp & 0x80)
           LED_SDA(1);
         else
           LED_SDA(0);
           
         temp = temp<<1;  
         LED_SCL(0);
         LED_SCL(1);
       }
     }
  }
  LED_LAT(1);
  LED_LAT(0);
}
void open_line(unsigned char x)
{
  switch (x)
  {  
    case 0 :open_line0;
            break;
    case 1 :open_line1;
            break;
    case 2 :open_line2;
            break;
    case 3 :open_line3;
            break;
    case 4 :open_line4;
            break;
    case 5 :open_line5;
            break;
    case 6 :open_line6;
            break;
    case 7 :open_line7;
            break;
    default: close_all_line;
            break;
  }
}

/********************************************************
Name:DispShowColor
Function:Fill a color in LED matrix
Parameter:R: the value of RED.   Range:RED 0~255
          G: the value of GREEN. Range:RED 0~255
          B: the value of BLUE.  Range:RED 0~255
********************************************************/
void DispShowColor(unsigned char R,unsigned char G,unsigned char B)
{
  unsigned char Page_Write,i,j;
  
  if(Page_Index == 0)
    Page_Write = 1;
  if(Page_Index == 1)
    Page_Write = 0;
    
  for (i = 0;i<8;i++)
    for(j = 0;j<8;j++)
    {
      dots[Page_Write][i][j][2] = R;
      dots[Page_Write][i][j][1] = G;
      dots[Page_Write][i][j][0] = B;
    }
  
  Page_Index = Page_Write;
}

/********************************************************
Name:DispSlideInt
Function:Display a number with to digits in LED matrix
Parameter:
          chr: the number want to show
          R: the value of RED.   Range:RED 0~255
          G: the value of GREEN. Range:RED 0~255
          B: the value of BLUE.  Range:RED 0~255
          speed: milliseconds between each step
********************************************************/
void DispSlideInt(int number,unsigned char R,unsigned char G,unsigned char B, int speed) {
  unsigned char i,j,Page_Write,temp;
  unsigned char chrtemp[8] = {0};
  
  int msb = number / 10 % 10;
  int lsb = number % 10;
  
  int msb_next = (number - 1) / 10 % 10;
  int lsb_next = (number - 1)  % 10;
  
  unsigned short col0 = pgm_read_byte(&(font8_3[msb_next][0])) << 8 | pgm_read_byte(&(font8_3[msb][0]));
  unsigned short col1 = pgm_read_byte(&(font8_3[msb_next][1])) << 8 | pgm_read_byte(&(font8_3[msb][1]));
  unsigned short col2 = pgm_read_byte(&(font8_3[msb_next][2])) << 8 | pgm_read_byte(&(font8_3[msb][2]));
  
  unsigned short col5 = pgm_read_byte(&(font8_3[lsb_next][0])) << 8 | pgm_read_byte(&(font8_3[lsb][0]));
  unsigned short col6 = pgm_read_byte(&(font8_3[lsb_next][1])) << 8 | pgm_read_byte(&(font8_3[lsb][1]));
  unsigned short col7 = pgm_read_byte(&(font8_3[lsb_next][2])) << 8 | pgm_read_byte(&(font8_3[lsb][2])); 
  
  chrtemp[3] = 0x00;
  chrtemp[4] = 0x00;
  
  for (int bias = 0; bias < 9; bias++) {
    if(Page_Index == 0)
      Page_Write = 1;
    if(Page_Index == 1)
      Page_Write = 0;
    
    if (msb == msb_next) {
      chrtemp[0] = col0;
      chrtemp[1] = col1;
      chrtemp[2] = col2;
    } else {
      chrtemp[0] = ( col0 ) >> bias;
      chrtemp[1] = ( col1 ) >> bias;
      chrtemp[2] = ( col2 ) >> bias;  
    }
    
    chrtemp[5] = ( col5 ) >> bias;
    chrtemp[6] = ( col6 ) >> bias;
    chrtemp[7] = ( col7 ) >> bias;
    
    for(i = 0;i < 8;i++) {
      temp = chrtemp[i];
      for(j = 0;j < 8;j++) {
        if(temp & 0x80) {
          dots[Page_Write][j][i][0] = R;
          dots[Page_Write][j][i][1] = G;
          dots[Page_Write][j][i][2] = B;
        } else {
          dots[Page_Write][j][i][0] = 0;
          dots[Page_Write][j][i][1] = 0;
          dots[Page_Write][j][i][2] = 0;
        }
        temp = temp << 1;
      }
    }
    Page_Index = Page_Write;
    
    delay(speed);
  }
}

/********************************************************
Name:DispShowInt
Function:Display a number with to digits in LED matrix
Parameter:
          chr: the number want to show
          R: the value of RED.   Range:RED 0~255
          G: the value of GREEN. Range:RED 0~255
          B: the value of BLUE.  Range:RED 0~255
********************************************************/
void DispShowInt(int number,unsigned char R,unsigned char G,unsigned char B) {
  unsigned char i,j,Page_Write,temp;
  unsigned char chrtemp[8] = {0};
  
  int msb = number / 10 % 10;
  int lsb = number % 10;
    
  if(Page_Index == 0)
    Page_Write = 1;
  if(Page_Index == 1)
    Page_Write = 0;
    
  chrtemp[0] = pgm_read_byte(&(font8_3[msb][0]));
  chrtemp[1] = pgm_read_byte(&(font8_3[msb][1]));
  chrtemp[2] = pgm_read_byte(&(font8_3[msb][2]));
  chrtemp[3] = 0x00;
  chrtemp[4] = 0x00;
  chrtemp[5] = pgm_read_byte(&(font8_3[lsb][0]));
  chrtemp[6] = pgm_read_byte(&(font8_3[lsb][1]));
  chrtemp[7] = pgm_read_byte(&(font8_3[lsb][2]));
    
  for(i = 0;i < 8;i++) {
    temp = chrtemp[i];
    for(j = 0;j < 8;j++) {
      if(temp & 0x80) {
        dots[Page_Write][j][i][0] = R;
        dots[Page_Write][j][i][1] = G;
        dots[Page_Write][j][i][2] = B;
      } else {
        dots[Page_Write][j][i][0] = 0;
        dots[Page_Write][j][i][1] = 0;
        dots[Page_Write][j][i][2] = 0;
      }
      temp = temp << 1;
    }
  }
  Page_Index = Page_Write;
}

/********************************************************
Name:DispBlinkInt
Function:Blink a number with to digits in LED matrix
Parameter:
          number: the number want to show
          R: the value of RED.   Range:RED 0~255
          G: the value of GREEN. Range:RED 0~255
          B: the value of BLUE.  Range:RED 0~255
          delay_on: milliseconds of displaying the number
          delay_off: milliseconds of not displaying the number
          count: number how often the given number should blink
********************************************************/
void DispBlinkInt(int number,unsigned char R,unsigned char G,unsigned char B, int delay_on, int delay_off, int count) {
  unsigned char i,j,Page_Write,temp;
  unsigned char chrtemp[8] = {0};
  
  int msb = number / 10 % 10;
  int lsb = number % 10;
  
  for (count; count > 0; count--) {
    
    DispShowInt(number, R, G, B);
    
    delay(delay_on);
    
    DispShowColor(0,0,0);
    
    delay(delay_off);
  }
}

/******************************************
the other operate functions zone
******************************************/
void LED_Delay(unsigned char i) {
  unsigned int y;
  y = i * 10;
  while(y--);
}
/****************************************************
Main Functions zone
****************************************************/
void setup() {
  _IO_Init();           //Init IO
  _LED_Init();          //Init LED Hardware
  _TC2_Init();          //Init Timer/Count2
}

void loop() {
  unsigned int wait = 100;
  
  int count_down = 25;
  
  unsigned long time_before, time_after;
 
  int red = 0;
  int green = 100;
  int blue = 0;
  
  delay(50 * wait);
  
  for (count_down; count_down > 0; count_down--) {
    time_before = millis();
    if (count_down <= 4) {
      red = 100;
      green = 0;
      blue = 0;
    }
    DispSlideInt(count_down, red, green, blue, 30);
    time_after = millis();
    delay( 1000 - (time_after-time_before) );
  }
  
  DispBlinkInt(count_down, red, green, blue, 100, 100, 5);
  
  delay(50 * wait);
}





