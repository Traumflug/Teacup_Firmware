#ifdef LCD
/*
  SimpleLCD.h - Library for interfacing 20x4 LCD character display.
  Created by Raivis Strogonovs, August 6, 2013.
  Released into the public domain.

  Altered by NickE for Teacup
*/
#include <stdint.h>
#include "SimpleLCD.h"
#include "config.h"
#include "delay.h"

void lcdGoToXY(uint8_t x, uint8_t y)
{
  uint8_t addr = 0x00;
  switch(x)
  {
     case 0: addr = 0x00; break; //Starting address of 1st line
     case 1: addr = 0x40; break; //Starting address of 2nd line
     case 2: addr = 0x14; break; //Starting address of 3rd line
     case 3: addr = 0x54; break; //Starting address of 4th line
     default: ;
  }

  addr +=y;

  lcdGoToAddr(addr);
}

void lcdGoToAddr(uint8_t addr)
{
    uint8_t cmd = 0x80 | addr;
    WRITE(LCD_RS_PIN, 0);
    sendCommand4Bit(cmd);
}

void lcdInit()
{
   uint8_t bedTemp[8] =
    {
        0x00,
        0x1F,
        0x15,
        0x11,
        0x15,
        0x1F,
        0x00,
        0x00
    }; //thanks Sonny Mounicou
    uint8_t degree[8] =
    {
        0x0C,
        0x12,
        0x12,
        0x0C,
        0x00,
        0x00,
        0x00,
        0x00
    };
    uint8_t thermometer[8] =
    {
        0x04,
        0x0A,
        0x0A,
        0x0A,
        0x0A,
        0x11,
        0x11,
        0x0E
    };
    uint8_t uplevel[8]={
        0x04,
        0x0E,
        0x1F,
        0x04,
        0x1C,
        0x00,
        0x00,
        0x00
    }; //thanks joris
    uint8_t refresh[8]={
        0x00,
        0x06,
        0x19,
        0x18,
        0x03,
        0x13,
        0x0C,
        0x00,
    }; //thanks joris
    uint8_t folder [8]={
        0x00,
        0x1C,
        0x1F,
        0x11,
        0x11,
        0x1F,
        0x00,
        0x00
    }; //thanks joris
    uint8_t feedrate [8]={
        0x1C,
        0x10,
        0x18,
        0x17,
        0x05,
        0x06,
        0x05,
        0x00
    }; //thanks Sonny Mounicou
    uint8_t clock [8]={
        0x00,
        0x0E,
        0x13,
        0x15,
        0x11,
        0x0E,
        0x00,
        0x00
    }; //thanks Sonny Mounicou

  SET_OUTPUT(LCD_RS_PIN);
  SET_OUTPUT(LCD_EN_PIN);
  SET_OUTPUT(LCD_D4_PIN);
  SET_OUTPUT(LCD_D5_PIN);
  SET_OUTPUT(LCD_D6_PIN);
  SET_OUTPUT(LCD_D7_PIN);


  //Set all the control pins to logic Zero
  WRITE(LCD_RS_PIN, 0);
  WRITE(LCD_EN_PIN, 0);

  //Do the wake up call
  delay_ms(20);
  sendCommand(0x30);
  delay_ms(20);
  sendCommand(0x30);
  delay_ms(20);
  sendCommand(0x30);
  delay_ms(20);
  sendCommand(0x20);  //Let's make it 4 bit mode
  delay_ms(10);
  //That's it LCD is initialized in 4 bit mode.
  sendCommand4Bit(0x28); //N = 1 (2 line display) F = 0 (5x8 characters)
  sendCommand4Bit(0x08); //Display on/off control D=0,C=0, B=0
  sendCommand4Bit(0x01); //Clear Display
  sendCommand4Bit(0x06); //Entry mode set - I/D = 1 (increment cursor) & S = 0 (no shift)
  sendCommand4Bit(0x0C); //Display on/off control. D = 1, C and B = 0. (Cursor and blink, last two bits)


  lcdCreateChar(LCD_STR_BEDTEMP[0], bedTemp);
  lcdCreateChar(LCD_STR_DEGREE[0], degree);
  lcdCreateChar(LCD_STR_THERMOMETER[0], thermometer);
  lcdCreateChar(LCD_STR_UPLEVEL[0], uplevel);
  lcdCreateChar(LCD_STR_REFRESH[0], refresh);
  lcdCreateChar(LCD_STR_FOLDER[0], folder);
  lcdCreateChar(LCD_STR_FEEDRATE[0], feedrate);
  lcdCreateChar(LCD_STR_CLOCK[0], clock);

  lcdClear();
}

void lcdCreateChar(uint8_t location, uint8_t charmap[]){
  uint8_t i;
  location &= 0x7; // we only have 8 locations 0-7
  WRITE(LCD_RS_PIN, 0);
  sendCommand4Bit(0x40 | (location << 3));
  for (i=0; i<8; i++) {
    WRITE(LCD_RS_PIN, 1);
    sendCommand4Bit(charmap[i]);
  }
}

void lcdClear()
{
   WRITE(LCD_RS_PIN, 0);
   sendCommand4Bit(0x01);
}

 void lcdWriteText(char *text)
 {

	while( *text)
	{
    WRITE(LCD_RS_PIN,1);
		sendCommand4Bit(*text++);
	}
 }

 void lcdWriteChar(char *text)
 {
    WRITE(LCD_RS_PIN,1);
    sendCommand4Bit(*text);
 }

void sendCommand(uint8_t opCode)
{
  WRITE(LCD_D4_PIN, opCode & 0x10);
  WRITE(LCD_D5_PIN, opCode & 0x20);
  WRITE(LCD_D6_PIN, opCode & 0x40);
  WRITE(LCD_D7_PIN, opCode & 0x80);

  WRITE(LCD_EN_PIN,1);
  delay_us(1);
  WRITE(LCD_EN_PIN,0);
  delay_us(750);

  WRITE(LCD_D4_PIN, opCode & 0x01);
  WRITE(LCD_D5_PIN, opCode & 0x02);
  WRITE(LCD_D6_PIN, opCode & 0x04);
  WRITE(LCD_D7_PIN, opCode & 0x08);
  WRITE(LCD_EN_PIN,1);
  delay_us(1);
  WRITE(LCD_EN_PIN,0);
  delay_us(750);
}

void sendCommand4Bit(uint8_t opCode)
{
  WRITE(LCD_D4_PIN, opCode & 0x10);
  WRITE(LCD_D5_PIN, opCode & 0x20);
  WRITE(LCD_D6_PIN, opCode & 0x40);
  WRITE(LCD_D7_PIN, opCode & 0x80);
  WRITE(LCD_EN_PIN,1);
  delay_us(5);
  WRITE(LCD_EN_PIN,0);
  delay_us(5);
  WRITE(LCD_D4_PIN, opCode & 0x01);
  WRITE(LCD_D5_PIN, opCode & 0x02);
  WRITE(LCD_D6_PIN, opCode & 0x04);
  WRITE(LCD_D7_PIN, opCode & 0x08);
  WRITE(LCD_EN_PIN,1);
  delay_us(5);
  WRITE(LCD_EN_PIN,0);
  delay_us(750);

}
#endif /* LCD */
