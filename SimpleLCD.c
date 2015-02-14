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
  uint8_t addr;
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
}

void sendCommand4Bit(uint8_t opCode)
{
  WRITE(LCD_D4_PIN, opCode & 0x10);
  WRITE(LCD_D5_PIN, opCode & 0x20);
  WRITE(LCD_D6_PIN, opCode & 0x40);
  WRITE(LCD_D7_PIN, opCode & 0x80);
  WRITE(LCD_EN_PIN,1);
  WRITE(LCD_EN_PIN,0);
  WRITE(LCD_D4_PIN, opCode & 0x01);
  WRITE(LCD_D5_PIN, opCode & 0x02);
  WRITE(LCD_D6_PIN, opCode & 0x04);
  WRITE(LCD_D7_PIN, opCode & 0x08);
  WRITE(LCD_EN_PIN,1);
  WRITE(LCD_EN_PIN,0);
  delay_ms(2);
}
