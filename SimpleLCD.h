/*
  SimpleLCD.h - Library for interfacing 20x4 LCD character display.
  Created by Raivis Strogonovs, August 6, 2013.
  Released into the public domain.

  Altered by NickE for Teacup
*/
#include <stdint.h>

#ifndef SimpleLCD_h
#define SimpleLCD_h

/* Custom characters defined in the first 8 characters of the LCD
Lifted directly from ultralcd_implementation_hitachi_HD44780.h in Marlin*/
#define LCD_STR_BEDTEMP     "\x00"
#define LCD_STR_DEGREE      "\x01"
#define LCD_STR_THERMOMETER "\x02"
#define LCD_STR_UPLEVEL     "\x03"
#define LCD_STR_REFRESH     "\x04"
#define LCD_STR_FOLDER      "\x05"
#define LCD_STR_FEEDRATE    "\x06"
#define LCD_STR_CLOCK       "\x07"
#define LCD_STR_ARROW_RIGHT "\x7E"  /* from the default character set */

void lcdGoToXY(uint8_t x, uint8_t y);
void lcdGoToAddr(uint8_t addr);
void lcdInit();
void lcdClear();
void lcdWriteText(char *text);
void lcdWriteChar(char *text);
void lcdBusy();
void sendCommand(uint8_t opCode);
void sendCommand4Bit(uint8_t opCode);
void lcdCreateChar(uint8_t location, uint8_t charmap[]);

#endif
