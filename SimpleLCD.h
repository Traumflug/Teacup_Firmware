/*
  SimpleLCD.h - Library for interfacing 20x4 LCD character display.
  Created by Raivis Strogonovs, August 6, 2013.
  Released into the public domain.

  Altered by NickE for Teacup
*/
#include <stdint.h>

#ifndef SimpleLCD_h
#define SimpleLCD_h


void lcdGoToXY(uint8_t x, uint8_t y);
void lcdGoToAddr(uint8_t addr);
void lcdInit();
void lcdClear();
void lcdWriteText(char *text);
void lcdWriteChar(char *text);
void lcdBusy();
void sendCommand(uint8_t opCode);
void sendCommand4Bit(uint8_t opCode);

#endif
