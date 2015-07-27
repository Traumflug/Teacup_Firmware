#include "config_wrapper.h"

#ifndef SIMULATOR
  #include <avr/pgmspace.h>
#else
  #define PROGMEM
#endif

#ifdef LCD
#include "SimpleLCD.h"
#include "clock.h"
#include "lcdMenu.h"
#include "delay.h"

uint8_t prevPos;
//#define encCursorLine

#define mainScreen[] {PSTR("Information","Prepare","Control","SD","Back")}
#define mainScreenMax 4
//#define infoScreen[]{PSTR("X","Y","Z",)}

void splashScreen (){
    lcdClear();
    lcdWriteText((uint8_t *)"->Teacup LCD Init<-");
    lcdGoToAddr(0x54);
    lcdWriteText((uint8_t *)"Teacup Firmware");
    delay_ms(1000);
    lcdClear();
}


void encCursor (uint8_t pos){
    //lcdClear();
    if(pos != prevPos){
      lcdGoToXY(0,prevPos);
      lcdWriteText(" ");
    }
    lcdGoToXY(0,pos);
    lcdWriteText(">");
    prevPos = pos;
}


void disp (){
    
}




#endif

