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

uint8_t prevPos = 0;
//#define encCursorLine

static char* mainScreen[] = {" Information","Prepare","Control","SD"};

void splashScreen (){
    lcdClear();
    lcdWriteText((uint8_t *)"->Teacup LCD Init<-");
    lcdGoToAddr(0x54);
    lcdWriteText((uint8_t *)"Teacup Firmware");
    delay_ms(1000);
    lcdClear();
}


void encCursor (uint8_t pos){
    if(pos != prevPos){
      lcdGoToXY(0,prevPos);
      lcdWriteText(" ");
    }
    lcdGoToXY(0,pos);
    lcdWriteText(">");
    prevPos = pos;
}


void disp (){
    lcdClear();
    uint8_t i;
    for(i = 0; i<=3; i++){
      lcdGoToXY(1,i);
      lcdWriteText((uint8_t *)mainScreen[i]);
    }
}




#endif

