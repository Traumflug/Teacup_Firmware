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
//#include "sersendf.h"

uint8_t prevPos = 0; //Previous cursor position to be replaced by empty space

//uint8_t selection = 0;
uint8_t Pos = 0;
uint8_t menuPos = 0;
uint8_t curStart; //cursor start position
uint8_t curEnd; //cursor end position

/*
int8_t itemLevel0 = -1; //-1 = not yet selected
int8_t itemLevel1 = -1;
int8_t itemLevel2 = -1;
*/

//which 4 items in the menu are displayed at a time
uint8_t lcdStartLn = 0;
uint8_t lcdEndLn = 3;

uint8_t curScreen = 0; //variable for selct function
uint8_t prevScreen = 0; //variable for back function

//Menu UI
static char* mainScreen[] = {" Watch","Control","SD Card","Settings","shift test"}; //need to save setting in EEPROM //0
  static char* watchScreen[] = {" X:000.00 Ext:000 C","Y:000.00 Bed:000 C","Z:000.00","Pause/Resume print","back"}; //1
  static char* controlScreen[] = {" Home","Jog","Heat","Cooldown","back"}; //2
    static char* homeScreen[] = {" Home All","X Axis","Y Axis","Z Axis","Homw All Max","X Max","Y Max","Z Max","back"}; //5
    static char* jogScreen[] = {" K Axis","Step Size xxxmm","+     000.00      -","back"}; //step size either 0.1/1/10 mm //6
    static char* heatScreen[] = {" Extruder","T: 000°C C:000°C","Bed","back"}; //7
  static char* sdScreen[] = {" Print from SD","Mount/Unmount Card","back"}; //3
    static char* cardPrintScreen[] = {" Select File","Start Print","Pause Print","Stop Print","back"}; //8
  static char* settingsScreen[] = {" Cool Down Conf","Beeper           ON","Auto Home        ON","System Information","back"}; //4
    static char* coolDnConfScreen[] = {" Fan Cutoff Temp","000 C","back"}; //9
    static char* sysInfoScreen[] = {" Teacup Firmware","Firmware Ver: 1.0","LCD Menu Ver: 1.0","Print Lifetime:0000h","back"}; //need to securely save total print time in EEPROM //10

void splashScreen (){ // should add teacup logo later and display printer name
    lcdClear();
    lcdWriteText((uint8_t *)"->Teacup LCD Init<-");
    lcdGoToAddr(0x54);
    lcdWriteText((uint8_t *)"Teacup Firmware");
    delay_ms(1000);
    lcdClear();
}

//Encoder Stuff
void select (){ //uint8_t Pos){  
   //lcdStartLn=0;
   //lcdEndLn=3;
   
   switch(curScreen){
     case 0: switch(menuPos){ //Main
       case 0: disp(1); break; //display watch screen
       case 1: disp(2); break; // control screen
       case 2: disp(3); break; // sd card menu
       case 3: disp(4); break; // settings screen
     } break;
     
     //case 1: switch(Pos){ //Watch
       //case x: break; //pause/play
       //case x: back(); break;
     //} break;
     /*
     case 2: switch(menuPos){ //Control
       case 0: disp(5); break; //Home
       case 1: disp(6); break; //Jog
       case 2: disp(7); break; //Heat
       //case 3: ; break; //cooldown
       //case x: back(); break;
     } break;
     
     case 3: switch(menuPos){ //SD
       case 0: disp(8); break;
       //case 1: disp(2); break; //mount/unmount
       //case 2: back(); break; //back
     } break;
     case 4: switch(menuPos){ //Settings
       case 0: disp(9); break;
       //case 1: ; break; //toggle beeper
       case 2: disp(10); break;
       //case 3: back(); break; //back
     } break;
     /*
     case 5: switch(Pos){
       case 0: disp(1); break;
       case 1: disp(2); break;
       case 2: disp(3); break;
       case 3: disp(4); break; //back
     } break;
     case 6: switch(Pos){
       case 0: disp(1); break; //move through axes
       case 1: disp(2); break; //move through step size
       case 2: disp(3); break; //move position
       case 3: disp(4); break; //back
     } break;
     case 7: switch(Pos){
       case 0: disp(1); break;
       case 1: disp(2); break;
       case 2: disp(3); break;
       case 3: disp(4); break;
     } break;
     case 8: switch(Pos){
       case 0: disp(1); break;
       case 1: disp(2); break;
       case 2: disp(3); break;
       case 3: disp(4); break;
     } break;
     case 9: switch(Pos){
       case 0: disp(1); break;
       case 1: disp(2); break;
       case 2: disp(3); break;
       case 3: disp(4); break;
     } break;
     case 10: switch(Pos){
       case 0: disp(1); break;
       case 1: disp(2); break;
       case 2: disp(3); break;
       case 3: disp(4); break;
     } break;
     //*/
   }

    //selection = 1;
    menuPos = 0;
   
}

void encCursor (int8_t pos){ //uint8_t start, uint8_t end, uint8_t pos){ // function to draw the cursor and move up or down it when the encoder detects motion
    //increment position in current menu
    if(pos>prevPos){menuPos=menuPos+1;}
    if(pos<prevPos){menuPos=menuPos-1;}
    if(menuPos>10){menuPos=10;}
    
    //Limit cursor postions to available entries on screen and shift screen up and down if cursor reaches the end  
    if(pos<curStart){/*shiftUp();*/ pos=curStart;}
    if(pos>curEnd){/*shiftDown();*/ pos=curEnd;}
    
    if(pos != prevPos){
      lcdGoToXY(0,prevPos);
      lcdWriteText(" "); //clear previous position of cursor
    }
    lcdGoToXY(0,pos);
    lcdWriteText(">"); //cursor shape, change it according to preference
    
    menuPos = pos;
    prevPos = pos;
}
///*
//Display Stuff
void shiftUp(){
  lcdStartLn = lcdStartLn+1;
  lcdEndLn = lcdEndLn+1;
  refresh();
}

void shiftDown(){
  lcdStartLn = lcdStartLn-1;
  lcdEndLn = lcdEndLn-1;
  refresh();
}

void disp(uint8_t screen){
  curScreen = screen;
  
  lcdClear();
  
  switch(screen){
    //Level 0
    case 0: curStart=0; curEnd=3; lcdStartLn=0; lcdEndLn=3; dispMainScreen(); break; //no shift
    //Level 1
    case 1: curStart=0; curEnd=3; dispWatchScreen(); break;
    case 2: curStart=0; curEnd=3; dispControlScreen(); break;
    case 3: curStart=0; curEnd=2; lcdStartLn=0; lcdEndLn=2; dispSdScreen(); break; //no shift
    case 4: curStart=0; curEnd=3; dispSettingsScreen(); break;
    //Level 2
    case 5: curStart=0; curEnd=3; dispHomeScreen(); break;
    case 6: curStart=0; curEnd=3; dispJogScreen(); break;
    case 7: curStart=0; curEnd=3; dispHeatScreen(); break;
    case 8: curStart=0; curEnd=3; dispCardPrintScreen(); break;
    case 9: curStart=0; curEnd=3; dispCoolDnConfScreen(); break;
    case 10: curStart=0; curEnd=3; dispSysInfoScreen(); break;
  }
  prevScreen = screen;
}

void refresh(){disp(curScreen);}

void back(){disp(prevScreen);}

//*/
//Menu Stuff
void dispMainScreen (){
    uint8_t i;
    for(i = lcdStartLn; i<=lcdEndLn; i++){
      lcdGoToXY(1,i);
      lcdWriteText((uint8_t *)mainScreen[i]);
    }
}

void dispWatchScreen (){
    uint8_t i;
    for(i = lcdStartLn; i<=lcdEndLn; i++){
      lcdGoToXY(1,i);
      lcdWriteText((uint8_t *)watchScreen[i]);
    }
}

void dispControlScreen (){
    uint8_t i;
    for(i = lcdStartLn; i<=lcdEndLn; i++){
      lcdGoToXY(1,i);
      lcdWriteText((uint8_t *)controlScreen[i]);
    }
}

void dispHomeScreen (){
    uint8_t i;
    for(i = lcdStartLn; i<=lcdEndLn; i++){
      lcdGoToXY(1,i);
      lcdWriteText((uint8_t *)homeScreen[i]);
    }
}

void dispJogScreen (){
    uint8_t i;
    for(i = lcdStartLn; i<=lcdEndLn; i++){
      lcdGoToXY(1,i);
      lcdWriteText((uint8_t *)jogScreen[i]);
    }
}

void dispHeatScreen (){
    uint8_t i;
    for(i = lcdStartLn; i<=lcdEndLn; i++){
      lcdGoToXY(1,i);
      lcdWriteText((uint8_t *)heatScreen[i]);
    }
}

void dispSdScreen (){
    uint8_t i;
    for(i = lcdStartLn; i<=lcdEndLn; i++){
      lcdGoToXY(1,i);
      lcdWriteText((uint8_t *)sdScreen[i]);
    }
}

void dispCardPrintScreen (){
    uint8_t i;
    for(i = lcdStartLn; i<=lcdEndLn; i++){
      lcdGoToXY(1,i);
      lcdWriteText((uint8_t *)cardPrintScreen[i]);
    }
}

void dispSettingsScreen (){
    uint8_t i;
    for(i = lcdStartLn; i<=lcdEndLn; i++){
      lcdGoToXY(1,i);
      lcdWriteText((uint8_t *)settingsScreen[i]);
    }
}

void dispCoolDnConfScreen (){
    uint8_t i;
    for(i = lcdStartLn; i<=lcdEndLn; i++){
      lcdGoToXY(1,i);
      lcdWriteText((uint8_t *)coolDnConfScreen[i]);
    }
}

void dispSysInfoScreen (){
    uint8_t i;
    for(i = lcdStartLn; i<=lcdEndLn; i++){
      lcdGoToXY(1,i);
      lcdWriteText((uint8_t *)sysInfoScreen[i]);
    }
}

#endif
