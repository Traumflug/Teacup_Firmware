#include "config_wrapper.h"

#ifdef LCD

//Start up screen
void splashScreen(void);

//screen control
void encCursor(uint8_t pos);
void shiftUp(void);
void shiftDn(void);
void click(void);

//Menu
static char* mainScreen[];
/*
  static char* informationScreen[];
  static char* prepareScreen[];
  static char* controlScreen[];
  static char* sdScreen[];
*/

void disp(void);



#endif
