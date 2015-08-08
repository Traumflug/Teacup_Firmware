#include "config_wrapper.h"

#ifdef LCD

//Start up screen
void splashScreen(void);

//screen control
void encCursor(/*uint8_t start, uint8_t end,*/ int8_t pos);
void shiftUp(void);
void shiftDown(void);
void select();//uint8_t Pos);
void disp(uint8_t screen);
void refresh(void);
void back(void);

//Menu
static char* mainScreen[];
  static char* watchScreen[];
  static char* controlScreen[];
    static char* homeScreen[];
    static char* jogScreen[];
    static char* heatScreen[];
    static char* coolDown[];
  static char* sdScreen[];
  static char* settingsScreen[];
    static char* coolDnScreen[];
    static char* sysInfoScreen[];

void dispMainScreen(void);
  void dispWatchScreen(void);
  void dispControlScreen(void);
    void dispHomeScreen(void);
    void dispJogScreen(void);
    void dispHeatScreen(void);
  void dispSdScreen(void);
    void dispCardPrintScreen(void);
  void dispSettingsScreen(void);
    void dispCoolDnConfScreen(void);
    void dispSysInfoScreen(void);

#endif
