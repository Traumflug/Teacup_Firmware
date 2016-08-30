#include <stdint.h>

#define KEY_1 0x01
#define KEY_2 0x02
#define KEY_3 0x04
#define KEY_4 0x08
#define KEY_5 0x10

void key_init(void);
void key_clock(void);
void key_tick(void);

extern volatile uint8_t keys;
// simply use this macro to add functionality to it in any code
// if_key(KEY_1) { do_something } else { do_somethingelse }
#define if_key(key) if (! (keys & key))