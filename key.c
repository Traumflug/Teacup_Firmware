/*
  Key for debouncing keys

  Most code comes from https://www.mikrocontroller.net/articles/Entprellung#Timer-Verfahren_.28nach_Peter_Dannegger.29
  Excellent work and very simple to implement into Teacup
*/

#include "key.h"
#include "pinio.h"
// #include "sersendf.h"
#include "memory_barrier.h"

#define S1_PIN DIO11
#define S2_PIN DIO12
#define L1_PIN DIO3
#define B1_PIN DIO9
#define B2_PIN DIO10

// #define LED1_PIN DIO6
// #define LED2_PIN DIO7
// #define LED3_PIN DIO8

#define REPEAT_MASK (KEY_3 | KEY_4)   // add keys for repeat function
#define REPEAT_START 50               // after 500ms
#define REPEAT_NEXT 20                // every 200ms

volatile uint8_t key_state;   // debounced and inverted key state:
                              // bit = 1: key pressed
volatile uint8_t key_press;   // key press detect
volatile uint8_t key_rpt;     // key long press and repeat

volatile uint8_t keys;

void key_init() {
  SET_INPUT(S1_PIN);
  PULLUP_ON(S1_PIN);

  SET_INPUT(S2_PIN);
  PULLUP_ON(S2_PIN);

  SET_INPUT(B1_PIN);
  PULLUP_ON(B1_PIN);

  SET_INPUT(B2_PIN);
  PULLUP_ON(B2_PIN);

  SET_INPUT(L1_PIN);
  PULLUP_ON(L1_PIN);

  // SET_OUTPUT(LED1_PIN);
  // SET_OUTPUT(LED2_PIN);
  // SET_OUTPUT(LED3_PIN);
}

/*
  when you need more than 8 keys you need to add them in key.h and
  change uint8_t for any vars below from uint8_t to uint16_t
*/
void key_clock() {
  static uint8_t ct0 = 0xFF, ct1 = 0xFF, rpt;
  uint8_t i;
  uint8_t key_read;


  key_read = (READ(S1_PIN) ? KEY_1 : 0) |
             (READ(S2_PIN) ? KEY_2 : 0) |
             (READ(B1_PIN) ? KEY_3 : 0) |
             (READ(B2_PIN) ? KEY_4 : 0) |
             (READ(L1_PIN) ? KEY_5 : 0);
  // sersendf_P(PSTR("%sx\n"), key_read);
  i = key_state ^ ~key_read;            // key changed?
  ct0 = ~(ct0 & i);                     // reset or count ct0
  ct1 = ct0 ^ (ct1 & i);                // reset or count ct1
  i &= ct0 & ct1;                       // count until roll over?
  key_state ^= i;                       // then toggle debounce state
  key_press |= key_state & i;           // 0->1: key press detect

  if ((key_state & REPEAT_MASK) == 0)   // check repeat function
    rpt = REPEAT_START;                 // start delay
  if (--rpt == 0) {
    rpt = REPEAT_NEXT;                  // repeat delay
    key_rpt |= key_state & REPEAT_MASK;
  }
}

/*
  check if a key has been pressed
  each pressed key is reported only once
*/
uint8_t get_key_press(uint8_t key_mask) {
  ATOMIC_START                  // read and clear atomic!
  key_mask &= key_press;        // read key(s)
  key_press ^= key_mask;        // clear key(s)
  ATOMIC_END
  return key_mask;
}

/*
  check if a key has been pressed long enough such that the
  key repeat functionality kicks in. After a small setup delay
  the key is reported being pressed in subsequent calls
  to this function. This simulates the user repeatedly
  pressing and releasing the key.
*/
uint8_t get_key_rpt(uint8_t key_mask) {
  ATOMIC_START                  // read and clear atomic!
  key_mask &= key_rpt;          // read key(s)
  key_rpt ^= key_mask;          // clear key(s)
  ATOMIC_END
  return key_mask;
}

/*
  check if a key is pressed right now
*/
uint8_t get_key_state(uint8_t key_mask) {
  key_mask &= key_state;
  return key_mask;
}

// we have a short key press?
uint8_t get_key_short(uint8_t key_mask) {
  ATOMIC_START              // read key state and key press atomic!
                            // little bit repeated code of get_key_press
                            // because we can't start with ATOMIC here and
                            // end this later in get_key_press
  key_mask = ~key_state & key_mask;
  key_mask &= key_press;
  key_press ^= key_mask;
  ATOMIC_END
  return key_mask;
}

// we have a long key press?
uint8_t get_key_long(uint8_t key_mask) {
  return get_key_press(get_key_rpt(key_mask));
}

// we press two keys?
uint8_t get_key_common(uint8_t key_mask) {
  return get_key_press((key_press & key_mask) == key_mask ? key_mask : 0);
}

void key_tick() {
  /* 
    some debugging can be found here
    also you can see some examples how your keys could work
  */
  // static uint8_t toggle;
  // if (get_key_short(KEY_3)) {
  //   toggle ^= 0x01;
  //   sersendf_P(PSTR("st: %sx\n"), toggle);
  // }

  // if (get_key_long(KEY_3)) {
  //   toggle ^= 0x01;
  //   sersendf_P(PSTR("lt: %sx\n"), toggle);
  // }

  // if (get_key_press(KEY_4) || get_key_rpt(KEY_4)) {
  //   toggle ^= 0x02;
  //   sersendf_P(PSTR("rt: %sx\n"), toggle);
  // }

  // WRITE(LED1_PIN, toggle & 0x01);
  // WRITE(LED2_PIN, toggle & 0x02);

  // WRITE(LED3_PIN, get_key_state(KEY_5));

  if (get_key_short(KEY_3))
    keys ^= KEY_3;

  if (get_key_long(KEY_3))
    keys ^= KEY_3 << 8;

  if (get_key_state(KEY_5))
    keys |= KEY_5;
  else
    keys &= ~KEY_5;

}
