

#include "rot_enc.h"
#include "pinio.h"
#include "config_wrapper.h"

#ifdef ROTARY_ENCODER

// ENCODER_SPEED
#if defined ROT_ENC_SPEED_FULL
const int8_t rot_enc_tv[16] PROGMEM = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0}; // Full speed
#endif
#if defined ROT_ENC_SPEED_HALF
const int8_t rot_enc_tv[16] PROGMEM = {0,0,1,0,0,0,0,-1,-1,0,0,0,0,1,0,0}; // Half speed
#endif
#if defined ROT_ENC_SPEED_QUARTER
const int8_t rot_enc_tv[16] PROGMEM = {0,0,0,0,0,0,0,0,0,0,0,1,0,0,-1,0}; // Quart speed
#endif

void rot_enc_init(void){
	SET_INPUT(ROT_EN1_PIN);
	SET_INPUT(ROT_EN2_PIN);
	SET_INPUT(BTN_ENC_PIN);
	PULLUP_ON(ROT_EN1_PIN);
	PULLUP_ON(ROT_EN2_PIN);
	PULLUP_ON(BTN_ENC_PIN);
}


// Read Encoder state
int8_t rot_enc_read(uint8_t rot_enc_clear) {
   static uint8_t rot_enc_val = 0;  		// this is needed for the encoder previous position
   static int8_t rot_enc_count = 0; 		// this is a local accumulator
   int8_t temp;

   rot_enc_val = (rot_enc_val << 2) & 0x0F;
   
   #if defined ROT_ENC_REVERSE
   	rot_enc_val += (READ(ROT_EN1_PIN) ? 1 : 0) << 1 | (READ(ROT_EN2_PIN) ? 1 : 0);
   #else
    rot_enc_val += (READ(ROT_EN2_PIN) ? 1 : 0) << 1 | (READ(ROT_EN1_PIN) ? 1 : 0);
   #endif
   rot_enc_count += pgm_read_byte(&rot_enc_tv[rot_enc_val]);
  
   temp = rot_enc_count;
   if(rot_enc_clear)
		rot_enc_count =0;
   return temp;
}


// Signal if button was pressed, then released (crude de-bounce)
uint8_t rot_enc_but(uint8_t rot_enc_bclr){
	static uint8_t rot_enc_but_stat =0; // 0 = default; 1=pressed (pre-latch); latch completes when the button is released
	uint8_t temp;
	
	if((READ(BTN_ENC_PIN) ? 0 : 1)) // <- reverse logic, low level means button pressed
		rot_enc_but_stat = 1;		 // pre-latch - button pressed
	else
		if(rot_enc_but_stat == 1)
			rot_enc_but_stat = 2; // reporting key press, release pre-latch
	
	temp = rot_enc_but_stat;
	if(rot_enc_bclr)
		rot_enc_but_stat = 0;
	
	if(temp == 2)
		return 1;
	else
		return 0;
}

#endif /* ROTARY_ENCODER */