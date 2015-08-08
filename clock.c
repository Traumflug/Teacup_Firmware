#include	"clock.h"

/** \file
	\brief Do stuff periodically
*/

#include	"pinio.h"
#include	"sersendf.h"
#include	"dda_queue.h"
#include	"watchdog.h"
#include	"timer.h"
#include	"debug.h"
#include	"heater.h"
#include	"serial.h"
#include        "temp.h"
#ifdef	TEMP_INTERCOM
	#include	"intercom.h"
#endif
#include	"memory_barrier.h"
#include "SimpleLCD.h"
#include "lcdMenu.h"

/// Every time our clock fires we increment this,
/// so we know when 10ms has elapsed.
uint8_t clock_counter_10ms = 0;
/// Keep track of when 250ms has elapsed.
uint8_t clock_counter_250ms = 0;
/// Keep track of when 1s has elapsed.
uint8_t clock_counter_1s = 0;
/// Keep track of when 3s has elapsed.
uint8_t clock_counter_3s = 0;

/// Flags to tell main loop when above have elapsed.
volatile uint8_t clock_flag_10ms = 0;
volatile uint8_t clock_flag_250ms = 0;
volatile uint8_t clock_flag_1s = 0;
volatile uint8_t clock_flag_3s = 0;

unsigned char enc_C = 1; // variable for the encoder click, 1 by default as it goes low when clicked
unsigned char enc_A; // variables for the encoder pins
unsigned char enc_B;
unsigned char enc_A_prev = 0;
uint8_t count = 0; //position in the menu itself
int8_t pos = 0; //position on screen
uint8_t prevcnt = 0; //previuos count
uint8_t debounce_counter_enc_C = 0; //debounce testing
uint8_t enc_C_STEPS = 4;
//uint8_t start = 0,end = 3;
/** Advance our clock by a tick.

  Update clock counters accordingly. Should be called from the TICK_TIME
  Interrupt in timer.c.
*/
void clock_tick(void) {

  clock_counter_10ms += TICK_TIME_MS;
  if (clock_counter_10ms >= 10) {
    clock_counter_10ms -= 10;
    clock_flag_10ms = 1;

    clock_counter_250ms++;
    if (clock_counter_250ms >= 25) {
      clock_counter_250ms = 0;
      clock_flag_250ms = 1;

      clock_counter_1s++;
      if (clock_counter_1s >= 4) {
        clock_counter_1s = 0;
        clock_flag_1s = 1;
      }
	  clock_counter_3s++;
	  if (clock_counter_3s >= 12) {
		  clock_counter_3s = 0;
		  clock_flag_3s = 1;
	  }
    }
  }
}

/*!	do stuff every 1/4 second

	called from clock_10ms(), do not call directly
*/
static void clock_250ms(void) {





  if (heaters_all_zero()) {
		if (psu_timeout > (30 * 4)) {
			power_off();
		}
		else {
      ATOMIC_START
        psu_timeout++;
      ATOMIC_END
		}
	}

	ifclock(clock_flag_1s) {
		#ifndef LCD //ifndef to disable this module to enable me to work on other parts
		   if (queue_empty() != 0) {
		      update_current_position();
                       lcdGoToXY(1,0);
                       lcdsendf_P(PSTR("X"));
                       lcdGoToXY(2,0);
                       lcdsendf_P(PSTR("%lq"),current_position.axis[X]);
                       #ifdef HEATER_EXTRUDER
                           //lcdGoToAddr(0xE);
                           //lcdsendf_P(PSTR("      "));
                           lcdGoToXY(10,0);
                           lcdsendf_P(PSTR("Ext:"));
                           temp_lcd(HEATER_EXTRUDER);
                       #endif
                       lcdGoToAddr(0x41);
                       lcdsendf_P(PSTR("Y      "));
                       lcdGoToAddr(0x42);
                       lcdsendf_P(PSTR("%lq"),current_position.axis[Y]);
                       #ifdef HEATER_BED
                           lcdGoToAddr(0x4E);
                           lcdsendf_P(PSTR("      "));
                           lcdGoToAddr(0x4A);
                           lcdsendf_P(PSTR("Bed:"));
                           temp_lcd(HEATER_BED);
                       #endif
                       lcdGoToAddr(0x15);
                       lcdsendf_P(PSTR("Z      "));
                       lcdGoToAddr(0x16);
                       lcdsendf_P(PSTR("%lq"),current_position.axis[Z]);
			lcdGoToAddr(0x1E);//  display on screen for the pos variable
			 lcdsendf_P(PSTR("P      "));
			lcdGoToAddr(0x1F);
			lcdsendf_P(PSTR("%su"),pos);
                        
                        lcdGoToXY(13,2);//  display on screen for the pos variable
			 lcdsendf_P(PSTR("C      "));
			lcdGoToXY(14,2);
			lcdsendf_P(PSTR("%su"),count);
		        
		    




		   }
		#endif
		if (DEBUG_POSITION && (debug_flags & DEBUG_POSITION)) {
			// current position
			update_current_position();
      sersendf_P(PSTR("Pos: %lq,%lq,%lq,%lq,%lu\n"), current_position.axis[X], current_position.axis[Y], current_position.axis[Z], current_position.axis[E], current_position.F);

			// target position
      sersendf_P(PSTR("Dst: %lq,%lq,%lq,%lq,%lu\n"), movebuffer[mb_tail].endpoint.axis[X], movebuffer[mb_tail].endpoint.axis[Y], movebuffer[mb_tail].endpoint.axis[Z], movebuffer[mb_tail].endpoint.axis[E], movebuffer[mb_tail].endpoint.F);

			// Queue
			print_queue();

			// newline
			serial_writechar('\n');
		}
		// temperature
		/*		if (temp_get_target())
		temp_print();*/
	}
	ifclock(clock_flag_3s) {
		#ifndef LCD //ifndef to disable this module to enable me to work on other parts
			#ifdef HEATER_EXTRUDER
			lcdGoToAddr(0xE);
			lcdsendf_P(PSTR("      "));
			lcdGoToAddr(0x0A);
			lcdsendf_P(PSTR("Ext:"));
			temp_lcd(HEATER_EXTRUDER);
			#endif
			#ifdef HEATER_EXTRUDER
			lcdGoToAddr(0x4E);
			lcdsendf_P(PSTR("      "));
			lcdGoToAddr(0x4A);
			lcdsendf_P(PSTR("Bed:"));
			temp_lcd(HEATER_BED);
			#endif
		#endif
                
	}
	#ifdef	TEMP_INTERCOM
	start_send();
	#endif
}

/*! do stuff every 10 milliseconds

	called from clock(), do not call directly
*/
static void clock_10ms(void) {


	
	// reset watchdog
	wd_reset();

	temp_sensor_tick();

	ifclock(clock_flag_250ms) {
		clock_250ms();
	}
}

/*! do reoccuring stuff

	call it occasionally in busy loops
*/
void clock() {
        
      #ifdef ENCODER
	enc_A = READ(BTN_EN1);//=======================   code for rotary encoder
	enc_B = READ(BTN_EN2);
        
        if(pos>3){pos = 3;} //will add menu shift up/down 
        if(pos<0){pos = 0;}
        
	if((!enc_A)&&(enc_A_prev)){
		if(enc_B){
			pos=pos-1;
                        count=count-1;	
		}
		else	
		{
			pos=pos+1;
                        count=count+1;
		}
            
  
        }
        
enc_A_prev=enc_A;//============  code for rotary encoder

	
	ifclock(clock_flag_10ms) {
		clock_10ms();
	}

          encCursor(/*start,end,*/pos);

          //read encoder psuh button, debounce it and make selection
          enc_C = READ(BTN_ENC);
          if(enc_C == 0){debounce_counter_enc_C++;}else{debounce_counter_enc_C = 0;}
          if(debounce_counter_enc_C >= enc_C_STEPS){select(); pos=0;}//pos);}
                    
#endif //encoder       
            
#ifdef SIMULATOR
  sim_time_warp();
#endif
}

void setCursorLim(uint8_t start,uint8_t end){start=start; end=end;}


