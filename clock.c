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


/// Every time our clock fires we increment this,
/// so we know when 10ms has elapsed.
uint8_t clock_counter_10ms = 0;
/// Keep track of when 250ms has elapsed.
uint8_t clock_counter_250ms = 0;
/// Keep track of when 1s has elapsed.
uint8_t clock_counter_1s = 0;
/// Keep track of when 1s has elapsed.
uint8_t clock_counter_3s = 0;

/// Flags to tell main loop when above have elapsed.
volatile uint8_t clock_flag_10ms = 0;
volatile uint8_t clock_flag_250ms = 0;
volatile uint8_t clock_flag_1s = 0;
volatile uint8_t clock_flag_3s = 0;

unsigned char enc_A; // variables for the encoder pins
unsigned char enc_B;
unsigned char enc_A_prev=0;
unsigned char enc_B_prev=0;
uint16_t count=0;
uint8_t prevcnt=0;
uint8_t butdwn=0;
uint8_t prvbutdwn=0;
uint8_t flimit=0;


static uint8_t listingfiles=0;


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
		#ifdef LCD
		   if (queue_empty() != 0) {
			if (listingfiles==0){
		      update_current_position();
                       lcdGoToAddr(0x00);
                       lcdsendf_P(PSTR("X       "));
                       lcdGoToAddr(0x01);
                       lcdsendf_P(PSTR("%lq"),
		                 current_position.axis[X]);
                       #ifdef HEATER_EXTRUDER
                           lcdGoToAddr(0xD);
                           lcdsendf_P(PSTR("       "));
                           lcdGoToAddr(0x09);
                           lcdsendf_P(PSTR("Ext:"));
                           temp_lcd(HEATER_EXTRUDER);
                       #endif
                       lcdGoToAddr(0x40);
                       lcdsendf_P(PSTR("Y       "));
                       lcdGoToAddr(0x41);
                       lcdsendf_P(PSTR("%lq"),
		                 current_position.axis[Y]);
                       #ifdef HEATER_EXTRUDER
                           lcdGoToAddr(0x4D);
                           lcdsendf_P(PSTR("       "));
                           lcdGoToAddr(0x49);
                           lcdsendf_P(PSTR("Bed:"));
                           temp_lcd(HEATER_BED);
                       #endif
                       lcdGoToAddr(0x14);
                       lcdsendf_P(PSTR("Z       "));
                       lcdGoToAddr(0x15);
                       lcdsendf_P(PSTR("%lq"),
		                 current_position.axis[Z]);
			
			}

			

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
		#ifdef LCD
			#ifdef HEATER_EXTRUDER
			if (listingfiles==0){
			lcdGoToAddr(0xD);
			lcdsendf_P(PSTR("       "));
			lcdGoToAddr(0x09);
			lcdsendf_P(PSTR("Ext:"));
			temp_lcd(HEATER_EXTRUDER);


			#endif
			#ifdef HEATER_EXTRUDER
			lcdGoToAddr(0x4D);
			lcdsendf_P(PSTR("       "));
			lcdGoToAddr(0x49);
			lcdsendf_P(PSTR("Bed:"));
			temp_lcd(HEATER_BED);
			#endif
			}
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
//this is my extremely simple menu system for teacup, originally i had ideas of a scrolling system similar to marlin,
			//however it is desirable to keep it as simple as possible while maintaining the widest range of function as needed
			//for the system to run in standalone operation - jgrjgr

			//todo - overflow handling for the variable count
			
			if (prevcnt!=count){// avoid writing to screen unnessessarily
			menuprocess();
			prevcnt=count;// avoid writing to screen unnessessarily
			}

}

/*! do reoccuring stuff

	call it occasionally in busy loops
*/
void clock() {


// todo this code needs fixing so that the encoder is read when it click on both channels, current it's 2 clicks per movement
// which is livable but annoying after a while
// also needs to be wrapped in a if define specific to the reprapdiscount smart lcd
// also on the todo list is to add more generic code for simple buttons eg up/down/enter like thos found on a lot of arduino lcd shields

	enc_A = READ(BTN_EN1);//=======================   code for rotary encoder
	enc_B = READ(BTN_EN2);
	if((!enc_A)&&(enc_A_prev)){
		if(enc_B){

			if(count!=0){
				count=count-1;
			}
				
		}
		else	
		{

			if(listingfiles==0){
				if(count!=7){// limiter for the main menu
				count=count+1;
				}
			}
			else
			{
				count=count+1;
			}

			
			
		}


	}



enc_A_prev=enc_A;//============  code for rotary encoder

if (READ(BTN_ENC)){// code for the button of the rotary encoder
				//lcdGoToAddr(0x54); 
				//lcdWriteText((uint8_t *)"!");
				butdwn=0;

}
else
{
				//lcdGoToAddr(0x54); 
				//lcdWriteText((uint8_t *)"?");
				butdwn=1;

}


if (prvbutdwn==1){
	if(butdwn==0){
	menuclick();
}
}
prvbutdwn=butdwn;// code for the button on the rotary encoder


	
	ifclock(clock_flag_10ms) {
		clock_10ms();
	}
#ifdef SIMULATOR
  sim_time_warp();
#endif
}
void menuprocess(){
			lcdGoToAddr(0x1D);//  display on screen for the count variable
			lcdsendf_P(PSTR("C       "));
			lcdGoToAddr(0x1E);
			lcdsendf_P(PSTR("%su"),count);
if(listingfiles==0){
	if(count==0){
		lcdGoToAddr(0x54); 
		lcdWriteText((uint8_t *)"Run Gcode File     >");
			
	}
	if(count==1){
		lcdGoToAddr(0x54); 
		lcdWriteText((uint8_t *)"Preheat Extruder  <>");
	}
	if(count==2){
		lcdGoToAddr(0x54); 
		lcdWriteText((uint8_t *)"Preheat Bed       <>");
			
	}
	if(count==3){
		lcdGoToAddr(0x54); 
		lcdWriteText((uint8_t *)"Shutdown Steppers <>");
			
	}
	if(count==4){
		lcdGoToAddr(0x54); 
		lcdWriteText((uint8_t *)"Shutdown Heaters  <>");
			
	}
	if(count==5){
		lcdGoToAddr(0x54); 
		lcdWriteText((uint8_t *)"Zero X Y Z E      <>");
			
	}
	if(count==6){
		lcdGoToAddr(0x54); 
		lcdWriteText((uint8_t *)"Start Spindle     <>");
			
	}
	if(count==7){
		lcdGoToAddr(0x54); 
		lcdWriteText((uint8_t *)"Stop Spindle      < ");
			
	}




}
else
{
if(count!=0){
	sd_list_offset("/",count);
}
else
{
	lcdGoToAddr(0x54); 
	lcdWriteText((uint8_t *)"back to menu        ");
}
}

}

void menuclick(){



if (count==0){// sets menu mode so that turning the encoder moves up and down the file list

if(listingfiles==0){
	listingfiles=1;
	lcdGoToAddr(0x54);
	lcdWriteText((uint8_t *)"back to menu        ");

}
else
{
if(listingfiles==1){
	listingfiles=0;

}
}// end file listing




}


}

