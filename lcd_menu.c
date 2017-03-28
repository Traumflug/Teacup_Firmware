
// This is part of the Teacup Firmware

/*Copyright (c) 2017 stecor@email.com

MIT License

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

*/



#include "lcd_menu.h"
#include "lcd_menu_define.h"
#include "display.h"
#include "rot_enc.h"
#include "sendf.h"

#include "dda.h"		// For setting/getting axis positions
#include "temp.h"		// For setting/getting temps
#include "home.h"		// For homing
#include "pinio.h"		// for STEPPER Disabling
#include "gcode_parse.h"// to add SD as GCODE source


// Implementation based on MICROMENU.
// Thank you Dean Camera, 2012
//      www.fourwalledcubicle.com
//   dean [at] fourwalledcubicle.com


/* Quick description, rationale, etc
** This is the Screen and Menu logic driving the menu system and associated actions
** Below are the principles driving the code:
** - code will NOT cause waits during menu selection; rather, functions are called and return, then resume 
**		screen/menu processing upon next cycle (hence the state vars in lcd_menu_process())
** - code will be as "lean" as possible, while:
**		- storing constants and strings in FLASH (we have plenty of it)
**		- avoid creating globals and static vars - for now, the following are defined:
**		- GLOBAL: lcd_menu_re_val - this may be removed later on - it is the running count for the input device (rot enc or buttons)
**				  lcd_menu_call - this is the LCD STATE MACHINE variable. stores pointer to function to be called
**		- LOCAL static: static int8_t cur_row current menu row
**						static Menu_Item_t* Menu  menu branch we're in
**
** FUNCTION TREE
** ENTRY POINT -> display_clock() called via clock.c; this calls the state machine: lcd_menu_state();
**		LCD STATE MACHINE lcd_menu_state(): this determines whether we are currently in a screen (default is System Status) or 
**				in menu processing mode, and will call the appropriate screen update/menu process function
**			SCREEN UPDATE FUNCTIONS - SCREENS are defined in the lcd_menu_define.h file and are explained there.
**				once a screen is selected, a one-time call to its template display function is made (which prints the static text)
**				followed by screen update function calls (which only updates the variable screen content) via the LCD STATE MACHINE
**				There are multiple pairs of screen display/update functions, one pair/screen.
**			MENU PROCESS lcd_menu_process(uint8_t menu_redisplay) - this is the menu workhorse, taking care of the following events:
**				- menu display
**				- cursor updates
**				- menu navigation
**				- it is currently limited to 2 display pages only (2x4 rows) - to keep things very simple (plus I hate long menus :) )
**				again, the LCD STATE MACHINE dispatches calls to lcd_menu_process() until menu navigation completes
**
**	Most of the data related to menu/screen is located in FLASH. Usage is about 4k, which seems excessive - probably optimizations are in order
**	but for recent AVR units, 4k of flash is unconsequential, plus this should not impede performance since it's only read during navigation
*/

// Implementation
Menu_Item_t PROGMEM NULL_MENU = {0};
uint16_t lcd_menu_re_val;				// used to input different values based on input device (rot enc)
void (*lcd_menu_call)(void) = NULL;	// STATE MACHINE - pointer to function to call upon next cycle (can be SCREEN or MENU)


//**********************************
// SD Stuff - could be improved upon
// *********************************
#include "pff.h"						// we need to access SD functionality directly, since the SD lib talks via Serial
// we'll try to be as frugal with resources as possible, however would be good to unify SD approach...
// BTW - why have SD while tethered anyway?
// a bit of a hack here - FATFS objects are locally re-defined... would be good to deal with this... maybe review SD.c/h?
// duplicated resource usage for now...
static FATFS sdfile;					// see sd.c
static FRESULT result;					// see sd.c

const char *SD_STAT = sd_stat_unk;		// pointer to SD Status message


// local function prototypes - private
// this way we keep the lcd_menu.h clean of non-interface stuff...
// the print functions are in the lcd_menu_define file (as they are needed for menus; the update ones could be moved there too...)
void lcd_menu_process(uint8_t menu_redisplay);
void lcd_menu_proc(void);
void lcd_menu_proc_refresh(void);

void lcd_menu_sy_st_u(void);
void lcd_menu_t_u(char heater);
void lcd_menu_t_ext_u(void);
void lcd_menu_t_bed_u(void);

void lcd_menu_mv_x_u(void);
void lcd_menu_mv_y_u(void);
void lcd_menu_mv_z_u(void);
void lcd_menu_mv_e_u(void);

void lcd_menu_sd_u(void);			// sd update function prototype

// *************************************************************************
// *			SCREEN  FUNCTIONS										   *
// *************************************************************************

// Screen template print 
void lcd_menu_screen(const char* const *screen){
	display_set_cursor(0,0);
	display_writestr_P(SCREEN(&screen[0]));
	display_writestr_P(SCREEN(&screen[2]));
	
	display_set_cursor(1,0);
	display_writestr_P(SCREEN(&screen[1]));
	display_writestr_P(SCREEN(&screen[3]));
}


// **************************************************************************
// * 			SYS STATUS Screen FUNCTIONS									*
// **************************************************************************

// Screen Functions come in pairs: a screen print and a screen update function
// screen definitions are in "lcd_menu_define.h"
void lcd_menu_sy_st_p(void){				// system status screen display function
	lcd_menu_screen(sy_st_scr);				// print screen template
	lcd_menu_call = lcd_menu_sy_st_u;		// set next call to Sys Stat Update
}

void lcd_menu_sy_st_u(void){  				// System Status screen update function. 
	uint16_t temperature;
	uint16_t tgt_temp;

	display_set_cursor(0, 1);	// Update Extr Temp
	temperature = temp_get(TEMP_SENSOR_extruder);
	tgt_temp = tgt_temp_get(TEMP_SENSOR_extruder);
	sendf_P(display_writechar, PSTR("%u/%u"),
            temperature >> 2, tgt_temp >> 2);

	display_set_cursor(0, 10); // Update Bed Temp
    temperature = temp_get(TEMP_SENSOR_bed);
	tgt_temp = tgt_temp_get(TEMP_SENSOR_bed);		
	sendf_P(display_writechar, PSTR("%u/%u"),
            temperature >> 2, tgt_temp >> 2);	
	
	update_current_position();
	display_set_cursor(1, 1); // Update X Pos
	sendf_P(display_writechar, PSTR("%lq"),
          current_position.axis[X]);

	display_set_cursor(1, 10); // Update X Pos
	sendf_P(display_writechar, PSTR("%lq"),
          current_position.axis[Y]);

	display_set_cursor(2, 1); // Update X Pos
	sendf_P(display_writechar, PSTR("%lq"),
          current_position.axis[Z]);
		  
	display_set_cursor(3, 3); // Update SD Status
	display_writestr_P(SD_STAT);
	

	if(rot_enc_but(1)){  				// if we pressed the button
	//	lcd_menu_state_var = SCR_MENU;	// Update state global var	> go back to menu	
		lcd_menu_call = lcd_menu_proc;
		rot_enc_read(1);				// clear rotary encoder value
		lcd_menu_process(1);	
		
	}
}

// **************************************************************************
// * 				TEMPERATURE Screen FUNCTIONS							*
// **************************************************************************
void lcd_menu_t_p(char heater){			// Extruder Temp setup screen display
	switch (heater){
	case 'E':
		lcd_menu_screen(t_ext_scr);
		lcd_menu_call = lcd_menu_t_ext_u;		//x1
		lcd_menu_re_val = (tgt_temp_get(TEMP_SENSOR_extruder))>>2;
		break;
	case 'B':
		lcd_menu_screen(t_bed_scr);
		lcd_menu_call = lcd_menu_t_bed_u;		//x1
		lcd_menu_re_val = (tgt_temp_get(TEMP_SENSOR_bed))>>2;
		break;
	}
	
	rot_enc_read(1);					// clear rot enc counter
	lcd_menu_t_u(heater);
}

void lcd_menu_t_u(char heater){  		// screen update. 
	lcd_menu_re_val += rot_enc_read(1);	// update values
	
	uint16_t temperature;
	uint16_t tgt_temp;

	display_set_cursor(3, 1);	// Update Extr Temp
	temperature = temp_get(TEMP_SENSOR_extruder);
	tgt_temp = tgt_temp_get(TEMP_SENSOR_extruder);
	sendf_P(display_writechar, PSTR("%u/%u"),
            temperature >> 2, tgt_temp >> 2);

	display_set_cursor(3, 10); // Update Bed Temp
    temperature = temp_get(TEMP_SENSOR_bed);
	tgt_temp = tgt_temp_get(TEMP_SENSOR_bed);		
	sendf_P(display_writechar, PSTR("%u/%u"),
            temperature >> 2, tgt_temp >> 2);	
	
	display_set_cursor(1, 5); 			// Update Ext temp
	sendf_P(display_writechar, PSTR("%u"),lcd_menu_re_val);
	display_writechar(LCD_STR_DEGREE); 
	display_writechar(' '); 			// quick hack to blank out leftover digits
	display_writechar(' ');

	if(rot_enc_but(1)){  				// if we pressed the button
		switch (heater){
		case 'E':
			temp_set(TEMP_SENSOR_extruder, lcd_menu_re_val<<2);
			break;
		case 'B':
			temp_set(TEMP_SENSOR_bed, lcd_menu_re_val<<2);
			break;
		}
		
		lcd_menu_call = lcd_menu_proc;
		lcd_menu_proc_refresh();		
	}
}
// Extruder Temp Print
void lcd_menu_t_ext_p(void){			// Bed Temp setup screen display
	lcd_menu_t_p('E');
}
// Bed Temp Print
void lcd_menu_t_bed_p(void){			// Bed Temp setup screen display
	lcd_menu_t_p('B');
}
// Extruder Temp Update
void lcd_menu_t_ext_u(void){  			// screen update. 
	lcd_menu_t_u('E');
}
// Bed Temp Update
void lcd_menu_t_bed_u(void){  			// screen update. 
	lcd_menu_t_u('B');
}
// Cooldown
void lcd_menu_t_coold(void){
	temp_set(TEMP_SENSOR_extruder, 	0);
	temp_set(TEMP_SENSOR_bed, 		0);
}

// **************************************************************************
// * 				MOVE Screen FUNCTIONS									*
// **************************************************************************
void lcd_menu_home(void){
	home();
}

void lcd_menu_mv_p(char axis){				// Move screen display
	switch (axis){
	case 'X':
		lcd_menu_screen(mv_x_scr);
		lcd_menu_call = lcd_menu_mv_x_u;
		break;
	case 'Y':
		lcd_menu_screen(mv_y_scr);
		lcd_menu_call = lcd_menu_mv_y_u;
		break;
	case 'Z':
		lcd_menu_screen(mv_z_scr);
		lcd_menu_call = lcd_menu_mv_z_u;
		break;
	case 'E':
		lcd_menu_screen(mv_e_scr);	
		lcd_menu_call = lcd_menu_mv_e_u;	
		break;
	}
	
	lcd_menu_re_val = 0;				// we will get the tgt temp 
	//lcd_menu_re_val = tgt_temp_get(TEMP_SENSOR_bed);
	rot_enc_read(1);					// clear rot enc counter
}

void lcd_menu_mv_u(char axis){ 			// MOVE screen update. 
	lcd_menu_re_val += rot_enc_read(1);	// update values
	
	display_set_cursor(2, 5); 			// Update Ext temp
	sendf_P(display_writechar, PSTR("%u"),lcd_menu_re_val); 
	display_writechar(' '); 			// quick hack to blank out leftover digits
	display_writechar(' ');

	if(rot_enc_but(1)){  				// if we pressed the button
	
		update_current_position();
         // current_position.axis[X]
         // current_position.axis[Y]
	/*
	
		// implement axis limits
		#ifdef	X_MIN
		if (next_target.target.axis[X] < (int32_t)(X_MIN * 1000.))
			next_target.target.axis[X] = (int32_t)(X_MIN * 1000.);
		#endif
		#ifdef	X_MAX
		if (next_target.target.axis[X] > (int32_t)(X_MAX * 1000.))
			next_target.target.axis[X] = (int32_t)(X_MAX * 1000.);
		#endif
		#ifdef	Y_MIN
		if (next_target.target.axis[Y] < (int32_t)(Y_MIN * 1000.))
			next_target.target.axis[Y] = (int32_t)(Y_MIN * 1000.);
		#endif
		#ifdef	Y_MAX
		if (next_target.target.axis[Y] > (int32_t)(Y_MAX * 1000.))
			next_target.target.axis[Y] = (int32_t)(Y_MAX * 1000.);
		#endif
		#ifdef	Z_MIN
		if (next_target.target.axis[Z] < (int32_t)(Z_MIN * 1000.))
			next_target.target.axis[Z] = (int32_t)(Z_MIN * 1000.);
		#endif
		#ifdef	Z_MAX
		if (next_target.target.axis[Z] > (int32_t)(Z_MAX * 1000.))
			next_target.target.axis[Z] = (int32_t)(Z_MAX * 1000.);
		#endif
		*/
	
		lcd_menu_call = lcd_menu_proc;
		lcd_menu_proc_refresh();		
	}
}


void lcd_menu_mv_x_p(void){
	lcd_menu_mv_p('X');
}
void lcd_menu_mv_y_p(void){
	lcd_menu_mv_p('Y');
}
void lcd_menu_mv_z_p(void){
	lcd_menu_mv_p('Z');
}
void lcd_menu_mv_e_p(void){
	lcd_menu_mv_p('E');
}

void lcd_menu_mv_x_u(void){ 	
	lcd_menu_mv_u('X'); 	
}
void lcd_menu_mv_y_u(void){ 	
	lcd_menu_mv_u('Y'); 	
}
void lcd_menu_mv_z_u(void){ 	
	lcd_menu_mv_u('Z'); 	
}
void lcd_menu_mv_e_u(void){ 	
	lcd_menu_mv_u('E'); 	
}

void lcd_menu_stp_dis(void){
	stepper_disable();
	x_disable();
	y_disable();
	z_disable();
	e_disable();
}


// **************************************************************************
// *			SD CARD FUNCTIONS												*
// **************************************************************************

// SD File Print function 
void lcd_menu_sd_p(void){				// SD screen display
	for(uint8_t i=0; i<4; i++){		// Blank screen (don't use CLS as it's blocking)
		display_set_cursor(0,0);
		display_writestr_P(blank);
		display_writestr_P(blank);
		display_set_cursor(1,0);
		display_writestr_P(blank);
		display_writestr_P(blank);
	}
	display_set_cursor(0,0);
	result = pf_mount(&sdfile);			// attempt SD mount
	if (result != FR_OK) display_writestr_P(sd_mnt_err);
	
	lcd_menu_call = lcd_menu_sd_u; 		// call update function next
	rot_enc_read(1);					// clear rot enc counter
	
	lcd_menu_sd_u();					// update screen now...
}

void lcd_menu_sd_u(void){  				// SD screen update. 
	static uint8_t max_files = 0;
	int8_t rot_enc_val = rot_enc_read(1);

	if(rot_enc_val < 0)
		if(-rot_enc_val > lcd_menu_re_val) lcd_menu_re_val = 0;	// avoid overflow
		else lcd_menu_re_val += rot_enc_val;
	else lcd_menu_re_val += rot_enc_val;
	
	if (lcd_menu_re_val > max_files) lcd_menu_re_val = max_files;  	// clamp values up

	DIR dir;	
	char path[] = "/";		
	FILINFO fno;

	result = pf_opendir(&dir, path);
	if (result != FR_OK) display_writestr_P(sd_opd_err);
	else{													//OpenDir OK
		for(uint8_t i = 0; i < lcd_menu_re_val; i++){
			result = pf_readdir(&dir, &fno);				// move to selected item
		}	
		
		uint8_t i = 0;
		while(i < 4) {										// and print file names...
			result = pf_readdir(&dir, &fno);
			if (result != FR_OK ) display_writestr_P(sd_rdd_err);
			else{											// read dir OK...
				if( fno.fname[0] != 0){
					display_set_cursor(i, 1);
					uint8_t r = 0;
					while( fno.fname[r]) {
						display_writechar(fno.fname[r]);
						r++;
					}
					if (fno.fattrib & AM_DIR){				// print a / sign after SC card directories
						display_writechar('/');
						r++;
					}
					while(r < 19) {							// and blank the rest of the row :)
						display_writechar(' ');
						r++;
					}
					i++;
				}else
					break;						// file name is NULL; we reached the end.
			}  // end else
		}  // end while i<4
		
		if(i < 4){								// if the screen has (at least one) empty row,
			display_set_cursor(i, 0);
			display_writestr_P(sd_bck_row); 	// display the "virtual" back entry
			i++;
		}
		max_files = lcd_menu_re_val + i -1;		// update our max_files counter
		
		while(i < 4){							// if there is still space on the screen,
			display_set_cursor(i, 0);
			display_writestr_P(blank); 			// fill with blank space
			i++;								// we don't count empty space as files...
		}
		
		display_set_cursor(0, 0);display_writechar('>'); // print cursor
		// x1
	//	display_set_cursor(2, 15);sendf_P(display_writechar, PSTR("%u"),max_files);  // print cursor
	//	display_set_cursor(3, 15);sendf_P(display_writechar, PSTR("%u"),lcd_menu_re_val);  // print cursor
	}
	
	if(rot_enc_but(1)){  					// if we pressed the button
		// temp - blank screen
		display_set_cursor(0,0);
		display_writestr_P(blank);
		display_writestr_P(blank);
		display_set_cursor(1,0);
		display_writestr_P(blank);
		display_writestr_P(blank);
		// end temp
		result = pf_opendir(&dir, path);
		for(uint8_t i = 0; i <= lcd_menu_re_val; i++)
			pf_readdir(&dir, &fno);		// move to selected item
	
		if( fno.fname[0] != 0){
			display_set_cursor(0,0);
	//		uint8_t r = 0;					// temp check to display file
	//		while( fno.fname[r]) {
	//			display_writechar(fno.fname[r]);
	//			r++;
	//		}
			
			result = pf_open(fno.fname);	// open selected file
			if (result != FR_OK ) display_writestr_P(sd_opf_err);
			gcode_sources |= GCODE_SOURCE_SD;// and start printing
			
			SD_STAT = sd_stat_prn;			// update SD stat message
		}	
	//	}else{ 								// back was selected
		lcd_menu_re_val = 0;				// set rot enc accumulator to 0
		lcd_menu_call = lcd_menu_proc;		// go back to the menu
		lcd_menu_proc_refresh();			// refresh the menu
		
		
	//		display_writestr_P(sd_bck_row); 
	//	}
	}
	
}

void lcd_menu_sd_pse(void){ // pause SD print
	gcode_sources &= ! GCODE_SOURCE_SD;
	SD_STAT = sd_stat_pse;
}

void lcd_menu_sd_stp(void){ // STOP SD print
	gcode_sources &= ! GCODE_SOURCE_SD;
	pf_unmount(&sdfile);
	SD_STAT = sd_stat_umnt;
}


// **************************************************************************
// *			MENU FUNCTIONS												*
// **************************************************************************

// Get menu total row #
uint8_t lcd_menu_get_rows(Menu_Item_t* Menu){
	uint8_t rows = 1;
	while(MENU_NEXT(Menu) != &NULL_MENU){
		Menu = MENU_NEXT(Menu);
		rows++;
	}
	return rows;
}

// Print menu on the LCD screen, starting at menu row "start_row".
// basically displays one menu page (4 rows) on the LCD
void lcd_menu_list(Menu_Item_t* Menu, uint8_t start_row){
	if (Menu == &NULL_MENU) return;
	uint8_t i;
	
	for(i=0; i < start_row; i++)  // advance to start_row
		Menu = MENU_NEXT(Menu);

	for(i=0; i < 4; i++){
		display_set_cursor(i,0);
		if(Menu != &NULL_MENU){  
			display_writestr_P(Menu->Text);
			Menu = MENU_NEXT(Menu);
		}else					// if we don't have enough items to fill the page
			display_writestr_P(blank);
	}
}

// Move cursor to new position
void lcd_menu_cursor(uint8_t cursor_pos, uint8_t mode ){
	display_set_cursor(cursor_pos,0);  	// set cursor position
	if(mode)
		display_writechar('>');
	else
		display_writechar(' ');  
}

// Process Menu events
// - Menu page changes *(limited to 2 pages currently)
// - Cursor update
// - Menu actions (navigation up/down + actions when item is selected)
void lcd_menu_process(uint8_t menu_redisplay){
	static int8_t cur_row = 0;					// current menu row we're at
	static Menu_Item_t* Menu = &sysstat;		// menu branch we're in
	
	if(menu_redisplay){ 						// Re-display current menu
		cur_row = 0; 
		while(MENU_PREVIOUS(Menu) != &NULL_MENU) // Roll back until the first item in the menu
			Menu = MENU_PREVIOUS(Menu);		
		lcd_menu_list(Menu,0);
		lcd_menu_cursor(0,1);					// Draw cursor at pos 0
		return;
	}
	
	uint8_t menu_rows = lcd_menu_get_rows(Menu);

	// we are here either because the rotary encoder was rotated or because the button was pushed
	// if both rotate and press happened, rotate has priority over button press 
	// we disregard the button press *assuming* is was a fluke
	int8_t last_row;
	int8_t rot_enc_val = rot_enc_read(1);
	if(rot_enc_val != 0){
		rot_enc_but(1);  								// clear button press event
		
		last_row = cur_row;
		cur_row += rot_enc_val;
	
		if(cur_row < 0) cur_row = 0;					// clamp min val
		if(cur_row > (menu_rows - 1) ) 
			cur_row = menu_rows - 1; 					//clamp max val
		
		// Limitation - this is currently limited to max 8 rows total; 
		// can be easily improved, but we don't want long menus, do we? ;) 
		if(cur_row != last_row){  						// move cursor
			if(cur_row >= 4){ 							// screen refresh?
				if (last_row < 4)
					lcd_menu_list(Menu, 4);
				else 
					lcd_menu_cursor(last_row - 4 ,0);  // erase old cursor  
				lcd_menu_cursor(cur_row - 4 ,1);
			}else{										// curr_row < 4
				if (last_row >= 4)
					lcd_menu_list(Menu, 0);
				else
					lcd_menu_cursor(last_row,0);  		// erase old cursor
				lcd_menu_cursor(cur_row,1);				// write the new one
			}
		}	
	}else{									// if we're here, the button was pushed (while NOT rotating the encoder shaft)
		if(rot_enc_but(1)){					// check button press, and acknowledge button press to clear it
			for(uint8_t i=0; i < cur_row; i++)  		// advance to currently selected row
				Menu = MENU_NEXT(Menu);

			if(MENU_CHILD(Menu) != &NULL_MENU){		// this menu has children
				Menu = MENU_CHILD(Menu);				// we always point to the first child
				cur_row = 0;last_row = 0;
				lcd_menu_list(Menu, 0);
				lcd_menu_cursor(0,1);					//write cursor at position 0
			}else{
				if(MENU_PARENT(Menu) != &NULL_MENU){	// this menu has a parent
					Menu = MENU_PARENT(Menu);			// go to the parent
					while(MENU_PREVIOUS(Menu) != &NULL_MENU) // then roll back until the first item in the menu
						Menu = MENU_PREVIOUS(Menu);		// so we're okay
					cur_row = 0;last_row = 0;
					lcd_menu_list(Menu, 0);
					lcd_menu_cursor(0,1);				//write cursor at position 0
				}else{
					void (*EnterCall)(void) = MENU_ENTERCALL(Menu);
					if(EnterCall) 
						EnterCall();
					while(MENU_PREVIOUS(Menu) != &NULL_MENU) // roll back until the first item in the menu
						Menu = MENU_PREVIOUS(Menu);			
				}
			}
		}
	
	}
}

// Helper function - lcd_menu_process(0) is redefined as void (void)
void lcd_menu_proc(void){
	lcd_menu_process(0);
}

void lcd_menu_proc_refresh(void){
	lcd_menu_process(1);
}


// This is the LCD state machine
//void lcd_menu_state(void){
//	lcd_menu_call();
//}


void lcd_menu_init(void){
	display_init();							// init display
	rot_enc_init();  						// init ROT ENC
	
	display_greeting(); 					// say something - just briefly

//	lcd_menu_call = lcd_menu_proc;			// we are in a menu now...
//	lcd_menu_proc_refresh();				// we need a printed menu on screen			
	lcd_menu_call = lcd_menu_sy_st_u;		// Startup with System Status
	lcd_menu_sy_st_p();
}

/** Regular update of the display. Typically called once a second from clock.c.
	
*/
void display_clock(void) {
	if(lcd_menu_call == lcd_menu_sy_st_u)	// If we are in the System Status screen this gets updated without user intervention
		lcd_menu_call();					// // *** Note -throttle calls, currently done each 1/4s
	else									// if we're not in System Status, only process menu if					
		if((rot_enc_read(0)!= 0) ||rot_enc_but(0) )// user interaction is recorded
			lcd_menu_call();
}

/**
  Forwards a character or a control command from the display queue to display
  Also updates encoder shaft position and button press status
*/
void display_tick(void) {
	display_putchar();
	rot_enc_read(0); 						// read encoder and retain position.
	rot_enc_but(0);  						// read button, retain status
}

