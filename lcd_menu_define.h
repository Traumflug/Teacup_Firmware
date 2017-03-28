
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




#ifndef _LCD_MENU_DEFINE_H
#define _LCD_MENU_DEFINE_H

// *************************************************************************
// *		LCD	MENU & SCREEN DEFINITION							   	   *
// *************************************************************************

// define function prototypes for functions used as ENTERCALL arguments
void lcd_menu_sy_st_p(void);		// System Status Print (display) screen
void lcd_menu_t_ext_p(void);		// Extruder Temp
void lcd_menu_t_bed_p(void);		// Bed Temp Screen
void lcd_menu_t_coold(void);		// Cooldown function call
void lcd_menu_mv_x_p(void);			// Move X Screen
void lcd_menu_mv_y_p(void);
void lcd_menu_mv_z_p(void);
void lcd_menu_mv_e_p(void);
void lcd_menu_home(void);			// Home function call
void lcd_menu_stp_dis(void);		// Stepper Disable function call

void lcd_menu_sd_p(void);			// SD file select and print Screen
void lcd_menu_sd_pse(void); 		// pause SD print
void lcd_menu_sd_stp(void); 		// pause SD print


// *************************************************************************
// *		LCD	MENU DEFINITION										   	   *
// *************************************************************************

// This is adapted from Micromenu V2. Please take a look at Micromenu for a more in-depth explanation
// You will also find some information in lcd_menu.h
// Basically, each menu item is described as below: Menu name + relationships (siblings, children, parent) + an optional
// entercall function which is executed when the item is entered.
// *** Important Note: the evaluation order is:
//		- siblings - all siblings will be displayed to form the given menu level
//		- child - when clicked, if the menu has a child, the lower level menu (child menu)
// 				will be navigated to. 
//		- parent - if an item has a parent, it will be evaluated after checking for children
//		- ENTERCALL - this is only evaluated when there is no child nor parent menu item
// Quick rules: items with CHILD should not have PARENT (it's useless to specify a parent)
//				items that need to execute a function should have NO parent nor child. In
// 				this sense, the function is the menu's child. 
// 				to work around what may seem a limitation, always use a "back" item in a lower
//				level menu. 
// *** Also, ensure that the first menu item in each branch has NULL_MENU as a previous item
//		and similarly the last menu branch's item has a NULL_MENU next item.

//         	NAME  , NEXT , 		PREV , 		PARENT,		CHILD,		ENTERCALL,			TEXT 
MENU_ITEM(sysstat,	temp,	 	NULL_MENU,	NULL_MENU,	NULL_MENU,	lcd_menu_sy_st_p,	" System Status      ");	// OK
MENU_ITEM(temp,		move,		sysstat,	NULL_MENU,	temp_ext,	NULL,				" Temperature        ");	// OK
MENU_ITEM(move,		sd, 		temp,		NULL_MENU,	move_hme,	NULL,				" Move               ");	// OK
MENU_ITEM(sd,		fan,		move,		NULL_MENU,	sd_sel,		NULL,				" SD Card            ");	// OK
MENU_ITEM(fan,		back,		sd,			NULL_MENU,	fan_he1,	NULL,				" FAN                ");	// OK
MENU_ITEM(back,		NULL_MENU,	fan,		NULL_MENU,	NULL_MENU,	lcd_menu_sy_st_p,	" \x04 back             ");	// OK

MENU_ITEM(temp_ext,	temp_bed,	NULL_MENU,	NULL_MENU,	NULL_MENU,	lcd_menu_t_ext_p,	" Extruder Temp      ");	// OK
MENU_ITEM(temp_bed,	temp_coo,	temp_ext,	NULL_MENU,	NULL_MENU,	lcd_menu_t_bed_p,	" Bed Temp           ");	// OK
MENU_ITEM(temp_coo,	temp_bck,	temp_bed,	NULL_MENU,	NULL_MENU,	lcd_menu_t_coold,	" Cooldown           ");	// needs testing
MENU_ITEM(temp_bck,	NULL_MENU,	temp_coo,	temp,		NULL_MENU,	NULL,				" \x04 back             ");	// OK

MENU_ITEM(move_hme,	move_x,		NULL_MENU,	NULL_MENU,	NULL_MENU,	lcd_menu_home,		" Home All           ");	// needs testing
MENU_ITEM(move_x,	move_y,		move_hme,	NULL_MENU,	NULL_MENU,	lcd_menu_mv_x_p,	" Move X             ");	// OK
MENU_ITEM(move_y,	move_z,		move_x,		NULL_MENU,	NULL_MENU,	lcd_menu_mv_y_p,	" Move Y             ");	// OK
MENU_ITEM(move_z,	move_e,		move_y,		NULL_MENU,	NULL_MENU,	lcd_menu_mv_z_p,	" Move Z             ");	// OK
MENU_ITEM(move_e,	move_dis,	move_z,		NULL_MENU,	NULL_MENU,	lcd_menu_mv_e_p,	" Move E             ");	// OK
MENU_ITEM(move_dis,	move_bck,	move_e,		NULL_MENU,	NULL_MENU,	lcd_menu_stp_dis,	" Disable Motors     ");	// needs testing
MENU_ITEM(move_bck,	NULL_MENU,	move_dis,	move,		NULL_MENU,	NULL,				" \x04 back             ");	// OK

MENU_ITEM(sd_sel,	sd_stp,		NULL_MENU,	NULL_MENU,	NULL_MENU,	lcd_menu_sd_p,		" Select File        ");	// OK
MENU_ITEM(sd_stp,	sd_pse,		sd_sel,		NULL_MENU,	NULL_MENU,	lcd_menu_sd_stp,	" Stop Print         ");	// OK
MENU_ITEM(sd_pse,	sd_bck,		sd_stp,		NULL_MENU,	NULL_MENU,	lcd_menu_sd_pse,	" Pause Print        ");	// OK
MENU_ITEM(sd_bck,	NULL_MENU,	sd_pse,		sd,			NULL_MENU,	NULL,				" \x04 back             ");	// OK

MENU_ITEM(fan_he1,	fan_pcf,	NULL_MENU,	NULL_MENU,	NULL_MENU,	NULL,				" Hotend Fan         ");
MENU_ITEM(fan_pcf,	fan_he2,	fan_he1,	NULL_MENU,	NULL_MENU,	NULL,				" Part Cooling Fan   ");
MENU_ITEM(fan_he2,	fan_bck,	fan_pcf,	NULL_MENU,	NULL_MENU,	NULL,				" Hotend Fan2        ");
MENU_ITEM(fan_bck,	NULL_MENU,	fan_he2,	fan,		NULL_MENU,	NULL,				" \x04 back             ");	// OK


// *************************************************************************
// *			SCREEN DEFINITION										   *
// *************************************************************************
// Screens are different from menus; screens usually only display information, or allow limited user interaction. 
// clicking the button in a screen completes data entry (if data can be entered) and jump back to the menu we came from
// Also, each screen comprises:
//	- a SCREEN template (below) - which is printed on the screen using the screen print function
// 	- a SCREEN update function - which then updates values on the scree, and handles button presses 


/* Info Screen Template and update positions

	*****************************************************
	*  0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19*
	*  T 1 1 1 / 2 2 2   B  1  1  1  /  2  2  2     W  1*
	*  X 2 0 0 . 0 1 5   Y  2  0  0  .  5  1  4         * 
	*  Z 1 0 0 . 0 0 5                                  *
	*  S D _ i n s e r t e  d     S  T  :  I  D  L  E   *
	***************************************************** 
 To avoid a CLRSCR, we blank out every empty space as well as enter the template chars;
 Since a CLRSCR costs 2000us and each char is about 50us, 40 chars ~=CLRSCR; However, after CLS we need to write something on 
 the screen, so the actual overhead is just 20-30 or so to fill out the screen blanks (we come out on top not using CLS).
 **** PLUS (big +) these updates are done in the msg queue (non-blocking) vs delay_ms(2) which is blocking.
*/

// Info Screen               01234567890123456789
const char i1[21] PROGMEM = "\x03        \x01          ";
const char i2[21] PROGMEM = "X        Y          ";
const char i3[21] PROGMEM = "Z                   ";
const char i4[21] PROGMEM = "SD          ST:     ";
const char* const sy_st_scr[4] PROGMEM= {i1, i2, i3, i4};

// Blank row   					01234567890123456789
const char blank[21] PROGMEM = "                    ";

// Extruder Temp set Screen    01234567890123456789
const char te1[21] PROGMEM = "    EXTRUDER        "; 
const char te2[21] PROGMEM = "   \x03:               ";
const char* const t_ext_scr[4] PROGMEM= {te1, te2, blank, i1};

// Bed Temp set Screen  		  01234567890123456789
const char tb1[21] PROGMEM = "    BED TEMP        "; 
const char tb2[21] PROGMEM = "   \x01:               ";
const char* const t_bed_scr[4] PROGMEM= {tb1, tb2, blank, i1};

// Move X Screen  		  01234567890123456789
const char mx1[21] PROGMEM = "      MOVE X        "; 
const char mx3[21] PROGMEM = "   X:               ";
const char* const mv_x_scr[4] PROGMEM= {mx1, blank, mx3, blank};
// Move Y Screen  		  01234567890123456789
const char my1[21] PROGMEM = "      MOVE Y        "; 
const char my3[21] PROGMEM = "   Y:               ";
const char* const mv_y_scr[4] PROGMEM= {my1, blank, my3, blank};
// Move Z Screen  		  01234567890123456789
const char mz1[21] PROGMEM = "      MOVE Z        "; 
const char mz3[21] PROGMEM = "   Z:               ";
const char* const mv_z_scr[4] PROGMEM= {mz1, blank, mz3, blank};
// Move E Screen  		  01234567890123456789
const char me1[21] PROGMEM = "      MOVE E        "; 
const char me3[21] PROGMEM = "   E:               ";
const char* const mv_e_scr[4] PROGMEM= {me1, blank, me3, blank};

// SD Card strings				 	 01234567890123456789
const char sd_mnt_err[21] PROGMEM = "SD CARD MNT ERR:    ";
const char sd_opd_err[21] PROGMEM = "SD CARD OPEN DIR ERR";
const char sd_rdd_err[21] PROGMEM = "SD CARD READ DIR ERR";
const char sd_opf_err[21] PROGMEM = "SD CARD OPN FILE ERR";
const char sd_bck_row[21] PROGMEM =	" \x04 back             ";
// SD Status messages
const char sd_stat_unk[] PROGMEM  = 	"Unknown ";
const char sd_stat_umnt[] PROGMEM  = 	"STOP    ";
const char sd_stat_prn[] PROGMEM  = 	"Printing";
const char sd_stat_pse[] PROGMEM  = 	"Paused  ";



#endif