
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



#ifndef _LCD_MENU_H
#define _LCD_MENU_H

#include <stdint.h>

//#define DISPLAY

// Micromenu
	#include <avr/pgmspace.h>
	#include <stddef.h>
	
	#define MENU_ITEM_READ_POINTER(Addr)   (void*)pgm_read_word(Addr)

	#define MENU_PARENT(Menu)	MENU_ITEM_READ_POINTER(&Menu->Parent)
	#define MENU_CHILD(Menu)	MENU_ITEM_READ_POINTER(&Menu->Child)
	#define MENU_NEXT(Menu)		MENU_ITEM_READ_POINTER(&Menu->Next)
	#define MENU_PREVIOUS(Menu)	MENU_ITEM_READ_POINTER(&Menu->Previous)
	#define MENU_ENTERCALL(Menu) MENU_ITEM_READ_POINTER(&Menu->EnterCall)
	
	// Type define for a menu item. Menu items should be initialized via the helper
	// MENU_ITEM(), not created from this type directly in user-code.
	typedef const struct Menu_Item {
		const struct Menu_Item *Next; 
		const struct Menu_Item *Previous; 
		const struct Menu_Item *Parent; 
		const struct Menu_Item *Child; 
		void (*EnterCall)(void); 
		const char Text[21];			//20 chars plus null string terminator
	} Menu_Item_t;

	/** Creates a new menu item entry with the specified links and callbacks.
	 *  Name      Name of the menu entry, must be unique.
	 *  Next      Name of the next linked menu item, or NULL_MENU if no menu link.
	 *  Previous  Name of the previous linked menu item, or \ref NULL_MENU if no menu link.
	 *  Parent    Name of the parent linked menu item, or \ref NULL_MENU if no menu link.
	 *  Child     Name of the child linked menu item, or \ref NULL_MENU if no menu link.
	 *  SelectFunc  Function callback to execute when the menu item is selected, or \c NULL for no callback.
	 *  EnterFunc   Function callback to execute when the menu item is entered, or \c NULL for no callback.
	 */
	#define MENU_ITEM(Name, Next, Previous, Parent, Child, EnterFunc, Text) \
		extern Menu_Item_t PROGMEM Next;     \
		extern Menu_Item_t PROGMEM Previous; \
		extern Menu_Item_t PROGMEM Parent;   \
		extern Menu_Item_t PROGMEM Child;  \
		Menu_Item_t PROGMEM Name = {&Next, &Previous, &Parent, &Child, EnterFunc, Text}
			
	extern Menu_Item_t PROGMEM NULL_MENU;

	#define SCREEN(Addr)		MENU_ITEM_READ_POINTER(Addr)
	
// END Micromenu

void lcd_menu_init(void);
void display_clock(void);
void display_tick(void);


#endif /* _LCD_MENU_H */