/* 
      These pin definitions match the PJRC Teensy 3.0  as used in the 
      Teensy++ 2.0 carrier per http://www.pjrc.com/teensy/pinout.html

      See Sprinter's https://github.com/kliment/Sprinter/blob/master/Sprinter/fastio.h or 
      Marlin's lincomatic fork https://github.com/lincomatic/Marlin/blob/Marlin_v1/Marlin/fastio.h 

 */

// SPI
//#define SCK            DIO9
//#define MISO           DIO11
//#define MOSI           DIO10
//#define SS             DIO8



#include <WProgram.h> // cli(),sei(), others.
/* some workarounds ************************************

 disable interrupts etc 
 following bobc at https://github.com/bobc/Teacup_Firmware/blob/master/app/sysfuncs.c
*/  

/* compatibility with AVR #####################
 */
#define ADC ADC0_RA
#define ADC_vect adc0_isr
//#######################################

#ifndef ISR  

// attributes not useful for ARM per 
// http://forum.pjrc.com/threads/17760-AVR-like-ISR%28%29?p=28111&viewfull=1#post28111
# ifdef __cplusplus
#  define ISR(vector, ...)            \
    extern "C" void vector (void) ; \
    void vector (void)
# else
#  define ISR(vector, ...)            \
    void vector (void) ; \
    void vector (void)
# endif
#endif



// change for your board
//#define DEBUG_LED      DIO13 

// Redefine some AVR macros for ARM
#undef _SET_OUTPUT
// Set GPIOx_PDDR for the port's bit and set PORTx_PCRn for HIGH output, SLOW slew GPIO
#define _SET_OUTPUT(IO) do{  IO ## _DDR |= MASK(IO ## _PIN); \
     IO ## _CONFIG = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);} while(0)

#undef _SET_INPUT
// Clear GPIOx_PDDR for the port's bit and set PORTx_PCRn the pin for GPIO
#define _SET_INPUT(IO) do{  IO ## _DDR &= ~MASK(IO ## _PIN); \
     IO ## _CONFIG = PORT_PCR_MUX(1);} while(0)




/*
pins for READ, WRITE, TOGGLE, SET_OUPUT, GET_INPUT GET_OUTPUT as defined in arduino.h
*/
/* 
  These targets are defined in files like 
/Applications/Arduino.app/Contents//Resources/Java/hardware/tools/avr/avr/include/avr/iom32u4.h

    DIOn and AIOn aren't defined anywhere, just as text-munged DIOn_whatever macros which arr defined here
    READ port, read port's pin
    Write port, write port's pin (piggy backs on read port's pin ratehr than PORTD and PORTD6)
    Data direction register also piggy backs off PIND1 instead of DDRD with DDRD1
    PWM register for pin 

 */



/* Some python code to generate this section:
#/usr/bin/env python

pwms=[3,4,5,6,9, 10,20,21,22,23]
FTMxCnVs=["FTM1_C0V","FTM1_C1V", "FTM0_C7V","FTM0_C4V","FTM0_C2V",
          "FTM0_C3V","FTM0_C5V","FTM0_C6V","FTM0_C0V","FTM0_C1V"]

for x in range(0,34):
   pwm = "NULL" if not x in pwms else "&{reg}".format(reg=FTMxCnVs[pwms.index(x)])

   print """// pin {x}
#define DIO{x}_PIN       CORE_PIN{x}_BIT  // bit for bitmask within registers  
#define DIO{x}_RPORT     CORE_PIN{x}_PORTREG
#define DIO{x}_WPORT     CORE_PIN{x}_PORTREG
#define DIO{x}_PWM       {pwm}          
#define DIO{x}_DDR       CORE_PIN{x}_DDRREG
#define DIO{x}_CONFIG    CORE_PIN{x}_CONFIG
#define DIO{x}_BAREPIN   {x}
""".format(x=x,pwm=pwm)  

#### end of python code

 */
// pin 0
#define DIO0_PIN       CORE_PIN0_BIT  // bit for bitmask within registers  
#define DIO0_RPORT     CORE_PIN0_PORTREG
#define DIO0_WPORT     CORE_PIN0_PORTREG
#define DIO0_PWM       NULL          
#define DIO0_DDR       CORE_PIN0_DDRREG
#define DIO0_CONFIG    CORE_PIN0_CONFIG
#define DIO0_BAREPIN   0

// pin 1
#define DIO1_PIN       CORE_PIN1_BIT  // bit for bitmask within registers  
#define DIO1_RPORT     CORE_PIN1_PORTREG
#define DIO1_WPORT     CORE_PIN1_PORTREG
#define DIO1_PWM       NULL          
#define DIO1_DDR       CORE_PIN1_DDRREG
#define DIO1_CONFIG    CORE_PIN1_CONFIG
#define DIO1_BAREPIN   1

// pin 2
#define DIO2_PIN       CORE_PIN2_BIT  // bit for bitmask within registers  
#define DIO2_RPORT     CORE_PIN2_PORTREG
#define DIO2_WPORT     CORE_PIN2_PORTREG
#define DIO2_PWM       NULL          
#define DIO2_DDR       CORE_PIN2_DDRREG
#define DIO2_CONFIG    CORE_PIN2_CONFIG
#define DIO2_BAREPIN   2

// pin 3
#define DIO3_PIN       CORE_PIN3_BIT  // bit for bitmask within registers  
#define DIO3_RPORT     CORE_PIN3_PORTREG
#define DIO3_WPORT     CORE_PIN3_PORTREG
#define DIO3_PWM       &FTM1_C0V          
#define DIO3_DDR       CORE_PIN3_DDRREG
#define DIO3_CONFIG    CORE_PIN3_CONFIG
#define DIO3_BAREPIN   3

// pin 4
#define DIO4_PIN       CORE_PIN4_BIT  // bit for bitmask within registers  
#define DIO4_RPORT     CORE_PIN4_PORTREG
#define DIO4_WPORT     CORE_PIN4_PORTREG
#define DIO4_PWM       &FTM1_C1V          
#define DIO4_DDR       CORE_PIN4_DDRREG
#define DIO4_CONFIG    CORE_PIN4_CONFIG
#define DIO4_BAREPIN   4

// pin 5
#define DIO5_PIN       CORE_PIN5_BIT  // bit for bitmask within registers  
#define DIO5_RPORT     CORE_PIN5_PORTREG
#define DIO5_WPORT     CORE_PIN5_PORTREG
#define DIO5_PWM       &FTM0_C7V          
#define DIO5_DDR       CORE_PIN5_DDRREG
#define DIO5_CONFIG    CORE_PIN5_CONFIG
#define DIO5_BAREPIN   5

// pin 6
#define DIO6_PIN       CORE_PIN6_BIT  // bit for bitmask within registers  
#define DIO6_RPORT     CORE_PIN6_PORTREG
#define DIO6_WPORT     CORE_PIN6_PORTREG
#define DIO6_PWM       &FTM0_C4V          
#define DIO6_DDR       CORE_PIN6_DDRREG
#define DIO6_CONFIG    CORE_PIN6_CONFIG
#define DIO6_BAREPIN   6

// pin 7
#define DIO7_PIN       CORE_PIN7_BIT  // bit for bitmask within registers  
#define DIO7_RPORT     CORE_PIN7_PORTREG
#define DIO7_WPORT     CORE_PIN7_PORTREG
#define DIO7_PWM       NULL          
#define DIO7_DDR       CORE_PIN7_DDRREG
#define DIO7_CONFIG    CORE_PIN7_CONFIG
#define DIO7_BAREPIN   7

// pin 8
#define DIO8_PIN       CORE_PIN8_BIT  // bit for bitmask within registers  
#define DIO8_RPORT     CORE_PIN8_PORTREG
#define DIO8_WPORT     CORE_PIN8_PORTREG
#define DIO8_PWM       NULL          
#define DIO8_DDR       CORE_PIN8_DDRREG
#define DIO8_CONFIG    CORE_PIN8_CONFIG
#define DIO8_BAREPIN   8

// pin 9
#define DIO9_PIN       CORE_PIN9_BIT  // bit for bitmask within registers  
#define DIO9_RPORT     CORE_PIN9_PORTREG
#define DIO9_WPORT     CORE_PIN9_PORTREG
#define DIO9_PWM       &FTM0_C2V          
#define DIO9_DDR       CORE_PIN9_DDRREG
#define DIO9_CONFIG    CORE_PIN9_CONFIG
#define DIO9_BAREPIN   9

// pin 10
#define DIO10_PIN       CORE_PIN10_BIT  // bit for bitmask within registers  
#define DIO10_RPORT     CORE_PIN10_PORTREG
#define DIO10_WPORT     CORE_PIN10_PORTREG
#define DIO10_PWM       &FTM0_C3V          
#define DIO10_DDR       CORE_PIN10_DDRREG
#define DIO10_CONFIG    CORE_PIN10_CONFIG
#define DIO10_BAREPIN   10

// pin 11
#define DIO11_PIN       CORE_PIN11_BIT  // bit for bitmask within registers  
#define DIO11_RPORT     CORE_PIN11_PORTREG
#define DIO11_WPORT     CORE_PIN11_PORTREG
#define DIO11_PWM       NULL          
#define DIO11_DDR       CORE_PIN11_DDRREG
#define DIO11_CONFIG    CORE_PIN11_CONFIG
#define DIO11_BAREPIN   11

// pin 12
#define DIO12_PIN       CORE_PIN12_BIT  // bit for bitmask within registers  
#define DIO12_RPORT     CORE_PIN12_PORTREG
#define DIO12_WPORT     CORE_PIN12_PORTREG
#define DIO12_PWM       NULL          
#define DIO12_DDR       CORE_PIN12_DDRREG
#define DIO12_CONFIG    CORE_PIN12_CONFIG
#define DIO12_BAREPIN   12

// pin 13
#define DIO13_PIN       CORE_PIN13_BIT  // bit for bitmask within registers  
#define DIO13_RPORT     CORE_PIN13_PORTREG
#define DIO13_WPORT     CORE_PIN13_PORTREG
#define DIO13_PWM       NULL          
#define DIO13_DDR       CORE_PIN13_DDRREG
#define DIO13_CONFIG    CORE_PIN13_CONFIG
#define DIO13_BAREPIN   13

// pin 14
#define DIO14_PIN       CORE_PIN14_BIT  // bit for bitmask within registers  
#define DIO14_RPORT     CORE_PIN14_PORTREG
#define DIO14_WPORT     CORE_PIN14_PORTREG
#define DIO14_PWM       NULL          
#define DIO14_DDR       CORE_PIN14_DDRREG
#define DIO14_CONFIG    CORE_PIN14_CONFIG
#define DIO14_BAREPIN   14

// pin 15
#define DIO15_PIN       CORE_PIN15_BIT  // bit for bitmask within registers  
#define DIO15_RPORT     CORE_PIN15_PORTREG
#define DIO15_WPORT     CORE_PIN15_PORTREG
#define DIO15_PWM       NULL          
#define DIO15_DDR       CORE_PIN15_DDRREG
#define DIO15_CONFIG    CORE_PIN15_CONFIG
#define DIO15_BAREPIN   15

// pin 16
#define DIO16_PIN       CORE_PIN16_BIT  // bit for bitmask within registers  
#define DIO16_RPORT     CORE_PIN16_PORTREG
#define DIO16_WPORT     CORE_PIN16_PORTREG
#define DIO16_PWM       NULL          
#define DIO16_DDR       CORE_PIN16_DDRREG
#define DIO16_CONFIG    CORE_PIN16_CONFIG
#define DIO16_BAREPIN   16

// pin 17
#define DIO17_PIN       CORE_PIN17_BIT  // bit for bitmask within registers  
#define DIO17_RPORT     CORE_PIN17_PORTREG
#define DIO17_WPORT     CORE_PIN17_PORTREG
#define DIO17_PWM       NULL          
#define DIO17_DDR       CORE_PIN17_DDRREG
#define DIO17_CONFIG    CORE_PIN17_CONFIG
#define DIO17_BAREPIN   17

// pin 18
#define DIO18_PIN       CORE_PIN18_BIT  // bit for bitmask within registers  
#define DIO18_RPORT     CORE_PIN18_PORTREG
#define DIO18_WPORT     CORE_PIN18_PORTREG
#define DIO18_PWM       NULL          
#define DIO18_DDR       CORE_PIN18_DDRREG
#define DIO18_CONFIG    CORE_PIN18_CONFIG
#define DIO18_BAREPIN   18

// pin 19
#define DIO19_PIN       CORE_PIN19_BIT  // bit for bitmask within registers  
#define DIO19_RPORT     CORE_PIN19_PORTREG
#define DIO19_WPORT     CORE_PIN19_PORTREG
#define DIO19_PWM       NULL          
#define DIO19_DDR       CORE_PIN19_DDRREG
#define DIO19_CONFIG    CORE_PIN19_CONFIG
#define DIO19_BAREPIN   19

// pin 20
#define DIO20_PIN       CORE_PIN20_BIT  // bit for bitmask within registers  
#define DIO20_RPORT     CORE_PIN20_PORTREG
#define DIO20_WPORT     CORE_PIN20_PORTREG
#define DIO20_PWM       &FTM0_C5V          
#define DIO20_DDR       CORE_PIN20_DDRREG
#define DIO20_CONFIG    CORE_PIN20_CONFIG
#define DIO20_BAREPIN   20

// pin 21
#define DIO21_PIN       CORE_PIN21_BIT  // bit for bitmask within registers  
#define DIO21_RPORT     CORE_PIN21_PORTREG
#define DIO21_WPORT     CORE_PIN21_PORTREG
#define DIO21_PWM       &FTM0_C6V          
#define DIO21_DDR       CORE_PIN21_DDRREG
#define DIO21_CONFIG    CORE_PIN21_CONFIG
#define DIO21_BAREPIN   21

// pin 22
#define DIO22_PIN       CORE_PIN22_BIT  // bit for bitmask within registers  
#define DIO22_RPORT     CORE_PIN22_PORTREG
#define DIO22_WPORT     CORE_PIN22_PORTREG
#define DIO22_PWM       &FTM0_C0V          
#define DIO22_DDR       CORE_PIN22_DDRREG
#define DIO22_CONFIG    CORE_PIN22_CONFIG
#define DIO22_BAREPIN   22

// pin 23
#define DIO23_PIN       CORE_PIN23_BIT  // bit for bitmask within registers  
#define DIO23_RPORT     CORE_PIN23_PORTREG
#define DIO23_WPORT     CORE_PIN23_PORTREG
#define DIO23_PWM       &FTM0_C1V          
#define DIO23_DDR       CORE_PIN23_DDRREG
#define DIO23_CONFIG    CORE_PIN23_CONFIG
#define DIO23_BAREPIN   23

// pin 24
#define DIO24_PIN       CORE_PIN24_BIT  // bit for bitmask within registers  
#define DIO24_RPORT     CORE_PIN24_PORTREG
#define DIO24_WPORT     CORE_PIN24_PORTREG
#define DIO24_PWM       NULL          
#define DIO24_DDR       CORE_PIN24_DDRREG
#define DIO24_CONFIG    CORE_PIN24_CONFIG
#define DIO24_BAREPIN   24

// pin 25
#define DIO25_PIN       CORE_PIN25_BIT  // bit for bitmask within registers  
#define DIO25_RPORT     CORE_PIN25_PORTREG
#define DIO25_WPORT     CORE_PIN25_PORTREG
#define DIO25_PWM       NULL          
#define DIO25_DDR       CORE_PIN25_DDRREG
#define DIO25_CONFIG    CORE_PIN25_CONFIG
#define DIO25_BAREPIN   25

// pin 26
#define DIO26_PIN       CORE_PIN26_BIT  // bit for bitmask within registers  
#define DIO26_RPORT     CORE_PIN26_PORTREG
#define DIO26_WPORT     CORE_PIN26_PORTREG
#define DIO26_PWM       NULL          
#define DIO26_DDR       CORE_PIN26_DDRREG
#define DIO26_CONFIG    CORE_PIN26_CONFIG
#define DIO26_BAREPIN   26

// pin 27
#define DIO27_PIN       CORE_PIN27_BIT  // bit for bitmask within registers  
#define DIO27_RPORT     CORE_PIN27_PORTREG
#define DIO27_WPORT     CORE_PIN27_PORTREG
#define DIO27_PWM       NULL          
#define DIO27_DDR       CORE_PIN27_DDRREG
#define DIO27_CONFIG    CORE_PIN27_CONFIG
#define DIO27_BAREPIN   27

// pin 28
#define DIO28_PIN       CORE_PIN28_BIT  // bit for bitmask within registers  
#define DIO28_RPORT     CORE_PIN28_PORTREG
#define DIO28_WPORT     CORE_PIN28_PORTREG
#define DIO28_PWM       NULL          
#define DIO28_DDR       CORE_PIN28_DDRREG
#define DIO28_CONFIG    CORE_PIN28_CONFIG
#define DIO28_BAREPIN   28

// pin 29
#define DIO29_PIN       CORE_PIN29_BIT  // bit for bitmask within registers  
#define DIO29_RPORT     CORE_PIN29_PORTREG
#define DIO29_WPORT     CORE_PIN29_PORTREG
#define DIO29_PWM       NULL          
#define DIO29_DDR       CORE_PIN29_DDRREG
#define DIO29_CONFIG    CORE_PIN29_CONFIG
#define DIO29_BAREPIN   29

// pin 30
#define DIO30_PIN       CORE_PIN30_BIT  // bit for bitmask within registers  
#define DIO30_RPORT     CORE_PIN30_PORTREG
#define DIO30_WPORT     CORE_PIN30_PORTREG
#define DIO30_PWM       NULL          
#define DIO30_DDR       CORE_PIN30_DDRREG
#define DIO30_CONFIG    CORE_PIN30_CONFIG
#define DIO30_BAREPIN   30

// pin 31
#define DIO31_PIN       CORE_PIN31_BIT  // bit for bitmask within registers  
#define DIO31_RPORT     CORE_PIN31_PORTREG
#define DIO31_WPORT     CORE_PIN31_PORTREG
#define DIO31_PWM       NULL          
#define DIO31_DDR       CORE_PIN31_DDRREG
#define DIO31_CONFIG    CORE_PIN31_CONFIG
#define DIO31_BAREPIN   31

// pin 32
#define DIO32_PIN       CORE_PIN32_BIT  // bit for bitmask within registers  
#define DIO32_RPORT     CORE_PIN32_PORTREG
#define DIO32_WPORT     CORE_PIN32_PORTREG
#define DIO32_PWM       NULL          
#define DIO32_DDR       CORE_PIN32_DDRREG
#define DIO32_CONFIG    CORE_PIN32_CONFIG
#define DIO32_BAREPIN   32

// pin 33
#define DIO33_PIN       CORE_PIN33_BIT  // bit for bitmask within registers  
#define DIO33_RPORT     CORE_PIN33_PORTREG
#define DIO33_WPORT     CORE_PIN33_PORTREG
#define DIO33_PWM       NULL          
#define DIO33_DDR       CORE_PIN33_DDRREG
#define DIO33_CONFIG    CORE_PIN33_CONFIG
#define DIO33_BAREPIN   33






/* python code for these

#!/usr/bin/env python

# AIOn -> DIOn is n+14 
channel=[ 5, 14, 8, 9, 13, 12, 6, 7, 15, 4,
        0, 19, 3, 21, 26, 22]

for x in range(0,14):  
   y = "" if x < 10 else "  (not a digital IO pin)"
   dio = x+14 if x < 10 else ""
   print """// ADC {x}: {y}
#define AIO{x}_PIN        CORE_ADC{x}_PIN    
#define AIO{x}_RPORT           
#define AIO{x}_WPORT     
#define AIO{x}_PWM       
#define AIO{x}_DDR        
#define AIO{x}_ADC        ADC_SC1_ADCH({ch})
#define AIO{x}_DIO        {dio}
#define AIO{x}_CONFIG    CORE_PIN{dio}_CONFIG
""".format(x=x,y=y,ch=channel[x],dio=dio )
####  end python code


};


 */

// ADC 0: 
#define AIO0_PIN        CORE_ADC0_PIN    
#define AIO0_RPORT           
#define AIO0_WPORT     
#define AIO0_PWM       
#define AIO0_DDR        
#define AIO0_ADC        ADC_SC1_ADCH(5)
#define AIO0_DIO        14

// ADC 1: 
#define AIO1_PIN        CORE_ADC1_PIN    
#define AIO1_RPORT           
#define AIO1_WPORT     
#define AIO1_PWM       
#define AIO1_DDR        
#define AIO1_ADC        ADC_SC1_ADCH(14)
#define AIO1_DIO        15

// ADC 2: 
#define AIO2_PIN        CORE_ADC2_PIN    
#define AIO2_RPORT           
#define AIO2_WPORT     
#define AIO2_PWM       
#define AIO2_DDR        
#define AIO2_ADC        ADC_SC1_ADCH(8)
#define AIO2_DIO        16

// ADC 3: 
#define AIO3_PIN        CORE_ADC3_PIN    
#define AIO3_RPORT           
#define AIO3_WPORT     
#define AIO3_PWM       
#define AIO3_DDR        
#define AIO3_ADC        ADC_SC1_ADCH(9)
#define AIO3_DIO        17

// ADC 4: 
#define AIO4_PIN        CORE_ADC4_PIN    
#define AIO4_RPORT           
#define AIO4_WPORT     
#define AIO4_PWM       
#define AIO4_DDR        
#define AIO4_ADC        ADC_SC1_ADCH(13)
#define AIO4_DIO        18

// ADC 5: 
#define AIO5_PIN        CORE_ADC5_PIN    
#define AIO5_RPORT           
#define AIO5_WPORT     
#define AIO5_PWM       
#define AIO5_DDR        
#define AIO5_ADC        ADC_SC1_ADCH(12)
#define AIO5_DIO        19

// ADC 6: 
#define AIO6_PIN        CORE_ADC6_PIN    
#define AIO6_RPORT           
#define AIO6_WPORT     
#define AIO6_PWM       
#define AIO6_DDR        
#define AIO6_ADC        ADC_SC1_ADCH(6)
#define AIO6_DIO        20

// ADC 7: 
#define AIO7_PIN        CORE_ADC7_PIN    
#define AIO7_RPORT           
#define AIO7_WPORT     
#define AIO7_PWM       
#define AIO7_DDR        
#define AIO7_ADC        ADC_SC1_ADCH(7)
#define AIO7_DIO        21

// ADC 8: 
#define AIO8_PIN        CORE_ADC8_PIN    
#define AIO8_RPORT           
#define AIO8_WPORT     
#define AIO8_PWM       
#define AIO8_DDR        
#define AIO8_ADC        ADC_SC1_ADCH(15)
#define AIO8_DIO        22

// ADC 9: 
#define AIO9_PIN        CORE_ADC9_PIN    
#define AIO9_RPORT           
#define AIO9_WPORT     
#define AIO9_PWM       
#define AIO9_DDR        
#define AIO9_ADC        ADC_SC1_ADCH(4)
#define AIO9_DIO        23

// ADC 10:   (not a digital IO pin)
#define AIO10_PIN        CORE_ADC10_PIN    
#define AIO10_RPORT           
#define AIO10_WPORT     
#define AIO10_PWM       
#define AIO10_DDR        
#define AIO10_ADC        ADC_SC1_ADCH(0)
#define AIO10_DIO        

// ADC 11:   (not a digital IO pin)
#define AIO11_PIN        CORE_ADC11_PIN    
#define AIO11_RPORT           
#define AIO11_WPORT     
#define AIO11_PWM       
#define AIO11_DDR        
#define AIO11_ADC        ADC_SC1_ADCH(19)
#define AIO11_DIO        

// ADC 12:   (not a digital IO pin)
#define AIO12_PIN        CORE_ADC12_PIN    
#define AIO12_RPORT           
#define AIO12_WPORT     
#define AIO12_PWM       
#define AIO12_DDR        
#define AIO12_ADC        ADC_SC1_ADCH(3)
#define AIO12_DIO        

// ADC 13:   (not a digital IO pin)
#define AIO13_PIN        CORE_ADC13_PIN    
#define AIO13_RPORT           
#define AIO13_WPORT     
#define AIO13_PWM       
#define AIO13_DDR        
#define AIO13_ADC        ADC_SC1_ADCH(21)
#define AIO13_DIO        



