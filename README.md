
**Small modification and tested on custom made 3d/cnc board with arduino** 



nano 328p. For GCODE sender, i use my own gcodesender (on github too)

1. Repetier Style EEPROM configuration, M206 and M205, share same EEPROM address with Repetier, for example, to adjust Zmax, can use M206 P153 S100. Config on eeprom are: Maxfeedrate XYZE, Step/mmXYZE, Acceleration,Zmax, and RodLen,HorizontalRadius, TowerOffsetXYZ(DELTA PRINTER)
2. Implement M109, to set heater 0 temperature and wait
3. Implement ARC G2 and G3, configurable in configtool
4. Implement Adjust Temp, (see EEPROM), basically just add /reduce temperature for M109, because who knows same gcode file request same 180degree, but on different printer, different filament loaded, need to add few degree, without editing the Gcode file/reslice.
5. DELTA Printer , work in progress for nano 328p, of course need low number of line buffer (8 is ok). and maybe other fixes so can work with my gcode sender better.

10-6-2017
1. Add JerkXY,JerkZ, and segments to EEPROM.
2. Reset factory M502 (still have bug on zmax and accel)
3. Limit the F from gcodeprocess, so when calculate segment_total, it will be correct.
4. Optional to include INCH coordinate (nowdays mosly slicer using mm)
5. Update configtool

My Machine using TEACUP3D PCB

![image](https://user-images.githubusercontent.com/11457832/26998799-82d1581e-4db3-11e7-9b53-a2f20cf818a3.png)

Dragon on 0.3mm layer.

![image](https://user-images.githubusercontent.com/11457832/26998803-9b35f89c-4db3-11e7-8128-b8e439bd99cc.png)

Using Repetier Host, can use EEPROM manager too

![image](https://user-images.githubusercontent.com/11457832/27000800-17e81552-4de4-11e7-863d-b2c5c0c6e362.png)



![Teacup3D PCB](https://raw.githubusercontent.com/ryannining/Teacup_Firmware/master/pcb/schematic.png)

    ##############################################################################
    #                                                                            #
    # Teacup - lean and efficient firmware for RepRap printers                   #
    #                                                                            #
    # by Triffid Hunter, Traumflug, jakepoz, many others.                        #
    #                                                                            #
    ##############################################################################
    
    For installation instructions, see
    http://reprap.org/wiki/Teacup_Firmware#Simple_Installation and/or
    http://reprap.org/wiki/Teacup_Firmware#Developer_Installation
    
    For documentation, see
    http://reprap.org/wiki/Teacup_Firmware
    
    
    ##############################################################################
    #                                                                            #
    # This firmware is Copyright (c) ...                                         #
    #   2009 - 2010 Michael Moon aka Triffid_Hunter                              #
    #   2010 - 2013 Markus "Traumflug" Hitter <mah@jump-ing.de>                  #
    #                                                                            #
    # This program is free software; you can redistribute it and/or modify       #
    # it under the terms of the GNU General Public License as published by       #
    # the Free Software Foundation; either version 2 of the License, or          #
    # (at your option) any later version.                                        #
    #                                                                            #
    # This program is distributed in the hope that it will be useful,            #
    # but WITHOUT ANY WARRANTY; without even the implied warranty of             #
    # MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the              #
    # GNU General Public License for more details.                               #
    #                                                                            #
    # You should have received a copy of the GNU General Public License          #
    # along with this program; if not, write to the Free Software                #
    # Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA #
    #                                                                            #
    ##############################################################################
