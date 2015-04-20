README file for the Pikado project
==================================

What is Pikado ?
----------------
Pikado is the croatian word for darts. This project has the goal to build a PC
based e-dart machine. The project consists of a interface electronic and a GUI
application.

Interface electronic
--------------------
The interface is a PCB which connects an electronic dart board (only the 
switch matrix) and optional some push buttons to the PC. Using that PCB the
GUI application can detect the thrown darts of the players. The PCB uses an
AVR micro-controller and connects via RS232 serial connection to the PC.
I have used the interface successfully with the US232R USB<->RS232 converter
cable from FTDI (FT232R chipset).

Until now I designed two versions of the PCB:
- V1: This is a small universal PCB which can be mounted in cheap home e-dart
      boards. (The normal electronic of the home e-dart board must be removed)
      This PCB can be connected to any switch matrix with a maximum of 
      24 signal lines.

- V2: For my homemade dart machine I searched a switch matrix with a long spare
      part availability. I selected the throwing circle and switch matrix of 
      the dart machines from the NSM-LÖWEN ENTERTAINMENT GmbH.
      => This project has nothing in common with the NSM-LÖWEN ENTERTAINMENT GmbH.
      => I only used their spare parts.
      
      Therefore I designed a new PCB which fits directly to the connector of
      the LÖWEN switch matrix.

My future plans for the PCB:
- Use a vibration sensor to detect darts which hits the catch ring or only the 
  case of the board. (At the moment the player has to press a push-button if
  less than three darts are in the board)
- Use a movement sensor to detect the removal of the darts by the player.
  (At the moment the player has to press a push-button after removing the darts)

Software used for the interface:
- For the schematic and PCB layout:  The freeware version of EAGLE 5.6 <http://cadsoft.de/>
- For the micro-controller software: avr-gcc + avr-libc
- For flashing the micro-controller: avrdude

