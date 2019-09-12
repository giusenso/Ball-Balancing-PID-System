
# Ball Balancing PID Systems 

Compile and run with one of this flag:

[1] 	"./run "
 	standard mode: better performance, minimal GUI

[2]	"./run -s"
	setting mode: set pid gains and computer vision parameters
 
[3]	"./run -d"
  	debug moded: start debug mode: a better GUI and print utilities,
 	little bit slower than standard mode
  
[4]	"./run -m"
  	manual mode: platform can be controlled directly from terminal/joypad.

	*note: one and only one flag can be used

===================================================================

Language: C/C++
Additional libs: OpenCV 3.4.1

Hardware:
- microcontroller: Atmega2560
- Servos: DS3218MG 5V-6.8V
- USB camera: 640x480
- Battery pack: 6V, 2850maAh

*** This project is designed to be modular. Every module can work stand-alone with it's own testing unit. ***

[1] Computer Vision Module: acquire ball position using OpenCV libs.

[2] PID Module: filter and compute signal.

[3] Serial Communication Module: provide beetwen PC and AVR device.

[4] Actuation Module: this code run on avr, wait for incoming packet and move servos.

