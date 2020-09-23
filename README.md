# Ball Balancing PID Systems

## table of contents
* [Project description](#project-description)
* [Hardware](#hardware)
* [Technologies](#technologies)
* [How to use it](#how-to-use-it)
* [Test](#test)

## Project description
This project is designed to be modular. Every module can work stand-alone with it's own testing unit.

[1] Computer Vision Module: acquire ball position using OpenCV libs.

[2] PID Module: filter and compute signal.

[3] Serial Communication Module: provide beetwen PC and AVR device.

[4] Actuation Module: this code run on avr, wait for incoming packet and move servos.

## Hardware:
* microcontroller: Atmega2560
* Servos: DS3218MG 5V-6.8V
* USB camera: 640x480
* Battery pack: 6V, 2850maAh

## Technologies
* OS: Ubuntu 16.04
* Language: C/C++
* Compilers: gcc 5.4.0, avr-gcc 4.9.2
* Additional libs: OpenCV 3.4.1


## How to use it
Compile and run with one of this flag:

* [1] "./run -s"
 	standard mode: better performance but minimal GUI

* [2]	"./run -settings"
	setting mode: set pid gains and computer vision parameters

* [3]	"./run -debug"
  	debug mode: start debug mode: a better GUI and print utilities,
 	little bit slower than standard mode

* [4]	"./run -manual"
  	manual mode: platform can be controlled directly from terminal.

	*note: one and only one flag can be used*

## Test
