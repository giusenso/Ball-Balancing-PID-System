# Ball Balancing PID Systems
![picture](img/platform.jpg)

## table of contents
* [Project description](#project-description)
* [Hardware](#hardware)
* [Technologies](#technologies)
* [How to use it](#how-to-use-it)

## Project description
The _Ball Balancing PID System_ is a cheap multidisciplinary project I developed in 2018 during my BSc in Automation Engineering.
The project is designed to be modular. There are 4 main modules, every one of them can also work stand-alone.

1. **Computer Vision** Module: acquire ball position using OpenCV libs.

2. **PID Module**: filter and compute signal.

3. **Serial Communication** Module: provide beetwen PC and AVR device.

4. **Actuation** Module: this code run on avr, wait for incoming packet and move servos.

![picture](img/computer_vision_algorithm.png)

## Hardware
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

- **Standard mode**: better performance but minimal GUI  
```
"./run -s"
```

- **Setting mode**: set pid gains and computer vision parameters 	
```
./run -settings
```

- **Debug mode**: start debug mode: a better GUI and print utilities, little bit slower than standard mode
```
./run -debug
```

- **Manual mode**: platform can be controlled directly from terminal.	
```
./run -manual
```

*note: one and only one flag can be used*
