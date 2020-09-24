# Ball Balancing PID Systems
![picture](img/platform.jpg)

## table of contents
* [Project description](#project-description)
* [Hardware](#hardware)
* [Technologies](#technologies)
* [How to use it](#how-to-use-it)

## Project description
The _Ball Balancing PID System_ is a cheap multidisciplinary project I developed in 2018 during my BSc in Automation Engineering.
The project is designed to be modular; there are 4 main stand-alone modules:

1. _**Computer Vision** Module_: acquires and preprocess ball position using OpenCV libs.

2. _**PID** Module_: filters and computes the control input through a digital PI-D.

3. _**Serial Communication** Module_: it provides communication beetwen PC and microcontroller.

4. _**Actuation** Module_: embedded software running on microcontroller. It uses interrupts to detects incoming packets and it moves servo motors.

![picture](img/computer_vision_algorithm.png)

## Hardware
* microcontroller: Atmega2560
* Servos: DS3218MG 5V-6.8V
* USB camera: 640x480
* Battery pack: 6V, 2850mAh

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
