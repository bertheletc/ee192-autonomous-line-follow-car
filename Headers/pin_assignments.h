/*
 * pin_assignments.h
 *
 *  Created on: May 7, 2017
 *      Author: Nick
 *
 *      Header file assigning pins for various functions on FRDM-K64F
 *      Make sure to include in main.cpp
 */

#ifndef HEADERS_PIN_ASSIGNMENTS_H_
#define HEADERS_PIN_ASSIGNMENTS_H_

#include "FastAnalogIn.h"

//CAM1//
//AnalogIn A0_CAM1(PTC10);
FastAnalogIn A0_CAM1(PTB2, false);
DigitalOut SI_CAM1(PTC16);
DigitalOut CLK_CAM1(PTC17);
//CAM2//
//AnalogIn A0_CAM2(PTC11);
FastAnalogIn A0_CAM2(PTB3, false);

DigitalOut SI_CAM2(PTC2);
DigitalOut CLK_CAM2(PTC3);

//Hall Sensor Pins//
InterruptIn hallA(PTC12);
InterruptIn hallB(PTA2);

//Control Loop Ticker//
Ticker timestep;

//Motor Pins//
PwmOut motor(PTD1);
DigitalOut brakeEN(PTB9);

//Servo Pin//
PwmOut servo(PTA1);

//Push Buttons Pins//
InterruptIn SELECT(PTC4);
InterruptIn UP(PTD0);
InterruptIn DOWN(PTD3);



#endif /* HEADERS_PIN_ASSIGNMENTS_H_ */
