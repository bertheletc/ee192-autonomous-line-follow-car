/*
 * velocity_control.h
 *
 *  Created on: May 7, 2017
 *      Author: Nick
 *
 *      Header file for velocity control related variables and constants
 *      Make sure to include in main.cpp
 */

#ifndef HEADERS_VELOCITY_CONTROL_H_
#define HEADERS_VELOCITY_CONTROL_H_

const float dt = 0.010; //CONTROL LOOP TIME!

//VELOCITY CONTROL//
const float refVel_max = 3.30; //TUNE!
const float refVel_min = 2.40; //TUNE!
const float refVel_incr = 0.05;//TUNE!
const float refVel_decr = 1.50;//TUNE!

volatile float refVel = refVel_min; //start out with slow speed
volatile float velDiff = 0.0;

const float Kp_vel = 0.15; //TUNE!
const float Ki_vel = 0.002; //TUNE!

//VELOCITY//
//Velocity Clamps
const float maxPWM = 0.30;
const float minPWM = 0.0;
//Transform to m/s with gear ratio (9:1 gear ratio) (r = 32.5 mm)
const float r = 0.0325; // meters
const float gear = 7.5; // 9:1 gear down ratio
const float velTransform = (2.0*3.141592*r) / (gear*4*dt);
//Variable other instantiations
volatile int revs = 0;
volatile int revsA = 0;
volatile int revsB = 0;
volatile float vel = 0.0;
volatile float velMA = 0.0;
volatile float velSum = 0.0;
volatile int resolution = 10;
volatile int count = 1;
volatile int ramp = 1;
volatile float dutyInput = 0.0;
volatile float intgrlTerm = 0.0;
volatile float err_vel = 0.0;

#endif /* HEADERS_VELOCITY_CONTROL_H_ */
