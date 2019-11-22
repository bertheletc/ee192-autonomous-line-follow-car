/*
 * steering_control.h
 *
 *  Created on: May 7, 2017
 *      Author: Nick
 *
 *      Header file for steering control related variables and constants
 *      Make sure to include in main.cpp
 */

#ifndef HEADERS_STEERING_CONTROL_H_
#define HEADERS_STEERING_CONTROL_H_

//STEERING CONTROL//
const int refSteer = 67; //TUNE!

const float Kp_max = 5.2; //TUNE!
const float Kp_min = 3.8; //TUNE!
volatile float Kp_steer = Kp_max; //TUNE!

const float Kd_max = 0.25; //TUNE!
const float Kd_min = 0.15; //TUNE!
volatile float Kd_steer = Kd_max; //TUNE!

//Servo bounds
const float l_end = 900.0;
const float middle = 1300.0;
const float r_end = 1700.0;
// Servo position
volatile float pw = 0.0;
volatile int max_i_diff = 0;
volatile float pos = 0.0;

//Servo control
volatile int err_steer = 0;
volatile int last_err_steer = 0;
volatile int derivTerm = 0;
volatile float input_steer = 0.0;



#endif /* HEADERS_STEERING_CONTROL_H_ */
