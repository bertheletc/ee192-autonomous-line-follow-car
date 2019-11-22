#include "mbed.h"
#include "camera.h"
#include "pin_assignments.h"
#include "steering_control.h"
#include "velocity_control.h"
#include "FastAnalogIn.h"

////TELEMETRY
//#include "telemetry.h"
//#include "MODSERIAL.h"
//#include "init_telemetry.h";

void readCamera();

void initCalibration() {
	for (int i = 0; i < cam1.cal_iterations; i++) {
		readCamera();
		cam1.findTrackWidth();
		cam2.findTrackWidth();
		cam1.cal_trackWidth += cam1.trackWidth;
		cam2.cal_trackWidth += cam2.trackWidth;
	}
	cam1.cal_trackWidth /= cam1.cal_iterations;
	cam2.cal_trackWidth /= cam2.cal_iterations;
//	tele_calTrackWidth = cam1.cal_trackWidth;

}

void calcTurnSpeed() {
	if (cam2.straight_flag && cam1.straight_flag) {
		refVel += refVel_incr;
		if (refVel > refVel_max) {
			refVel = refVel_max;
		}
	} else {
		refVel -= refVel_decr;
		if (refVel < refVel_min) {
			refVel = refVel_min;
		}
	}
}

void calcKp() {
	if (cam2.straight_flag && cam1.straight_flag) {
		Kp_steer = Kp_min;
		Kd_steer = Kd_min;
	} else if (refVel == refVel_max) {
		Kp_steer = Kp_min;
		Kd_steer = Kd_min;
	} else if (refVel == refVel_min) {
		Kp_steer = Kp_max;
		Kd_steer = Kd_max;
	} else {
		velDiff = (refVel_max - refVel) / (refVel_max - refVel_min);
		Kp_steer = velDiff * (Kp_max - Kp_min) + Kp_min;
		Kd_steer = velDiff * (Kd_max - Kd_min) + Kd_min;
		if (Kp_steer > Kp_max) {
			Kp_steer = Kp_max;
		} else if (Kp_steer < Kp_min) {
			Kp_steer = Kp_min;
		}
		if (Kd_steer > Kd_max) {
			Kd_steer = Kd_max;
		} else if (Kd_steer < Kd_min) {
			Kd_steer = Kd_min;
		}
	}
}

void gainScheduler() {
	cam1.checkForStraight();
	cam2.checkForStraight();
	calcTurnSpeed();
	calcKp();
}

//VELOCITY!
//ISR for HALL sensor reading
void rpmCounterA() {
    revsA++;
}

void rpmCounterB() {
    revsB++;
}

void read_cam_real() {

	cam1.max = 0;
	cam2.max = 0;
	SI_CAM1 = 1;
	SI_CAM2 = 1;
	CLK_CAM1 = 1;
	CLK_CAM2 = 1;
	SI_CAM1 = 0;
	SI_CAM2 = 0;
	cam1.data[0] = A0_CAM1.read_u16();
	CLK_CAM1 = 0;
	cam2.data[0] = A0_CAM2.read_u16();
	CLK_CAM2 = 0;

	for(int i = 1; i < PIXELS; i++) {
		CLK_CAM1 = 1;
		CLK_CAM2 = 1;
		cam1.data[i] = A0_CAM1.read_u16();
		cam2.data[i] = A0_CAM2.read_u16();
//		//TELEMETRY
//		tele_linescan_cam1[i] = cam1.data[i];
//		tele_linescan_cam2[i] = cam2.data[i];
		if (cam1.data[i] > cam1.max) { //calc max
			cam1.max = cam1.data[i];
			cam1.max_i = i;
		}
		if (cam2.data[i] > cam2.max) {
			cam2.max = cam2.data[i];
			cam2.max_i = i;
		}
		CLK_CAM1 = 0;
		CLK_CAM2 = 0;
	}
//	tele_max_1 = cam1.max;
//	tele_max_2 = cam2.max;
}

void read_cam_fake() {
	//Fake ANALOG READ
	SI_CAM1 = 1;
	SI_CAM2 = 1;
	CLK_CAM1 = 1;
	CLK_CAM2 = 1;
	SI_CAM1 = 0;
	SI_CAM2 = 0;
	CLK_CAM1 = 0;
	CLK_CAM2 = 0;
	for(int i = 0; i < PIXELS; i++) {
		CLK_CAM1 = 1;
		CLK_CAM2 = 1;
		CLK_CAM1 = 0;
		CLK_CAM2 = 0;
	}

	wait(exp_time);
}

void readCamera() {

	cam1.max = 0;
	cam2.max = 0;

	read_cam_fake();
	read_cam_real();
}

void PDcalculation(volatile int max_i) {
	//Line is to the Right
	if (max_i > refSteer) {
		err_steer = max_i - refSteer;
		derivTerm = (err_steer - last_err_steer) / dt;

		input_steer = (Kp_steer * err_steer) + (Kd_steer * derivTerm);
		pos = middle - input_steer;

		//CLAMP to servo limits
		if (pos < l_end) {
			pos = l_end;
		} else if (pos > r_end) {
			pos = r_end;
		}

		last_err_steer = err_steer;
	}
	//Line is to the Left
	else if (max_i < refSteer) {
		err_steer = refSteer - max_i;
		derivTerm = (err_steer - last_err_steer) / dt;

		input_steer = (Kp_steer * err_steer) + (Kd_steer * derivTerm);

		pos = middle + input_steer;

		//CLAMP to servo limits
		if (pos < l_end) {
			pos = l_end;
		} else if (pos > r_end) {
			pos = r_end;
		}

		last_err_steer = err_steer;
	} else {
		pw = pw;
	}

//	//TELEMETRY!
//	tele_lateral_error = max_i - refSteer;

}

void steeringPD() {
	if (cam1.trackDetected()) {
		PDcalculation(cam1.max_i);
		pw = pos * (0.000001);

	} else {
		if (pos > middle) {
			pw = r_end * 0.000001;
		} else {
			pw = l_end * 0.000001;
		}
	}

	//ACTUATION!
	//Send INPUT value to servo
	servo.pulsewidth(pw);
//	tele_servo_pos = pw * 1000000;
}

void velocityPI() {
	//Calculate velocity (revolutions/fixed time unit)
	revs = revsA + revsB;
	vel = (float) (revs) * velTransform;

	//Code begins when switch is turned on
	if (vel > 0.0){
		//Calculate moving average
		if (count < resolution) {
			velSum = velSum + vel;
			velMA = vel;
			count++;
		} else if (count == resolution) {
			velMA = (velSum + vel) / resolution;
			count++;
		} else {
			velMA = velMA + (vel / resolution) - (velMA / resolution);
		}

		//TELEMETRY
		//tele_velocity_velMA = velMA;

		//Calculate ERROR
		err_vel = refVel - velMA;

		//Accumulate error for integral term
		intgrlTerm = intgrlTerm + (Ki_vel * err_vel);

		//Anti-Windup check!
		if (intgrlTerm > maxPWM){
			intgrlTerm = maxPWM;
		} else if (intgrlTerm < minPWM){
			intgrlTerm = minPWM;
		}

		//Calculate INPUT value
		dutyInput = (Kp_vel*err_vel) + intgrlTerm;
		//dutyInput = (Kp_vel * err_vel);

		//Clamp REFERENCE value if needed
		if (dutyInput > maxPWM) {
			dutyInput = maxPWM;
		} else if (dutyInput < minPWM) {
			dutyInput = minPWM;
		}

		if (ramp<100){
			dutyInput = 0.10f;
			ramp++;
			intgrlTerm = 0.0;
		}
		//ACTUATION! Send INPUT value to motor
		motor.write(dutyInput);
		revsA = 0;
		revsB = 0;
	}
}

void control() {

	readCamera();

	gainScheduler();

	steeringPD();

	velocityPI();

	//RESET parameters
}

//TELEMETRY
//void print_serial() {
//	tele_time_ms = telemetry_hal.get_time_ms();
//    telemetry_obj.do_io();
//}

void setup() {

//	//TELEMETRY
//	telemetry_serial.baud(38400);
//	telemetry_obj.transmit_header();
//	//tele_lateral_error = 0.0;
//	//tele_velocity_velMA = 0.0;

	//cameras

//	Camera cam1;
//	Camera cam2;

	//Calibration setup
	SELECT.rise(&initCalibration);

	//set pwm periods
    motor.period_us(50); // 20 kHz frequency
    servo.period_ms(10); //Set servo PWM period = 3ms

    //set regular interrupts
    hallA.rise(&rpmCounterA);
    hallA.fall(&rpmCounterA);

    hallB.rise(&rpmCounterB);
    hallB.fall(&rpmCounterB);

    //set Ticker interrupts
    timestep.attach(&control,dt);

    brakeEN.write(1);

    //set warm-up speed and command motor
    dutyInput = 0.10f;

    motor.write(dutyInput);
}

//Main CODE
int main() {
	//run setup function
	setup();

	//run while loop
	//int counter = 0;
	while (1) {
//		//TELEMETRY
//		counter++;
//		if (counter % 2 == 1) {
//			print_serial();
//		}
	}
}
