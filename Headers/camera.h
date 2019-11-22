/*
 * camera.h
 *
 *  Created on: May 7, 2017
 *      Author: Nick
 *
 *      Header file for camera related variables and constants
 *      Definition of Camera class
 *    		-for each camera, instantiate a Camera object to store data
 *    		and access functions
 *      Make sure to include in main.cpp
 */


#ifndef HEADERS_CAMERA_H_
#define HEADERS_CAMERA_H_

#include <iostream>
#include <map>

volatile float exp_time = 0.0040; //TUNE!
const float thresh = 0.7;
const int PIXELS = 128;

class Camera {
	private:
		bool rising_edge = false;
		int trackWidth_pm = 4;
		int trackWidth_upperBound = 0;
		int trackWidth_lowerBound = 0;
		int track_num = 0;
		const int refStraight = 64;
		const int straightBound = 10;


	public:
		int cal_trackWidth = 0;
		int cal_iterations = 50;
		int trackWidth = 0;
		volatile float max = 0;
		volatile float avg = 0;
		volatile int max_i = 0;
		volatile unsigned short data[128];
		float cal_background = 0;
		bool straight_flag = false;
/////////CLASS FUNCTIONS///////////////
		bool trackDetected(void);
		void findTrackWidth(void);
		void checkForStraight(void);
		void filter(void);

};


void Camera::findTrackWidth() {
	//find the track width in pixels
	rising_edge = false;
	trackWidth = 0;
	for (int i=0; i< PIXELS; i++) {
		avg += data[i];
		if (data[i] > thresh * max) {
			trackWidth += 1;
			rising_edge = true;
		} else if (rising_edge) {
			rising_edge = false;
		}
	}
	avg /= PIXELS;
}

bool Camera::trackDetected() {
	//determines whether a track is in the view of the camera based on track width
	Camera::findTrackWidth();
	trackWidth_upperBound = cal_trackWidth + 12;
	trackWidth_lowerBound = cal_trackWidth - 2;
	if (trackWidth < trackWidth_upperBound && trackWidth >= trackWidth_lowerBound) {
		return true;
	} else {
		return false;
	}
}

void Camera::checkForStraight() {
	//checks for straight-away
	if (Camera::trackDetected()) {
		if (max_i > refStraight - straightBound && max_i < refStraight + straightBound) {
			straight_flag = true;
		} else {
			straight_flag = false;
		}
	}
}

Camera cam1;
Camera cam2;

#endif
