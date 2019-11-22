/*
 * init_telemetry.h
 *
 *  Created on: May 8, 2017
 *      Author: Nick
 *
 *      initializes telemetry containers and relevant variables
 */

#ifndef HEADERS_INIT_TELEMETRY_H_
#define HEADERS_INIT_TELEMETRY_H_

////TELEMETRY
#include "telemetry.h"
#include "MODSERIAL.h"

//TELEMETRY////////////////////////////////////////////////////////////////////////////////////////////////////////////
MODSERIAL telemetry_serial(PTC15, PTC14);
//
telemetry::MbedHal telemetry_hal(telemetry_serial);
telemetry::Telemetry telemetry_obj(telemetry_hal);


//
////VARIABLES
telemetry::Numeric<uint32_t> tele_time_ms(telemetry_obj, "time", "Time", "ms", 0);
telemetry::NumericArray<uint16_t, 128> tele_linescan_cam1(telemetry_obj, "Camera 1", "Linescan", "ADC", 0);
//telemetry::NumericArray<uint16_t, 128> tele_filtered(telemetry_obj, "Camera 1", "Linescan", "ADC", 0);
//telemetry::Numeric<float> tele_trackWidth(telemetry_obj, "Camera 1", "Linescan", "track width", 0);
telemetry::NumericArray<uint16_t, 128> tele_linescan_cam2(telemetry_obj, "Camera 2", "Linescan", "ADC", 0);
//telemetry::Numeric<float> tele_lateral_error(telemetry_obj, "Lateral Error", "Lateral Error", "pixels", 0);
//telemetry::Numeric<float> tele_velocity_velMA(telemetry_obj, "VELOCITY", "VELOCITY", "m/s", 0);
//telemetry::Numeric<float> tele_servo_pos(telemetry_obj, "servo", "servo position", "pulsewidth (us)", 0);
//telemetry::Numeric<float> tele_calTrackWidth(telemetry_obj, "Linescan", "TrackWidth", "Pixels", 0);
//telemetry::Numeric<float> tele_trackInView(telemetry_obj, "Linescan", "Track in View", "Yes or No", 0);
//telemetry::Numeric<float> tele_background(telemetry_obj, "Linescan", "background values", "intensity", 0);
telemetry::Numeric<float> tele_max_1(telemetry_obj, "Linescan", "max", "intensity", 0);
telemetry::Numeric<float> tele_max_2(telemetry_obj, "Linescan", "max", "intensity", 0);

#endif /* HEADERS_INIT_TELEMETRY_H_ */
