/*
 * sensors.h
 *
 *  Created on: Feb 5, 2020
 *      Author: E_CAD
 */

#pragma once

#ifndef SENSORS_H_
#define SENSORS_H_

#include <Arduino.h>
#include <string>
#include "globals.h"
#include "ext_def.h"


enum class PmSensorCmd {
	Start,
	Stop,
	ContinuousMode,
	VersionDate
};


	template<typename T, std::size_t N> constexpr std::size_t array_num_elements(const T(&)[N]) {
		return N;
	}

	template<typename T, std::size_t N> constexpr std::size_t capacity_null_terminated_char_array(const T(&)[N]) {
		return N - 1;
	}

	bool SDS_cmd(PmSensorCmd cmd);
	bool PMS_cmd(PmSensorCmd cmd);
	bool HPM_cmd(PmSensorCmd cmd);

	String SDS_version_date();
	String sensorSDS();
	String sensorPMS();
	String sensorHPM();
	String sensorGPS();


	void disable_unneeded_nmea();

	#ifdef CFG_BME280
		String sensorBME280();
	#endif



		/*****************************************************************
		 * check display values, return '-' if undefined				 *
		 *****************************************************************/
		String check_display_value(double value, double undef, uint8_t len, uint8_t str_len);
		/*****************************************************************
		 * convert float to string with a								 *
		 * precision of two (or a given number of) decimal places		 *
		 *****************************************************************/
		String Float2String(const double value);

		String Float2String(const double value, uint8_t digits);
		/*****************************************************************
		 * convert value to json string									 *
		 *****************************************************************/
		String Value2Json(const String& type, const String& value);
		/*****************************************************************
		 * convert string value to json string							 *
		 *****************************************************************/
		String Var2Json(const String& name, const String& value);

		/*****************************************************************
		 * convert boolean value to json string							 *
		 *****************************************************************/
		String Var2Json(const String& name, const bool value);

		/*****************************************************************
		 * convert integer value to json string							 *
		 *****************************************************************/
		String Var2Json(const String& name, const int value);
		/*****************************************************************
		 * convert double value to json string							 *
		 *****************************************************************/
		String Var2Json(const String& name, const double value);




#endif /* SENSORS_H_ */
