/*
 * sensors.h
 *
 *  Created on: Feb 5, 2020
 *      Author: E_CAD
 */

#ifndef SENSORS_H_
#define SENSORS_H_


#include <string>
#include "convert.h"
#include "globals.h"


	template<typename T, std::size_t N> constexpr std::size_t array_num_elements(const T(&)[N]) {
		return N;
	}

	template<typename T, std::size_t N> constexpr std::size_t capacity_null_terminated_char_array(const T(&)[N]) {
		return N - 1;
	}




	static bool SDS_cmd(PmSensorCmd cmd);
	static bool PMS_cmd(PmSensorCmd cmd);
	static bool HPM_cmd(PmSensorCmd cmd);

	String SDS_version_date();
	String sensorSDS();
	String sensorPMS();
	String sensorHPM();
	String sensorGPS();


	void disable_unneeded_nmea();

	#ifdef CFG_BME280
		static String sensorBME280();
	#endif





#endif /* SENSORS_H_ */
