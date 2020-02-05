/*
 * convert.h
 *
 *  Created on: Feb 5, 2020
 *      Author: E_CAD
 */

#ifndef CONVERT_H_
#define CONVERT_H_

#include <Arduino.h>
#include <string>
#include "globals.h"

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



#endif /* CONVERT_H_ */
