/*
 * globals.h
 *
 *  Created on: Feb 5, 2020
 *      Author: E_CAD
 */

#ifndef GLOBALS_H_
#define GLOBALS_H_

#include "ext_def.h"

#define msSince(timestamp_before) (act_milli - (timestamp_before))

#include "sensors.h"




/*****************************************************************
 * Debug output																									*
 *****************************************************************/
void debug_out(const String& text, const int level, const bool linebreak);

/*****************************************************************
 * display values																								*
 *****************************************************************/
void display_debug(const String& text1, const String& text2);

String printLocalTime();

// Global variables

extern const char INTL_LANG[];


int sds_pm10_sum = 0;
int sds_pm25_sum = 0;
int sds_val_count = 0;
int sds_pm10_max = 0;
int sds_pm10_min = 20000;
int sds_pm25_max = 0;
int sds_pm25_min = 20000;

int pms_pm1_sum = 0;
int pms_pm10_sum = 0;
int pms_pm25_sum = 0;
int pms_val_count = 0;
int pms_pm1_max = 0;
int pms_pm1_min = 20000;
int pms_pm10_max = 0;
int pms_pm10_min = 20000;
int pms_pm25_max = 0;
int pms_pm25_min = 20000;

int hpm_pm10_sum = 0;
int hpm_pm25_sum = 0;
int hpm_val_count = 0;
int hpm_pm10_max = 0;
int hpm_pm10_min = 20000;
int hpm_pm25_max = 0;
int hpm_pm25_min = 20000;

double last_value_PPD_P1 = -1.0;
double last_value_PPD_P2 = -1.0;
double last_value_SDS_P1 = -1.0;
double last_value_SDS_P2 = -1.0;
double last_value_PMS_P0 = -1.0;  // PM  1.0
double last_value_PMS_P1 = -1.0;  // PM 10.0
double last_value_PMS_P2 = -1.0;  // PM  2.5
double last_value_HPM_P1 = -1.0;
double last_value_HPM_P2 = -1.0;
double last_value_DHT_T = -128.0;
double last_value_DHT_H = -1.0;
double last_value_HTU21D_T = -128.0;
double last_value_HTU21D_H = -1.0;
double last_value_BMP_T = -128.0;
double last_value_BMP_P = -1.0;
double last_value_BMP280_T = -128.0;
double last_value_BMP280_P = -1.0;
double last_value_BME280_T = -128.0;
double last_value_BME280_H = -1.0;
double last_value_BME280_P = -1.0;
double last_value_DS18B20_T = -1.0;
double last_value_GPS_lat = -200.0;
double last_value_GPS_lon = -200.0;
double last_value_GPS_alt = -1000.0;
String last_value_GPS_date = "";
String last_value_GPS_time = "";
String last_data_string = "";

double values_PMS[130] = {0.0};


bool is_SDS_running = true;
bool is_PMS_running = true;
bool is_HPM_running = true;



#pragma once

/*****************************************************************
 * Includes																											*
 *****************************************************************/
#include <FS.h>										 // must be first
#include "ext_def.h"



#ifdef ESP32
	#include <esp_wifi.h>						 // must be first

	#include <WiFiClient.h>
	#include <WiFiClientSecure.h>
	#include <WebServer.h>
	#include <ESPmDNS.h>
	#include "SPIFFS.h"
	#include "time.h"

#endif


#include "SoftwareSerial.h"
#include "Credentials.h"
#include <DNSServer.h>
#include <time.h>
#include <assert.h>
#include <base64.h>
#include <ArduinoJson.h>



// For Custom WDT
#include <Ticker.h>

#if defined(INTL_BG)
#include "intl_bg.h"
#elif defined(INTL_CZ)
#include "intl_cz.h"
#elif defined(INTL_EN)
#include "intl_en.h"
#elif defined(INTL_ES)
#include "intl_es.h"
#elif defined(INTL_FR)
#include "intl_fr.h"
#elif defined(INTL_IT)
#include "intl_it.h"
#elif defined(INTL_LU)
#include "intl_lu.h"
#elif defined(INTL_NL)
#include "intl_nl.h"
#elif defined(INTL_PL)
#include "intl_pl.h"
#elif defined(INTL_PT)
#include "intl_pt.h"
#elif defined(INTL_RU)
#include "intl_ru.h"
#elif defined(INTL_SE)
#include "intl_se.h"
//#else
//#include "intl_de.h"
#endif

#include "html-content.h"


#ifdef CFG_LCD
	#include "./oledfont.h"				// avoids including the default Arial font, needs to be included before SSD1306.h
	#include <SSD1306.h>
	#include <LiquidCrystal_I2C.h>
#endif
#ifdef CFG_BME280
	#include <Adafruit_BME280.h>
#endif
#ifdef CFG_GPS
	#include <TinyGPS++.h>
#endif
#ifdef CFG_GSHEET
	#include "lib/HTTPSRedirect.h"
#endif
#ifdef CFG_SQL
	#include <sqlite3.h>
	#include "SD.h"
	/*
	Connections:
	 * SD Card | ESP32
	 *	DAT2			 -
	 *	DAT3			 SS (D5)
	 *	CMD				MOSI (D23)
	 *	VSS				GND
	 *	VDD				3.3V
	 *	CLK				SCK (D18)
	 *	DAT0			 MISO (D19)
	 *	DAT1			 -
	*/
#endif



/******************************************************************
 * Constants																											*
 ******************************************************************/
const unsigned long SAMPLETIME_MS = 30000;
const unsigned long SAMPLETIME_SDS_MS = 1000;
const unsigned long WARMUPTIME_SDS_MS = 15000;
const unsigned long READINGTIME_SDS_MS = 5000;
const unsigned long SAMPLETIME_GPS_MS = 50;
const unsigned long DISPLAY_UPDATE_INTERVAL_MS = 5000;
const unsigned long ONE_DAY_IN_MS = 24 * 60 * 60 * 1000;
const unsigned long PAUSE_BETWEEN_UPDATE_ATTEMPTS_MS = ONE_DAY_IN_MS;				// check for firmware updates once a day
const unsigned long DURATION_BEFORE_FORCED_RESTART_MS = ONE_DAY_IN_MS * 28;	// force a reboot every ~4 weeks

#ifdef CFG_GSHEET
	const char* host = "script.google.com";
	const int httpsPort = 443;
	const char* fingerprint = "";
	int error_count = 0;
#endif

/******************************************************************
 * The variables inside the cfg namespace are persistent					*
 * configuration values. They have defaults which can be					*
 * configured at compile-time via the ext_def.h file							*
 * They can be changed by the user via the web interface, the		 *
 * changes are persisted to the flash and read back after reboot. *
 * Note that the names of these variables can't be easily changed *
 * as they are part of the json format used to persist the data.	*
 ******************************************************************/
namespace cfg {
	char wlanssid[35] = WLANSSID;
	char wlanpwd[65] = WLANPWD;

	char current_lang[3] = "DE";
	char www_username[65] = WWW_USERNAME;
	char www_password[65] = WWW_PASSWORD;
	bool www_basicauth_enabled = WWW_BASICAUTH_ENABLED;

	char fs_ssid[33] = FS_SSID;
	char fs_pwd[65] = FS_PWD;

	char version_from_local_config[20] = "";

	bool dht_read = DHT_READ;
	bool htu21d_read = HTU21D_READ;
	bool ppd_read = PPD_READ;
	bool sds_read = SDS_READ;
	bool pms_read = PMS_READ;
	bool hpm_read = HPM_READ;
	bool bmp_read = BMP_READ;
	bool bmp280_read = BMP280_READ;
	bool bme280_read = BME280_READ;
	bool ds18b20_read = DS18B20_READ;
	bool gps_read = GPS_READ;
	bool send2dusti = SEND2DUSTI;
	bool send2madavi = SEND2MADAVI;
	bool send2sensemap = SEND2SENSEMAP;
	bool send2fsapp = SEND2FSAPP;
	bool send2custom = SEND2CUSTOM;
	bool send2lora = SEND2LORA;
	bool send2influx = SEND2INFLUX;
	bool send2csv = SEND2CSV;
	bool auto_update = AUTO_UPDATE;
	bool use_beta = USE_BETA;
	bool has_display = HAS_DISPLAY;
	bool has_sh1106 = HAS_SH1106;
	bool has_lcd1602 = HAS_LCD1602;
	bool has_lcd1602_27 = HAS_LCD1602_27;
	bool has_lcd2004_27 = HAS_LCD2004_27;
	int	debug = DEBUG;

	bool ssl_madavi = SSL_MADAVI;
	bool ssl_dusti = SSL_DUSTI;
	char senseboxid[30] = SENSEBOXID;

	int port_influx = PORT_INFLUX;
	char user_influx[65] = USER_INFLUX;
	char pwd_influx[65] = PWD_INFLUX;

	char host_custom[100] = HOST_CUSTOM;
	char url_custom[100] = URL_CUSTOM;
	int port_custom = PORT_CUSTOM;
	char user_custom[65] = USER_CUSTOM;
	char pwd_custom[65] = PWD_CUSTOM;

	char host_influx[100] = HOST_INFLUX;
	char url_influx[100] = URL_INFLUX;

	unsigned long time_for_wifi_config = 10000;
	unsigned long sending_intervall_ms = 60000;	 //145000;

	void initNonTrivials(const char* id) {
		strcpy(cfg::current_lang, INTL_LANG);
		if (fs_ssid[0] == '\0') {
			strcpy(fs_ssid, "PM-");
			strcat(fs_ssid, id);
		}
	}
}

/*****************************************************************
 * Variable Definitions for PPD24NS															*
 * P1 for PM10 & P2 for PM25																		 *
 *****************************************************************/

unsigned long durationP1;
unsigned long durationP2;

boolean trigP1 = false;
boolean trigP2 = false;
unsigned long trigOnP1;
unsigned long trigOnP2;

unsigned long lowpulseoccupancyP1 = 0;
unsigned long lowpulseoccupancyP2 = 0;

bool after_send = false;
bool send_now = false;
unsigned long starttime;
unsigned long time_point_device_start_ms;
unsigned long starttime_SDS;
unsigned long starttime_GPS;
unsigned long act_micro;
unsigned long act_milli;
unsigned long last_micro = 0;
unsigned long min_micro = 1000000000;
unsigned long max_micro = 0;



unsigned long sending_time = 0;
unsigned long last_update_attempt;




SoftwareSerial Serial;

String basic_auth_influx = "";
String basic_auth_custom = "";

long int sample_count = 0;
bool bmp_init_failed = false;
bool bmp280_init_failed = false;
bool bme280_init_failed = false;

#ifdef ESP32
	WebServer server(80);
#endif

int TimeZone = 2;
String time_str = "";

#ifdef CFG_LCD
/*****************************************************************
 * Display definitions																					 *
 *****************************************************************/
SSD1306 display(0x3c, I2C_PIN_SDA, I2C_PIN_SCL); // OLED_ADDRESS

#endif

/*****************************************************************
 * SDS011 declarations																					 *
 *****************************************************************/
#ifdef ESP32

	HardwareSerial serialSDS(0);
	HardwareSerial serialPMS(1);
	HardwareSerial serialGPS(2);

#else
	SoftwareSerial serialSDS(PM_SERIAL_RX, PM_SERIAL_TX, false, 128);
	SoftwareSerial serialGPS(GPS_SERIAL_RX, GPS_SERIAL_TX, false, 512);
#endif



#ifdef CFG_BME280
/*****************************************************************
 * BME280 declaration																						*
 *****************************************************************/
Adafruit_BME280 bme280;
#endif

#ifdef CFG_GPS
/*****************************************************************
 * GPS declaration																							 *
 *****************************************************************/
	TinyGPSPlus gps;
	bool 		GPS_EN = false;
#endif





	String esp_chipid;

	long last_page_load = millis();

	bool wificonfig_loop = false;

	bool first_cycle = true;

	bool got_ntp = false;

	unsigned long count_sends = 0;
	unsigned long next_display_millis = 0;
	unsigned long next_display_count = 0;

	unsigned long timeUpdate=0;

	// Led blink delay
	// unsigned long LEDoff_millis = 0;

	struct struct_wifiInfo {
		char ssid[35];
		uint8_t encryptionType;
		int32_t RSSI;
		int32_t channel;
		bool isHidden;
	};

	struct struct_wifiInfo *wifiInfo;
	uint8_t count_wifiInfo;

	// buttons filter
	int BUT_A_CAP=0;
	int BUT_B_CAP=0;
	bool BUT_A_PRESS=false;
	bool BUT_B_PRESS=false;


	#ifdef CFG_GSHEET
		// For google spreadsheets:
		String url = String("/macros/s/") + GScriptId + "/exec?value=Hello";	// Write to Google Spreadsheet
		String url2 = String("/macros/s/") + GScriptId + "/exec?cal";				 // Fetch Google Calendar events for 1 week ahead
		String url3 = String("/macros/s/") + GScriptId + "/exec?read";				// Read from Google Spreadsheet
		String payload_base =	"{\"command\": \"appendRow\", \"sheet_name\": \"DATA\", \"values\": ";
		String payload = "";

		HTTPSRedirect* client = nullptr;
	#endif

	#ifdef CFG_AIn
	// For sensor on analog input
	double				last_value_A0_Max = -9999.0; // unitialized value. Maximum
	double				last_value_A0_Min = -9999.0; // unitialized value. Minimum
	double				last_value_A0_Avg = -9999.0; // unitialized value. Average
	uint8_t			 A0_Cnt						= 0;			 // number of measurements
	unsigned long last_A0_millis		= 0;			 // Time of last measurement
	#endif

	// Custom WDT
	#define OSWATCH_RESET_TIME 300

	static unsigned long WDT_last_loop;
	Ticker tickerOSWatch;



	const char data_first_part[] PROGMEM = "{\"software_version\": \"{v}\", \"sensordatavalues\":[";



	#ifdef CFG_SQL
		sqlite3 *db;
		int rc;
		sqlite3_stmt *res;
		int rec_count = 0;
		char *zErrMsg = 0;

		const char* data = "Callback function called";

		static int callback(void *data, int argc, char **argv, char **azColName);
		int db_open(const char *filename, sqlite3 **db);
		int db_exec(sqlite3 *db, const char *sql);

	#endif



#endif /* GLOBALS_H_ */
