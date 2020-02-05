#pragma once

// Language config
#define CURRENT_LANG INTL_LANG

#include "Credentials.h"


// BasicAuth config
#define WWW_USERNAME "admin"
#define WWW_PASSWORD "feinstaub"
#define WWW_BASICAUTH_ENABLED 0
  
// Sensor Wifi config (config mode)
#define FS_SSID "sensor_cfg"
#define FS_PWD  ""

// Wohin gehen die Daten?
#define SEND2DUSTI 		0
#define SSL_DUSTI 		0
#define SEND2MADAVI 	0
#define SSL_MADAVI 		0
#define SEND2SENSEMAP 	0
#define SEND2FSAPP 		0
#define SEND2MQTT 		0
#define SEND2INFLUX 	0
#define SEND2LORA 		0
#define SEND2CSV 		0
#define SEND2CUSTOM 	0

// NTP Server
#define NTP_SERVER "0.europe.pool.ntp.org"

#define TZ_INFO "EET-2EEST,M3.5.0/3,M10.5.0/4" //"Europe Kyiv,Ukraine"
// see https://knowledgebase.progress.com/articles/Article/P129473

// OpenSenseMap
#define SENSEBOXID ""


// IMPORTANT: NO MORE CHANGES TO VARIABLE NAMES NEEDED FOR EXTERNAL APIS

// Definition eigene API
#define HOST_CUSTOM "192.168.234.1"
#define URL_CUSTOM "/data.php"
#define PORT_CUSTOM 80
#define USER_CUSTOM ""
#define PWD_CUSTOM ""

// Definition eigene InfluxDB
#define HOST_INFLUX "influx.server"
#define URL_INFLUX "/write?db=luftdaten"
#define PORT_INFLUX 8086
#define USER_INFLUX ""
#define PWD_INFLUX ""

// define pins for I2C
  #define I2C_PIN_SCL OLED_SCL
  #define I2C_PIN_SDA OLED_SDA



// define serial interface pins for particle sensors
// Serial confusion: These definitions are based on SoftSerial
// TX (transmitting) pin on one side goes to RX (receiving) pin on other side
// SoftSerial RX PIN is D1 and goes to SDS TX
// SoftSerial TX PIN is D2 and goes to SDS RX

// define serial interface pins for GPS modules

#if defined(ESP32)

	#define PM_SERIAL_RX    35	// SDS
	#define PM_SERIAL_TX    32	// SDS

	#define PM2_SERIAL_RX   25  // PMS
	#define PM2_SERIAL_TX   26	// PMS

	#define GPS_SERIAL_RX   16	// GPS
	#define GPS_SERIAL_TX   17	// GPS

	#define DEB_RX    		3	// Debug UART pins
	#define DEB_TX    		1

	#define BUT_A 			2
	#define BUT_B 			4
  
	//#define ONEWIRE_PIN     12

#endif


// DHT22, temperature, humidity
#define DHT_READ 0
#define DHT_TYPE DHT22
#define DHT_API_PIN 7

// HTU21D, temperature, humidity
#define HTU21D_READ 0
#define HTU21D_API_PIN 7

#define PPD_READ 0


// SDS011, der etwas teuerere Feinstaubsensor
#define SDS_READ 1
#define SDS_API_PIN 1

// PMS1003, PMS300, 3PMS5003, PMS6003, PMS7003
#define PMS_READ 1
#define PMS_API_PIN 1

// Honeywell PM sensor
#define HPM_READ 0
#define HPM_API_PIN 1

// BMP180, temperature, pressure
#define BMP_READ 0
#define BMP_API_PIN 3

// BMP280, temperature, pressure
#define BMP280_READ 0
#define BMP280_API_PIN 3

// BME280, temperature, humidity, pressure
#define BME280_READ 1
#define BME280_API_PIN 11

// DS18B20, temperature
#define DS18B20_READ 0
#define DS18B20_API_PIN 13


// GPS, bevorzugt Neo-6M
#define GPS_READ 1
#define GPS_API_PIN 9

// automatic firmware updates
#define AUTO_UPDATE 0

// use beta firmware
#define USE_BETA 0

// OLED Display SSD1306 angeschlossen?
#define HAS_DISPLAY 1

// OLED Display SH1106 angeschlossen?
#define HAS_SH1106 0

// LCD Display LCD1602 angeschlossen?
#define HAS_LCD1602 0

// LCD Display LCD1602 (0x27) angeschlossen?
#define HAS_LCD1602_27 0

// LCD Display LCD2004 (0x27) angeschlossen?
#define HAS_LCD2004_27 0

// Wieviele Informationen sollenber die serielle Schnittstelle ausgegeben werden?
#define DEBUG 3

// Definition der Debuglevel
#define DEBUG_ERROR 1
#define DEBUG_WARNING 2
#define DEBUG_MIN_INFO 3
#define DEBUG_MED_INFO 4
#define DEBUG_MAX_INFO 5




#if defined(ESP32)
//GPIO Pins
#include "SSD1306Wire.h" 
#define OLED_CLASS_OBJ  SSD1306Wire
#define OLED_ADDRESS    0x3C
#define OLED_SDA    21
#define OLED_SCL    22
#define OLED_RST    -1

#define SDCARD_MOSI 23
#define SDCARD_MISO 19
#define SDCARD_SCLK 18
#define SDCARD_CS   5

#endif

#define SQLITE_OK           0   /* Successful result */

enum class PmSensorCmd {
	Start,
	Stop,
	ContinuousMode,
	VersionDate
};
