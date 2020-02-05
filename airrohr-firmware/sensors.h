



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
