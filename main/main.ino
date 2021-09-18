/* --- Sensor Setup --- */
#include "Adafruit_PM25AQI.h"
#include <Wire.h>
#include <SPI.h>
#include "math.h"
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include <SoftwareSerial.h>

// define the default sea-level pressure for altitude calculations
#define SEALEVELPRESSURE_HPA (1013.25)

// setup software serial for talking to PM2.5 sensor
SoftwareSerial pmSerial(16, 17);
Adafruit_PM25AQI aqi = Adafruit_PM25AQI();
Adafruit_BME680 bme; // I2C

// Store defined Air Quality Index break points
int AQIlow[] = {0,51,101,151,201,301};
int AQIhigh[] = {50,100,150,200,300,500};
double PM2_5ConLow[] ={0,12.1,35.5,55.5,150.5,250.5}; 
double PM2_5ConHigh[] ={12.0,35.4,55.4,150.4,250.4,500.4};

// Store defined Air Quality Index categories associated with break points
enum AQIcategory 
{   good,
    moderate, 
    UFSG,  //unhealthy for sensitive groups
    unhealthy,
    veryUnhealthy,
    hazardous,
    invalid 
};

// based on PM2.5 concentration, get AQI category
AQIcategory getRange(int pmCount){  
  
  if (pmCount < PM2_5ConHigh[0])
  {/* constant-expression */
   return good;
  }
  else if (pmCount < PM2_5ConHigh[1])
  {
    return moderate;
  }
  else if (pmCount < PM2_5ConHigh[2])/* constant-expression */
  {  
    return UFSG;
  }
  else if (pmCount < PM2_5ConHigh[3])
  {
    return unhealthy;
  }
  else if (pmCount < PM2_5ConHigh[4])
  {
    return veryUnhealthy;
  }
  else if (pmCount < PM2_5ConHigh[5])
  {
    return hazardous;
  }
  return invalid;
}

// based on PM2.5 concentration, get Air Quality Index number (0-500)
int getAQI(int pmCount){
  AQIcategory AQIcat = getRange(pmCount);
  if(AQIcat == invalid){
    return 999;
  }
  int AQI = ((AQIhigh[AQIcat] - AQIlow[AQIcat])/(PM2_5ConHigh[AQIcat]-PM2_5ConLow[AQIcat]))*(pmCount-PM2_5ConLow[AQIcat])+AQIlow[AQIcat];
  return AQI;
}

// struct to contain all reported data
struct air_data_struct {
  float temp = 0;
  float pressure = 0;
  float humidity = 0;
  float VOC = 0;
  int AQI = 0;
};

// oversample & average out data 
void collectData(int samples, int delay_time, struct air_data_struct &output){
   int AQIsamples = samples;
   int BMEsamples = samples;
       
   for(int i = 0; i < samples; i++){
      PM25_AQI_Data data;
      bool writeAQI = true;
      bool writeBME = true;
  
      // Tell BME680 to begin measurement.
      unsigned long endTime = bme.beginReading();
      if (endTime == 0) {
          Serial.println(F("Failed to begin reading :("));
          BMEsamples --;
          writeBME = false;
      }
      delay(1);
    
      if (! aqi.read(&data)) {
        Serial.println("Could not read from AQI");
        AQIsamples --;
        writeAQI = false;
      }
          
      // Obtain measurement results from BME680. Note that this operation isn't
      // instantaneous even if milli() >= endTime due to I2C/SPI latency.
      if (!bme.endReading()) {
        Serial.println(F("Failed to complete reading :("));
        if(writeBME){
          BMEsamples --;
          writeBME = false;
        }
      }

      if(writeBME){
          output.temp += bme.temperature;
          output.pressure += (bme.pressure/101325.0);
          output.humidity += bme.humidity;
          output.VOC += (bme.gas_resistance/1000.0);  
      }
      if(writeAQI){
          output.AQI += getAQI(data.pm25_standard); 
      }
      
      delay(delay_time);
   }
    output.temp /= BMEsamples;
    output.pressure /= BMEsamples;
    output.humidity /= BMEsamples;
    output.VOC /= BMEsamples;
    output.AQI /= AQIsamples;
}



/* --- WiFi Setup --- */

#if defined(ESP32)
#include <WiFiMulti.h>
WiFiMulti wifiMulti;
#define DEVICE "ESP32"
#elif defined(ESP8266)
#include <ESP8266WiFiMulti.h>
ESP8266WiFiMulti wifiMulti;
#define DEVICE "ESP8266"
#endif

// WiFi ssid & pwd
#define WIFI_SSID "bill_wi_the_science_fi"
#define WIFI_PASSWORD "adventureeveryday484"



/* --- InfluxDB Setup --- */
#include <InfluxDbClient.h>

// InfluxDB  server url. Don't use localhost, always server name or ip address.
// E.g. http://192.168.1.48:8086 (In InfluxDB 2 UI -> Load Data -> Client Libraries), 
#define INFLUXDB_URL "http://192.168.1.100:8086"
// InfluxDB v1 database name 
#define INFLUXDB_DB_NAME "air_metrics"
// InfluxDB client instance for InfluxDB 1
InfluxDBClient client(INFLUXDB_URL, INFLUXDB_DB_NAME);

// Data point
Point sensor("air_status");


/* --- Not used, more complex config, maybe someday? --- */
// InfluxDB 2 server or cloud API authentication token (Use: InfluxDB UI -> Load Data -> Tokens -> <select token>)
#define INFLUXDB_TOKEN "toked-id"
// InfluxDB 2 organization id (Use: InfluxDB UI -> Settings -> Profile -> <name under tile> )
#define INFLUXDB_ORG "org"
// InfluxDB 2 bucket name (Use: InfluxDB UI -> Load Data -> Buckets)
#define INFLUXDB_BUCKET "bucket"
// InfluxDB client instance
//InfluxDBClient client(INFLUXDB_URL, INFLUXDB_ORG, INFLUXDB_BUCKET, INFLUXDB_TOKEN);


void setup() {
  Serial.begin(9600);
  while (!Serial);

  if (!bme.begin()) {
    Serial.println(F("Could not find a valid BME680 sensor, check wiring!"));
    while (1);
  }

  pmSerial.begin(9600);
  if (! aqi.begin_UART(&pmSerial)) { // connect to the sensor over software serial 
    Serial.println("Could not find PM 2.5 sensor!");
    while (1) delay(10);
  }

  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms

  // Connect WiFi
  Serial.println("Connecting to WiFi");
  WiFi.mode(WIFI_STA);
  wifiMulti.addAP(WIFI_SSID, WIFI_PASSWORD);
  while (wifiMulti.run() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println();

  // Set InfluxDB 1 authentication params
  //client.setConnectionParamsV1(INFLUXDB_URL, INFLUXDB_DB_NAME, INFLUXDB_USER, INFLUXDB_PASSWORD);

  // Add constant tags - only once
  sensor.addTag("device", DEVICE);

  // Check server connection
  if (client.validateConnection()) {
    Serial.print("Connected to InfluxDB: ");
    Serial.println(client.getServerUrl());
  } else {
    Serial.print("InfluxDB connection failed: ");
    Serial.println(client.getLastErrorMessage());
  }
  //delay to allow sensors to boot
  delay(5000);
}

void loop() {

  air_data_struct air_data;
  collectData(10, 1000, air_data);

  // Store measured value into point
  sensor.clearFields();
  // Report RSSI of currently connected network
  sensor.addField("Temperature (°C)", air_data.temp);
  sensor.addField("Pressure (atm)", air_data.pressure);
  sensor.addField("Humidity (%)", air_data.humidity);
  sensor.addField("VOC Gas (kOhm)", air_data.VOC);
  sensor.addField("PM2.5 AQI", air_data.AQI);
  // Print what are we exactly writing
  Serial.print("Writing: ");
  Serial.println(client.pointToLineProtocol(sensor));
  // If no Wifi signal, try to reconnect it
  if (wifiMulti.run() != WL_CONNECTED) {
    Serial.println("Wifi connection lost");
  }
  // Write point
  if (!client.writePoint(sensor)) {
    Serial.print("InfluxDB write failed: ");
    Serial.println(client.getLastErrorMessage());
  }  

/* --- Old data Acquition 
   PM25_AQI_Data data;

  // Tell BME680 to begin measurement.
  unsigned long endTime = bme.beginReading();
  if (endTime == 0) {
    Serial.println(F("Failed to begin reading :("));
    return;
  }
  Serial.print(F("Reading started at "));
  Serial.print(millis());
  Serial.print(F(" and will finish at "));
  Serial.println(endTime);

  if (! aqi.read(&data)) {
    Serial.println("Could not read from AQI");
    delay(500);  // try again in a bit!
    return;
  } else {
    Serial.println("Read PM Sensor data");
  }

  // delay(50); // This represents parallel work.
  // There's no need to delay() until millis() >= endTime: bme.endReading()
  // takes care of that. It's okay for parallel work to take longer than
  // BME680's measurement time.

  // Obtain measurement results from BME680. Note that this operation isn't
  // instantaneous even if milli() >= endTime due to I2C/SPI latency.
  if (!bme.endReading()) {
    Serial.println(F("Failed to complete reading :("));
    return;
  }
  Serial.print(F("Reading completed at "));
  Serial.println(millis());

  Serial.print(F("Temperature = "));
  Serial.print(bme.temperature);
  Serial.println(F(" *C"));

  Serial.print(F("Pressure = "));
  Serial.print(bme.pressure / 100.0);
  Serial.println(F(" hPa"));

  Serial.print(F("Humidity = "));
  Serial.print(bme.humidity);
  Serial.println(F(" %"));

  Serial.print(F("Gas = "));
  Serial.print(bme.gas_resistance / 1000.0);
  Serial.println(F(" KOhms"));

  Serial.print(F("Approx. Altitude = "));
  Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(F(" m"));
  
  Serial.print("Air Quality Index: ");
  Serial.println(getAQI(data.pm25_standard));
  Serial.println();
  Serial.println();
 
  // Store measured value into point
  sensor.clearFields();
  // Report RSSI of currently connected network
  sensor.addField("Temperature (°C)", bme.temperature);
  sensor.addField("Pressure (atm)", bme.pressure/101325.0);
  sensor.addField("Humidity (%)", bme.humidity);
  sensor.addField("VOC Gas (kOhm)", bme.gas_resistance/1000.0);
  sensor.addField("PM2.5 AQI", getAQI(data.pm25_standard));
  // Print what are we exactly writing
  Serial.print("Writing: ");
  Serial.println(client.pointToLineProtocol(sensor));
  // If no Wifi signal, try to reconnect it
  if (wifiMulti.run() != WL_CONNECTED) {
    Serial.println("Wifi connection lost");
  }
  // Write point
  if (!client.writePoint(sensor)) {
    Serial.print("InfluxDB write failed: ");
    Serial.println(client.getLastErrorMessage());
  }  
  
  delay(10000);
   ---- */

}
