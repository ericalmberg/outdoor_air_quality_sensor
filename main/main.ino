/* --- Sensor Setup --- */
//#include "Adafruit_PM25AQI.h"
#include <Wire.h>
#include <SPI.h>
#include "math.h"
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include <SoftwareSerial.h>

// define the default sea-level pressure for altitude calculations
#define SEALEVELPRESSURE_HPA (1013.25)

// setup software serial for talking to PM2.5 sensor
SoftwareSerial pmsSerial(16, 17);
//Adafruit_PM25AQI aqi = Adafruit_PM25AQI();
Adafruit_BME680 bme; // I2C

// struct for storing raw PMS5003 Particlate matter data
struct pms5003data {
  uint16_t framelen;
  uint16_t pm10_standard, pm25_standard, pm100_standard;
  uint16_t pm10_env, pm25_env, pm100_env;
  uint16_t particles_03um, particles_05um, particles_10um, particles_25um, particles_50um, particles_100um;
  uint16_t unused;
  uint16_t checksum;
};

// struct to contain all reported data
struct data_struct {
  float temp = 0;
  float pressure = 0;
  float humidity = 0;
  float VOC = 0;
  float lux = 0;
  int AQI = 0;
};
 
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

// create globally accessible struct instance to dump raw PM sensor data to
struct pms5003data data;
struct data_struct output_data;

int luxSensorPin = 36;    // select the input pin for the potentiometer

float rawRange = 4096; // 3.3v
float logRange = 5.0; // 3.3v = 10^5 lux

float RawToLux(float raw)
{
  float logLux = raw * logRange / rawRange;
  return pow(10, logLux);
}


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

// Serial interface funtion to read PM2.5 data to fix bug of Adafruit lib (not reading after a while)
boolean readPMSdata(Stream *s) {
  if (! s->available()) {
    return false;
  }
  
  // Read a byte at a time until we get to the special '0x42' start-byte
  if (s->peek() != 0x42) {
    s->read();
    return false;
  }
 
  // Now read all 32 bytes
  if (s->available() < 32) {
    return false;
  }
    
  uint8_t buffer[32];    
  uint16_t sum = 0;
  s->readBytes(buffer, 32);
 
  // get checksum ready
  for (uint8_t i=0; i<30; i++) {
    sum += buffer[i];
  }
 
  /* debugging
  for (uint8_t i=2; i<32; i++) {
    Serial.print("0x"); Serial.print(buffer[i], HEX); Serial.print(", ");
  }
  Serial.println();
  */
  
  // The data comes in endian'd, this solves it so it works on all platforms
  uint16_t buffer_u16[15];
  for (uint8_t i=0; i<15; i++) {
    buffer_u16[i] = buffer[2 + i*2 + 1];
    buffer_u16[i] += (buffer[2 + i*2] << 8);
  }
 
  // put it into a nice struct :)
  memcpy((void *)&data, (void *)buffer_u16, 30);
 
  if (sum != data.checksum) {
    Serial.println("Checksum failure");
    return false;
  }
  // success!
  return true;
}

void reset_output_data(){
    output_data.temp = 0;
    output_data.pressure = 0;
    output_data.humidity = 0;
    output_data.VOC = 0;
    output_data.AQI = 0;
    output_data.lux = 0;
}

bool retry_logic(){
    for(int j = 1; j <= 5; j++){
       Serial.print("retry #");
       Serial.println(j);
       if (!readPMSdata(&pmsSerial)){  
          // if we are not reading error condition, exit and return true that we got a valid datapoint
          if(data.pm25_standard < 5000 && data.pm25_standard != 999 && data.pm25_standard != 66 && data.pm25_standard != 28 && data.pm25_standard != 151 && data.pm25_standard != 256 && data.pm25_standard != 322 && data.pm25_standard != 512 && data.pm25_standard != 578 && data.pm25_standard != 834 && data.pm25_standard != 768 && data.pm25_standard != 1024 && data.pm25_standard != 1090 && data.pm25_standard != 1280 && data.pm25_standard != 1346 && data.pm25_standard != 1536 && data.pm25_standard != 1858 && data.pm25_standard != 2048 && data.pm25_standard != 2114 && data.pm25_standard != 2304 && data.pm25_standard != 2370 && data.pm25_standard != 2560 && data.pm25_standard != 3072 && data.pm25_standard != 3138 && data.pm25_standard != 4096 && data.pm25_standard != 4674 && data.pm25_standard != 19712){
              return true;
          }
       }
       delay(500);
    }
    
   // if all else fails restart serial connection
    pmsSerial.end();
    Serial.println("restarting PMS sensor serial...");
    delay(1000);
    pmsSerial.begin(9600);
    while(!pmsSerial);
    delay(1000);
    return false;
}

// oversample & average out data 
void collectData(int samples, int delay_time){
   float AQIsamples = samples;
   float BMEsamples = samples;
   int last_reading = 0;
   int bad_read_count = 0;
   delay_time -= 300;
   reset_output_data();
       
   for(int i = 0; i < samples; i++){
      bool writeAQI = true;
      bool writeBME = true;
  
      // Tell BME680 to begin measurement.
      unsigned long endTime = bme.beginReading();
      if (endTime == 0) {
          Serial.println(F("Failed to begin reading :("));
          BMEsamples --;
          writeBME = false;
      }
      delay(100);

      // Obtain measurement results from BME680. Note that this operation isn't
      // instantaneous even if milli() >= endTime due to I2C/SPI latency.
      if (!bme.endReading()) {
        Serial.println(F("Failed to complete reading :("));
        if(writeBME){
          BMEsamples --;
          writeBME = false;
        }
      }
      delay(100);

      // Obtain measurements from PM2.5 sensor, handles Serial disconnect and hung condition bugs
      if (!readPMSdata(&pmsSerial)) {
        Serial.println("Could not read from AQI");
        AQIsamples --;
        writeAQI = false;
        delay(50);
        //retry logic
        if(retry_logic()){
          AQIsamples ++;
          writeAQI = true;
        }
      }
      if(data.pm25_standard > 100 && data.pm25_standard == last_reading) {
         Serial.println("Suspected bad AQI reading");
         bad_read_count ++;
         if(bad_read_count >=5){
            AQIsamples -= 5;
            writeAQI = false;
            output_data.AQI = 0;
         }
      } else {
         last_reading = data.pm25_standard;
         bad_read_count = 0;
      }


      // Add datapoints to sum if they were successfully read
      if(writeBME){
          output_data.temp += bme.temperature;
          output_data.pressure += (bme.pressure/101325.0);
          output_data.humidity += bme.humidity;
          output_data.VOC += (bme.gas_resistance/1000.0);  
      }
      if(writeAQI){
          output_data.AQI += getAQI(data.pm25_standard); 
          Serial.print("write AQI ");
          Serial.println(getAQI(data.pm25_standard));
          Serial.print("Raw PM2.5: ");
          Serial.println(data.pm25_standard);
      } else if (writeAQI){
          AQIsamples --;
      }
      float raw_lux = analogRead(luxSensorPin);
      float lux_value = 0;
      lux_value = RawToLux(raw_lux);
      output_data.lux += lux_value; 
      delay(delay_time);
   }
   if(BMEsamples != 0){
      output_data.temp /= BMEsamples;
      output_data.pressure /= BMEsamples;
      output_data.humidity /= BMEsamples;
      output_data.VOC /= BMEsamples; 
   }
   if(AQIsamples != 0){
      output_data.AQI /= AQIsamples;
   }
   output_data.lux /= samples;
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

  analogSetAttenuation(ADC_11db); // set attenuation to configure most accurate ~.15-2.5V sensing on ADC
  pmsSerial.begin(9600); // start PMS5003 sensor serial stream
  
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
  sensor.addTag("location", "Greenhouse");

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

  // collect data with error handling, 10 samples 1s each
  collectData(10, 1000);

  // Clear and report all data fields
  sensor.clearFields();
  sensor.addField("Temperature (°C)", output_data.temp);
  sensor.addField("Pressure (atm)", output_data.pressure);
  sensor.addField("Humidity (%)", output_data.humidity);
  sensor.addField("VOC Gas (kOhm)", output_data.VOC);
  sensor.addField("PM2.5 AQI", output_data.AQI);
  sensor.addField("Brightness (lux)", output_data.lux);

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

}
