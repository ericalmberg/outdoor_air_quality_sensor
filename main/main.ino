/* --- Sensor Setup --- */
#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_VEML7700.h"
#include "Adafruit_SHT31.h"

// struct to contain all reported data
struct data_struct {
  float temp = 0;
  float soil_temp = 0;
  float soil_moisture = 0;
  float humidity = 0;
  float lux = 0;
};



// instantiate globals
Adafruit_VEML7700 veml = Adafruit_VEML7700();
Adafruit_SHT31 sht31 = Adafruit_SHT31();
struct data_struct output_data;
bool enableHeater = false;



void reset_output_data(){
    output_data.temp = 0;
    output_data.humidity = 0;
    output_data.lux = 0;
}



// oversample & average out data 
void collectData(int samples, int delay_time){
   float SHTsamples = samples;
   reset_output_data();
       
   for(int i = 0; i < samples; i++){
      bool writeSHT = true;
  
      // get SHT temp & humidity measurement.
      float t = sht31.readTemperature();
      float h = sht31.readHumidity();

      // check temp reading validity
      if (! isnan(t)) {  // check if 'is not a number'
        //Serial.print("Temp *C = "); Serial.print(t);
      } else { 
        if(writeSHT){
          SHTsamples --;
          writeSHT = false;
          Serial.println("Failed to read temperature");
        }
      }

      // check humidity reading validity
      if (! isnan(h)) {  // check if 'is not a number'
        //Serial.print("Hum. % = "); Serial.println(h);
      } else { 
        if(writeSHT){
          SHTsamples --;
          writeSHT = false;
          Serial.println("Failed to read humidity");
        }
      }
            
      // Add datapoints to sum if they were successfully read
      if(writeSHT){
          output_data.temp += t;
          output_data.humidity += h;
          Serial.print("write SHT - temp: ");
          Serial.print(t);
          Serial.print(" humidity: ");
          Serial.print(h);
      }
      
      // get Lux reading from VEML
      float lux_value = veml.readLux();
      output_data.lux += lux_value; 
      Serial.print(" write lux: ");
      Serial.println(lux_value);
      delay(delay_time);
   }
   if(SHTsamples != 0){
      output_data.temp /= SHTsamples;
      output_data.humidity /= SHTsamples;
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
#define WIFI_SSID "#"
#define WIFI_PASSWORD "#"

/* --- InfluxDB Setup --- */
#include <InfluxDbClient.h>

// InfluxDB  server url. Don't use localhost, always server name or ip address.
// E.g. http://192.168.1.48:8086 (In InfluxDB 2 UI -> Load Data -> Client Libraries), 
#define INFLUXDB_URL "http://192.168.2.100:8086"
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

  // start veml7700 sensor via i2c
  if (!veml.begin()) {
    Serial.println("VEML7700 not found");
    while (1);
  }
  Serial.println("VEML7700 found");

  // set gain to 1/8 and integration time to 25ms to achieve maximum resolution to 120klux
  // lux scales linearly with increases to IT and gain i.e. 100ms IT = 30klux max reading
  veml.setGain(VEML7700_GAIN_1_8);
  veml.setIntegrationTime(VEML7700_IT_25MS);
  veml.powerSaveEnable(true);
  veml.setPowerSaveMode(VEML7700_POWERSAVE_MODE1);

  //start sht31-d sensor via i2c
  if (! sht31.begin(0x44)) {   // Set to 0x45 for alternate i2c addr
     Serial.println("Couldn't find SHT31");
  } else {
    Serial.println("SHT31 found");
  }
  Serial.print("Heater Enabled State: ");
  if (sht31.isHeaterEnabled())
    Serial.println("ENABLED");
  else
    Serial.println("DISABLED");



  // Connect WiFi
  Serial.println("Connecting to WiFi");
  WiFi.mode(WIFI_STA);
  wifiMulti.addAP(WIFI_SSID, WIFI_PASSWORD);
  while (wifiMulti.run() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.print("My ip address: ");
  Serial.println(WiFi.localIP());

  // Set InfluxDB 1 authentication params
  //client.setConnectionParamsV1(INFLUXDB_URL, INFLUXDB_DB_NAME, INFLUXDB_USER, INFLUXDB_PASSWORD);

  // Add constant tags - only once
  sensor.addTag("device", DEVICE);
  //sensor.addTag("type", "temp-humidity-lux");
  sensor.addTag("location", "test");

  // Check server connection
  if (client.validateConnection()) {
    Serial.print("Connected to InfluxDB: ");
    Serial.println(client.getServerUrl());
  } else {
    Serial.print("InfluxDB connection failed: ");
    Serial.println(client.getLastErrorMessage());
  }
  //delay to allow sensors to boot
  delay(1000);
}

void loop() {

  // collect data with error handling, 10 samples 1s each
  collectData(10, 1000);

  // Clear and report all data fields
  sensor.clearFields();
  sensor.addField("Temperature (°C)", output_data.temp);
  sensor.addField("Humidity (%)", output_data.humidity);
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
