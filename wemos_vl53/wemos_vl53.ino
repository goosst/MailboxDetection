/* sg 25 july 2020
* code for wemos d1 mini, reads VL53 sensor and broadcasts its status through an MQTT message
* - goes into deepsleep after a successful reading, and wakes up through an external reset
* - disables the VL53 sensor by making use of the shutoff functionality
* - communicates status to (an arduino pro mini), choice of pins is linked to pullups used in the wemos 
*/

#include <ESP8266WiFi.h>  //For ESP8266
#include <PubSubClient.h> //For MQTT
#include <ESP8266mDNS.h>  //For OTA
#include <WiFiUdp.h>      //For OTA
#include <ArduinoOTA.h>   //For OTA
#include <Wire.h>
#include <VL53L0X.h>

#include "configuration.h" // to store passwords of mqtt and wifi

VL53L0X sensor; //distance in mm
//#define LONG_RANGE
//#define HIGH_SPEED
#define HIGH_ACCURACY

int Distance;
const int pin_enablesensor = D8;
const int pin_informarduino = D3; //send information to arduino
const int PWM_frequency_start = 10;
const int PWM_frequency = 500;

String mqtt_client_id = "ESP8266-"; //This text is concatenated with ChipId to get unique client_id
//MQTT Topic configuration
String mqtt_base_topic = "/sensor/" + mqtt_client_id + "/data";
#define distance_topic "/distance"

//MQTT client
WiFiClient espClient;
PubSubClient mqtt_client(espClient);

//Necesary to make Arduino Software autodetect OTA device
WiFiServer TelnetServer(8266);

void setup_wifi()
{
  delay(10);
  Serial.print("Connecting to ");
  Serial.print(wifi_ssid);
  WiFi.begin(wifi_ssid, wifi_password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("OK");
  Serial.print("   IP address: ");
  Serial.println(WiFi.localIP());
}

void setup()
{
  Serial.begin(115200);
  Serial.println("\r\nBooting...");

  setup_wifi();

  Serial.print("Configuring OTA device...");
  TelnetServer.begin();   //Necesary to make Arduino Software autodetect OTA device
  ArduinoOTA.onStart([]() {
    Serial.println("OTA starting...");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("OTA update finished!");
    Serial.println("Rebooting...");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("OTA in progress: %u%%\r\n", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
  Serial.println("OK");

  Serial.println("Configuring MQTT server...");
  mqtt_client_id = mqtt_client_id + ESP.getChipId();
  mqtt_base_topic = "/sensor/" + mqtt_client_id + "/data";
  mqtt_client.setServer(mqtt_server, 1883);
  Serial.printf("   Server IP: %s\r\n", mqtt_server);
  Serial.printf("   Username:  %s\r\n", mqtt_user);
  Serial.println("   Cliend Id: " + mqtt_client_id);
  Serial.println("   MQTT configured!");

  Serial.println("Setup completed! Running app...");


  Wire.begin();
  pinMode(pin_enablesensor, OUTPUT);
  digitalWrite(pin_enablesensor, HIGH);

  pinMode(pin_informarduino, OUTPUT);
  digitalWrite(pin_informarduino, HIGH);

  //  analogWrite(pin_informarduino, 0);

  //  pinMode(pin_informarduino, OUTPUT);
  //  digitalWrite(pin_informarduino, LOW);


  sensor.init();
  sensor.setTimeout(500);

#if defined LONG_RANGE
  // lower the return signal rate limit (default is 0.25 MCPS)
  sensor.setSignalRateLimit(0.1);
  // increase laser pulse periods (defaults are 14 and 10 PCLKs)
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
#endif

#if defined HIGH_SPEED
  // reduce timing budget to 20 ms (default is about 33 ms)
  sensor.setMeasurementTimingBudget(20000);
#elif defined HIGH_ACCURACY
  // increase timing budget to 200 ms
  sensor.setMeasurementTimingBudget(200000);
#endif

}


void mqtt_reconnect()
{
  // Loop until we're reconnected
  while (!mqtt_client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    // If you do not want to use a username and password, change next line to
    // if (client.connect("ESP8266Client")) {
    if (mqtt_client.connect(mqtt_client_id.c_str(), mqtt_user, mqtt_password)) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqtt_client.state());
      Serial.println(" try again in 5 seconds");
      // Wait milliseconds before retrying
      delay(3000);
    }
  }
}


bool checkBound(float newValue, float prevValue, float maxDiff) {
  return (true);
  return newValue < prevValue - maxDiff || newValue > prevValue + maxDiff;
}


int min_timeout = 20000; //in ms
int count_reads = 0;

void loop()
{

  ArduinoOTA.handle();

  if (!mqtt_client.connected())
  {
    mqtt_reconnect();
  }
  mqtt_client.loop();

  Distance = sensor.readRangeSingleMillimeters();
  if (sensor.timeoutOccurred()) {
    Distance = 32766;
  }
  mqtt_client.publish((mqtt_base_topic + distance_topic).c_str(), String(Distance).c_str(), true);


  //reading was ok, amount of attempts has passed
  if (Distance < 10000 || count_reads > 4)
  {
    digitalWrite(pin_enablesensor, LOW); //disable VL53 sensor (and its current consumption)

    //inform arduino successful measurement was done, by sending PWM at 500Hz with 70% duty ratio
    //    analogWriteFreq(PWM_frequency);
    //    analogWrite(pin_informarduino, 714);
    //    analogWrite(pin_informarduino, 512);

    digitalWrite(pin_informarduino, LOW);
    delay(2000); //wait for arduino to shut esp down
    
    ESP.deepSleep(0); //this is in case the ESP has not shutdown because there was still enough battery voltage remaining
    //the RST pin will be enabled 
    

  }
  else
  {
    delay(min_timeout); //try again after x ms
  }



  if (count_reads < 32767)
  {
    count_reads = count_reads + 1;
  }
  else
  {
    count_reads = 0;
  }
}
