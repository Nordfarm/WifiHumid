//WiFi
#include <SPI.h>
#include <WiFiNINA.h>
#include "arduino_secrets.h" 
///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[]    = SECRET_SSID;        // your network SSID (name)
char pass[]    = SECRET_PASS;        // your network password (use for WPA, or use as key for WEP)

int WIFIstatus = WL_IDLE_STATUS;     // the WiFi radio's status
bool remoteOK  = false;              // WiFi and MQTT is connected?
int remoteLED  = 7;                  // Pin for remoteOK indicator LED


//MQTT
#include <ArduinoMqttClient.h>
WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

const char broker[]   = MQTT_BROKER;
int        port       = 1883;
//const char topic[]  = "plants_kitchen/simple";
const unsigned long MQTTtimeout = 5000; //[ms]

//Air temperature/humidity
#include "DHT.h"
#define DHTTYPE DHT22
#define DHTPIN 2
DHT airTempHumid(DHTPIN, DHTTYPE);

const char* const airTempHumid_topicTemp  = "plants_kitchen/airTemp";
const char* const airTempHumid_topicHumid = "plants_kitchen/airHumid";


//Soil humidity (capacitive sensor)
const int soilhumid_num    = 2;
const int soilhumid_pins[] = {A0, A1};
const char* const soilhumid_topic[] = {"plants_kitchen/soilhumid0",
                                       "plants_kitchen/soilhumid1"};
float soilhumid_data[2];

// Calibration of soil humidity
const float soil100 = 2.48; //[V] Water
const float soil0   = 3.65; //[V] Air

// Acquisition parameters for soil humidity
const int           soil_nsamples     = 10;
const unsigned long soil_sampleDelay = 10; //[ms]

// Soilhumid LEDs and their limits
const size_t soilhumid_nLEDS  = 6;
const int    soilhumid_LEDS[] = { 8,  9, 10, 11, 12, 13};
const float  soilhumid_LIMS[] = {50, 60, 70, 80, 90,100};

//General config
const unsigned long update_interval         = 1000L;      //[ms] How often to loop (general)
const unsigned long update_interval_DHT     = 5000L;      //[ms] How often to loop the DHT
const unsigned long update_interval_WIFI    = 5*60*1000L; //[ms] How often to try reconnecting the WiFi

//Global vars
unsigned long prevUpdateTime      = 0L;
unsigned long prevUpdateTime_DHT  = 0L;
unsigned long prevUpdateTime_WIFI = 0L;

#define serialBuff_len 100
char serialBuff[serialBuff_len];

// *** THE CODE ***

void setup() {

  Serial.begin(9600);
  int serialCounter=0;
  while (!Serial) {
    //However only wait for 1 second before giving up,
    // so that it can work also when not connected to USB
    delay(100);
    serialCounter++;
    if (serialCounter > 10) break;
  }
  Serial.println("WifiHumid initializing...");

  pinMode(remoteLED, OUTPUT);
  
  // check for the WiFi module:
  WIFIstatus = WiFi.status();
  if (WIFIstatus == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }
  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }
  connectWifi(true); //Also MQTT

  //Initialize air temperature & humidity sensor
  airTempHumid.begin();

  //Initialize LEDS
  for (size_t i=0; i < soilhumid_nLEDS; i++) {
    pinMode     (soilhumid_LEDS[i],OUTPUT);
    digitalWrite(soilhumid_LEDS[i],HIGH);
    delay(50);
    digitalWrite(soilhumid_LEDS[i],LOW);
  }
  
  
  
  Serial.println("WifiHumid ready!");
}




void loop() {
  // ** Flood control **
  //Only update at intervals, not "as fast as we can possibly go"
  //This is the minimum interval; if e.g. web server takes more time, it will be slowed down.
  unsigned long thisUpdateTime = millis();
  if (not (thisUpdateTime - prevUpdateTime >= update_interval)) {
    //Not ready yet.
    //Note that this should be safe when millis rolls over,
    // however the interval when it happens will almost certainly be shorter
    return;
  }
  prevUpdateTime = thisUpdateTime;

  // Housekeeping
  connectWifi(false);
  if (remoteOK) {
    mqttClient.poll();
  }

  soilhumid_read();

  // ** Flood control **
  //Only update at intervals, not "as fast as we can possibly go"
  //This is the minimum interval; if e.g. web server takes more time, it will be slowed down.
  if (not (thisUpdateTime - prevUpdateTime_DHT >= update_interval_DHT)) {
    //Not ready yet.
    //Note that this should be safe when millis rolls over,
    // however the interval when it happens will almost certainly be shorter
    return;
  }
  prevUpdateTime_DHT = thisUpdateTime;

  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float humid = airTempHumid.readHumidity();
  // Read temperature as Celsius (the default)
  float temp = airTempHumid.readTemperature();
  if (isnan(humid) || isnan(temp)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }

  snprintf(serialBuff,serialBuff_len, "%s [degC] = ", airTempHumid_topicTemp);
  Serial.print(serialBuff);
  Serial.println(temp);
  if (remoteOK) {
    mqttClient.beginMessage(airTempHumid_topicTemp);
    mqttClient.print(temp);
    mqttClient.endMessage();
  }  
  snprintf(serialBuff,serialBuff_len, "%s [%%] = ", airTempHumid_topicHumid);
  Serial.print(serialBuff);
  Serial.println(humid);
  if (remoteOK) {
    mqttClient.beginMessage(airTempHumid_topicHumid);
    mqttClient.print(humid);
    mqttClient.endMessage();
  }
}

// **** SOILHUMID CODE ***** //

void soilhumid_read() {
  // Read soil humidity data
  for (int i=0; i<soilhumid_num; i++) {
    float soilhumid_data_tmp1 = 0;
    for (int j=0; j < soil_nsamples; j++) {
      float soilhumid_data_tmp2 = (float(analogRead(soilhumid_pins[i]))/1023.0)*5.0; // read sensor
      soilhumid_data_tmp2       = 100*(soil0-soilhumid_data_tmp2)/(soil0-soil100); //Calibrate to percent
      soilhumid_data_tmp1      += soilhumid_data_tmp2;

      delay(soil_sampleDelay);
    }
    soilhumid_data[i] = soilhumid_data_tmp1/soil_nsamples;

    //Serial.print("Soilhumid[");
    //Serial.print(soilhumid_data[i]);
    snprintf(serialBuff,serialBuff_len, "plants_kitchen/soilhumid%d [%%] = ", i);
    Serial.print(serialBuff);
    Serial.println(soilhumid_data[i]);

    if (remoteOK) {
      mqttClient.beginMessage(soilhumid_topic[i]);
      mqttClient.print(soilhumid_data[i]);
      mqttClient.endMessage();
    }
  }

  static int showLED = 0;
  soilhumid_leds(showLED);
  showLED = (showLED+1)%2;
  
}

void soilhumid_leds(size_t sensorID) {
  for (size_t i=0; i < soilhumid_nLEDS; i++) {
    if (soilhumid_data[sensorID] > soilhumid_LIMS[i]) {
      digitalWrite(soilhumid_LEDS[i],HIGH);
    }
    else {
      digitalWrite(soilhumid_LEDS[i],LOW);
    }
  }
}

// **** WIFI CODE ***** //


void connectWifi(bool doItNow) {
  // ** Flood control **
  //Only update at intervals, not "as fast as we can possibly go"
  //This is the minimum interval; if something takes more time, it will be slowed down.
  unsigned long thisUpdateTime = millis();
  if (not doItNow) {
    if (not ( (thisUpdateTime - prevUpdateTime_WIFI) >= update_interval_WIFI) ) {
      //Not ready yet.
      //Note that this should be safe when millis rolls over,
      // however the interval when it happens will almost certainly be shorter
      return;
    }
  }
  prevUpdateTime_WIFI = thisUpdateTime;
  // Note: Also updated at bottom of function,
  // to only start counting time once fuction is done
  
  WIFIstatus = WiFi.status();
  if (WIFIstatus != WL_CONNECTED) {
    // attempt to connect to WiFi network:
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network:
    WIFIstatus = WiFi.begin(ssid, pass);
    // wait for connection:
    int loopCounter = 0;
    while ((WIFIstatus=WiFi.status()) == WL_IDLE_STATUS && loopCounter < 10) {
      Serial.print(".");
      loopCounter++;
      delay(100);
    }
    if (loopCounter == 10) {
      Serial.println("Wifi connection timed out.");
    }
    Serial.println();
    if (WIFIstatus == WL_CONNECTED) {
      // you're connected now, so print out the data:
      Serial.print("You're connected to the network");
      printCurrentNet();
      printWifiData();
      Serial.println();
    }
    else {
      Serial.println("Network connection failed!");
    }
  }

  //MQTT setup
  if (WIFIstatus == WL_CONNECTED) {
    // You can provide a unique client ID, if not set the library uses Arduino-millis()
    // Each client must have a unique client ID
    // mqttClient.setId("clientId");

    // You can provide a username and password for authentication
    // mqttClient.setUsernamePassword("username", "password");
    if (! mqttClient.connected()) {
      Serial.print("Attempting to connect to the MQTT broker: ");
      Serial.println(broker);

      mqttClient.setConnectionTimeout(MQTTtimeout);
      if (!mqttClient.connect(broker, port)) {
        Serial.print("MQTT connection failed! Error code = ");
        Serial.println(mqttClient.connectError());
        Serial.println("Will try again later");
      }
      else {
        Serial.println("You're connected to the MQTT broker!");
        Serial.println();
      }
    }
  }

  // Set the overal remote status bit
  if (WIFIstatus == WL_CONNECTED && mqttClient.connected()) {
    remoteOK = true;
    digitalWrite(remoteLED,HIGH);
  }
  else {
    remoteOK = false;
    digitalWrite(remoteLED,LOW);
  }
  prevUpdateTime_WIFI = millis();
}

void printWifiData() {
  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);
  Serial.println(ip);

  // print your MAC address:
  byte mac[6];
  WiFi.macAddress(mac);
  Serial.print("MAC address: ");
  printMacAddress(mac);
}

void printCurrentNet() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print the MAC address of the router you're attached to:
  byte bssid[6];
  WiFi.BSSID(bssid);
  Serial.print("BSSID: ");
  printMacAddress(bssid);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.println(rssi);

  // print the encryption type:
  byte encryption = WiFi.encryptionType();
  Serial.print("Encryption Type:");
  Serial.println(encryption, HEX);
  Serial.println();
}

void printMacAddress(byte mac[]) {
  for (int i = 5; i >= 0; i--) {
    if (mac[i] < 16) {
      Serial.print("0");
    }
    Serial.print(mac[i], HEX);
    if (i > 0) {
      Serial.print(":");
    }
  }
  Serial.println();
}
