//WiFi
#include <SPI.h>
#include <WiFiNINA.h>
#include "arduino_secrets.h" 
///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)
int status = WL_IDLE_STATUS;     // the WiFi radio's status

//MQTT
#include <ArduinoMqttClient.h>
WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

const char broker[] = MQTT_BROKER;
int        port     = 1883;
//const char topic[]  = "plants_kitchen/simple";

//Air temperature/humidity
#include "DHT.h"
#define DHTTYPE DHT11
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

//General config
const unsigned long update_interval         = 1000; //[ms] How often to loop (general)
const unsigned long update_interval_DHT     = 5000; //[ms] How often to loop the DHT

//Global vars
unsigned long prevUpdateTime     = 0;
unsigned long prevUpdateTime_DHT = 0;
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


  
  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }
  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }
  connectWifi();


  // You can provide a unique client ID, if not set the library uses Arduino-millis()
  // Each client must have a unique client ID
  // mqttClient.setId("clientId");

  // You can provide a username and password for authentication
  // mqttClient.setUsernamePassword("username", "password");

  Serial.print("Attempting to connect to the MQTT broker: ");
  Serial.println(broker);

  if (!mqttClient.connect(broker, port)) {
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());

    while (1);
  }

  Serial.println("You're connected to the MQTT broker!");
  Serial.println();


  
  
  airTempHumid.begin();

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
  mqttClient.poll();


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

    mqttClient.beginMessage(soilhumid_topic[i]);
    mqttClient.print(soilhumid_data[i]);
    mqttClient.endMessage();
  }

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
  mqttClient.beginMessage(airTempHumid_topicTemp);
  mqttClient.print(temp);
  mqttClient.endMessage();
    
  snprintf(serialBuff,serialBuff_len, "%s [%%] = ", airTempHumid_topicHumid);
  Serial.print(serialBuff);
  Serial.println(humid);
  mqttClient.beginMessage(airTempHumid_topicHumid);
  mqttClient.print(humid);
  mqttClient.endMessage();

}


// **** WIFI CODE ***** //


void connectWifi() {
  if (status == WL_CONNECTED) {
    return;
  }
  
  // attempt to connect to WiFi network:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network:
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    //delay(10000);
    while (WiFi.status() == WL_IDLE_STATUS) {
      Serial.print(".");
      delay(100);
    }
  }
  Serial.println();
  // you're connected now, so print out the data:
  Serial.print("You're connected to the network");
  printCurrentNet();
  printWifiData();
  Serial.println();
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
