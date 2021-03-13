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
const char topic[]  = "arduino/simple";

//Air temperature/humidity
#include "DHT.h"
#define DHTTYPE DHT11
#define DHTPIN 2
DHT dht(DHTPIN, DHTTYPE);

//Soil humidity
const int soilhumid_num    = 2;
const int soilhumid_pins[] = {A0, A1};
// Calibration
const float soil100 = 2.48; //[V] Water
const float soil0   = 3.65; //[V] Air

float soilhumid_data[2];

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


  
  
  dht.begin();

  Serial.println("WifiHumid ready!");
}

void loop() {
  // Wait a few seconds between measurements.
  delay(2000);

  mqttClient.poll();

  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
    if (isnan(h) || isnan(t)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }

  Serial.print(h);
  //Serial.print(F(" Temperature[C]: "));
  Serial.print(", ");
  Serial.print(t);
  //Serial.println(F("")); 


  for (int i=0; i<soilhumid_num; i++) {
    soilhumid_data[i] = (float(analogRead(soilhumid_pins[i]))/1023.0)*5.0; // read sensor
    soilhumid_data[i] = 100*(soil0-soilhumid_data[i])/(soil0-soil100); //Calibrate to percent
    Serial.print(", ");
    Serial.print(soilhumid_data[i]);
  }
  Serial.println();

  mqttClient.beginMessage(topic);
  mqttClient.print("Humid = ");
  mqttClient.print(soilhumid_data[0]);
  mqttClient.endMessage();
  
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
