
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
  
  dht.begin();


  Serial.println("WifiHumid ready!");
}

void loop() {
  // Wait a few seconds between measurements.
  delay(2000);

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
}
