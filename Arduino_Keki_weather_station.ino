#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include "DHT.h"
#include <WiFi.h>
#include <PubSubClient.h>

#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 16, 2);

#define DHTPIN 4
#define DHTTYPE DHT11

DHT dht(DHTPIN, DHTTYPE);

Adafruit_BMP280 bmp; // use I2C interface
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();

#define sw1 33
#define buz 32

const char* ssid = "Your_WiFi";
const char* password = "Your_password";
const char* mqtt_server = "broker.hivemq.com";

WiFiClient espClient;

byte i = 0;

long t1 = 0;

String DataString;
char msg[100];


PubSubClient client(mqtt_server, 1883, espClient);
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    /*
      YOU MIGHT NEED TO CHANGE THIS LINE, IF YOU'RE HAVING PROBLEMS WITH MQTT MULTIPLE CONNECTIONS
      To change the ESP device ID, you will have to give a new name to the ESP8266.
      Here's how it looks:
       if (client.connect("ESP8266Client")) {
      You can do it like this:
       if (client.connect("ESP1_Office")) {
      Then, for the other ESP:
       if (client.connect("ESP2_Garage")) {
      That should solve your MQTT multiple connections problem
    */
    if (client.connect("ESP32Client")) {
      Serial.println("connected");
      // Subscribe or resubscribe to a topic
      // You can subscribe to more topics (to control more LEDs in this example)

    }
    else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}



void setup() {
  Serial.begin(9600);
  dht.begin();
  bmp.begin(0x76);
  lcd.begin();
  pinMode(sw1, INPUT);
  pinMode(buz, OUTPUT);


  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);



  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");


    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
    client.setServer(mqtt_server, 1883);



  }
}

void loop() {

  if (!client.connected()) {
    reconnect();
  }
  if (!client.loop())
    client.connect("ESP32Client");

  sensors_event_t temp_event, pressure_event;
  bmp_temp->getEvent(&temp_event);
  bmp_pressure->getEvent(&pressure_event);
  float t = dht.readTemperature();
  float h = dht.readHumidity();
  float hic = dht.computeHeatIndex(t, h, false);
  float f = dht.readTemperature(true);


  if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }

  //  Serial.print(F("Temperature = "));
  //  Serial.print(temp_event.temperature);
  //  Serial.println(" *C");
  //
  //  Serial.print(F("Humidity = "));
  //  Serial.print(h);
  //  Serial.println(" %");
  //
  //  Serial.print(F("Pressure = "));
  //  Serial.print(pressure_event.pressure);
  //  Serial.println(" hPa");
  //
  //  Serial.print(F("Heat Index = "));
  //  Serial.print(hic);
  //  Serial.println(" *C");

  if (digitalRead(sw1) == 1) {
    digitalWrite(buz, HIGH);
    i++;
    delay(250);
    lcd.clear();
  }
  else {
    digitalWrite(buz, LOW);
  }

  if (i == 0) {
    lcd.setCursor(0, 0);
    lcd.print("Temperature");
    lcd.setCursor(0, 1);
    lcd.print(t);
    lcd.setCursor(9, 1);
    lcd.print("*C");

  }

  if (i == 1) {
    lcd.setCursor(0, 0);
    lcd.print("Humidity");
    lcd.setCursor(0, 1);
    lcd.print(h);
    lcd.setCursor(9, 1);
    lcd.print("%");

  }

  if (i == 2) {
    lcd.setCursor(0, 0);
    lcd.print("Pressure");
    lcd.setCursor(0, 1);
    lcd.print(pressure_event.pressure);
    lcd.setCursor(9, 1);
    lcd.print("hPa");

  }

  if (i == 3) {
    lcd.setCursor(0, 0);
    lcd.print("Heat Index");
    lcd.setCursor(0, 1);
    lcd.print(hic);
    lcd.setCursor(9, 1);
    lcd.print("*C");

  }

  if (i > 3) {
    i = 0;
  }

  client.loop();

  DataString = "{\"temp\": " + String(t) + ", " + "\"humidity\" :" + String(h) + ", " + "\"Pressure\" :" + String(pressure_event.pressure) + ", " + "\"Index\" :" + String(hic) + "}";
  DataString.toCharArray(msg, 120);


  if (millis() - t1 > 5000) {
    client.publish("esp32/station", msg);
    Serial.print("Publish message: ");
    Serial.println(msg);
    t1 = millis();

  }
}
