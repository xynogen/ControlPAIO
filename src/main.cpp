#include <Arduino.h>
#include <WiFiEsp.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// Global Var
#ifndef STASSID
#define STASSID "PAIO"
#define STAPSK "complexity"
#endif

const char *ssid = STASSID;
const char *password = STAPSK;

// Set Static IP Configuration
IPAddress local_IP(192, 168, 0, 100);
IPAddress gateway(192, 168, 0, 1);
IPAddress subnet(255, 255, 255, 0);

WiFiEspServer server(80);
RingBuffer buf(8);

// Emulate One Wire
OneWire oneWire(4);
DallasTemperature sensors(&oneWire);

// Setting IO Pin
const uint8_t TRIGPIN = 7;
const uint8_t ECHOPIN = 6;
const uint8_t RXWIFIPIN = 3;
const uint8_t TXWIFIPIN = 2;
const uint8_t TEMPPIN = 4;
const uint8_t TURBPIN = A5;
const uint8_t PHPIN = A4;

// Serial Com to WiFi Module
#ifndef HAVE_HWSERIAL1
#include "SoftwareSerial.h"
SoftwareSerial Serial2(TXWIFIPIN, RXWIFIPIN); // TX, RX
#endif

// Prototype
float Distance();
float Turbidity();
float Temp();
float PH();

// Globab Var
float distance;
float turbidity;
float temp;
float ph;
String response;

void setup()
{
  pinMode(TRIGPIN, OUTPUT);
  pinMode(ECHOPIN, INPUT);
  pinMode(TURBPIN, INPUT);
  pinMode(TEMPPIN, INPUT);
  pinMode(PHPIN, INPUT);

  Serial.begin(9600);
  Serial2.begin(9600);
  WiFi.init(&Serial2);

  // Clear the serial output
  Serial.println(F(""));
  Serial.println(F(""));
  Serial.print(F("Connecting to : "));
  Serial.println(ssid);

  if (WiFi.status() == WL_NO_SHIELD)
  {
    Serial.println(F("[WiFi] WiFi shield not present"));
    // don't continue
    while (true)
      ;
  }
  Serial.println(F("[WiFi] WiFi Shield Present"));

  // Connect to The WiFi
  while (WiFi.begin(ssid, password) != WL_CONNECTED)
  {
    Serial.print(F("[WiFi] Attempting to connect to WPA SSID: "));
    Serial.println(ssid);
  }
  Serial.println(F("[WiFi] Connected"));
  WiFi.config(local_IP);

  //Print The Some Debug data
  Serial.print("[WIFI]: SSID: ");
  Serial.println(WiFi.SSID());
  IPAddress ip = WiFi.localIP();
  Serial.print("[WIFI]: IP Address: ");
  Serial.println(ip);
  long rssi = WiFi.RSSI();
  Serial.print("[WIFI]: Kekuatan Sinyal (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");

  server.begin();
}

void loop()
{
  WiFiEspClient client = server.available();

  if (!client)
    return;

  // Default Value is 1000
  // client.setTimeout(5000);

  while (client.available())
    client.read();
  

  distance = Distance();
  turbidity = Turbidity();
  temp = Temp();
  ph = PH();
  
  response.reserve(200);
  response = "HTTP/1.1 200 OK\r\nContent-type: application/json\r\n\r\n";
  response += "{\"distance\": ";
  response += distance;
  response += ", \"turbidity\":";
  response += turbidity;
  response += ", \"temp\":";
  response += temp;
  response += ", \"ph\":";
  response += ph;
  response += "}";
  
  client.print(response);
  client.stop();
  Serial.println(F("Client Disconnected"));
 
  
}

float Distance()
{
  unsigned long duration;
  float dist;

  digitalWrite(TRIGPIN, LOW);
  delay(5);

  digitalWrite(TRIGPIN, HIGH);
  delay(10);
  digitalWrite(TRIGPIN, LOW);

  duration = pulseIn(ECHOPIN, HIGH);

  dist = (float)duration * 0.017;
  return dist;
}

float Turbidity()
{
  int sensorValue = analogRead(TURBPIN);
  float voltage = sensorValue * (5.0 / 1024.0) - 0.36;
  float turb = -1120.4 * voltage * voltage + 5742.3 * voltage - 4352.9;
  return turb;
}

float Temp()
{
  sensors.requestTemperatures();
  return sensors.getTempCByIndex(0);
}

float PH()
{
  int analogSensor = analogRead(PHPIN);
  float volt = analogSensor * (5.0 / 1024.0);
  float PH = 7.00 + ((2.5 - volt) / 0.2);
  return PH;
}