#include "DHTesp.h"
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <ESP32Servo.h>

unsigned long previousMillis = 0;
const unsigned long interval = 2000;

DHTesp dht;     //Define the DHT object
int dhtPin = 13;//Define the dht pin

const char *SSID = "DGSE_mobile";
const char *PWD = "vincent28$";

const int red_pin = 5;   
const int green_pin = 18; 
const int blue_pin = 19; 

const int frequency = 5000;
const int redChannel = 0;
const int greenChannel = 1;
const int blueChannel = 2;
const int resolution = 8;

WebServer server(80);

StaticJsonDocument<250> jsonDocument;
char buffer[250];

float temperature;
float humidity;

LiquidCrystal_I2C lcd(0x27,16,2);
#define SDA 33
#define SCL 32

#define PIN_ANALOG_IN   34
#define PIN_LED         12
#define CHAN            0
int ledIntensity = 0;

Servo myservo; 
int posVal = 0; 
int servoPin = 25; 

#define trigPin 15 // define trigPin
#define echoPin 2 // define echoPin.
#define MAX_DISTANCE 700 // Maximum sensor distance is rated at 400-500cm. //timeOut= 2*MAX_DISTANCE /100 /340 *1000000 = MAX_DISTANCE*58.8
float timeOut = MAX_DISTANCE * 60;
int soundVelocity = 340; // define sound speed=340m/s

#define PIN_BUZZER 26

void create_json(char *tag, float value, char *unit) {  
  jsonDocument.clear();
  jsonDocument["type"] = tag;
  jsonDocument["value"] = value;
  jsonDocument["unit"] = unit;
  serializeJson(jsonDocument, buffer);
}
 
void add_json_object(char *tag, float value, char *unit) {
  JsonObject obj = jsonDocument.createNestedObject();
  obj["type"] = tag;
  obj["value"] = value;
  obj["unit"] = unit; 
}

void read_sensor_data() {
  TempAndHumidity values = dht.getTempAndHumidity();
  temperature = values.temperature;
  humidity = values.humidity;
  Serial.println("Read sensor data");    
}
 
void getTemperature() {
  Serial.println("Get temperature");
  read_sensor_data();
  create_json("temperature", temperature, "°C");
  server.send(200, "application/json", buffer);
}
 
void getHumidity() {
  Serial.println("Get humidity");
  read_sensor_data();
  create_json("humidity", humidity, "%");
  server.send(200, "application/json", buffer);
}
 

 
void getData() {
  Serial.println("Get BME280 Sensor Data");
  jsonDocument.clear();
  read_sensor_data();
  add_json_object("temperature", temperature, "°C");
  add_json_object("humidity", humidity, "%");
  serializeJson(jsonDocument, buffer);
  server.send(200, "application/json", buffer);
}

void handlePost() {
  Serial.println("handlePost");
  if (server.hasArg("plain") == false) {
  }
  String body = server.arg("plain");
  deserializeJson(jsonDocument, body);

  int red_value = jsonDocument["red"];
  int green_value = jsonDocument["green"];
  int blue_value = jsonDocument["blue"];

  ledcWrite(redChannel, red_value);
  ledcWrite(greenChannel,green_value);
  ledcWrite(blueChannel, blue_value);

  server.send(200, "application/json", "{}");
}

void setText() {
  Serial.println("setText");
  if (server.hasArg("plain") == false) {
  }
  String body = server.arg("plain");
  deserializeJson(jsonDocument, body);

  String text = jsonDocument["text"];
  
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(text);

  server.send(200, "application/json", "{}");
}
void getBrightness() {
  Serial.println("getBrightness");
  float pourcentage = (static_cast<float>(ledIntensity) / static_cast<float>(255)) * 100.0;

  create_json("brightness", pourcentage, "%");
  server.send(200, "application/json", buffer);
}
void openLock(){
  Serial.println("Open");
  for (posVal = 0; posVal <= 180; posVal += 1) { 
    myservo.write(posVal);  
    delay(15); 
  }
}
void closeLock(){
  Serial.println("close");
  for (posVal = 180; posVal >= 0; posVal -= 1) { 
    myservo.write(posVal); 
    delay(15);                  
  }
}

void setup_routing() {     
  server.on("/temperature", getTemperature);     
  server.on("/humidity", getHumidity);     
  server.on("/data", getData);     
  server.on("/led", HTTP_POST, handlePost);    
  server.on("/text", HTTP_POST, setText);    
  server.on("/brightness", getBrightness);    
  server.on("/open", HTTP_POST, openLock);    
  server.on("/close", HTTP_POST, closeLock); 

  server.begin();    
}
bool i2CAddrTest(uint8_t addr) {
  Wire.begin();
  Wire.beginTransmission(addr);
  if (Wire.endTransmission() == 0) {
    return true;
  }
  return false;
}
void setup() {
  dht.setup(dhtPin, DHTesp::DHT11);//Initialize the dht pin and dht object 
  Serial.begin(115200); 

  ledcSetup(redChannel, frequency, resolution);
  ledcSetup(greenChannel, frequency, resolution);
  ledcSetup(blueChannel, frequency, resolution);
 
  ledcAttachPin(red_pin, redChannel);
  ledcAttachPin(green_pin, greenChannel);
  ledcAttachPin(blue_pin, blueChannel);

  Serial.print("Connecting to Wi-Fi");
  WiFi.begin(SSID, PWD);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
 
  Serial.print("Connected! IP Address: ");
  Serial.println(WiFi.localIP());
  setup_routing();  

//Set up ecran LCD
  Wire.begin(SDA, SCL);
  if (!i2CAddrTest(0x27)) {
    lcd = LiquidCrystal_I2C(0x3F, 16, 2);
  }
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print(WiFi.localIP());


  ledcSetup(CHAN, 1000, 12);
  ledcAttachPin(PIN_LED, CHAN);

  myservo.setPeriodHertz(50);
  myservo.attach(servoPin, 500, 2500);

  pinMode(trigPin,OUTPUT);
  pinMode(echoPin,INPUT); 

  pinMode(PIN_BUZZER, OUTPUT);
}
float getSonar() {
  unsigned long pingTime;
  float distance;
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  pingTime = pulseIn(echoPin, HIGH, timeOut);
  distance = (float)pingTime * soundVelocity / 2 / 10000; 
  return distance; 
}
void loop() {
  unsigned long currentMillis = millis();

  server.handleClient();   

  int potValue = analogRead(PIN_ANALOG_IN); 
  ledIntensity = map(potValue, 0, 4095, 0, 255);
  ledcWrite(CHAN, ledIntensity);
  float dist = getSonar();
  Serial.printf("Distance: ");
  Serial.print(dist); 
  Serial.println("cm");

if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // Code à exécuter toutes les 2 secondes
    if (dist <= 10.0) {
      Serial.println("Objet présent");
        digitalWrite(PIN_BUZZER, HIGH);
    } else {
        digitalWrite(PIN_BUZZER, LOW);
    }
  }
}