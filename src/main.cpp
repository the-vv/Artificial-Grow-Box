#include <Arduino.h>
#include <DHT_U.h>
#include <LiquidCrystal.h>

#define MOISTURE_SENSOR_PIN_ANALOG A0 // Moisture Sensor
#define TEMP_HUMIDITY_SENSOR_PIN 2    // DHT11
#define PELTIER_FAN_PIN 9             // Peltier Fan
#define PUMP_PIN 8                    // Pump
#define LIGHT_PIN 7                   // Light

// System Specific values
#define TemperatureThreshold 25                  // 25 degrees, turn on fan
#define MoistureThreshold 50                     // 50%, turn on pump
#define LightOnTime 8UL * 60UL * 60UL * 1000UL   // 8 hours
#define LightOffTime 16UL * 60UL * 60UL * 1000UL // 16 hours

// For DHT11
DHT_Unified dht(TEMP_HUMIDITY_SENSOR_PIN, DHT11);

// Liquid Crystal Display
const int rs = 4, en = 9, d4 = 10, d5 = 11, d6 = 12, d7 = 13;
LiquidCrystal lcd(4, 6, 10, 11, 12, 13);

// Function Prototypes
float getTemperature();
float getMoisturePercentage();
void lightScheduler(int pin);
void displayEnviornmentInfo(float temperature, int moisture);
void displayPeripheralsStatus();

// Global Variables
uint8_t lightStatus = 0; // This is to track the state of the light timer
int8_t prevLightState = -1;
int8_t prevFanState = -1;
int8_t prevPumpState = -1;
static unsigned long previousMillis = 0;
float previousTemperature = 0;
float previousMoisture = 0;

void setup()
{
  dht.begin();
  lcd.begin(16, 2);

  pinMode(MOISTURE_SENSOR_PIN_ANALOG, INPUT); // Moisture Sensor connected to analog pin A0 as input
  pinMode(PELTIER_FAN_PIN, OUTPUT);           // Peltier Fan connected to digital pin 3 as output
  pinMode(PUMP_PIN, OUTPUT);                  // Pump connected to digital pin 4 as output
  pinMode(LIGHT_PIN, OUTPUT);                 // Light connected to digital pin 5 as output

  digitalWrite(PELTIER_FAN_PIN, HIGH);  // Turn off the fan when the system starts
  digitalWrite(PUMP_PIN, HIGH);         // Turn off the pump when the system starts
  digitalWrite(LIGHT_PIN, lightStatus); // Turn on the light when the system starts

  Serial.begin(9600);

  Serial.println("Smart Greenhouse System is starting...");

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Smart Greenhouse");
  lcd.setCursor(0, 1);
  lcd.print("Starting System...");

  delay(5000); // Delay to let system boot up
}

void loop()
{
  // Temperature
  float temperature = getTemperature();
  if (temperature > TemperatureThreshold)
  {
    digitalWrite(PELTIER_FAN_PIN, LOW);
    prevFanState = 0;
  }
  else
  {
    digitalWrite(PELTIER_FAN_PIN, HIGH);
    prevFanState = 1;
  }

  // Moisture
  int moisture = getMoisturePercentage();
  if (moisture < MoistureThreshold)
  {
    digitalWrite(PUMP_PIN, LOW);
    prevPumpState = 0;
  }
  else
  {
    digitalWrite(PUMP_PIN, HIGH);
    prevPumpState = 1;
  }

  if (temperature != previousTemperature || moisture != previousMoisture)
  {
    displayEnviornmentInfo(temperature, moisture);
    previousTemperature = temperature;
    previousMoisture = moisture;
  }

  displayPeripheralsStatus();

  lightScheduler(LIGHT_PIN);
}

float getTemperature()
{
  sensors_event_t event;
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature))
  {
    Serial.println("Error reading temperature!");
    return -1;
  }
  return event.temperature;
}

float getMoisturePercentage()
{
  int sensorValue = analogRead(MOISTURE_SENSOR_PIN_ANALOG);
  return map(sensorValue, 0, 1023, 100, 0);
}

void lightScheduler(int pin)
{
  unsigned long currentMillis = millis(); // Get the current time

  if (currentMillis - previousMillis >= (lightStatus ? LightOffTime : LightOnTime))
  {
    previousMillis = currentMillis;
    lightStatus = !lightStatus;
    digitalWrite(pin, lightStatus);
    Serial.print("Light is ");
    Serial.println(!lightStatus ? "on" : "off");
  }
}

void displayEnviornmentInfo(float temperature, int moisture)
{
  lcd.setCursor(0, 1);
  lcd.print("T: ");
  lcd.print(temperature);
  lcd.print("C");
  lcd.print(" M: ");
  lcd.print(moisture);
  lcd.print("%");
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.print("C, Moisture: ");
  Serial.print(moisture);
  Serial.println("%");
}

void displayPeripheralsStatus()
{
  uint8_t currentFanState = digitalRead(PELTIER_FAN_PIN);
  uint8_t currentPumpState = digitalRead(PUMP_PIN);
  uint8_t currentLightState = digitalRead(LIGHT_PIN);

  if (currentFanState != prevFanState || currentPumpState != prevPumpState || currentLightState != prevLightState)
  {
    lcd.setCursor(0, 0);
    lcd.print("F:");
    lcd.print(currentFanState ? "OFF" : "ON");
    lcd.print("  P:");
    lcd.print(currentPumpState ? "OFF" : "ON");
    lcd.print("  L:");
    lcd.print(currentLightState ? "OFF" : "ON");
    Serial.print("Fan: ");
    Serial.print(currentFanState ? "OFF" : "ON");
    Serial.print(", Pump: ");
    Serial.print(currentPumpState ? "OFF" : "ON");
    Serial.print(", Light: ");
    Serial.println(currentLightState ? "OFF" : "ON");

    prevFanState = currentFanState;
    prevPumpState = currentPumpState;
    prevLightState = currentLightState;
  }
}