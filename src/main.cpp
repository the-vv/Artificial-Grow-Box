#include <Arduino.h>
#include <DHT.h>
#include <DHT_U.h>

#define MOISTURE_SENSOR_PIN_ANALOG A0
#define TEMP_HUMIDITY_SENSOR_PIN 2
#define PELTIER_FAN_PIN 3
#define PUMP_PIN 4
#define LIGHT_PIN 5

// System Specific values
#define TemperatureThreshold 25
#define MoistureThreshold 50

DHT_Unified dht(TEMP_HUMIDITY_SENSOR_PIN, DHT11);

float getTemperature();
float getMoisturePercentage();
void lightScheduler(int pin);
uint8_t lightState = HIGH;

// Blink without delay for light
static unsigned long previousMillisOn = 0;
static unsigned long previousMillisOff = 0;

void setup()
{
  // Setup
  dht.begin();
  pinMode(MOISTURE_SENSOR_PIN_ANALOG, INPUT);
  pinMode(PELTIER_FAN_PIN, OUTPUT);
  pinMode(PUMP_PIN, OUTPUT);
  pinMode(LIGHT_PIN, OUTPUT);

  digitalWrite(PELTIER_FAN_PIN, HIGH);
  digitalWrite(PUMP_PIN, HIGH);
  digitalWrite(LIGHT_PIN, HIGH);

  Serial.begin(9600);

  delay(1000);

  // digitalWrite(LIGHT_PIN, lightState);
}

void loop()
{
  // Temperature
  float temperature = getTemperature();
  Serial.print("Temperature: ");
  Serial.println(temperature);

  // Moisture
  int moisture = getMoisturePercentage();
  Serial.print("Moisture: ");
  Serial.println(moisture);

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
  // logic be like
  // when arduino starts, light should be on for 8 hours
  // then off for 16 hours
  // then on for 8 hours
  unsigned long currentMillis = millis();
  // unsigned long delayTimeOn = 8 * 60 * 60 * 1000; // 8 hours
  // unsigned long delayTimeOff = 16 * 60 * 60 * 1000; // 16 hours
  if (lightState == HIGH)
  {
    if (currentMillis - previousMillisOff >= 2000)
    {
      previousMillisOff = currentMillis;
      lightState = LOW;
      digitalWrite(pin, lightState);
      previousMillisOn = currentMillis;
    }
  }
  else
  {
    if (currentMillis - previousMillisOn >= 500)
    {
      previousMillisOn = currentMillis;
      lightState = HIGH;
      digitalWrite(pin, lightState);
    }
  }

}