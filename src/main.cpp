#include <Arduino.h>
#include <DHT.h>
#include <DHT_U.h>

#define MOISTURE_SENSOR_PIN_ANALOG A0
#define TEMP_HUMIDITY_SENSOR_PIN 2
#define PELTIER_FAN_PIN 3
#define PUMP_PIN 4
#define LIGHT_PIN 5

DHT_Unified dht(TEMP_HUMIDITY_SENSOR_PIN, DHT11);

float getTemperature();

void setup() {

  dht.begin();

  Serial.begin(9600);

  Serial.println("Starting DHT");
}

void loop() {
  float temperature = getTemperature();
  Serial.print("Temperature: ");
  Serial.println(temperature);
  delay(2000);
}


float getTemperature() {
  sensors_event_t event;
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    Serial.println("Error reading temperature!");
    return -1;
  }
  return event.temperature;
}