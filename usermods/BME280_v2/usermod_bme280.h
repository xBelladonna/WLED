#pragma once

#include "wled.h"
#include <Arduino.h>
#include <Wire.h>
#include <BME280I2C.h>               // BME280 sensor
#include <EnvironmentCalculations.h> // BME280 extended measurements

class UsermodBME280 : public Usermod
{
private:
// User-defined configuration
#define Celsius               // Show temperature mesaurement in Celcius. Comment out for Fahrenheit
#define TemperatureDecimals 1 // Number of decimal places in published temperaure values
#define HumidityDecimals 0    // Number of decimal places in published humidity values
#define PressureDecimals 2    // Number of decimal places in published pressure values
#define TemperatureInterval 5 // Interval to measure temperature (and humidity if available) in seconds
#define PressureInterval 300  // Interval to measure pressure in seconds

  // BME280 sensor settings
  BME280I2C::Settings settings{
      BME280::OSR_X16, // Temperature oversampling x16
      BME280::OSR_X16, // Humidity oversampling x16
      BME280::OSR_X16, // Pressure oversampling x16
      // Defaults
      BME280::Mode_Forced,
      BME280::StandbyTime_1000ms,
      BME280::Filter_Off,
      BME280::SpiEnable_False,
      BME280I2C::I2CAddr_0x76 // I2C address. Default 0x76
  };

// Pin definitions
#ifdef ARDUINO_ARCH_ESP32 // ESP32 boards
#define SCL_PIN (uint8_t)22
#define SDA_PIN (uint8_t)21
#else // ESP8266 boards
#define SCL_PIN (uint8_t)5
#define SDA_PIN (uint8_t)4
#endif

// Sanity checks
#if !defined(TemperatureDecimals) || TemperatureDecimals < 0
#define TemperatureDecimals 0
#endif
#if !defined(HumidityDecimals) || HumidityDecimals < 0
#define HumidityDecimals 0
#endif
#if !defined(PressureDecimals) || PressureDecimals < 0
#define PressureDecimals 0
#endif
#if !defined(TemperatureInterval) || TemperatureInterval < 0
#define TemperatureInterval 5
#endif
#if !defined(PressureInterval) || PressureInterval < 0
#define PressureInterval TemperatureInterval
#endif

  BME280I2C bme{settings};
  uint8_t SensorType = 0;

  // Measurement timers
  unsigned long timer;
  unsigned long lastTemperatureMeasure = 0;
  unsigned long lastPressureMeasure = 0;

  // Current sensor values
  float Temperature;
  float HeatIndex;
  float Humidity;
  float DewPoint;
  float Pressure;
  // Track previous sensor values
  float lastTemperature;
  float lastHeatIndex;
  float lastHumidity;
  float lastDewPoint;
  float lastPressure;

  // Store packet IDs of MQTT publications
  uint16_t mqttTemperaturePub = 0;
  uint16_t mqttPressurePub = 0;

  void UpdateSensorData(uint8_t SensorType)
  {
    float _temperature, _humidity, _pressure;
#ifdef Celsius
    BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
    EnvironmentCalculations::TempUnit envTempUnit(EnvironmentCalculations::TempUnit_Celsius);
#else
    BME280::TempUnit tempUnit(BME280::TempUnit_Fahrenheit);
    EnvironmentCalculations::TempUnit envTempUnit(EnvironmentCalculations::TempUnit_Fahrenheit);
#endif
    BME280::PresUnit presUnit(BME280::PresUnit_hPa);

    bme.read(_pressure, _temperature, _humidity, tempUnit, presUnit);

    Temperature = roundf(_temperature * pow(10, TemperatureDecimals)) / pow(10, TemperatureDecimals);
    if (SensorType > 1)
    {
      float _heat_index(EnvironmentCalculations::HeatIndex(_temperature, _humidity, envTempUnit));
      float _dew_point(EnvironmentCalculations::DewPoint(_temperature, _humidity, envTempUnit));

      HeatIndex = roundf(_heat_index * pow(10, TemperatureDecimals)) / pow(10, TemperatureDecimals);
      Humidity = roundf(_humidity * pow(10, HumidityDecimals)) / pow(10, HumidityDecimals);
      DewPoint = roundf(_dew_point * pow(10, TemperatureDecimals)) / pow(10, TemperatureDecimals);
    }
    Pressure = roundf(_pressure * pow(10, PressureDecimals)) / pow(10, PressureDecimals);
  }

public:
  void setup()
  {
    Serial.println("\n"); // Insert buffer line from other serial data

    if (!mqttEnabled)
      Serial.println("MQTT is not enabled!");
    else
    {
      Wire.begin(SDA_PIN, SCL_PIN); // Connect to I2C bus
      if (!bme.begin())             // Connect to sensor
        Serial.println("Could not find BME280 I2C sensor!");
      else
      {
        switch (bme.chipModel()) // Identify sensor type
        {
        case BME280::ChipModel_BME280:
          SensorType = 2;
          Serial.println("Found BME280 sensor!");
          break;
        case BME280::ChipModel_BMP280:
          SensorType = 1;
          Serial.println("Found BMP280 sensor. No humidity available.");
          break;
        default:
          Serial.println("Found unknown sensor! Cannot be used.");
        }
      }
    }

    if (SensorType)
      Serial.println("Connecting to " + String(mqttServer) + " and publishing on " + String(mqttDeviceTopic));
    else
      Serial.println("BME280 has been disabled. WLED will continue to work normally without it. Reboot WLED to re-initialize.");
  }

  void loop()
  {
    // BME280 sensor MQTT publishing
    // Check if sensor present and MQTT Connected, otherwise it will crash the MCU
    if (!SensorType || !WLED_MQTT_CONNECTED)
      return;

    // Timer to fetch new temperature, humidity and pressure data at intervals
    timer = millis();

    if (timer - lastTemperatureMeasure >= TemperatureInterval * 1000 || !mqttTemperaturePub)
    {
      lastTemperatureMeasure = timer;

      UpdateSensorData(SensorType);

      // If temperature has changed since last measure, create string populated with device topic
      // from the UI and values read from sensor, then publish to broker
      if (Temperature != lastTemperature)
      {
        String topic = String(mqttDeviceTopic) + "/temperature";
        mqttTemperaturePub = mqtt->publish(topic.c_str(), 0, true, String(Temperature, TemperatureDecimals).c_str());
      }

      lastTemperature = Temperature; // Update last sensor temperature for next loop

      if (SensorType > 1) // Only if sensor is a BME280
      {
        if (HeatIndex != lastHeatIndex)
        {
          String topic = String(mqttDeviceTopic) + "/heat_index";
          mqtt->publish(topic.c_str(), 0, true, String(HeatIndex, TemperatureDecimals).c_str());
        }

        if (Humidity != lastHumidity)
        {
          String topic = String(mqttDeviceTopic) + "/humidity";
          mqtt->publish(topic.c_str(), 0, true, String(Humidity, HumidityDecimals).c_str());
        }

        if (DewPoint != lastDewPoint)
        {
          String topic = String(mqttDeviceTopic) + "/dew_point";
          mqtt->publish(topic.c_str(), 0, true, String(DewPoint, TemperatureDecimals).c_str());
        }

        lastHumidity = Humidity;
        lastHeatIndex = HeatIndex;
        lastDewPoint = DewPoint;
      }
    }

    if (timer - lastPressureMeasure >= PressureInterval * 1000 || !mqttPressurePub)
    {
      lastPressureMeasure = timer;

      UpdateSensorData(SensorType);

      if (Pressure != lastPressure)
      {
        String topic = String(mqttDeviceTopic) + "/pressure";
        mqttPressurePub = mqtt->publish(topic.c_str(), 0, true, String(Pressure, PressureDecimals).c_str());
      }

      lastPressure = Pressure;
    }
  }

  void addToJsonInfo(JsonObject &root)
  {
    // Don't add temperature to info object if BME280 is disabled
    if (!SensorType)
      return;

    // Set up the API
    JsonObject user = root["u"];
    if (user.isNull())
      user = root.createNestedObject("u");

    // Declare the JsonArrays that will contain all the nested arrays here for later use; scoping issues
    JsonArray temperature;
    JsonArray heatIndex;
    JsonArray humidity;
    JsonArray dewPoint;
    JsonArray pressure;

    // If a measure has been made, create a nested array and populate the values
    if (lastTemperature) // Add temperature
    {
      temperature = user.createNestedArray("Temperature");
      temperature.add(Temperature);
#ifdef Celsius
      temperature.add(" °C");
#else
      temperature.add(" °F");
#endif
    }

    if (SensorType > 1) // Only if a BME280 is in use
    {
      // Add BME280-specific features
      if (lastHeatIndex) // Add heat index
      {
        heatIndex = user.createNestedArray("Apparent Temperature");
        heatIndex.add(HeatIndex);
#ifdef Celsius
        heatIndex.add(" °C");
#else
        heatIndex.add(" °F");
#endif
      }

      if (lastHumidity) // Add humidity
      {
        humidity = user.createNestedArray("Humidity");
        humidity.add(Humidity);
        humidity.add("%");
      }

      if (lastDewPoint) // Add dew point
      {
        dewPoint = user.createNestedArray("Dew Point");
        dewPoint.add(DewPoint);
#ifdef Celsius
        dewPoint.add(" °C");
#else
        dewPoint.add(" °F");
#endif
      }
    }

    if (lastPressure) // Add pressure
    {
      pressure = user.createNestedArray("Pressure");
      pressure.add(Pressure);
      pressure.add(" hPa");
    }
  }

  uint16_t getId()
  {
    return USERMOD_ID_BME280;
  }
};