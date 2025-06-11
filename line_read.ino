#include "globals.h"
#include <Arduino.h>
#include <EEPROM.h>


ZumoReflectanceSensorArray reflectanceSensors;

float get_line_deviation() {
  uint16_t SensorValues[sensor_count];
  float position = reflectanceSensors.readLine(SensorValues);
  //Serial.println(position);
  /*int eepromAddr2 = runs*(sizeof(RobotPos)+sizeof(float))+sizeof(RobotPos);
  EEPROM.put(eepromAddr2, position);*/
  float line_deviation = (position - 2500) / 83333.3;
  /*for (uint8_t i = 0; i < 6; i++) {
    Serial.print(SensorValues[i]);
    Serial.print("\t");
  }
  Serial.println();
  Serial.println();  */
  /*Serial.print("line Position = ");
  Serial.println(line_deviation,4);*/
  delay(50);
  return line_deviation;
}

