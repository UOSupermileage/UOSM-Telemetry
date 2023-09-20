//
// Created by Jeremy Cote on 2023-08-29.
//

#ifndef UOSM_TELEMETRY_THINGPROPERTIES_H
#define UOSM_TELEMETRY_THINGPROPERTIES_H

#include <ArduinoIoTCloud.h>
#include <Arduino_ConnectionHandler.h>
#include "Mutex.hpp"
#include "Secrets.h"

Mutex iotMutex;

// Name of variables is important. They map to definitions in our IOT Cloud Dashboard
CloudElectricCurrent batteryCurrent;
CloudElectricPotential batteryVoltage;

String canMessage;
int motorRPM;
CloudPercentage throttle;

// Acceleration Sensor
CloudAcceleration accelerationX;
CloudAcceleration accelerationY;
CloudAcceleration accelerationZ;

// Pressure Sensor
CloudPressure pressure;
CloudTemperatureSensor temperature;

// GPS Sensor
float latitude;
float longitude;
float gps_speed; // in km / h
float heading;
float altitude;

// CAN Sensor
CloudVelocity speed;

void initProperties(){
    ArduinoCloud.setBoardId(DEVICE_LOGIN_NAME);
    ArduinoCloud.setSecretDeviceKey(DEVICE_KEY);

    ArduinoCloud.addProperty(accelerationX, READ, ON_CHANGE, NULL);
    ArduinoCloud.addProperty(accelerationY, READ, ON_CHANGE, NULL);
    ArduinoCloud.addProperty(accelerationZ, READ, ON_CHANGE, NULL);
    ArduinoCloud.addProperty(canMessage, READ, ON_CHANGE, NULL);
    ArduinoCloud.addProperty(batteryCurrent, READ, ON_CHANGE, NULL);
    ArduinoCloud.addProperty(batteryVoltage, READ, ON_CHANGE, NULL);
    ArduinoCloud.addProperty(temperature, READ, ON_CHANGE, NULL);
    ArduinoCloud.addProperty(motorRPM, READ, ON_CHANGE, NULL);
    ArduinoCloud.addProperty(throttle, READ, ON_CHANGE, NULL);
    ArduinoCloud.addProperty(pressure, READ, ON_CHANGE, NULL);
    ArduinoCloud.addProperty(speed, READ, ON_CHANGE, NULL);
}

void updateBatteryVoltage(float voltage) {
    batteryVoltage = voltage;
}

void updateAcceleration(acceleration_t acceleration){
    accelerationX = acceleration.x;
    accelerationY = acceleration.y;
    accelerationZ = acceleration.z;
}

void updateCanMessages(const char* message) {
    canMessage = message;
}

void updatePressure(pressure_t p){
    pressure = p.pressure;
    temperature = p.temp;
}

void updateGPS(gps_coordinate_t coordinate) {
    latitude = coordinate.latitude;
    longitude = coordinate.longitude;
    gps_speed = coordinate.speed_kmh;
    heading = coordinate.heading;
    altitude = coordinate.altitude;
}

void updateRPM(velocity_t r) {
    motorRPM = r;
}

void updateSpeed(speed_t s) {
    speed = s;
}

void updateThrottle(percentage_t p) {
    throttle = (float) p / 10;
}

WiFiConnectionHandler ArduinoIoTPreferredConnection(SSID, PASS);

#endif //UOSM_TELEMETRY_THINGPROPERTIES_H
