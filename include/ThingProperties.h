//
// Created by Jeremy Cote on 2023-08-29.
//

#ifndef UOSM_TELEMETRY_THINGPROPERTIES_H
#define UOSM_TELEMETRY_THINGPROPERTIES_H

#include "ApplicationTypes.h"

#include <ArduinoIoTCloud.h>
#include <Arduino_ConnectionHandler.h>
#include "Mutex.hpp"
#include "Secrets.h"

#define MOTOR_ON_TIMEOUT 500

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
CloudLocation gpsCoordinates;
float gps_speed; // in km / h
float heading;
float altitude;

// CAN Sensor
CloudVelocity speed;

// Events
bool motorOn = false;
uint32_t motorOnTimestamp;

void initProperties(){
    ArduinoCloud.setThingId(DEVICE_LOGIN_NAME);

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
    ArduinoCloud.addProperty(motorOn, READ, ON_CHANGE, NULL);
    ArduinoCloud.addProperty(gpsCoordinates, READ, ON_CHANGE, NULL);
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
    gpsCoordinates = { coordinate.longitude, coordinate.latitude };
    gps_speed = coordinate.speed_kmh;
    heading = coordinate.heading;
    altitude = coordinate.altitude;
}

void updateRPM(velocity_t r) {
    // Store inverted because the rpm is inverse
    motorRPM = r * -1;
}

void updateMotorOn(boolean isOn) {
    motorOn = isOn;
    motorOnTimestamp = 0; //TODO: Fix this
}

void periodicMotorOn() {
    if (motorOn && (motorOnTimestamp + MOTOR_ON_TIMEOUT) < 0) {
        motorOn = false;
    }
}

void updateSpeed(speed_t s) {
    // store in km / h
    speed = (float) s / 1000;
}

void updateThrottle(percentage_t p) {
    throttle = (float) p / 10;
}

WiFiConnectionHandler ArduinoIoTPreferredConnection(SSID, PASS);

#endif //UOSM_TELEMETRY_THINGPROPERTIES_H
