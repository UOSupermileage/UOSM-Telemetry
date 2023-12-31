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
#include <vector>

#define MOTOR_ON_TIMEOUT 500

Mutex iotMutex;

struct LapData {
    float totalJoules;
    int totalTime;

    LapData(float totalJoules, int totalTime) : totalJoules(totalJoules), totalTime(totalTime) {}
};

struct MotorData {
    int rpm = 0;
    bool isOn = false;
    uint32_t heartbeatTimestamp = 0;
};

struct GPSData {
    CloudLocation coordinates;
    float speed = 0; // km/h
    float heading = 0;
    float altitude = 0;
};

std::vector<LapData> laps = std::vector<LapData>(1, LapData(0, 0));;

CloudElectricCurrent batteryCurrent;
CloudElectricPotential batteryVoltage;

String canMessage;
CloudPercentage throttle;

MotorData motor;

// Acceleration Sensor
CloudAcceleration accelerationX;
CloudAcceleration accelerationY;
CloudAcceleration accelerationZ;

// Pressure Sensor
CloudPressure pressure;
CloudTemperatureSensor temperature;

GPSData gps;

// CAN Sensor
CloudVelocity speed;

void SetupThing() {
    ArduinoCloud.setThingId(DEVICE_LOGIN_NAME);

    ArduinoCloud.addProperty(accelerationX, READ, ON_CHANGE, NULL);
    ArduinoCloud.addProperty(accelerationY, READ, ON_CHANGE, NULL);
    ArduinoCloud.addProperty(accelerationZ, READ, ON_CHANGE, NULL);
    ArduinoCloud.addProperty(canMessage, READ, ON_CHANGE, NULL);
    ArduinoCloud.addProperty(batteryCurrent, READ, ON_CHANGE, NULL);
    ArduinoCloud.addProperty(batteryVoltage, READ, ON_CHANGE, NULL);
    ArduinoCloud.addProperty(temperature, READ, ON_CHANGE, NULL);
    ArduinoCloud.addProperty(motor.rpm, READ, ON_CHANGE, NULL);
    ArduinoCloud.addProperty(throttle, READ, ON_CHANGE, NULL);
    ArduinoCloud.addProperty(pressure, READ, ON_CHANGE, NULL);
    ArduinoCloud.addProperty(speed, READ, ON_CHANGE, NULL);
    ArduinoCloud.addProperty(motor.isOn, READ, ON_CHANGE, NULL);
    ArduinoCloud.addProperty(gps.coordinates, READ, ON_CHANGE, NULL);
}

float getBatteryVoltage() { return batteryVoltage; }

float getBatteryCurrent() { return batteryCurrent; }

float getAccelerationX() { return accelerationX; }

float getAccelerationY() { return accelerationY; }

float getAccelerationZ() { return accelerationZ; }

int getRPM() { return motor.rpm; }

float getThrottle() { return throttle; }

float getSpeed() { return speed; }

void updateBattery(float voltage, float current) {
    batteryVoltage = voltage;
    batteryCurrent = current;

    LapData &currentLap = laps.back();
    currentLap.totalJoules += voltage * current;
}

void updateAcceleration(acceleration_t acceleration) {
    accelerationX = acceleration.x;
    accelerationY = acceleration.y;
    accelerationZ = acceleration.z;
}

void updateCanMessages(const char *message) {
    canMessage = message;
}

void updatePressure(pressure_t p) {
    pressure = p.pressure;
    temperature = p.temp;
}

void updateGPS(gps_coordinate_t coordinate) {
    gps.coordinates = {coordinate.longitude, coordinate.latitude};
    gps.speed = coordinate.speed_kmh;
    gps.heading = coordinate.heading;
    gps.altitude = coordinate.altitude;
}

void updateRPM(velocity_t r) {
    // Store inverted because the rpm is inverse
    motor.rpm = r * -1;
}

void updateMotorOn(boolean isOn) {
    motor.isOn = isOn;
    motor.heartbeatTimestamp = 0; //TODO: Fix this
}

void periodicMotorOn() {
    if (motor.isOn && (motor.heartbeatTimestamp + MOTOR_ON_TIMEOUT) < 0) {
        motor.isOn = false;
    }
}

void updateSpeed(speed_t s) {
    // store in km / h
    speed = (float) s / 1000;
}

void updateThrottle(percentage_t p) {
    throttle = (float) p / 10;
}

void triggerLap() {
    // Construct a new LapData
    LapData &currentLap = laps.back();
    currentLap.totalTime = 0;

    laps.emplace_back(0, 0);
}

#endif //UOSM_TELEMETRY_THINGPROPERTIES_H
