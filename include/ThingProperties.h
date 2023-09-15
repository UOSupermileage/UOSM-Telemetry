//
// Created by Jeremy Cote on 2023-08-29.
//

#ifndef UOSM_TELEMETRY_THINGPROPERTIES_H
#define UOSM_TELEMETRY_THINGPROPERTIES_H

#include <ArduinoIoTCloud.h>
#include <Arduino_ConnectionHandler.h>
#include "Mutex.hpp"

const char DEVICE_LOGIN_NAME[]  = "484c5baf-f235-40cf-9a75-3dfc3334a2dd";

const char SSID[]               = "Jeremy";    // Network SSID (name)
const char PASS[]               = "PeaceTea";    // Network password (use for WPA, or use as key for WEP)
const char DEVICE_KEY[]         = "RYVMGY6U1SV9YGEBWDG7";    // Secret device password

Mutex iotMutex;

// Name of variables is important. They map to definitions in our IOT Cloud Dashboard
float battery_voltage;

// Acceleration Sensor
CloudAcceleration acceleration_x;
CloudAcceleration acceleration_y;
CloudAcceleration acceleration_z;

// Pressure Sensor
double pressure;
double temp;

// GPS Sensor
float latitude;
float longitude;
float gps_speed; // in km / h
float heading;
float altitude;

// CAN Sensor
velocity_t rpm;
speed_t speed;
percentage_t throttle;

void initProperties(){
    ArduinoCloud.setBoardId(DEVICE_LOGIN_NAME);
    ArduinoCloud.setSecretDeviceKey(DEVICE_KEY);

    ArduinoCloud.addProperty(battery_voltage, READ, ON_CHANGE, NULL);
    ArduinoCloud.addProperty(acceleration_x, READ, ON_CHANGE, NULL);
    ArduinoCloud.addProperty(acceleration_y, READ, ON_CHANGE, NULL);
    ArduinoCloud.addProperty(acceleration_z, READ, ON_CHANGE, NULL);
}

void updateBatteryVoltage(float voltage) {
    DebugPrint("Collected Voltage: [%d]", voltage);
    battery_voltage = voltage;
}

void updateAcceleration(acceleration_t acceleration){
    acceleration_x = acceleration.x;
    acceleration_y = acceleration.y;
    acceleration_z = acceleration.z;

    DebugPrint("Accelerometer: %f %f %f", acceleration.x, acceleration.y, acceleration.z);
}

void updatePressure(pressure_t p){
    pressure = p.pressure;
    temp = p.temp;
}

void updateGPS(gps_coordinate_t coordinate) {
    latitude = coordinate.latitude;
    longitude = coordinate.longitude;
    gps_speed = coordinate.speed_kmh;
    heading = coordinate.heading;
    altitude = coordinate.altitude;
}

void updateRPM(velocity_t r) {
    rpm = r;
}

void updateSpeed(speed_t s) {
    speed = s;
}

void updateThrottle(percentage_t p) {
    throttle = p;
}

WiFiConnectionHandler ArduinoIoTPreferredConnection(SSID, PASS);

#endif //UOSM_TELEMETRY_THINGPROPERTIES_H
