//
// Created by Jeremy Cote on 2023-08-29.
//

#ifndef UOSM_TELEMETRY_THINGPROPERTIES_H
#define UOSM_TELEMETRY_THINGPROPERTIES_H

#include <ArduinoIoTCloud.h>
#include <Arduino_ConnectionHandler.h>

const char DEVICE_LOGIN_NAME[]  = "";

const char SSID[]               = "";    // Network SSID (name)
const char PASS[]               = "";    // Network password (use for WPA, or use as key for WEP)
const char DEVICE_KEY[]  = "";    // Secret device password

// Name of variables is important. They map to definitions in our IOT Cloud Dashboard
float battery_Voltage;

// Acceleration
float acceleration_x;
float acceleration_y;
float acceleration_z;

// GPS
float latitude;
float longitude;
float gps_speed; // in km / h
float heading;
float altitude;

void initProperties(){

    ArduinoCloud.setBoardId(DEVICE_LOGIN_NAME);
    ArduinoCloud.setSecretDeviceKey(DEVICE_KEY);

    ArduinoCloud.addProperty(battery_Voltage, READ, ON_CHANGE, NULL);
}

void updateBatteryVoltage(float voltage) {
    battery_Voltage = voltage;
}

void updateAcceleration(acceleration_t acceleration){
    acceleration_x = acceleration.x;
    acceleration_y = acceleration.y;
    acceleration_z = acceleration.z;
}

void updateGPS(gps_coordinate_t coordinate) {
    latitude = coordinate.latitude;
    longitude = coordinate.longitude;
    gps_speed = coordinate.speed_kmh;
    heading = coordinate.heading;
    altitude = coordinate.altitude;
}

WiFiConnectionHandler ArduinoIoTPreferredConnection(SSID, PASS);

#endif //UOSM_TELEMETRY_THINGPROPERTIES_H
