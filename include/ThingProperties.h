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

acceleration_t acceleration;
gps_coordinate_t coordinates;

void initProperties(){

    ArduinoCloud.setBoardId(DEVICE_LOGIN_NAME);
    ArduinoCloud.setSecretDeviceKey(DEVICE_KEY);

    ArduinoCloud.addProperty(battery_Voltage, READ, ON_CHANGE, NULL);
}

void updateBatteryVoltage(float voltage) {
    battery_Voltage = voltage;
}

void updateAcceleration(acceleration_t a){
    acceleration = a;
}

void updateGPS(gps_coordinate_t c) {
    coordinates = c;
}


WiFiConnectionHandler ArduinoIoTPreferredConnection(SSID, PASS);

#endif //UOSM_TELEMETRY_THINGPROPERTIES_H
