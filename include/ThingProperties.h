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

float battery_Voltage;

void initProperties(){

    ArduinoCloud.setBoardId(DEVICE_LOGIN_NAME);
    ArduinoCloud.setSecretDeviceKey(DEVICE_KEY);

    ArduinoCloud.addProperty(battery_Voltage, READ, ON_CHANGE, NULL);
}

void updateBatteryVoltage(float voltage) {
    battery_Voltage = voltage;
}

void updateAcceleration(accel_t acceleration_struct){ 
    new_accel_x = acceleration_struct.accel_x; 
    new_accel_y = acceleration_struct.accel_y; 
    new_accel_z = acceleration_struct.accel_z; 
}

WiFiConnectionHandler ArduinoIoTPreferredConnection(SSID, PASS);

#endif //UOSM_TELEMETRY_THINGPROPERTIES_H
