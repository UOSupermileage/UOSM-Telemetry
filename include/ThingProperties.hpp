//
// Created by Jeremy Cote on 2023-08-29.
//

#ifndef UOSM_TELEMETRY_THINGPROPERTIES_H
#define UOSM_TELEMETRY_THINGPROPERTIES_H

#include "ApplicationTypes.h"

#include <ArduinoIoTCloud.h>
#include "Mutex.hpp"
#include "Secrets.h"
#include <vector>

#define MOTOR_ON_TIMEOUT 500

struct LapData {
    uint64_t totalJoules;
    uint32_t totalTime;

    LapData(uint64_t totalJoules = 0, uint32_t totalTime = 0) : totalJoules(totalJoules), totalTime(totalTime) {}
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

class CloudDatabase {
private:
    LapData a;
    LapData b;
    LapData c;
    LapData d;

    CloudElectricCurrent batteryCurrent;
    current_t raw_batteryCurrent;
    CloudElectricPotential batteryVoltage;
    voltage_t raw_batteryVoltage;

    CloudVelocity speed;
    speed_t raw_speed;

    String canMessage;
    CloudPercentage throttle;
    MotorData motor;
    CloudAcceleration accelerationX;
    CloudAcceleration accelerationY;
    CloudAcceleration accelerationZ;
    CloudPressure pressure;
    CloudTemperature temperature;
    GPSData gps;
    CloudFloat airSpeed;
    CloudPercentage brakesPercentage;

public:
    static CloudDatabase instance;

    void SetupThing() {
//        ArduinoCloud.setDeviceId(DEVICE_LOGIN_NAME);
//        ArduinoCloud.setThingId(DEVICE_LOGIN_NAME);
        ArduinoCloud.addProperty(batteryCurrent, READ, ON_CHANGE, NULL);
        ArduinoCloud.addProperty(batteryVoltage, READ, ON_CHANGE, NULL);
        ArduinoCloud.addPropertyReal(motor.rpm, "motorRPM", READ, ON_CHANGE, NULL);
        ArduinoCloud.addProperty(throttle, READ, ON_CHANGE, NULL);
        ArduinoCloud.addProperty(temperature, READ, ON_CHANGE, NULL);
        ArduinoCloud.addProperty(pressure, READ, ON_CHANGE, NULL);
        ArduinoCloud.addProperty(speed, READ, ON_CHANGE, NULL);
        ArduinoCloud.addPropertyReal(motor.isOn, "motorOn", READ, ON_CHANGE, NULL);
        ArduinoCloud.addPropertyReal(gps.coordinates, "gpsCoordinates", READ, ON_CHANGE, NULL);
        ArduinoCloud.addPropertyReal(airSpeed, "airSpeed", READ, ON_CHANGE, NULL);
    }

    void PeriodicUpdate() {
//        periodicMotorOn();
        ArduinoCloud.update();
    }

    voltage_t getBatteryVoltage() { return raw_batteryVoltage; }

    current_t getBatteryCurrent() { return raw_batteryCurrent; }

    float getAccelerationX() { return accelerationX; }

    float getAccelerationY() { return accelerationY; }

    float getAccelerationZ() { return accelerationZ; }

    int getRPM() { return motor.rpm; }

    float getThrottle() { return throttle; }

    speed_t getSpeed() { return raw_speed; }

    float getPressure() { return pressure; }

    float getTemperature() { return temperature; }

    float getBrakesPercentage() {
        return brakesPercentage;
    }

    void updateBrakesPercentage(float percentage) {
        brakesPercentage = percentage;
    }

    void updateBatteryVoltage(voltage_t voltage) {
        raw_batteryVoltage = voltage;
        batteryVoltage = (float) voltage / 1000;
    }

    void updateBatteryCurrent(current_t current) {
        raw_batteryCurrent = current;
        batteryCurrent = (float) current / 1000;

        DebugPrint("Current %d, Voltage %d\n", raw_batteryCurrent, raw_batteryVoltage);

        d.totalJoules += (uint64_t) raw_batteryVoltage * raw_batteryCurrent;
    }

    void updateAcceleration(acceleration_t acceleration) {
        accelerationX = acceleration.x;
        accelerationY = acceleration.y;
        accelerationZ = acceleration.z;
    }

    void updateCanMessages(const char *message) {
        canMessage = message;
    }

    void updatePressure(pressure_t pressure, temperature_t temperature);

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
        raw_speed = s;
    }

    void updateThrottle(percentage_t p) {
        throttle = (float) p / 10;
    }

    void triggerLap() {
        a = b;
        b = c;
        c = d;
        d = LapData(0, 0);
    }

    void getLapEfficiencies(lap_efficiencies_t* efficiencies) {
        if (efficiencies == nullptr) { return; }

        efficiencies->lap_0 = a.totalJoules;
        efficiencies->lap_1 = 100;
        efficiencies->lap_2 = c.totalJoules;
        efficiencies->lap_3 = d.totalJoules;
    }
};

#endif //UOSM_TELEMETRY_THINGPROPERTIES_H
