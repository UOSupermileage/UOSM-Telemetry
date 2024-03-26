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

    LapData(uint64_t totalJoules, uint32_t totalTime) : totalJoules(totalJoules), totalTime(totalTime) {}
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
    std::vector<LapData> laps;

    CloudElectricCurrent batteryCurrent;
    current_t raw_batteryCurrent;
    CloudElectricPotential batteryVoltage;
    voltage_t raw_batteryVoltage;

    String canMessage;
    CloudPercentage throttle;
    MotorData motor;
    CloudAcceleration accelerationX;
    CloudAcceleration accelerationY;
    CloudAcceleration accelerationZ;
    CloudPressure pressure;
    CloudTemperature temperature;
    GPSData gps;
    CloudVelocity speed;
    CloudFloat airSpeed;
    CloudPercentage brakesPercentage;

    CloudDatabase() {
        laps.emplace_back(LapData(0, 0));
    }

public:
    static CloudDatabase instance;

    void SetupThing() {
        ArduinoCloud.setThingId(DEVICE_LOGIN_NAME);

        ArduinoCloud.addProperty(accelerationX, READ, ON_CHANGE, NULL);
        ArduinoCloud.addProperty(accelerationY, READ, ON_CHANGE, NULL);
        ArduinoCloud.addProperty(accelerationZ, READ, ON_CHANGE, NULL);
        ArduinoCloud.addProperty(canMessage, READ, ON_CHANGE, NULL);
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
        periodicMotorOn();
        ArduinoCloud.update();
    }

    voltage_t getBatteryVoltage() { return raw_batteryVoltage; }

    current_t getBatteryCurrent() { return raw_batteryCurrent; }

    float getAccelerationX() { return accelerationX; }

    float getAccelerationY() { return accelerationY; }

    float getAccelerationZ() { return accelerationZ; }

    int getRPM() { return motor.rpm; }

    float getThrottle() { return throttle; }

    float getSpeed() { return speed; }

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

        LapData &currentLap = laps.back();
        currentLap.totalJoules += (uint64_t) raw_batteryVoltage * raw_batteryCurrent;
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
    }

    void updateThrottle(percentage_t p) {
        throttle = (float) p / 10;
    }

    void triggerLap() {

        if (laps.size() > 4) {
            laps.erase(laps.begin());
        }

        // Construct a new LapData
        laps.emplace_back(0, 0);
    }

    void getLapEfficiencies(lap_efficiencies_t* efficiencies) {
        if (efficiencies == nullptr) { return; }

        uint8_t offset = 4 > laps.size() ? 4 : laps.size();

        uint64_t largest = 1;

        // Iterate over the last 4 elements in the vector to get the largest
        for (auto it = laps.end() - offset; it < laps.end(); it++) {
            if (it->totalJoules > largest) {
                largest = it->totalJoules;
            }
        }

        uint8_t efficiency_percentages[4] { 0, 0, 0, 0};
        for (auto it = laps.end() - offset; it < laps.end(); it++) {
            uint8_t index = laps.end() - it;

            if (index < 4) {
                efficiency_percentages[index] = (uint8_t) ((it->totalJoules / largest) * 200);
            }
        }

        efficiencies->lap_0 = efficiency_percentages[0];
        efficiencies->lap_1 = efficiency_percentages[1];
        efficiencies->lap_2 = efficiency_percentages[2];
        efficiencies->lap_3 = efficiency_percentages[3];
    }
};

#endif //UOSM_TELEMETRY_THINGPROPERTIES_H
