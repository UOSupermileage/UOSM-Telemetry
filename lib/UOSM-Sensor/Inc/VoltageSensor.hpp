//
// Created by Jeremy Cote on 2023-08-27.
//

#ifndef UOSM_TELEMETRY_VOLTAGESENSOR_HPP
#define UOSM_TELEMETRY_VOLTAGESENSOR_HPP

#include <Arduino.h>

#include "ApplicationTypes.h"
#include "Sensor.hpp"

#include <ADS1219.h>

class VoltageSensor: public Sensor<voltage_t> {
public:
    enum class VoltageSensorMode: uint8_t { Differential_0_1, Differential_2_3 };
private:
    ADS1219 ads;

    VoltageSensorMode mode;

public:
    explicit VoltageSensor(VoltageSensorMode mode, uint8_t drdy_pin, uint8_t reset_pin, uint16_t address, uint8_t bufferSize): Sensor<voltage_t>(bufferSize), mode(mode), ads(drdy_pin, address) {
        pinMode(drdy_pin, INPUT_PULLUP);
        pinMode(reset_pin, OUTPUT);

        digitalWrite(reset_pin, HIGH);

        ads.setVoltageReference(REF_EXTERNAL);
    }

    /**
     * Collect a voltage and store it in the sensor's internal buffer
     */
    void collect() override {

        long voltage = 0;
        switch (mode) {
            case VoltageSensorMode::Differential_0_1:
                voltage = ads.readDifferential_0_1();
                break;
            case VoltageSensorMode::Differential_2_3:
                voltage = ads.readDifferential_2_3();
                break;
        }

        // Get some arbitrary data for testing
        printf("Collected voltage %ld", voltage);
        // TODO: Is this losing accuracy in conversion? Long -> uint16_t. What does this imply.
        add(voltage);
    }
};

#endif //UOSM_TELEMETRY_VOLTAGESENSOR_HPP
