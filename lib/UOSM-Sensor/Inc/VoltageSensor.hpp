//
// Created by Jeremy Cote on 2023-08-27.
//

#ifndef UOSM_TELEMETRY_VOLTAGESENSOR_HPP
#define UOSM_TELEMETRY_VOLTAGESENSOR_HPP

#include "ApplicationTypes.h"
#include "Sensor.hpp"

class VoltageSensor: public Sensor<voltage_t> {
public:
    explicit VoltageSensor(uint8_t bufferSize): Sensor<voltage_t>(bufferSize) {}

    /**
     * Collect the battery's voltage and store it in the sensor's internal buffer
     */
    void collect() override {
        add(10);
    }
};

#endif //UOSM_TELEMETRY_VOLTAGESENSOR_HPP
