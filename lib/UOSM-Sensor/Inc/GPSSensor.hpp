//
// Created by Jeremy Cote on 2023-09-06.
//

#ifndef UOSM_TELEMETRY_GPSSENSOR_HPP
#define UOSM_TELEMETRY_GPSSENSOR_HPP

#include "Sensor.hpp"
#include "Fona3G.hpp"
#include <exception>

class GPSSensor: public Sensor<gps_coordinate_t> {
private:
    Fona3G* fona;
public:
    GPSSensor(Fona3G* fona, uint8_t bufferSize): Sensor<gps_coordinate_t>(bufferSize), fona(fona) {

    }

    void collect() override {
        if (!fona->awaitAvailability()) {
            return;
        }

        try {
            add(fona->getGPSCoordinate());
        } catch (std::runtime_error e) {
            Serial.printf("Failed to get GPS coordinates due to: %s\n", e.what());
        }
    };
};

#endif //UOSM_TELEMETRY_GPSSENSOR_HPP
