//
// Created by Jeremy Cote on 2023-09-08.
//

#ifndef UOSM_TELEMETRY_VALUESENSOR_HPP
#define UOSM_TELEMETRY_VALUESENSOR_HPP

#include "Sensor.hpp"

template<typename T>
class ValueSensor: public Sensor<T> {
protected:
    using Sensor<T>::add;
public:
    explicit ValueSensor(uint8_t bufferSize): Sensor<T>(bufferSize) {}

    void collect() override {};
    virtual void collect(T value) {
        add(value);
    }
};

#endif //UOSM_TELEMETRY_VALUESENSOR_HPP
