//
// Created by Jeremy Cote on 2023-09-08.
//

#ifndef UOSM_TELEMETRY_VALUESENSOR_HPP
#define UOSM_TELEMETRY_VALUESENSOR_HPP

#include "Sensor.hpp"

template<typename T>
class ValueSensor: public Sensor<T> {
protected:
    using Sensor<T>::notify;
public:
    void collect() override {};

    virtual void collect(T& value) {
        notify(value);
    }
};

#endif //UOSM_TELEMETRY_VALUESENSOR_HPP
