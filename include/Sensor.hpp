//
// Created by Jeremy Cote on 2023-08-26.
//

#ifndef UOSM_TELEMETRY_SENSOR_HPP
#define UOSM_TELEMETRY_SENSOR_HPP

template<typename T>
class Sensor {
public:
    virtual T GetValue() = 0;
    virtual bool UpdateValue() = 0;
    virtual void RegisterListener() {

    }
};

#endif //UOSM_TELEMETRY_SENSOR_HPP
