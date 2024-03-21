//
// Created by Jeremy Cote on 2023-08-26.
//

#ifndef UOSM_TELEMETRY_SENSOR_HPP
#define UOSM_TELEMETRY_SENSOR_HPP

#include "Collector.hpp"
#include "Observable.hpp"

template<typename T>
class Sensor: public Collector, public Observable<T> {

};

#endif //UOSM_TELEMETRY_SENSOR_HPP
