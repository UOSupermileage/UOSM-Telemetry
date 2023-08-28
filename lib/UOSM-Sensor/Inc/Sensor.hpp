//
// Created by Jeremy Cote on 2023-08-26.
//

#ifndef UOSM_TELEMETRY_SENSOR_HPP
#define UOSM_TELEMETRY_SENSOR_HPP

#include "ObservedDataQueue.hpp"
#include "Collector.hpp"
#include "ValueStore.hpp"

template<typename T>
class Sensor: public Collector, public ValueStore<T>, public ObservedObject<ValueStore<T>> {
private:
    DataQueue<T> data;
protected:
    void addValue(T value) {
        data.add(value);
    }
public:
    /**
     *  Create a new sensor.
     * @param bufferSize How many items of type T to store
     */
    explicit Sensor(uint8_t bufferSize) : data(bufferSize), ValueStore<T>(), ObservedObject<ValueStore<T>>(static_cast<ValueStore<T>*>(this), false) {

    }

    virtual ~Sensor() = default;

    /**
     * @return the newest value added to the collection.
     */
    T getValue() {
        return data.getLatestValue();
    }

    /**
     * Collect data from sensor and add as new datapoint in it's internal buffer.
     */
    virtual void collect() = 0;

    ObserverToken addListener(std::function<void(const ObservedDataQueue<Sensor<T>>&)> callback) {
        return ObservedObject<Sensor<T>>::addListener(callback);
    }
};

#endif //UOSM_TELEMETRY_SENSOR_HPP
