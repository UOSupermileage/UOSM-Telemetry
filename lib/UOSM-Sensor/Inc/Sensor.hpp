//
// Created by Jeremy Cote on 2023-08-26.
//

#ifndef UOSM_TELEMETRY_SENSOR_HPP
#define UOSM_TELEMETRY_SENSOR_HPP

#include "ObservedDataQueue.hpp"
#include "Collector.hpp"

template<typename T>
class Sensor: public Collector, public Observable<T> {
private:
    ObservedObject<DataQueue<T>>* data;
protected:
    void add(T value) {
        data->getMutable().add(value);
        data->publish();
    }
public:
    /**
     *  Create a new sensor.
     * @param bufferSize How many items of type T to store
     */
    explicit Sensor(uint8_t bufferSize) {
        auto* queue = new DataQueue<T>(10);
        data = new ObservedObject<DataQueue<T>>(queue, true);
    }

    virtual ~Sensor() {
        delete data;
    }

    // Observable Protocol

    /**
     * @return the newest value added to the collection.
     */
    T& get() const override {
        return data->get().getLatestValue();
    }

    ObserverToken addListener(std::function<void(const T&)> callback) override {
        return data->addListener([callback](const DataQueue<T>& q) {
            callback(q.getLatestValue());
        });
    }
};

#endif //UOSM_TELEMETRY_SENSOR_HPP
