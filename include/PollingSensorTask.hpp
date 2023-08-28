//
// Created by Jeremy Cote on 2023-08-27.
//

#ifndef UOSM_TELEMETRY_POLLINGSENSORTASK_HPP
#define UOSM_TELEMETRY_POLLINGSENSORTASK_HPP

#include "SensorTask.hpp"
#include <Arduino.h>

template<typename T>
class PollingSensorTask: public SensorTask {
private:
    TaskHandle_t handle = NULL;

    static void loop(void* parameters) {
        PollingSensorTaskArgs* args = (PollingSensorTaskArgs*) parameters;

        for(;;) {
            args->collector->collect();
            vTaskDelay(args->pollingRate);
        }
    }

public:
    struct PollingSensorTaskArgs {
        Collector* collector;
        uint16_t pollingRate;

        PollingSensorTaskArgs(Collector* collector, uint16_t pollingRate): collector(collector), pollingRate(pollingRate) {}
    };

    PollingSensorTask(Sensor<T>* sensor, const uint16_t pollingRate, const char* name, uint32_t stackSize, UBaseType_t priority) {
        auto* args = new PollingSensorTaskArgs(sensor, pollingRate);
        xTaskCreate(PollingSensorTask::loop, name, stackSize, args, priority, &handle);
    }
};

#endif //UOSM_TELEMETRY_POLLINGSENSORTASK_HPP
