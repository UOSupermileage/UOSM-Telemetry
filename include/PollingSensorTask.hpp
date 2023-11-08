//
// Created by Jeremy Cote on 2023-08-27.
//

#ifndef UOSM_TELEMETRY_POLLINGSENSORTASK_HPP
#define UOSM_TELEMETRY_POLLINGSENSORTASK_HPP

#include "SensorTask.hpp"
#include <Arduino.h>
#include <cmsis.h>
#include <rtos.h>

template<typename T>
class PollingSensorTask: public SensorTask {
private:
    rtos::Thread thread;

    static void loop(void* parameters) {
        auto* args = (PollingSensorTaskArgs*) parameters;

        for(;;) {
            args->collector->collect();
            rtos::ThisThread::sleep_for(args->pollingRate);
        }
    }

public:
    struct PollingSensorTaskArgs {
        Collector* collector;
        uint16_t pollingRate;

        PollingSensorTaskArgs(Collector* collector, uint16_t pollingRate): collector(collector), pollingRate(pollingRate) {}
    };

    PollingSensorTask(Sensor<T>* sensor, const uint16_t pollingRate, const char* name, uint32_t stackSize, osPriority_t priority) {
        // TODO: Is this a memory leak?
        auto* args = new PollingSensorTaskArgs(sensor, pollingRate);
        thread.set_priority(priority);
        thread.start(mbed::callback(PollingSensorTask::loop, args));
    }
};

#endif //UOSM_TELEMETRY_POLLINGSENSORTASK_HPP
