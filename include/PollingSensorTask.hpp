//
// Created by Jeremy Cote on 2023-08-27.
//

#ifndef UOSM_TELEMETRY_POLLINGSENSORTASK_HPP
#define UOSM_TELEMETRY_POLLINGSENSORTASK_HPP

#include "SensorTask.hpp"
#include "Config.h"
#include <Arduino.h>
#include <cmsis.h>
#include <rtos.h>
#include <LibPrintf.h>

template<typename T>
class PollingSensorTask: public SensorTask {
private:
    rtos::Thread thread;

    static void loop(void* parameters) {
        auto* args = (PollingSensorTaskArgs*) parameters;

        for(;;) {
            printf("PollingSensorTask loop\n");
            args->collector->collect();

            printf("Starting Thread Sleep for %d ms\n", args->pollingRate);
            rtos::ThisThread::sleep_for(std::chrono::milliseconds(args->pollingRate));
            printf("Ended Thread Sleep\n");
        }
    }

public:
    struct PollingSensorTaskArgs {
        Collector* collector;
        uint16_t pollingRate;

        PollingSensorTaskArgs(Collector* collector, uint16_t pollingRate): collector(collector), pollingRate(pollingRate) {}
    };

    PollingSensorTask(Sensor<T>* sensor, const uint16_t pollingRate, const char* name, uint32_t stackSize, osPriority_t priority) {
        // Not a memory leak because we use the args in the loop.
        printf("Creating PollingSensorTask with pollingRate: %d\n", pollingRate);

        auto* args = new PollingSensorTaskArgs(sensor, pollingRate);

        printf("Starting RTOS Thread\n");
        thread.start(mbed::callback(PollingSensorTask::loop, args));

//        DebugPrint("Setting priority");
//        thread.set_priority(priority);
//
//        DebugPrint("Set priority");
    }
};

#endif //UOSM_TELEMETRY_POLLINGSENSORTASK_HPP
