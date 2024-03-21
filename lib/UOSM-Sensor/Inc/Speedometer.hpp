//
// Created by Jeremy Cote on 2024-02-22.
//

#ifndef UOSM_TELEMETRY_PIO_SPEEDOMETER_HPP
#define UOSM_TELEMETRY_PIO_SPEEDOMETER_HPP

#include "Sensor.hpp"
#include "ApplicationTypes.h"
#include <Arduino.h>
#include "exception"

#define SPEEDOMETER_IDLE 0
#define SPEEDOMETER_DONE 1

#define HALL_BOLTS 4
#define HALL_RADIUS 0.2667
#define HALL_CIRC HALL_RADIUS * PI_T2

class Speedometer: public Sensor<speed_t> {
private:
    volatile uint8_t state = SPEEDOMETER_IDLE;
    volatile uint32_t time1 = 0;
    volatile uint32_t time2 = 0;
    volatile uint32_t nTicks = 0;
    volatile uint32_t hallFrequency = 0;
    volatile uint32_t clicks = 0;
public:
    void collect() override{
        printf("@%d | hallFrenquency: %d. Clicks: %d\n",osKernelGetTickCount(), hallFrequency, clicks);
        speed_t value = (uint32_t) (hallFrequency * 36 * HALL_CIRC) / (HALL_BOLTS);
        notify(value);
    }

    void hallCallback() {
        clicks++;
        if (state == SPEEDOMETER_IDLE) {
            time1 = osKernelGetTickCount();//TIM1->CCR1;
            state = SPEEDOMETER_DONE;
            return;
        }

        if (state != SPEEDOMETER_DONE) {
            return;
        }

        state = SPEEDOMETER_IDLE;

        time2 = osKernelGetTickCount();//TIM1->CCR1;

        if (time2 < time1) {
            return;
        }

        nTicks = time2 - time1;

        if (nTicks == 0) {
            return;
        }

        hallFrequency = (uint32_t)(100 * osKernelGetTickFreq() / nTicks);
    }
};

#endif //UOSM_TELEMETRY_PIO_SPEEDOMETER_HPP
