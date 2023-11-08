//
// Created by Jeremy Cote on 2023-09-15.
//

#include "CANTask.hpp"
#include "UOSMCoreConfig.h"

#include "InternalCommsModule.h"
#include <Arduino.h>

#include <rtos.h>

// RTOS Handles
static rtos::Thread canThread;

uint16_t pollingRate;

// RTOS Execution Loops
[[noreturn]] void CANTask() {

    pinMode(MCP2515_CS_PIN, OUTPUT);

    result_t isInitialized = RESULT_FAIL;

    while (true) {
        if (isInitialized == RESULT_FAIL) {
            isInitialized = IComms_Init();
        } else {
            IComms_PeriodicReceive();
        }

        rtos::ThisThread::sleep_for(pollingRate);
    }
}

void CANInit(
        uint32_t stackSize,
        osPriority_t priority,
        uint16_t _pollingRate
) {
    pollingRate = _pollingRate;
    canThread.set_priority(priority);
    canThread.start(mbed::callback(CANTask));
}

