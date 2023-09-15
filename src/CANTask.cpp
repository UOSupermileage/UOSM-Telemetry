//
// Created by Jeremy Cote on 2023-09-15.
//

#include "CANTask.hpp"

#include "InternalCommsModule.h"

// RTOS Handles
static TaskHandle_t handle = nullptr;

uint16_t pollingRate;

// RTOS Execution Loops
[[noreturn]] void CANTask(void* args) {
    result_t isInitialized = RESULT_FAIL;

    while (true) {
        if (isInitialized == RESULT_FAIL) {
            isInitialized = IComms_Init();
        } else {
            IComms_PeriodicReceive();
        }

        vTaskDelay(pollingRate);
    }
}

void CANInit(
        uint32_t stackSize,
        UBaseType_t priority,
        uint16_t _pollingRate
) {
    pollingRate = _pollingRate;
    xTaskCreate(CANTask, "CANTask", stackSize, nullptr, priority, &handle);
}

