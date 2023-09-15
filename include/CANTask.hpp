//
// Created by Jeremy Cote on 2023-09-15.
//

#ifndef UOSM_TELEMETRY_CANTASK_HPP
#define UOSM_TELEMETRY_CANTASK_HPP

#include <Arduino.h>
#include "ApplicationTypes.h"

void CANInit(
        uint32_t stackSize,
        UBaseType_t priority,
        uint16_t _pollingRate
);


#endif //UOSM_TELEMETRY_CANTASK_HPP
