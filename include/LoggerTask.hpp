//
// Created by Jeremy Cote on 2023-09-08.
//

#ifndef UOSM_TELEMETRY_LOGGERTASK_HPP
#define UOSM_TELEMETRY_LOGGERTASK_HPP

#include <Arduino.h>
#include <SD.h>
#include <cmsis_os.h>

void LoggerInit(
        uint8_t _chipSelectPin,
        uint8_t _signalLightPin,
        uint8_t _sdDetectPin,
        uint8_t _logButtonPin,
        const std::function<String()>& _constructRow,
        const String& _header,
        uint32_t stackSize,
        osPriority_t priority,
        uint16_t _logRate
);

#endif //UOSM_TELEMETRY_LOGGERTASK_HPP
