//
// Created by Jeremy Cote on 2023-09-08.
//

#ifndef UOSM_TELEMETRY_LOGGERTASK_HPP
#define UOSM_TELEMETRY_LOGGERTASK_HPP

#include <Arduino.h>
#include <SD.h>
#include <cmsis_os.h>

void LoggerInit(
        uint16_t _logRate,
        const std::function<String()>& _constructRow,
        const String& _header
);

#endif //UOSM_TELEMETRY_LOGGERTASK_HPP
