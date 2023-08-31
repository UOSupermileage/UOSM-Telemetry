//
// Created by Jeremy Cote on 2023-08-29.
//

#ifndef UOSM_TELEMETRY_CONVERSIONS_H
#define UOSM_TELEMETRY_CONVERSIONS_H

#include "ApplicationTypes.h"

/**
 * Convert the voltage ADC code to a voltage.
 * @param voltage
 * @return
 */
float convertVoltageToFloat(voltage_t voltage) {
    // TODO: Make this actually work
    return voltage * 1.5;
}

#endif //UOSM_TELEMETRY_CONVERSIONS_H
