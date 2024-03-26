//
// Created by Jeremy Cote on 2023-08-27.
//

#ifndef UOSM_TELEMETRY_VOLTAGESENSOR_HPP
#define UOSM_TELEMETRY_VOLTAGESENSOR_HPP

#include <Arduino.h>

#include "ApplicationTypes.h"
#include "Sensor.hpp"

#include "ADS1219.h"
#include <SparkFun_ADS1219.h>

class VoltageSensor: public Sensor<voltage_t> {
public:
    enum class VoltageSensorMode: uint8_t { Differential_0_1, Differential_2_3 };
private:
    ADS1219* ads;
    VoltageSensorMode mode;

    bool initialized = false;

public:
    explicit VoltageSensor(TwoWire& i2c, VoltageSensorMode mode, uint8_t drdy_pin, uint16_t address): ads(new ADS1219(i2c, drdy_pin, address)), mode(mode) {

    }

    virtual ~VoltageSensor() {
        delete ads;
    }

    /**
     * Collect a voltage and store it in the sensor's internal buffer
     */
    void collect() override {

        if (ads == nullptr) {
            DebugPrint("Voltage Sensor internal pointer in null. Aborting collection()\n");
            return;
        }

        if (!initialized) {
            initialized = ads->begin() && ads->set_voltage_ref(EXTERNAL) && ads->set_conversion_mode(SINGLE_SHOT);
            if (!initialized) {
                return;
            }
        }

        switch (mode) {
            case VoltageSensorMode::Differential_0_1:
                ads->set_input_mux(ADS1219_MUX::DIFF_P0_N1);
                break;
            case VoltageSensorMode::Differential_2_3:
                ads->set_input_mux(ADS1219_MUX::DIFF_P2_N3);
                break;
        }

        if (ads->start()) {
            uint16_t timeout = 10;
            while (!ads->is_data_ready() && timeout-- > 0) {
                osDelay(10);
            }

            if (timeout == 0) {
                return;
            }

            ads->read_conversion();
            voltage_t voltage = (voltage_t) ads->get_millivolts(3300.0);
            printf("Collected voltage (mV): %d\n", voltage);

            notify(voltage);
        }
    }
};

#endif //UOSM_TELEMETRY_VOLTAGESENSOR_HPP
