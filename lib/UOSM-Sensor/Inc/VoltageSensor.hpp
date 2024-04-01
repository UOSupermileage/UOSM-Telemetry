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
#include "Config.h"

class VoltageSensor: public Sensor<voltage_current_t> {
public:
    enum class VoltageSensorMode: uint8_t { Differential_0_1, Differential_2_3 };
private:
    ADS1219* ads;

    bool initialized = false;

public:
    explicit VoltageSensor(TwoWire& i2c, uint8_t drdy_pin, uint16_t address): ads(new ADS1219(i2c, drdy_pin, address)) {

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

        voltage_current_t results;

#if SENSOR_VOLTAGE == 1
        ads->set_input_mux(ADS1219_MUX::DIFF_P0_N1);
        if (ads->start()) {
            uint16_t timeout = 10;
            while (!ads->is_data_ready() && timeout-- > 0) {
                osDelay(10);
            }

            if (timeout == 0) {
                return;
            }

            ads->read_conversion();

            // Multiply result by 19.29 to convert from ADC voltage to Battery voltage (Scaling voltage from 3.3 to larger range)
            results.voltage = (voltage_t) ads->get_millivolts(3300.0) * 19.29;
        }
#endif

#if SENSOR_CURRENT == 1
        ads->set_input_mux(ADS1219_MUX::DIFF_P2_N3);
        if (ads->start()) {
            uint16_t timeout = 10;
            while (!ads->is_data_ready() && timeout-- > 0) {
                osDelay(10);
            }

            if (timeout == 0) {
                return;
            }

            ads->read_conversion();

            // Divide by 12.5 to get number of milliamps from current sensor
            Serial.print("Raw ads current: "); Serial.println(ads->get_raw());
            Serial.println("ads current: "); Serial.println(ads->get_millivolts(3300));
            results.current = ads->get_millivolts(3300.0) * 1000 / 12.5f;
        }
#endif

        notify(results);
    }
};

#endif //UOSM_TELEMETRY_VOLTAGESENSOR_HPP
