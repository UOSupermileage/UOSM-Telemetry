//
// Created by etara on 2023-08-28.
//
#include "Sensor.hpp"
#include "ApplicationTypes.h"
#include "SparkFunLIS3DH.h"

#ifdef MBED
#include <Arduino_PortentaBreakout.h>
#define CS GPIO_1
#else
#define CS 0
#endif
#define G_RANGE 2
#define GRAVITY 9.81f
#define SAMPLE_RATE 400
#define VERBOSE_SERIAL

class Accelerometer: public Sensor<acceleration_t>{
private:
    LIS3DH handle = LIS3DH(I2C_MODE,CS);

public:
    explicit Accelerometer(uint8_t buffer_size): Sensor<acceleration_t>(buffer_size) {
        handle.settings.accelRange = G_RANGE;
        handle.settings.accelSampleRate = SAMPLE_RATE;

        // Start LIS3DH module with applied settings
        handle.begin();
    };

    void collect() override {
        DebugPrint("Collecting Accelerometer data\n");
        acceleration_t acceleration;

        acceleration.x = handle.readFloatAccelX() * GRAVITY;
        acceleration.y = handle.readFloatAccelY() * GRAVITY;
        acceleration.z = handle.readFloatAccelZ() * GRAVITY;

        DebugPrint("Collected Accelerometer data, %f %f %f\n", acceleration.x, acceleration.y, acceleration.z);

        add(acceleration);
    }
};
