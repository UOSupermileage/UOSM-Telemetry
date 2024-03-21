//
// Created by etara on 2023-08-28.
//
#include "Sensor.hpp"
#include "ApplicationTypes.h"

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

/**
 * 50hz mode, high power, all axis.
 */
#define CTRL_REG1_VALUE 0x47

/**
 * Output Register ready when MSB and LSB are ready.
 * +/- 4G of range
 * High Resolution mode
 */
#define CTRL_REG4_VALUE 0b10010001

/**
 * Always convert to 10 bits.
 * TODO: Review this
 */
#define LIS3DH_LSB16_TO_KILO_LSB10 16000

class Accelerometer: public Sensor<acceleration_t> {
private:
    TwoWire& i2c;
    uint8_t address;
    uint8_t dataReadyPin;

    using Register = enum: uint8_t {
        CTRL_REG1 = 0x20,
        CTRL_REG4 = 0x23,
        OUT_X_L = 0x28
    };

    bool writeRegister(Register reg, uint8_t byte) {
        i2c.beginTransmission(address);
        i2c.write(byte);
        return i2c.endTransmission() == 0;
    }

    bool isInitialized = false;

    bool init() {
        if (!isInitialized) {
            if (!writeRegister(CTRL_REG1, CTRL_REG1_VALUE)) {
                return false;
            }

            if (!writeRegister(CTRL_REG4, CTRL_REG4_VALUE)) {
                return false;
            }

            isInitialized = true;
        }

        return isInitialized;
    }

    bool readAcceleration(acceleration_t& acceleration) {
        // 2 bytes for X, Y and Z accelerations
        uint8_t buffer[6];

        i2c.beginTransmission(address);

        // Autoincrement address to Y and then Z (8)
        // Enable multiple byte reception (1)
        i2c.write(OUT_X_L | 0x81);

        i2c.readBytes(buffer, 6);
        i2c.endTransmission();

        int16_t x, y, z;

        x = buffer[0];
        x |= ((uint16_t ) buffer[1]) << 8;

        y = buffer[2];
        y |= ((uint16_t ) buffer[3]) << 8;

        z = buffer[4];
        z |= ((uint16_t ) buffer[5]) << 8;

        // Due to range of 4G
        uint8_t lsbValue = 8;

        acceleration.x = lsbValue * ((float) x / LIS3DH_LSB16_TO_KILO_LSB10);
        acceleration.y = lsbValue * ((float) y / LIS3DH_LSB16_TO_KILO_LSB10);
        acceleration.z = lsbValue * ((float) z / LIS3DH_LSB16_TO_KILO_LSB10);
    }

public:
    explicit Accelerometer(TwoWire& i2c, uint8_t address, uint8_t dataReadyPin): i2c(i2c), address(address), dataReadyPin(dataReadyPin) {
        pinMode(dataReadyPin, INPUT);
        i2c.begin();
    };

    void collect() override {
        if (!init()) {
            return;
        }

        DebugPrint("Collecting Accelerometer data\n");
        acceleration_t acceleration;

        if (!readAcceleration(acceleration)) {
            return;
        }

        DebugPrint("Collected Accelerometer data, %f %f %f\n", acceleration.x, acceleration.y, acceleration.z);
        notify(acceleration);
    }
};
