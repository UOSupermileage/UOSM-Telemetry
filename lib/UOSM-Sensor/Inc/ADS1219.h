#ifndef ADS1219_H
#define ADS1219_H

#include <Arduino.h>
#include <functional>

const uint8_t ads1219_default_address = 0x40;

/**
    Input multiplexer configuration : Configuration Register Bits 7:5
    These bits configure the input multiplexer.
    000 : AINP = AIN0, AINN = AIN1 (default)
    001 : AINP = AIN2, AINN = AIN3
    010 : AINP = AIN1, AINN = AIN2
    011 : AINP = AIN0, AINN = AGND
    100 : AINP = AIN1, AINN = AGND
    101 : AINP = AIN2, AINN = AGND
    110 : AINP = AIN3, AINN = AGND
    111 : AINP and AINN shorted to AVDD / 2
 */
using ADS1219_MUX = enum: uint8_t {
    DIFF_P0_N1 = 0,
    DIFF_P2_N3,
    DIFF_P1_N2,
    SINGLE_0,
    SINGLE_1,
    SINGLE_2,
    SINGLE_3,
    SHORTED
};

// Gain configuration : Configuration Register Bit 4
using ADS1219_GAIN = enum: uint8_t {
    GAIN_1 = 0,
    GAIN_4
};

// Data rate configuration : Configuration Register Bits 3:2
using ADS1219_DATA_RATE = enum: uint8_t {
    SPS_20 = 0,
    SPS_90,
    SPS_330,
    SPS_1000
};

// Conversion mode configuration : Configuration Register Bit 1
using ADS1219_CONVERSION_MODE = enum: uint8_t {
    SINGLE_SHOT = 0,
    CONTINUOUS
};

// Voltage reference configuration : Configuration Register Bit 0
// Internal: internal 2.048V reference
// External: external REFP and REFN
using ADS1219_VREF = enum: uint8_t {
    INTERNAL = 0,
    EXTERNAL
};

const uint8_t ads1219_reg_config_write = 0x40;
const uint8_t ads1219_reg_config_read = 0x20;
const uint8_t ads1219_reg_status_read = 0x24;

const uint8_t ads1219_command_reset = 0x06;
const uint8_t ads1219_command_start = 0x08;
const uint8_t ads1219_command_shutdown = 0x02;
const uint8_t ads1219_command_read = 0x10;

// A union is used here so that individual values from the register can be
// accessed or the whole register can be accessed.
typedef union
{
    struct
    {
        uint8_t vref : 1; // Voltage reference configuration : Configuration Register Bit 0
        uint8_t cm : 1;   // Conversion mode configuration : Configuration Register Bit 1
        uint8_t dr : 2;   // Data rate configuration : Configuration Register Bits 3:2
        uint8_t gain : 1; // Gain configuration : Configuration Register Bit 4
        uint8_t mux : 3;  // Input multiplexer configuration : Configuration Register Bits 7:5
    };
    uint8_t byte;
} ads1219_config_t;

typedef union
{
    struct
    {
        uint8_t id : 7;   // Device ID = 0x60 - but datasheet says "Reserved. Values are subject to change without notice"
        uint8_t drdy : 1; // Conversion result ready flag : Status Register Bit 7
    };
    uint8_t byte;
} ads1219_status_t;

class ADS1219 {
private:
    ADS1219_GAIN gain;
    int32_t result;

    TwoWire& i2c;
    uint8_t data_ready;

    uint8_t address;
public:
    /**
     * Construct an ADS1219 Sensor
     * @param i2c Wire instance to use
     * @param data_ready GPIO Pin for hardware data_ready
     */
    explicit ADS1219(TwoWire& i2c, uint8_t data_ready, uint8_t address, ADS1219_GAIN gain = GAIN_1): gain(gain), result(0), i2c(i2c), data_ready(data_ready), address(address) {
        pinMode(data_ready, INPUT);
    }

    bool begin() {
        if (!reset()) {
            printf("Failed to reset ADS\n");
            return false;
        }

        osDelay(50);

        ads1219_config_t config;
        bool r = read_register(ads1219_reg_config_read, config.byte);
        if (r && config.byte == 0) {

            set_gain(gain);

            return true;
        }

        printf("Failed to configure ADS\n");
        return false;
    }

    bool reset() {
        return write_byte(ads1219_command_reset);
    }

    bool start() {
        return write_byte(ads1219_command_start);
    }

    bool shutdown() {
        return write_byte(ads1219_command_shutdown);
    }

    bool set_conversion_mode(const ADS1219_CONVERSION_MODE mode) {
        return patch_register([&mode](ads1219_config_t& config) {
            config.cm = mode;
        });
    }

    bool set_input_mux(const ADS1219_MUX mux) {
        return patch_register([&mux](ads1219_config_t& config) {
            config.mux = mux;
        });
    }

    bool set_data_rate(const ADS1219_DATA_RATE rate) {
        return patch_register([&rate](ads1219_config_t& config) {
            config.dr = rate;
        });
    }

    bool set_gain(const ADS1219_GAIN gain) {
        return patch_register([&gain](ads1219_config_t& config) {
            config.gain = gain;
        });
    }

    bool set_voltage_ref(const ADS1219_VREF vref) {
        return patch_register([&vref](ads1219_config_t& config) {
            config.vref = vref;
        });
    }

    bool read_conversion() {
        auto* raw_bytes = new uint8_t [3];
        if (!read_register_region(ads1219_command_read, raw_bytes, 3)) {
            delete[] raw_bytes;
            return false;
        }

        // Data is 3-bytes (24-bits), big-endian (MSB first).
        // Use a union to avoid signed / unsigned ambiguity
        union
        {
            int32_t i32;
            uint32_t u32;
        } iu32;

        iu32.u32 = raw_bytes[0];
        iu32.u32 = (iu32.u32 << 8) | raw_bytes[1];
        iu32.u32 = (iu32.u32 << 8) | raw_bytes[2];

        // Preserve the 2's complement.
        if (0x00800000 == (iu32.u32 & 0x00800000)) {
            iu32.u32 = iu32.u32 | 0xFF000000;
        }
        result = iu32.i32; // Store the signed result
        return true;
    }

    float get_millivolts(float vref_millivolts) {
        float mV = result;            // Convert int32_t to float
        mV /= 8388608.0;                  // Convert to a fraction of full-scale (2^23)
        mV *= vref_millivolts; // Convert to millivolts
        if (gain == GAIN_4)
            mV /= 4.0; // Correct for the gain
        return mV;
    }

    int32_t get_raw() {
        return result;
    }

    bool is_data_ready() {
        return digitalRead(data_ready) == LOW;
    }

private:
    bool write_byte(uint8_t byte, bool stop = true) {
        i2c.beginTransmission(address);
        i2c.write(byte);
        return i2c.endTransmission(stop) == 0;
    }

    bool write_register(uint8_t reg, uint8_t byte) {
        i2c.beginTransmission(address);
        i2c.write(reg);
        i2c.write(byte);
        return i2c.endTransmission() == 0;
    }

    bool read_register(uint8_t reg, uint8_t& data) {
        write_byte(reg, false);

        i2c.requestFrom(address, (uint8_t) 1);

        if (i2c.available() != 0) {
            data = i2c.read();
        } else {
            printf("Read_register failed, did not receive\n");
            return false;
        }

        return true;
    }

    bool patch_register(const std::function<void(ads1219_config_t&)>& patch) {
        ads1219_config_t config;
        if (!read_register(ads1219_reg_config_read, config.byte)) {
            printf("Failed to read config\n");
            return false;
        }

        patch(config);

        return write_register(ads1219_reg_config_write, config.byte);
    }

    bool read_register_region(uint8_t reg, uint8_t* data, size_t n) {
        if (data == nullptr) {
            return false;
        }

        write_byte(reg, false);

        size_t num_read_bytes = i2c.requestFrom(address, n, true);

        if (num_read_bytes != n) {
            return false;
        }

        for (uint8_t i = 0; i < num_read_bytes; i++) {
            *(data + i) = i2c.read();
        }

        return true;
    }
};

#endif