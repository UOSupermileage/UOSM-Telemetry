//
// Created by Jeremy Cote on 2023-08-27.
//

#include <unity.h>
#include "Sensor.hpp"

class MockSensor : public Sensor<uint8_t> {
private:
    uint8_t nextMockValue;

public:
    explicit MockSensor(uint8_t bufferSize) : Sensor(bufferSize) {

    }

    ~MockSensor() {

    }

    void collect() override {
        addValue(nextMockValue);
    }

    void setNextMockValue(uint8_t value) {
        nextMockValue = value;
    }
};

void test_voltage_sensor_value_collection() {
    // Create a mock sensor with a buffer size of 3
    MockSensor sensor(2);

    // Set next value that will be collected
    sensor.setNextMockValue(5);

    // Collect a value
    sensor.collect();

    // Check if the latest value is correct
    TEST_ASSERT_EQUAL_INT(5, sensor.getValue());
}

void test_voltage_sensor_latest_listener() {
    // Create a mock sensor with a buffer size of 3
    MockSensor sensor(2);

    // Create a callback function
    std::function<void(const uint8_t &)> callback = [](const uint8_t& newValue) {
        // Test the callback logic here
        TEST_ASSERT_EQUAL_INT(3, newValue);
    };

    // Set 3 as the next value to be collected
    sensor.setNextMockValue(3);

    // Add a listener for the ObservedDataQueue
//    ObserverToken token = sensor.addListenerForLatest(callback);

    // Add a value to the sensor
    sensor.collect();

    // Unregister the listener
//    token.cancel();
}

int main() {
    UNITY_BEGIN();

    RUN_TEST(test_voltage_sensor_value_collection);
    RUN_TEST(test_voltage_sensor_latest_listener);

    return UNITY_END();
}

//void loop() {}
