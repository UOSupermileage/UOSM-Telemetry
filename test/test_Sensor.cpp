//
// Created by Jeremy Cote on 2023-08-27.
//

#include <gtest/gtest.h>
#include "Sensor.hpp"

class MockSensor : public Sensor<uint8_t> {
private:
    uint8_t nextMockValue = 0;

public:
    explicit MockSensor(uint8_t bufferSize) : Sensor(bufferSize) {

    }

    ~MockSensor() {

    }

    void collect() override {
        Sensor::add(nextMockValue);
    }

    void setNextMockValue(uint8_t value) {
        nextMockValue = value;
    }
};

TEST(SensorTests, CollectData) {
    MockSensor sensor(5);

    sensor.setNextMockValue(10);
    sensor.collect();

    EXPECT_EQ(sensor.get(), 10);

    sensor.setNextMockValue(20);
    sensor.collect();

    EXPECT_EQ(sensor.get(), 20);
}

TEST(SensorTests, Listener) {
    MockSensor sensor(5);
    std::function<void(const uint8_t &)> callback = [](const uint8_t& newValue) {
        // Test the callback logic here
        EXPECT_EQ(newValue, 3);
    };

    sensor.setNextMockValue(3);

    ObserverToken token = sensor.addListener(callback);

    sensor.collect();

    token.cancel();
}

#if defined(ARDUINO)
#include <Arduino.h>

void setup()
{
    // should be the same value as for the `test_speed` option in "platformio.ini"
    // default value is test_speed=115200
    Serial.begin(115200);

    ::testing::InitGoogleTest();
    // if you plan to use GMock, replace the line above with
    // ::testing::InitGoogleMock();
}

void loop()
{
    // Run tests
    if (RUN_ALL_TESTS()) {}

    // sleep for 1 sec
    delay(1000);
}

#else
int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    // if you plan to use GMock, replace the line above with
    // ::testing::InitGoogleMock(&argc, argv);

    if (RUN_ALL_TESTS()) {}

    // Always return zero-code and allow PlatformIO to parse results
    return 0;
}
#endif