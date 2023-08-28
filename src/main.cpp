#include "VoltageSensor.hpp"

#ifdef ESP32

#include "../lib/UOSM-Sensor/Inc/VoltageSensor.hpp"
#include "PollingSensorTask.hpp"

#define DEFAULT_BUFFER_SIZE 10

VoltageSensor* voltageSensor;

PollingSensorTask<voltage_t>* voltageSensorTask;

void setup() {
    voltageSensor = new VoltageSensor(DEFAULT_BUFFER_SIZE);
    voltageSensorTask = new PollingSensorTask<voltage_t>(voltageSensor, 200, "VoltageSensorTask", 128 * 4, 5);
}

void loop() {
    Serial.print("Main Loop...");
    vTaskDelay(500);
}

#else
#ifndef PIO_UNIT_TESTING
int main() {
    printf("Starting");
    VoltageSensor sensor(10);
}
#endif
#endif
