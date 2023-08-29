#include "VoltageSensor.hpp"

#ifdef ESP32

#include <Arduino.h>
#include "PollingSensorTask.hpp"

#define DEFAULT_BUFFER_SIZE 10

VoltageSensor* voltageSensor;

PollingSensorTask<voltage_t>* voltageSensorTask;

void setup() {
    Serial.begin(115200);
    Serial.printf("Minimal stack size %d\n", configMINIMAL_STACK_SIZE);
    voltageSensor = new VoltageSensor(DEFAULT_BUFFER_SIZE);

    // TODO: Note that voltageSensor will throw an exception if collect is not called before get(). See if we can apply RAII

    voltageSensorTask = new PollingSensorTask<voltage_t>(voltageSensor, 200, "T_VoltageSensor", 128 * 10, 5);
}

void loop() {
    Serial.print("Main Loop...\n");
    vTaskDelay(500);
    Serial.printf("Value %d\n", voltageSensor->get());
}

#else
#ifndef PIO_UNIT_TESTING
int main() {
    printf("Starting");
    VoltageSensor sensor(10);
    sensor.collect();
    printf("Get Value %d", sensor.get());
    return 0;
}
#endif
#endif
