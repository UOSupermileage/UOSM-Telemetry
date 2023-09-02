#include "VoltageSensor.hpp"
#include "Accelerometer.hpp"
#include "AIotvarcollect.h" 
#ifdef ESP32

#include <Arduino.h>
#include "PollingSensorTask.hpp"

#define DEFAULT_BUFFER_SIZE 10

VoltageSensor* voltageSensor;
Accelerometer* accelSensor;

PollingSensorTask<voltage_t>* voltageSensorTask;
PollingSensorTask<accel_t>* accelSensorTask;

void setup() {
    Serial.begin(115200); 
    Serial.printf("Minimal stack size %d\n", configMINIMAL_STACK_SIZE);
    //voltageSensor = new VoltageSensor(DEFAULT_BUFFER_SIZE);
    accelSensor = new Accelerometer(DEFAULT_BUFFER_SIZE);
    // accelSensor->addListener([](const accel_t& newValue){ 
        accelSensor->addListener([](const accel_t& newValue){ 
            updateAcceleration(newValue); 
        }); 
    // }) 
    // TODO: Note that voltageSensor will throw an exception if collect is not called before get(). See if we can apply RAII

    //voltageSensorTask = new PollingSensorTask<voltage_t>(voltageSensor, 200, "T_VoltageSensor", 12800 * 100, 5);
    accelSensorTask = new PollingSensorTask<accel_t>(accelSensor,200,"T_AccelSensor",1280*100,5); 
    
    initProperties(); 
    ArduinoCloud.begin(AIoTPreferredConnection);  

}

void loop() {
    Serial.print("Main Loop...\n");
    vTaskDelay(100);
    //Serial.printf("Value %d\n", voltageSensor->get());
    // accelSensor->collect(); 
    // Serial.printf("Value Accelerometer X: %f\n", accelSensor->get().accel_x); 
    // Serial.printf("Value Accelerometer Y: %f\n", accelSensor->get().accel_y); 
    // Serial.printf("Value Accelerometer Z: %f\n", accelSensor->get().accel_z);  
    ArduinoCloud.update(); 
    
}

#else
#ifndef PIO_UNIT_TESTING
int main() {
    printf("Starting");
    VoltageSensor sensor(10);
    Accelerometer sens_accel(10);
    sensor.collect();
    sens_accel.collect(10);
    printf("Get Value %d", sensor.get());
    return 0;
}
#endif
#endif
