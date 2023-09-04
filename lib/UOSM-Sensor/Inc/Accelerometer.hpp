//
// Created by etara on 2023-08-28.
//
#include "Sensor.hpp"
#include "ApplicationTypes.h"
#include "SparkFunLIS3DH.h"
#include "SPI.h"


#define MISO 19
#define MOSI 23
#define SCLK 18
#define CS 5
#define G_RANGE 2
#define G 9.81
#define SAMPLE_RATE 400





class Accelerometer: public Sensor<accel_t>{

public:

    accel_t acceleration; 
    LIS3DH handle = LIS3DH(SPI_MODE,CS);    

    float readDatax, readDatay, readDataz = 0; 
    explicit Accelerometer(uint8_t buffer_size): Sensor<accel_t>(buffer_size) {
        
        handle.begin(); 
        handle.settings.accelRange = G_RANGE;
        handle.settings.accelSampleRate = SAMPLE_RATE; 
        

        
    }; 




    void collect() override {
        Serial.printf("Bonjour");
        readDatax = handle.readFloatAccelX(); 
        readDatay = handle.readFloatAccelY(); 
        readDataz = handle.readFloatAccelZ(); 
        acceleration.accel_x = readDatax*G; 
        acceleration.accel_y = readDatay*G; 
        acceleration.accel_z = readDataz*G; 


        add(acceleration); 

    }

};
