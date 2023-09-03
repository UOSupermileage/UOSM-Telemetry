//
// Created by etara on 2023-08-28.
//
#include "Sensor.hpp"
#include "ApplicationTypes.h"
#include "../SparkFun_LIS3DH_Arduino_Library-master/SparkFun_LIS3DH_Arduino_Library-master/src_files/SparkFunLIS3DH.h"
#include "../SparkFun_LIS3DH_Arduino_Library-master/SparkFun_LIS3DH_Arduino_Library-master/src_files/SparkFunLIS3DH.cpp" 
#include "SPI.h"


#define MISO 19
#define MOSI 23
#define SCLK 18
#define CS 5
#define G 2
#define SAMPLE_RATE 400
#define Address 0x18





class Accelerometer: public Sensor<accel_t>{

public:

    accel_t acceleration; 
    LIS3DH handle = LIS3DH(SPI_MODE,CS);    
   
    float readDatax, readDatay, readDataz = 0; 
    explicit Accelerometer(uint8_t buffer_size): Sensor<accel_t>(buffer_size) {
        
        handle.begin(); 
        handle.settings.accelRange = G; 
        handle.settings.accelSampleRate = SAMPLE_RATE; 
        
     
        
    }; 
   



    void collect() override {
        Serial.printf("Bonjour");
        readDatax = handle.readFloatAccelX(); 
        readDatay = handle.readFloatAccelY(); 
        readDataz = handle.readFloatAccelZ(); 
        acceleration.accel_x = readDatax*9.81; 
        acceleration.accel_y = readDatay*9.81; 
        acceleration.accel_z = readDataz*9.81; 

       
        add(acceleration); 

    }

};
