//
// Created by etara on 2023-08-28.
//
#include "Sensor.hpp"
#include "ApplicationTypes.h"
#include "../Adafruit_LIS3DH/Adafruit_LIS3DH.h"
#include "../Adafruit_LIS3DH/Adafruit_LIS3DH.cpp"
#include "../Adafruit_BusIO/Adafruit_BusIO_Register.h"
#include "../Adafruit_Sensor/Adafruit_Sensor.h"
#include "../SparkFun_LIS3DH_Arduino_Library-master/SparkFun_LIS3DH_Arduino_Library-master/src_files/SparkFunLIS3DH.h"
#include "../SparkFun_LIS3DH_Arduino_Library-master/SparkFun_LIS3DH_Arduino_Library-master/src_files/SparkFunLIS3DH.cpp" 
#include "SPI.h"


#define MISO 19
#define MOSI 23
#define SCLK 18
#define CS 5
#define G 2
#define SAMPLE_RATE 400





class Accelerometer: public Sensor<accel_t>{

public:

    accel_t acceleration; 
    LIS3DH handle = LIS3DH(SPI_MODE,CS);    
    // byte WriteData = 0; 
    float readDatax, readDatay, readDataz = 0; 
    explicit Accelerometer(uint8_t buffer_size): Sensor<accel_t>(buffer_size) {
        
        handle.begin(); 
        handle.settings.accelRange = G; 
        handle.settings.accelSampleRate = SAMPLE_RATE; 
        
      //  handle.readRegister(&readData,LIS3DH_WHO_AM_I); 
        
    }; 
   /* void setup(Adafruit_LIS3DH* _a){
        try{
            if (! _a ->begin()){ 
                throw("Couldn't start the accelerometer.");
            }
            _a->setDataRate(LIS3DH_DATARATE_10_HZ);
            _a->setRange(LIS3DH_RANGE_2_G);
            Serial.println("Initialization Successful!");
        }

        catch(error_t){
            printf("Error has occured.");
        }
    }*/



    void collect() override {
        Serial.printf("Bonjour");
        readDatax = handle.readFloatAccelX(); 
        readDatay = handle.readFloatAccelY(); 
        readDataz = handle.readFloatAccelZ(); 
        acceleration.accel_x = readDatax*9.81; 
        acceleration.accel_y = readDatay*9.81; 
        acceleration.accel_z = readDataz*9.81; 

        /*handle.read();
        handle.getEvent(&an_event);
        acceleration.accel_x = an_event.acceleration.x;
        acceleration.accel_y = an_event.acceleration.y;
        acceleration.accel_z = an_event.acceleration.z;
        acceleration.x = handle.x;
        acceleration.y = handle.y;
        acceleration.z = handle.z;*/
        add(acceleration); 

    }

};