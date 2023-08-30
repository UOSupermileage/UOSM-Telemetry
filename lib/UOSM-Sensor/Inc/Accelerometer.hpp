//
// Created by etara on 2023-08-28.
//
#include "Sensor.hpp"
#include "ApplicationTypes.h"
#include "../Adafruit_LIS3DH/Adafruit_LIS3DH.h"
#include "../Adafruit_LIS3DH/Adafruit_LIS3DH.cpp"
#include "../Adafruit_BusIO/Adafruit_BusIO_Register.h"
#include "../Adafruit_Sensor/Adafruit_Sensor.h"


#define MISO 37
#define MOSI 35
#define SCLK 36
#define CS 1
#define G LIS3DH_DATA





class Accelerometer: public Sensor<accel_t>{

public:

    Adafruit_LIS3DH handle;
     Accelerometer(uint8_t buffer_size): Sensor<accel_t>(buffer_size) {
        handle = Adafruit_LIS3DH(CS,MOSI, MISO, SCLK);
        setup(handle);
    };
    void setup(Adafruit_LIS3DH a_handle){
        try{
            if (! a_handle.begin(0x18)){
                throw("Couldn't start the accelerometer.");
            }
            a_handle.setDataRate(LIS3DH_DATARATE_10_HZ);
            a_handle.setRange(LIS3DH_RANGE_2_G);
            printf("Initialization Successful!");
        }

        catch(error_t){
            printf("Error has occured.");
        }
    }



    void collect() override {
        accel_t acceleration;
        handle.read();
        sensors_event_t an_event;
        handle.getEvent(&an_event);
        acceleration.accel_x = an_event.acceleration.x;
        acceleration.accel_y = an_event.acceleration.y;
        acceleration.accel_z = an_event.acceleration.z;
        acceleration.x = handle.x;
        acceleration.y = handle.y;
        acceleration.z = handle.z;
        //add(acceleration);

    }

};
