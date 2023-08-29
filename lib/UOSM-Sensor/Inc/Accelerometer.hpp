//
// Created by etara on 2023-08-28.
//
#include "Sensor.hpp"

class Accelerometer: public Sensor{
private:
    float vals[3];
public:

    Accelerometer(): Sensor(){


    };
    void AccelData(uint8_t buffer){
        try {

        }
        catch{
            printf("Error in saving data");
        }
    }
};