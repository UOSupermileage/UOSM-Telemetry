//
// Created by Mohamad on 2023-09-05.
//
#include "Sensor.hpp"
#include "ApplicationTypes.h"
#include "MS5525DSO.h"
#include "SPI.h"
#include "Wire.h"
#include <Arduino.h>
#include "exception"



class PressureSensor:public Sensor<pressure_t>{

    public: 
    
    MS5525DSO sensor_pres = MS5525DSO(pp005GS); //Need to look into the input pp005GS

    explicit PressureSensor(uint8_t buffer_size):Sensor<pressure_t>(buffer_size){
        
        Wire.begin();

        if (!sensor_pres.begin()) {
        throw std::runtime_error("Failed to initialize pressure sensor!");
        }

        sensor_pres.dumpCoefficients(Serial);
    }

    void collect() override{

        pressure_t struct_pressure;
        sensor_pres.readPressureAndTemperature(&struct_pressure.pressure, &struct_pressure.temp);
        add(struct_pressure);
    }




};