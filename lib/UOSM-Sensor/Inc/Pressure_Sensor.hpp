//
// Created by Mohamad on 2023-09-05.
//
#include "Sensor.hpp"
#include "ApplicationTypes.h"
#include "MS5525DSO.h"
#include "SPI.h"
#include "Wire.h"
#include <Arduino.h>



class PressureSensor:public Sensor<pressure_t>{

    public: 
    
    MS5525DSO sensor_pres = MS5525DSO(pp005GS); //Need to look into the input pp005GS

    explicit PressureSensor(uint8_t buffer_size):Sensor<pressure_t>(buffer_size){
        
        Wire.begin();

        if (!sensor_pres.begin()) {
        Serial.println("failed to initialize pressure sensor!");
        while (true) delay(100);
        }

        sensor_pres.dumpCoefficients(Serial);
    }

    void collect() override{

        pressure_t struct_pressure;
        sensor_pres.readPressureAndTemperature(&struct_pressure.pressure, &struct_pressure.temp);
        add(struct_pressure);
    }




};