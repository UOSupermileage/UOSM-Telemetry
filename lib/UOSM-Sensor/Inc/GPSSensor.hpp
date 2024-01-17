//
// Created by Jeremy Cote on 2023-09-06.
//

#ifndef UOSM_TELEMETRY_GPSSENSOR_HPP
#define UOSM_TELEMETRY_GPSSENSOR_HPP

#include "Sensor.hpp"
#include <exception>

#include <Arduino_PortentaBreakout.h>

#define TINY_GSM_MODEM_SIM7600
#include <TinyGsmClient.h>


class GPSSensor: public Sensor<gps_coordinate_t> {
private:
    TinyGsm modem;

    float lat      = 0;
    float lon      = 0;
    float speed    = 0;
    float alt      = 0;
    int   vsat     = 0;
    int   usat     = 0;
    float accuracy = 0;
    int   year     = 0;
    int   month    = 0;
    int   day      = 0;
    int   hour     = 0;
    int   min      = 0;
    int   sec      = 0;

public:
    GPSSensor(HardwareSerial& serial, uint8_t bufferSize): Sensor<gps_coordinate_t>(bufferSize), modem(serial)  {
        DebugPrint("Creating GPS Sensor");
        modem.enableGPS();
    }

    void collect() override {
        DebugPrint("Requesting current GPS location");
        if (modem.getGPS(&lat, &lon, &speed, &alt, &vsat, &usat, &accuracy,
                         &year, &month, &day, &hour, &min, &sec)) {
            DebugPrint("Latitude: %f, Longitude: %f", lat, lon);
            DebugPrint("Speed: %f, Altitude: %f", speed, alt);
            DebugPrint("Visible Satellites: %i, Used Satellites: %i", vsat, usat);
            DebugPrint("Accuracy: %f", accuracy);

            gps_coordinate_t coords;

            coords.altitude = alt;
            coords.latitude = lat;
            coords.longitude = lon;
            coords.speed_kmh = speed;

            add(coords);
            return;
        }

        DebugPrint("Couldn't get GPS location.");
    };
};

#endif //UOSM_TELEMETRY_GPSSENSOR_HPP
