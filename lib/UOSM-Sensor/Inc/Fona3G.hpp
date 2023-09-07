//
// Created by Jeremy Cote on 2023-09-06.
//

#ifndef UOSM_TELEMETRY_FONA3G_HPP
#define UOSM_TELEMETRY_FONA3G_HPP

#include <Arduino.h>
#include "Adafruit_FONA.h"
#include "exception"
#include "ApplicationTypes.h"

#define FONA_NETWORK_HOME 1
#define FONA_NETWORK_ROAMING 5

class Fona3G {
private:
    static bool inUse;
    Adafruit_FONA_3G fona;

    static void lock() {
        inUse = true;
    }

    static void unlock() {
        inUse = false;
    }

public:
    /**
     * Construct a Fona3G instance.
     * Throws a std::runtime_error if it fails to initialize the FONA.
     * The pointer to serial must be started or else a std::runtime_error is thrown.
     * @param serial Reference to HardwareSerial used to communicate with FONA.
     * @param resetPin connect to the Reset Pin of the FONA
     */
    Fona3G(HardwareSerial* serial, int resetPin) noexcept(false): fona(resetPin) {
        if (!fona.begin(*serial)) {
            throw std::runtime_error("Could not find FONA");
        }

        if (!fona.enableGPS(true)) {
            throw std::runtime_error("Could not enable GPS on FONA");
        }

        if (!fona.enableGPRS(true)) {
            throw std::runtime_error("Could not enable GPRS on FONA");
        }
    }

    /**
     * Await internal mutex to make sure FONA 3G is not in use.
     * @param retryInterval How long to wait using vTaskDelay between retries
     * @param maxRetries How many times to retry before returning false
     * @return true if the FONA 3G is available.
     */
    static bool awaitAvailability(uint16_t retryInterval = 200, uint8_t maxRetries = 3) {
        uint8_t attempts = 0;

        while (inUse && attempts < maxRetries) {
            vTaskDelay(retryInterval);
            attempts--;
        }

        return !inUse;
    }

    /**
     * Get network status of the FONA.
     * Throws a std::runtime_error is the FONA is in use.
     * @return true if the FONA 3G is connected to the cellular network.
     */
    bool getNetworkConnected() noexcept(false) {
        if (inUse) {
            throw std::runtime_error("Fona is busy");
        }

        // Lock the FONA before using it
        lock();

        uint8_t status = fona.getNetworkStatus();

        // Unlock it after use
        unlock();

        return status == FONA_NETWORK_HOME || status == FONA_NETWORK_ROAMING;
    }

    gps_coordinate_t getGPSCoordinate() noexcept(false) {
        if (inUse) {
            throw std::runtime_error("Fona is busy");
        }

        gps_coordinate_t coordinate;

        lock();
        if (!fona.getGPS(&coordinate.latitude, &coordinate.longitude, &coordinate.speed_kmh, &coordinate.heading, &coordinate.altitude)) {
            // If fona failed to get GPS coordinates, make sure to unlock the mutex.
            unlock();
            throw std::runtime_error("Failed to get GPS coordinates from FONA");
        }

        unlock();

        return coordinate;
    }
};

#endif //UOSM_TELEMETRY_FONA3G_HPP
