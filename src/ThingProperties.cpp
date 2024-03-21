//
// Created by Jeremy Cote on 2024-01-16.
//

#include <map>
#include "ThingProperties.hpp"

CloudDatabase CloudDatabase::instance;

// map of air density values at different temperatures (key, value) = (temperature, air density)
#define MAP_INTERVAL 5
const std::map<int, float> airDensityMap = {
        {-25, 1.4224},
        {-20, 1.3943},
        {-15, 1.3673},
        {-10, 1.3413},
        {-5, 1.3163},
        {0, 1.2922},
        {5, 1.2690},
        {10, 1.2466},
        {15, 1.2250},
        {20, 1.2041},
        {25, 1.1839},
        {30, 1.1644},
        {35, 1.1455},
        {40, 1.127},
        {45, 1.110},
        {50, 1.093},
        {55, 1.076},
        {60, 1.061}
};

void CloudDatabase::updatePressure(float pressure, float temperature) {
    this->pressure = pressure;
    this->temperature = temperature;

    // round air temperature to nearest 5
    int32_t roundedAirTemperature = (int) (MAP_INTERVAL * round(temperature / MAP_INTERVAL));

    try {
        float airDensity = airDensityMap.at(roundedAirTemperature);
        airSpeed = sqrt((2 * pressure) / airDensity);
    } catch (std::out_of_range& e) {
        return;
    }
}