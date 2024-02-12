#include <Arduino.h>

#include "ApplicationTypes.h"
#include "ThingProperties.hpp"
#include "TinyGSMConnectionHandler.hpp"

#include "ValueSensor.hpp"
#include "CANLogEntry.hpp"
#include "CANTask.hpp"
#include "CANDriver.h"
#include "PollingSensorTask.hpp"
#include "Config.h"
#include "LoggerTask.hpp"

constexpr uint32_t serialBaudrate = 115200;
constexpr uint8_t defaultBufferSize = 1;

#define SENSOR_GPS 0
#define SENSOR_VOLTAGE 0
#define SENSOR_CURRENT 0
#define SENSOR_ACCELEROMETER 0
#define SENSOR_PRESSURE 0
#define SENSOR_CAN_LOG 0
#define SENSOR_THROTTLE 0
#define SENSOR_SPEEDOMETER 0
#define SENSOR_RPM 0

#define LOGGER_SD 0
#define LOGGER_IOT 1

//enum class InternetConnection: uint8_t {
//    disabled = 0,
//    wifi = 1,
//    cellular = 2
//};

#define INTERNET_CONNECTION 2

//constexpr InternetConnection connection = InternetConnection::cellular;

#if INTERNET_CONNECTION == 1
WiFiConnectionHandler ArduinoIoTPreferredConnection(SSID, PASS);
#elif INTERNET_CONNECTION == 2
TinyGSMConnectionHandler ArduinoIoTPreferredConnection(Serial1, "", "LTEMOBILE.APN", "", "");
#endif

#if SENSOR_GPS == 1
#include "GPSSensor.hpp"

#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm        modem(debugger);
#else
TinyGsm*        modem;
#endif

GPSSensor* gpsSensor;
PollingSensorTask<gps_coordinate_t>* gpsSensorTask;
#endif

#if SENSOR_VOLTAGE == 1
#include "VoltageSensor.hpp"

VoltageSensor* voltageSensor;
PollingSensorTask<voltage_t>* voltageSensorTask;
#endif

#if SENSOR_CURRENT == 1
#include "VoltageSensor.hpp"

// The current sensor is just an ADS on the current sensor so it's technically a voltage reading that's extrapolated back into current output
VoltageSensor* currentSensor;
PollingSensorTask<voltage_t>* currentSensorTask;
#endif

#if SENSOR_ACCELEROMETER == 1
#include "Accelerometer.hpp"

Accelerometer* accelerationSensor;
PollingSensorTask<acceleration_t>* accelerationSensorTask;
#endif

#if SENSOR_PRESSURE == 1
#include "PressureSensor.hpp"
#include "Conversions.h"

ValueSensor<pressure_t>* pressureSensor = new ValueSensor<pressure_t>(defaultBufferSize);
#endif

// CAN Values
#if SENSOR_CAN_LOG == 1
ValueSensor<CANLogEntry*>* canLogsSensor = new ValueSensor<CANLogEntry*>(defaultBufferSize);
#endif

#if SENSOR_THROTTLE == 1
ValueSensor<percentage_t>* throttleSensor = new ValueSensor<percentage_t>(defaultBufferSize);
#endif

#if SENSOR_SPEEDOMETER == 1
ValueSensor<speed_t>* speedSensor = new ValueSensor<speed_t>(defaultBufferSize);
#endif

#if SENSOR_RPM == 1
ValueSensor<velocity_t>* rpmSensor = new ValueSensor<velocity_t >(defaultBufferSize);
#endif

void setup() {
    Serial.begin(serialBaudrate);

    printf("Hello");
    DebugPrint("Initializing Telemetry System...");
    Serial.print("HELLO");

#ifndef UOSM_SECRETS
    DebugPrint("Failed to find secrets... Make sure to create a Secrets.h file. Aborting!");
    while (true) {}
#endif

    // TODO: Note that sensors will throw an exception if collect is not called before get(). See if we can apply RAII

#if SENSOR_VOLTAGE == 1
    voltageSensor = new VoltageSensor(VoltageSensor::VoltageSensorMode::Differential_0_1, 15, 14, 0x40, defaultBufferSize);
    voltageSensor->addListener([](const voltage_t& newValue) {
        CloudDatabase::instance.updateBatteryVoltage(convertVoltageToFloat(newValue));
    });

    voltageSensorTask = new PollingSensorTask<voltage_t>(voltageSensor, 200, "T_VoltageSensor", 1024 * 10, static_cast<osPriority_t>(5));
#endif

#if SENSOR_CURRENT == 1
    currentSensor = new VoltageSensor(VoltageSensor::VoltageSensorMode::Differential_2_3, 15, 14, 0x40, defaultBufferSize);
    currentSensor->addListener([](const voltage_t& newValue) {

        // TODO Convert voltage into a current reading
       CloudDatabase::instance.updateBatteryCurrent(newValue * 2);
    });
#endif

#if SENSOR_ACCELEROMETER == 1
    accelerationSensor = new Accelerometer(defaultBufferSize);

    DebugPrint("Created accelerometer\n");

    accelerationSensor->addListener([](const acceleration_t & newValue){
        DebugPrint("Received new acceleration: %f %f %f\n", newValue.x, newValue.y, newValue.z);
        CloudDatabase::instance.updateAcceleration(newValue);
    });

    DebugPrint("Added listener\n");

    accelerationSensor->collect();

    DebugPrint("Forced collection\n");

    DebugPrint("Creating PollingSensorTask\n");
    accelerationSensorTask = new PollingSensorTask<acceleration_t>(accelerationSensor, 200, "T_AccelSensor", 1024 * 10,static_cast<osPriority_t>(5));

    DebugPrint("Created accelerometer task\n");
#endif

#if SENSOR_PRESSURE == 1
    pressureSensor->addListener([](const pressure_t & newValue){
        CloudDatabase::instance.updatePressure(newValue);
    });
#endif

#if SENSOR_CAN_LOG == 1
    canLogsSensor->addListener([](const CANLogEntry* newValue) {
        // Print all received CAN messages to Serial
        DebugPrint(newValue->getMessage());
        updateCanMessages(newValue->getMessage());
    });
#endif

#if SENSOR_THROTTLE == 1
    throttleSensor->addListener([](const percentage_t& newValue) {
        CloudDatabase::instance.updateThrottle(newValue);
    });
#endif

#if SENSOR_RPM == 1
    rpmSensor->addListener([](const velocity_t & newValue) {
        CloudDatabase::instance.updateRPM(newValue);
    });
#endif

#if SENSOR_SPEEDOMETER == 1
    speedSensor->addListener([](const speed_t& newValue) {
        CloudDatabase::instance.updateSpeed(newValue);
    });
#endif

#if SENSOR_GPS == 1
    modem = new TinyGsm(Serial1);

    gpsSensor = new GPSSensor(*modem, defaultBufferSize);
    gpsSensor->addListener([](const gps_coordinate_t& newValue) {
        CloudDatabase::instance.updateGPS(newValue);
    });
    gpsSensorTask = new PollingSensorTask<gps_coordinate_t>(gpsSensor, 200, "T_GPSSensor", 1024 * 10, static_cast<osPriority_t>(5));
#endif

#if SENSOR_CAN_LOG == 1 || SENSOR_RPM == 1 || SENSOR_SPEEDOMETER == 1 || SENSOR_THROTTLE == 1
     CANInit(1024 * 10, (osPriority_t) 5, 100);
#endif

#if LOGGER_SD == 1
    DebugPrint("Calling LoggerInit");
    LoggerInit(1000, []() {
        char* row = new char[100];
        sprintf(row, "%d,%f,%f,%d,%f,%f,%f,%f,%f\n", 0, CloudDatabase::instance.getThrottle(), CloudDatabase::instance.getSpeed(), CloudDatabase::instance.getRPM(), CloudDatabase::instance.getBatteryCurrent(), CloudDatabase::instance.getBatteryVoltage(), CloudDatabase::instance.getAccelerationX(), CloudDatabase::instance.getAccelerationY(), CloudDatabase::instance.getAccelerationZ());
        return row;
    }, "timestamp,throttle,speed,rpm,current,voltage,acc_x,acc_y,acc_z\n");
#endif

#if LOGGER_IOT == 1
    CloudDatabase::instance.SetupThing();
    ArduinoCloud.begin(ArduinoIoTPreferredConnection);
    setDebugMessageLevel(2);
    ArduinoCloud.printDebugInfo();
#endif

    DebugPrint("Setup Complete!");
}

void loop() {
#if LOGGER_IOT == 1
    CloudDatabase::instance.PeriodicUpdate();
#endif
}