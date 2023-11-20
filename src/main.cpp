#include <Arduino.h>

#include "ApplicationTypes.h"
#include "ThingProperties.h"

#include "PressureSensor.hpp"
#include "ValueSensor.hpp"
#include "CANLogEntry.hpp"
#include "CANTask.hpp"
#include "CANDriver.h"
#include "PollingSensorTask.hpp"
#include "Accelerometer.hpp"
#include "Config.h"

#define DEFAULT_BUFFER_SIZE 1

#define SENSOR_GPS 0
#define SENSOR_VOLTAGE 0
#define SENSOR_ACCELEROMETER 0
#define SENSOR_PRESSURE 0
#define SENSOR_CAN_LOG 1
#define SENSOR_THROTTLE 1
#define SENSOR_SPEEDOMETER 1
#define SENSOR_RPM 1

#define LOGGER_SD 0
#define LOGGER_IOT 0

 /*
 * 0 == WiFi
 * 1 == Cellular
 */
#define INTERNET_CONNECTION 0

#define SD_CS_PIN 18
#define SD_SIGNAL_LIGHT_PIN 0
#define SD_DETECT_PIN 0
#define SD_LOG_BUTTON_PIN 17
#define SD_LOGGER_STACK_SIZE 10240
#define SD_LOGGER_PRIORITY 5
#define SD_LOGGING_RATE 200

#if SENSOR_GPS == 1
GPSSensor* gpsSensor;
PollingSensorTask<gps_coordinate_t>* gpsSensorTask;
#endif

#if SENSOR_VOLTAGE == 1
VoltageSensor* voltageSensor;
PollingSensorTask<voltage_t>* voltageSensorTask;
#endif

#if SENSOR_ACCELEROMETER == 1
Accelerometer* accelerationSensor;
PollingSensorTask<acceleration_t>* accelerationSensorTask;
#endif

#if SENSOR_PRESSURE == 1
PressureSensor* pressureSensor;
PollingSensorTask<pressure_t>* pressureSensorTask;
#endif

// CAN Values
#if SENSOR_CAN_LOG == 1
ValueSensor<CANLogEntry*>* canLogsSensor = new ValueSensor<CANLogEntry*>(DEFAULT_BUFFER_SIZE);
#endif

#if SENSOR_THROTTLE == 1
ValueSensor<percentage_t>* throttleSensor = new ValueSensor<percentage_t>(DEFAULT_BUFFER_SIZE);
#endif

#if SENSOR_SPEEDOMETER == 1
ValueSensor<speed_t>* speedSensor = new ValueSensor<speed_t>(DEFAULT_BUFFER_SIZE);
#endif

#if SENSOR_RPM
ValueSensor<velocity_t>* rpmSensor = new ValueSensor<velocity_t>(DEFAULT_BUFFER_SIZE);
#endif

void setup() {
    Serial.begin(115200);

    delay(2000);

    DebugPrint("Initializing Telemetry System...");

#ifndef UOSM_SECRETS
    DebugPrint("Failed to find secrets... Make sure to create a Secrets.h file. Aborting!");
    while (true) {}
#endif

    // TODO: Note that sensors will throw an exception if collect is not called before get(). See if we can apply RAII

#if SENSOR_VOLTAGE == 1
    voltageSensor = new VoltageSensor(DEFAULT_BUFFER_SIZE);
    voltageSensor->addListener([](const voltage_t& newValue) {
        iotMutex.execute([newValue]() {
            updateBatteryVoltage(convertVoltageToFloat(newValue));
        });
    });

    voltageSensorTask = new PollingSensorTask<voltage_t>(voltageSensor, 200, "T_VoltageSensor", 1024 * 10, 5);
#endif

#if SENSOR_ACCELEROMETER == 1
    accelerationSensor = new Accelerometer(DEFAULT_BUFFER_SIZE);

    TelemetryPrint("Created accelerometer\n");

    accelerationSensor->addListener([](const acceleration_t & newValue){
        TelemetryPrint("Received new acceleration: %f %f %f\n", newValue.x, newValue.y, newValue.z);
        iotMutex.execute([newValue]() {
            updateAcceleration(newValue);
        });
    });

    TelemetryPrint("Added listener\n");

    accelerationSensor->collect();

    TelemetryPrint("Forced collection\n");

    TelemetryPrint("Creating PollingSensorTask\n");
    accelerationSensorTask = new PollingSensorTask<acceleration_t>(accelerationSensor, 200, "T_AccelSensor", 1024 * 10,static_cast<osPriority_t>(5));

    TelemetryPrint("Created accelerometer task\n");
#endif

#if SENSOR_PRESSURE == 1
    pressureSensor = new PressureSensor(DEFAULT_BUFFER_SIZE);
    pressureSensor->addListener([](const pressure_t & newValue){
        iotMutex.execute([newValue]() {
            updatePressure(newValue);
        });
    });

    pressureSensorTask = new PollingSensorTask<pressure_t>(pressureSensor, 200, "T_PressureSensor", 1024 * 10, (osPriority_t) 5);
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
        iotMutex.execute([newValue]() {
            updateThrottle(newValue);
        });
    });
#endif

#if SENSOR_RPM == 1
    rpmSensor->addListener([](const velocity_t & newValue) {
        iotMutex.execute([newValue]() {
            updateRPM(newValue);
        });
    });
#endif

#if SENSOR_SPEEDOMETER == 1
    speedSensor->addListener([](const speed_t& newValue) {
        iotMutex.execute([newValue]() {
            updateSpeed(newValue);
        });
    });
#endif

#if INTERNET_CONNECTION == 0

#elif INTERNET_CONNECTION == 1

#endif

#if SENSOR_GPS == 1
    gpsSensor = new GPSSensor(fona, DEFAULT_BUFFER_SIZE);
    gpsSensor->addListener([](const gps_coordinate_t& newValue) {
        iotMutex.execute([newValue]() {
            updateGPS(newValue);
        });
    });
    gpsSensorTask = new PollingSensorTask<gps_coordinate_t>(gpsSensor, 200, "T_GPSSensor", 1024 * 10, 5);
#endif

#if SENSOR_CAN_LOG == 1 || SENSOR_RPM == 1 || SENSOR_SPEEDOMETER == 1 || SENSOR_THROTTLE == 1
     CANInit(1024 * 10, (osPriority_t) 5, 100);
#endif

#if LOGGER_SD == 1
    LoggerInit(SD_CS_PIN, SD_SIGNAL_LIGHT_PIN, SD_DETECT_PIN, SD_LOG_BUTTON_PIN, []() {
        char* row = new char[100];
        sprintf(row, "%d,%f,%f,%d,%f,%f", xTaskGetTickCount(), (float) throttle, (float) speed, motorRPM, (float) batteryCurrent, (float) batteryVoltage);
        return row;
    }, "timestamp,throttle,speed,rpm,current,voltage", SD_LOGGER_STACK_SIZE, SD_LOGGER_PRIORITY, SD_LOGGING_RATE);
#endif

#if LOGGER_IOT == 1
    initProperties();
    ArduinoCloud.begin(ArduinoIoTPreferredConnection);
    setDebugMessageLevel(2);
    ArduinoCloud.printDebugInfo();
#endif

    DebugPrint("Setup Complete!");
}

void loop() {
#if LOGGER_IOT == 1
    if (!iotMutex.getLocked()) {
        iotMutex.lock();
        periodicMotorOn();
        ArduinoCloud.update();
        iotMutex.unlock();
    } else {
        rtos::ThisThread::sleep_for(std::chrono::milliseconds(DEFAULT_MUTEX_DELAY));
    }
#endif
}

#ifdef __cplusplus
extern "C" {
#endif
/************
 * Adapter to convert CAN Callbacks into Sensors
 ************/

/**
 * Listen for Throttle messages
 * @param msg
 */
void ThrottleDataCallback(iCommsMessage_t *msg) {
    // Read the CAN Message as a 32bit int, then cast it to a percentage_t
    auto throttle = (percentage_t) readMsg(msg);

    // Explicitly collect the throttle for the throttle sensor
#if SENSOR_THROTTLE == 1
    throttleSensor->collect(throttle);
#endif

#if SENSOR_CAN_LOG == 1
    canLogsSensor->collect(new CANLogEntry(THROTTLE_DATA_ID, throttle, CAN_DECIMAL));
#endif
}

/**
 * Listen for Error messages
 * @param msg
 */
void ErrorDataCallback(iCommsMessage_t *msg) {
    if (msg->dataLength == CANMessageLookUpTable[ERROR_DATA_ID].numberOfBytes) {
        auto code = (ErrorCode) msg->data[1];
        auto status = (flag_status_t) msg->data[0];

        switch (code) {
            default:
                break;
        }

#if SENSOR_CAN_LOG == 1
        canLogsSensor->collect(new CANLogEntry(ERROR_DATA_ID, code, status, CAN_DECIMAL));
#endif
    } else {
        DebugPrint("msg.dataLength does not match lookup table. %d != %d", msg->dataLength,
                   CANMessageLookUpTable[ERROR_DATA_ID].numberOfBytes);
    }
}

/**
 * Listen for Speed messages
 * @param msg
 */
void SpeedDataCallback(iCommsMessage_t *msg) {
    auto speed = (speed_t) readMsg(msg);

#if SENSOR_SPEEDOMETER == 1
    speedSensor->collect(speed);
#endif

#if SENSOR_CAN_LOG == 1
    canLogsSensor->collect(new CANLogEntry(SPEED_DATA_ID, speed, CAN_DECIMAL));
#endif
}

/**
 * Listen for Event messages
 * @param msg
 */
void EventDataCallback(iCommsMessage_t *msg) {
    if (msg->dataLength == CANMessageLookUpTable[EVENT_DATA_ID].numberOfBytes) {
        auto code = (EventCode) msg->data[1];
        auto status = (flag_status_t) msg->data[0];

        switch (code) {
            case DEADMAN:
                updateMotorOn(status);
            default:
                break;
        }

#if SENSOR_CAN_LOG == 1
        canLogsSensor->collect(new CANLogEntry(EVENT_DATA_ID, code, status, CAN_DECIMAL));
#endif
    } else {
        DebugPrint("msg.dataLength does not match lookup table. %d != %d", msg->dataLength,
                   CANMessageLookUpTable[ERROR_DATA_ID].numberOfBytes);
    }
}

/**
 * Listen for RPM messages
 * @param msg
 */
void MotorRPMDataCallback(iCommsMessage_t *msg) {
    auto rpm = (velocity_t) readMsg(msg);

#if SENSOR_RPM == 1
    rpmSensor->collect(rpm);
#endif

#if SENSOR_CAN_LOG == 1
    canLogsSensor->collect(new CANLogEntry(MOTOR_RPM_DATA_ID, rpm, CAN_DECIMAL));
#endif
}

/**
 * Listen for Current and Voltage messages
 * @param msg
 */
void CurrentVoltageDataCallback(iCommsMessage_t *msg) {
    // Do nothing, telemetry broadcasts this type of message
}

#ifdef __cplusplus
}
#endif
