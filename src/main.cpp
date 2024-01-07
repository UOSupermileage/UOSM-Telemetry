#include <Arduino.h>

#include "ApplicationTypes.h"
#include "ThingProperties.h"

#include "ValueSensor.hpp"
#include "CANLogEntry.hpp"
#include "CANTask.hpp"
#include "CANDriver.h"
#include "PollingSensorTask.hpp"
#include "Config.h"
#include "LoggerTask.hpp"

constexpr uint8_t defaultBufferSize = 1;

//#define SENSOR_GPS
//#define SENSOR_VOLTAGE
//#define SENSOR_ACCELEROMETER
//#define SENSOR_PRESSURE
//#define SENSOR_CAN_LOG
//#define SENSOR_THROTTLE
//#define SENSOR_SPEEDOMETER
//#define SENSOR_RPM

#define LOGGER_SD
#define LOGGER_IOT

enum class InternetConnection: uint8_t {
    disabled = 0,
    wifi = 1,
    cellular = 2
};

constexpr InternetConnection connection = InternetConnection::wifi;

WiFiConnectionHandler ArduinoIoTPreferredConnection(SSID, PASS);

#ifdef SENSOR_GPS
#include "GPSSensor.hpp"

std::unique_ptr<GPSSensor> gpsSensor;
std::unique_ptr<PollingSensorTask<gps_coordinate_t>> gpsSensorTask;
#endif

#ifdef SENSOR_VOLTAGE
#include "VoltageSensor.hpp"

std::unique_ptr<VoltageSensor> voltageSensor;
std::unique_ptr<PollingSensorTask<voltage_t>> voltageSensorTask;
#endif

#ifdef SENSOR_ACCELEROMETER
#include "Accelerometer.hpp"

std::unique_ptr<Accelerometer> accelerationSensor;
std::unique_ptr<PollingSensorTask<acceleration_t>> accelerationSensorTask;
#endif

#ifdef SENSOR_PRESSURE
#include "PressureSensor.hpp"
#include "Conversions.h"

std::unique_ptr<PressureSensor> pressureSensor;
std::unique_ptr<PollingSensorTask<pressure_t>> pressureSensorTask;
#endif

// CAN Values
#ifdef SENSOR_CAN_LOG
ValueSensor<CANLogEntry*>* canLogsSensor = new ValueSensor<CANLogEntry*>(defaultBufferSize);
#endif

#ifdef SENSOR_THROTTLE
std::unique_ptr<ValueSensor<percentage_t>> throttleSensor = std::make_unique<ValueSensor<percentage_t>>(defaultBufferSize);
#endif

#ifdef SENSOR_SPEEDOMETER
std::unique_ptr<ValueSensor<speed_t>> speedSensor = std::make_unique<ValueSensor<speed_t>>(defaultBufferSize);
#endif

#ifdef SENSOR_RPM
std::unique_ptr<ValueSensor<velocity_t>> rpmSensor = std::make_unique<ValueSensor<velocity_t >>(defaultBufferSize);
#endif

void setup() {
    Serial.begin(115200);

    TelemetryPrint("Initializing Telemetry System...");

#ifndef UOSM_SECRETS
    TelemetryPrint("Failed to find secrets... Make sure to create a Secrets.h file. Aborting!");
    while (true) {}
#endif

    // TODO: Note that sensors will throw an exception if collect is not called before get(). See if we can apply RAII

#ifdef SENSOR_VOLTAGE
    voltageSensor = std::make_unique<VoltageSensor>(defaultBufferSize);
    voltageSensor->addListener([](const voltage_t& newValue) {
        iotMutex.execute([newValue]() {
            updateBatteryVoltage(convertVoltageToFloat(newValue));
        });
    });

    voltageSensorTask = std::make_unique<PollingSensorTask<voltage_t>>(voltageSensor, 200, "T_VoltageSensor", 1024 * 10, 5);
#endif

#ifdef SENSOR_ACCELEROMETER
    accelerationSensor = new Accelerometer(defaultBufferSize);

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
    pressureSensor = new PressureSensor(defaultBufferSize);
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
    gpsSensor = new GPSSensor(fona, defaultBufferSize);
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

#ifdef LOGGER_SD
    DebugPrint("Calling LoggerInit");
    LoggerInit(1000, []() {
        char* row = new char[100];
        sprintf(row, "%d,%f,%f,%d,%f,%f,%f,%f,%f\n", 0, getThrottle(), getSpeed(), getRPM(), getBatteryCurrent(), getBatteryVoltage(), getAccelerationX(), getAccelerationY(), getAccelerationZ());
        return row;
    }, "timestamp,throttle,speed,rpm,current,voltage,acc_x,acc_y,acc_z\n");
#endif

#ifdef LOGGER_IOT
    SetupThing();
    ArduinoCloud.begin(ArduinoIoTPreferredConnection);
    setDebugMessageLevel(2);
    ArduinoCloud.printDebugInfo();
#endif

    DebugPrint("Setup Complete!");
}

void loop() {
#ifdef LOGGER_IOT
    if (!iotMutex.getLocked()) {
        iotMutex.lock();
        // TODO: Is there a cleaner way of doing this
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
