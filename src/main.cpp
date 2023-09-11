#include "VoltageSensor.hpp"
#include "Accelerometer.hpp"
 #include "PressureSensor.hpp"
#ifdef ESP32

#include <Arduino.h>
#include "PollingSensorTask.hpp"

#include "ThingProperties.h"
#include "Conversions.h"
#include "GPSSensor.hpp"
#include "InternalCommsModule.h"
#include "LoggerTask.hpp"
#include "ValueSensor.hpp"
#include "CANDriver.h"
#include "CANLogEntry.hpp"

#define SD_CS_PIN 0
#define SD_SIGNAL_LIGHT_PIN 0
#define SD_DETECT_PIN 0
#define SD_LOG_BUTTON_PIN 0
#define SD_LOGGER_STACK_SIZE 10240
#define SD_LOGGER_PRIORITY 5
#define SD_LOGGING_RATE 200

#define FONA_TX 10
#define FONA_RX 9
#define FONA_RESET 14
#define fonaSerial Serial1

#define DEFAULT_BUFFER_SIZE 1

Fona3G* fona;
GPSSensor* gpsSensor;

VoltageSensor* voltageSensor = new VoltageSensor(DEFAULT_BUFFER_SIZE);
Accelerometer* accelerationSensor = new Accelerometer(DEFAULT_BUFFER_SIZE);
PressureSensor* pressureSensor = new PressureSensor(DEFAULT_BUFFER_SIZE);

// CAN Values
ValueSensor<CANLogEntry*>* canLogsSensor = new ValueSensor<CANLogEntry*>(DEFAULT_BUFFER_SIZE);
ValueSensor<percentage_t>* throttleSensor = new ValueSensor<percentage_t>(DEFAULT_BUFFER_SIZE);
ValueSensor<speed_t>* speedSensor = new ValueSensor<speed_t>(DEFAULT_BUFFER_SIZE);
ValueSensor<velocity_t>* rpmSensor = new ValueSensor<velocity_t>(DEFAULT_BUFFER_SIZE);

// Polling Tasks
PollingSensorTask<voltage_t>* voltageSensorTask;
PollingSensorTask<acceleration_t>* accelerationSensorTask;
PollingSensorTask<pressure_t>* pressureSensorTask;
PollingSensorTask<gps_coordinate_t>* gpsSensorTask;

// RTOS Handles
TaskHandle_t canHandle = NULL;

// RTOS Execution Loops
[[noreturn]] void canTask(void* args) {
    result_t isInitialized = RESULT_FAIL;

    while (true) {
        if (isInitialized == RESULT_FAIL) {
            isInitialized = IComms_Init();
        } else {
            IComms_PeriodicReceive();
        }

        vTaskDelay(100);
    }
}

// ESP32 Setup
void setup() {
    Serial.begin(115200);
    fonaSerial.begin(4800, SERIAL_8N1, FONA_TX, FONA_RX);

    fona = new Fona3G(&fonaSerial, FONA_RESET);
    gpsSensor = new GPSSensor(fona, DEFAULT_BUFFER_SIZE);

    voltageSensor->addListener([](const voltage_t& newValue) {
        updateBatteryVoltage(convertVoltageToFloat(newValue));
    });

    accelerationSensor->addListener([](const acceleration_t & newValue){
        updateAcceleration(newValue);
    });

    pressureSensor->addListener([](const pressure_t & newValue){
        updatePressure(newValue);
    });

    gpsSensor->addListener([](const gps_coordinate_t& newValue) {
        updateGPS(newValue);
    });

    // Print all received CAN messages to Serial
    canLogsSensor->addListener([](const CANLogEntry* newValue) {
        DebugPrint(newValue->getMessage());
    });

    throttleSensor->addListener([](const percentage_t& newValue) {
        updateThrottle(newValue);
    });

    rpmSensor->addListener([](const velocity_t & newValue) {
        updateRPM(newValue);
    });

    speedSensor->addListener([](const speed_t& newValue) {
        updateSpeed(newValue);
    });

    LoggerInit(
            SD_CS_PIN,
            SD_SIGNAL_LIGHT_PIN,
            SD_DETECT_PIN,
            SD_LOG_BUTTON_PIN,
            []() {
                return "0,0,0,0,0,0,0,0,0,0";
            },
            "timestamp,throttle,speed,rpm,current,voltage,throttleTooHigh,motorInitializing,clockState,lastDeadman",
            SD_LOGGER_STACK_SIZE,
            SD_LOGGER_PRIORITY,
            SD_LOGGING_RATE
    );

    xTaskCreate(canTask, "CanTask", 1024 * 10, nullptr, 3, &canHandle);

    // TODO: Note that sensors will throw an exception if collect is not called before get(). See if we can apply RAII
    voltageSensorTask = new PollingSensorTask<voltage_t>(voltageSensor, 200, "T_VoltageSensor", 1024 * 10, 5);
    accelerationSensorTask = new PollingSensorTask<acceleration_t>(accelerationSensor, 200, "T_AccelSensor", 1024 * 10, 5);
    pressureSensorTask = new PollingSensorTask<pressure_t>(pressureSensor, 200, "T_PressureSensor", 1024 * 10, 5);
    gpsSensorTask = new PollingSensorTask<gps_coordinate_t>(gpsSensor, 200, "T_GPSSensor", 1024 * 10, 5);

    initProperties();
    ArduinoCloud.begin(ArduinoIoTPreferredConnection);
    setDebugMessageLevel(2);
    ArduinoCloud.printDebugInfo();
}

void loop() {
    ArduinoCloud.update();
}

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
    throttleSensor->collect(throttle);
    canLogsSensor->collect(new CANLogEntry(THROTTLE_DATA_ID, throttle, CAN_DECIMAL));
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

        canLogsSensor->collect(new CANLogEntry(ERROR_DATA_ID, code, status, CAN_DECIMAL));
    } else {
        DebugPrint("msg.dataLength does not match lookup table. %d != %d", msg->dataLength, CANMessageLookUpTable[ERROR_DATA_ID].numberOfBytes);
    }
}

/**
 * Listen for Speed messages
 * @param msg
 */
void SpeedDataCallback(iCommsMessage_t *msg) {
    auto speed = (speed_t) readMsg(msg);
    speedSensor->collect(speed);
    canLogsSensor->collect(new CANLogEntry(SPEED_DATA_ID, speed, CAN_DECIMAL));
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
            default:
                break;
        }

        canLogsSensor->collect(new CANLogEntry(EVENT_DATA_ID, code, status, CAN_DECIMAL));
    } else {
        DebugPrint("msg.dataLength does not match lookup table. %d != %d", msg->dataLength, CANMessageLookUpTable[ERROR_DATA_ID].numberOfBytes);
    }
}

/**
 * Listen for RPM messages
 * @param msg
 */
void MotorRPMDataCallback(iCommsMessage_t *msg) {
    auto rpm = (velocity_t) readMsg(msg);
    rpmSensor->collect(rpm);
    canLogsSensor->collect(new CANLogEntry(MOTOR_RPM_DATA_ID, rpm, CAN_DECIMAL));
}

/**
 * Listen for Current and Voltage messages
 * @param msg
 */
void CurrentVoltageDataCallback(iCommsMessage_t *msg) {
    // Do nothing, telemetry broadcasts this type of message
}

#else
#ifndef PIO_UNIT_TESTING
int main() {
    printf("Starting");
    VoltageSensor sensor(10);
    Accelerometer sens_accel(10);
    sensor.collect();
    sens_accel.collect(10);
    printf("Get Value %d", sensor.get());
    return 0;
}
#endif
#endif

