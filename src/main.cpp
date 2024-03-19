#include <Arduino.h>
#include <SPI.h>

#include "ApplicationTypes.h"
#include "ThingProperties.hpp"

#include "ValueSensor.hpp"
#include "CANLogEntry.hpp"
#include "CANTask.hpp"
#include "CANDriver.h"
#include "PollingSensorTask.hpp"
#include "Config.h"

constexpr uint32_t serialBaudrate = 115200;
constexpr uint8_t defaultBufferSize = 1;

#if LOGGER_SD == 1
#include "LoggerTask.hpp"
#endif

#define INTERNET_CONNECTION 2

#if INTERNET_CONNECTION == 1
WiFiConnectionHandler ArduinoIoTPreferredConnection(SSID, PASS);
#elif INTERNET_CONNECTION == 2
#include "TinyGSMConnectionHandler.hpp"
#include "InternalCommsModule.h"

TinyGSMConnectionHandler* ArduinoIoTPreferredConnection;
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
#include "Arduino_PortentaBreakout.h"

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

// CAN Values
#if SENSOR_CAN_LOG == 1
ValueSensor<CANLogEntry*>* canLogsSensor = new ValueSensor<CANLogEntry*>(defaultBufferSize);
#endif

#if SENSOR_THROTTLE == 1
ValueSensor<percentage_t>* throttleSensor = new ValueSensor<percentage_t>(defaultBufferSize);
#endif

#if SENSOR_SPEEDOMETER == 1
#include "Speedometer.hpp"

#define SPEEDOMETER_PIN 4

void hallInterupt();
Speedometer* speedometer;
PollingSensorTask<speed_t>* speedometerTask;
#endif

#if SENSOR_RPM == 1
ValueSensor<velocity_t>* rpmSensor = new ValueSensor<velocity_t >(defaultBufferSize);
#endif

static rtos::Thread statusLightThread;

void StatusLightTask() {
    bool isOn = false;
    while (true) {
        digitalWrite(LEDG, isOn ? HIGH : LOW);
        isOn = !isOn;
        osDelay(500);
    }
}

static rtos::Thread iotThread(osPriorityLow);

void IotTask() {
    CloudDatabase::instance.SetupThing();
    ArduinoIoTPreferredConnection = new TinyGSMConnectionHandler(Serial1, "", "LTEMOBILE.APN", "", "");
    ArduinoCloud.begin(*ArduinoIoTPreferredConnection);
    setDebugMessageLevel(2);
    ArduinoCloud.printDebugInfo();

    while (true) {
        CloudDatabase::instance.PeriodicUpdate();

        static bool isOn = false;
        digitalWrite(LEDB, isOn ? HIGH : LOW);
        isOn = !isOn;
        osDelay(50);
    }
}

void setup() {
    Serial.begin(serialBaudrate);

    delay(2000);

    printf("Initializing Telemetry System...\n");

#ifndef UOSM_SECRETS
    printf("Failed to find secrets... Make sure to create a Secrets.h file. Aborting!\n");
    while (true) {}
#endif

    // TODO: Note that sensors will throw an exception if collect is not called before get(). See if we can apply RAII

#if SENSOR_VOLTAGE == 1 || SENSOR_CURRENT == 1
    Wire.begin();
#endif

#if SENSOR_VOLTAGE == 1
    voltageSensor = new VoltageSensor(Wire, VoltageSensor::VoltageSensorMode::Differential_0_1, 0, 0x40, defaultBufferSize);
    voltageSensor->addListener([](const voltage_t& newValue) {
        CloudDatabase::instance.updateBatteryVoltage(newValue);
        printf("Voltage: %f", newValue);
    });
    voltageSensorTask = new PollingSensorTask<voltage_t>(voltageSensor, 200, "T_VoltageSensor", 1024 * 10, osPriorityNormal);
#endif

#if SENSOR_CURRENT == 1
    currentSensor = new VoltageSensor(Wire, VoltageSensor::VoltageSensorMode::Differential_2_3, 0, 0x40, defaultBufferSize);
    currentSensor->addListener([](const voltage_t& newValue) {
        CloudDatabase::instance.updateBatteryCurrent(newValue / 12.5f);
        printf("Current: %fA\n", newValue / 12.5f);
    });
    currentSensorTask = new PollingSensorTask<voltage_t>(currentSensor, 200, "T_CurrentSensor", 1024 * 10, osPriorityNormal);
#endif

#if SENSOR_ACCELEROMETER == 1
    accelerationSensor = new Accelerometer(defaultBufferSize);

    printf("Created accelerometer\n");

    accelerationSensor->addListener([](const acceleration_t & newValue){
        printf("Acceleration X: %f\n", newValue.x);
        printf("Acceleration Y: %f\n", newValue.y);
        printf("Acceleration Z: %f\n", newValue.z);

        CloudDatabase::instance.updateAcceleration(newValue);
    });

    printf("Added listener\n");

    accelerationSensor->collect();

    printf("Forced collection\n");

    printf("Creating PollingSensorTask\n");
    accelerationSensorTask = new PollingSensorTask<acceleration_t>(accelerationSensor, 200, "T_AccelSensor", 1024 * 10, osPriorityNormal);

    printf("Created accelerometer task\n");
#endif

#if SENSOR_CAN_LOG == 1
//    canLogsSensor->addListener([](const CANLogEntry* newValue) {
//        // Print all received CAN messages to Serial
//        CloudDatabase::instance.updateCanMessages(newValue->getMessage());
//    });
#endif

#if SENSOR_THROTTLE == 1
    throttleSensor->addListener([](const percentage_t& newValue) {
        printf("Throttle: %d", newValue);
        CloudDatabase::instance.updateThrottle(newValue);
    });
#endif

#if SENSOR_RPM == 1
    rpmSensor->addListener([](const velocity_t & newValue) {
        CloudDatabase::instance.updateRPM(newValue);
    });
#endif

#if SENSOR_SPEEDOMETER == 1
    speedometer = new Speedometer(defaultBufferSize);

    attachInterrupt(digitalPinToInterrupt(SPEEDOMETER_PIN), hallInterupt, HIGH);

    speedometer->addListener([](const speed_t& newValue) {
        CloudDatabase::instance.updateSpeed(newValue);
        printf("Speed: %d", newValue);
    });

    speedometerTask = new PollingSensorTask<speed_t>(speedometer, 200, "T_Speedometer", 1024 * 10, osPriorityNormal);
#endif

#if SENSOR_GPS == 1
    modem = new TinyGsm(Serial1);

    gpsSensor = new GPSSensor(*modem, defaultBufferSize);
    gpsSensor->addListener([](const gps_coordinate_t& newValue) {
        CloudDatabase::instance.updateGPS(newValue);
    });
    gpsSensorTask = new PollingSensorTask<gps_coordinate_t>(gpsSensor, 200, "T_GPSSensor", 1024 * 10, osPriorityNormal);
#endif

#if ENABLE_CAN == 1
    pinMode(MCP2515_CS_PIN, OUTPUT);
    SPI.begin();
    SPI.beginTransaction(SPISettings(100000, MSBFIRST, SPI_MODE3));
    printf("Starting CANInit!\n");
    CANInit(1024 * 10, (osPriority_t) 7, 400);
#endif

#if LOGGER_SD == 1
    printf("Calling LoggerInit\n");
    LoggerInit(1000, []() {
        char* row = new char[100];
        // TODO: Add pressure, and other metrics
        sprintf(row, "%d,%f,%f,%d,%f,%f,%f,%f,%f\n", 0, CloudDatabase::instance.getThrottle(), CloudDatabase::instance.getSpeed(), CloudDatabase::instance.getRPM(), CloudDatabase::instance.getBatteryCurrent(), CloudDatabase::instance.getBatteryVoltage(), CloudDatabase::instance.getAccelerationX(), CloudDatabase::instance.getAccelerationY(), CloudDatabase::instance.getAccelerationZ());
        return row;
    }, "timestamp,throttle,speed,rpm,current,voltage,acc_x,acc_y,acc_z\n");
#endif

#if LOGGER_IOT == 1
    iotThread.start(mbed::callback(IotTask));
#endif

    statusLightThread.start(mbed::callback(StatusLightTask));
    printf("Setup Complete!\n");

    pinMode(LEDB, OUTPUT);
    pinMode(LEDG, OUTPUT);
}

//const ICommsMessageInfo* speedInfo = CANMessageLookUpGetInfo(SPEED_DATA_ID);
//uint8_t speedTxCounter = 0;
//#define SPEED_RATE 400


void loop() {
    rtos::ThisThread::sleep_for(std::chrono::milliseconds(1000));
}

#if SENSOR_SPEEDOMETER == 1
void hallInterupt() {
    speedometer->hallCallback();
}
#endif

#include "CANMessageLookUpModule.h"

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

void SpeedDataCallback(iCommsMessage_t *msg) { }

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
//                CloudDatabase::instance.updateMotorOn(status);
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

#if DATA_AIR_SPEED == 1

// map of air density values at different temperatures (key, value) = (temperature, air density)
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

// air pressure value
int airPressure = 0;

// air temperature value
int airTemperature = 0;

void AirSpeedDataProcessing() {
    // round air temperature to nearest 5
    int roundedAirTemperature = (int) (5 * round((float) airTemperature / 5));

    // get air density
    float airDensity = airDensityMap.at(roundedAirTemperature);

    // calculate air speed
    float airSpeed = sqrt((2 * airPressure) / airDensity);

    // update air speed
    //CloudDatabase::instance.updateAirSpeed(airSpeed);
    DebugPrint("Air Speed: %f", airSpeed);
}
#endif

/**
 * Listen for Current and Voltage messages
 * @param msg
 */
void CurrentVoltageDataCallback(iCommsMessage_t *msg) {
    // Do nothing, telemetry broadcasts this type of message
}

void LightsDataCallback(iCommsMessage_t *msg) {
    // Do nothing, telemetry broadcasts this type of message
}

void PressureDataCallback(iCommsMessage_t *msg) {
    // Do nothing, telemetry broadcasts this type of message
    airPressure = readMsg(msg);
    AirSpeedDataProcessing();
}

void TemperatureDataCallback(iCommsMessage_t *msg) {
    // Do nothing, telemetry broadcasts this type of message
    airTemperature = readMsg(msg);
    AirSpeedDataProcessing();
}

#ifdef __cplusplus
}
#endif