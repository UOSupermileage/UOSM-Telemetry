#include <Arduino.h>
#include <SPI.h>

#include "ApplicationTypes.h"
#include "ThingProperties.hpp"

#include "ValueSensor.hpp"
#include "CANLogEntry.hpp"
#include "CANMessageLookUpModule.h"
#include "CANTask.hpp"
#include "CANDriver.h"
#include "PollingSensorTask.hpp"
#include "Config.h"

constexpr uint32_t serialBaudrate = 115200;
constexpr uint8_t defaultBufferSize = 1;

#define CAN_INTERUPT_PIN PG_7

#if LOGGER_SD == 1
#include "LoggerTask.hpp"
#endif

#define INTERNET_CONNECTION 2

#if INTERNET_CONNECTION == 1
WiFiConnectionHandler ArduinoIoTPreferredConnection(SSID, PASS);
#elif INTERNET_CONNECTION == 2

#include "TinyGSMConnectionHandler.hpp"
#include "InternalCommsModule.h"

TinyGSMConnectionHandler *ArduinoIoTPreferredConnection;
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

#if SENSOR_VOLTAGE == 1 || SENSOR_CURRENT == 1

#include "VoltageSensor.hpp"

VoltageSensor *voltageSensor;
PollingSensorTask<voltage_current_t> *voltageSensorTask;
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
ValueSensor<percentage_t> *throttleSensor = new ValueSensor<percentage_t>();
#endif

#if SENSOR_SPEEDOMETER == 1

#include "Speedometer.hpp"

#define SPEEDOMETER_PIN 4

void hallInterupt();

Speedometer *speedometer;
PollingSensorTask<speed_t> *speedometerTask;

#endif

#if SENSOR_RPM == 1
ValueSensor<velocity_t> *rpmSensor = new ValueSensor<velocity_t>();
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

#if LOGGER_IOT == 1
rtos::Thread iotThread(osPriorityHigh, 40960);

void GetData(TinyGSMConnectionHandler* c) {
    const char server[]   = "vsh.pp.ua";
    const char resource[] = "/TinyGSM/logo.txt";

    const int     port = 80;
    DBG("Connecting to", server);
    if (!c->getClient().connect(server, port)) {
        DBG("... failed");
    } else {
        // Make a HTTP GET request:
        c->getClient().print(String("GET ") + resource + " HTTP/1.0\r\n");
        c->getClient().print(String("Host: ") + server + "\r\n");
        c->getClient().print("Connection: close\r\n\r\n");

        // Wait for data to arrive
        uint32_t start = millis();
        while (c->getClient().connected() &&
               !c->getClient().available() &&
               millis() - start < 30000L) {
            osDelay(100);
        };

        // Read data
        start = millis();
        char logo[640] = {
                '\0',
        };
        int read_chars = 0;
        while (c->getClient().connected() && millis() - start < 10000L) {
            while (c->getClient().available()) {
                logo[read_chars] = c->getClient().read();
                logo[read_chars + 1] = '\0';
                read_chars++;
                start = millis();
            }
        }
        DebugPrint(logo);
    }
}

void IotTask() {
    ArduinoIoTPreferredConnection = new TinyGSMConnectionHandler(Serial1, "", "LTEMOBILE.APN", "", "");

    GetData(ArduinoIoTPreferredConnection);

    CloudDatabase::instance.SetupThing();
    ArduinoCloud.begin(*ArduinoIoTPreferredConnection);
    setDebugMessageLevel(4);
    Debug.setDebugLevel(DBG_VERBOSE);
    Debug.setDebugOutputStream(&Serial);
    ArduinoCloud.printDebugInfo();

    while (true) {

//        GetData(ArduinoIoTPreferredConnection);

        CloudDatabase::instance.PeriodicUpdate();


        static bool isOn = false;
        digitalWrite(LEDB, isOn ? HIGH : LOW);
        isOn = !isOn;

        osDelay(50);
    }
}
#endif

static rtos::Thread canThread;

uint16_t pollingRate;

#define EFFICIENCY_BROADCAST_RATE 2
#define VOLTAGE_CURRENT_BROADCAST_RATE 5
#define SPEED_BROADCAST_RATE 2
#define BRAKES_BROADCAST_RATE 2

// RTOS Execution Loops
[[noreturn]] void CANTask() {
    pinMode(MCP2515_CS_PIN, OUTPUT);
    pinMode(SPI_MISO, INPUT);

    SPI.begin();
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));

    result_t isInitialized = RESULT_FAIL;

    uint8_t broadcast_voltage_current_counter = 0;
    const ICommsMessageInfo *voltageCurrentInfo = CANMessageLookUpGetInfo(CURRENT_VOLTAGE_DATA_ID);

    uint8_t speedTxCounter = 0;
    const ICommsMessageInfo *speedInfo = CANMessageLookUpGetInfo(SPEED_DATA_ID);

    uint8_t efficiencyTxCounter = 0;
    const ICommsMessageInfo *efficiencyInfo = CANMessageLookUpGetInfo(EFFICIENCY_DATA_ID);

    uint8_t brakesTxCounter = 0;
    const ICommsMessageInfo *eventInfo = CANMessageLookUpGetInfo(EVENT_DATA_ID);

    while (true) {
        if (isInitialized == RESULT_FAIL) {
            isInitialized = IComms_Init();
            continue;
        }

        IComms_PeriodicReceive();

        if (broadcast_voltage_current_counter++ > VOLTAGE_CURRENT_BROADCAST_RATE) {
            iCommsMessage_t txMsg = IComms_CreatePairUInt16BitMessage(voltageCurrentInfo->messageID, CloudDatabase::instance.getBatteryCurrent(), CloudDatabase::instance.getBatteryVoltage());;
            result_t _ = IComms_Transmit(&txMsg);
            Serial.print("Sent voltage: "); Serial.println(CloudDatabase::instance.getBatteryVoltage());
            broadcast_voltage_current_counter = 0;
        }

        speedTxCounter++;
        if (speedTxCounter > SPEED_BROADCAST_RATE) {
            iCommsMessage_t speedTxMsg = IComms_CreateUint32BitMessage(speedInfo->messageID, CloudDatabase::instance.getSpeed());
            result_t r = IComms_Transmit(&speedTxMsg);
//            printf("Sending Speed [%d] Result: %d\n", CloudDatabase::instance.getSpeed(), r);
            speedTxCounter = 0;
        }

        efficiencyTxCounter++;
        if (efficiencyTxCounter > EFFICIENCY_BROADCAST_RATE) {
            Serial.println("Sending efficiencies.");
            lap_efficiencies_t efficiencies;
            CloudDatabase::instance.getLapEfficiencies(&efficiencies);
//            printf("Lap Efficiencies: %d %d %d %d\n", efficiencies.lap_0, efficiencies.lap_1, efficiencies.lap_2, efficiencies.lap_3);
            iCommsMessage_t efficiencyTxMsg = IComms_CreateEfficiencyMessage(efficiencyInfo->messageID, &efficiencies);
            result_t r = IComms_Transmit(&efficiencyTxMsg);
//            printf("Eff Transmission Result: %d", r);

            efficiencyTxCounter = 0;
        }

#if SENSOR_BRAKES == 1
            brakesTxCounter++;
            if (brakesTxCounter > BRAKES_BROADCAST_RATE) {

                flag_status_t brakesEnabled = Clear;

                int brakesPinReading = digitalRead(BRAKES_INPUT_PIN);//1; // analogRead(BRAKES_INPUT_PIN) > 500;
                float percentage = (float) (brakesPinReading) * 100;
                CloudDatabase::instance.updateBrakesPercentage(percentage);

                Serial.print("Brakes: ");
                Serial.print(percentage);
                Serial.println();

                if (brakesPinReading) {
                    DebugPrint("Brakes Enabled");
                }
                iCommsMessage_t brakesTxMsg = IComms_CreateEventMessage(eventInfo->messageID, BRAKES_ENABLED,
                                                                        brakesPinReading == HIGH ? Set : Clear);
                result_t r = IComms_Transmit(&brakesTxMsg);
                printf("brakes r: %d", r);
                brakesTxCounter = 0;
            }
#endif
        rtos::ThisThread::sleep_for(std::chrono::milliseconds(200));
    }
}

void CANInit(
        uint32_t stackSize,
        osPriority_t priority,
        uint16_t _pollingRate
) {
    printf("CANInit");
    canThread.start(mbed::callback(CANTask));
}

void setup() {
    Serial.begin(serialBaudrate);

    delay(1000);

    printf("Initializing Telemetry System...\n");

#ifndef UOSM_SECRETS
    printf("Failed to find secrets... Make sure to create a Secrets.h file. Aborting!\n");
    while (true) {}
#endif

#if SENSOR_VOLTAGE == 1 || SENSOR_CURRENT == 1
    Wire.begin();
#endif

#if SENSOR_VOLTAGE == 1 || SENSOR_CURRENT == 1
    voltageSensor = new VoltageSensor(Wire, 0, 0x40);
    voltageSensor->addListener([](const voltage_current_t &newValue) {
        CloudDatabase::instance.updateBatteryVoltage(newValue.voltage);
        Serial.print("Battery Voltage: ");
        Serial.println(newValue.voltage);

        CloudDatabase::instance.updateBatteryCurrent(newValue.current);
        Serial.print("Battery Current: ");
        Serial.println(newValue.current);
    });

    voltageSensorTask = new PollingSensorTask<voltage_current_t>(voltageSensor, 200, "T_VoltageSensor", 1024 * 10,
                                                                 osPriorityNormal);
#endif

#if SENSOR_ACCELEROMETER == 1
    // TODO: Validate address and drdy pin
    accelerationSensor = new Accelerometer(Wire, 0x16, 6);

    printf("Created accelerometer\n");

    accelerationSensor->addListener([](const acceleration_t & newValue){
        printf("Acceleration X: %f\n", newValue.x);
        printf("Acceleration Y: %f\n", newValue.y);
        printf("Acceleration Z: %f\n", newValue.z);

        CloudDatabase::instance.updateAcceleration(newValue);
    });

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
    throttleSensor->addListener([](const percentage_t &newValue) {
        Serial.print("Throttle: ");
        Serial.println(newValue);
        CloudDatabase::instance.updateThrottle(newValue);
    });
#endif

#if SENSOR_RPM == 1
    rpmSensor->addListener([](const velocity_t &newValue) {
        CloudDatabase::instance.updateRPM(newValue);
    });
#endif

#if SENSOR_SPEEDOMETER == 1
    pinMode(SPEEDOMETER_PIN, INPUT);
    speedometer = new Speedometer();

    if (digitalPinToInterrupt(SPEEDOMETER_PIN) != -1) {
        attachInterrupt(digitalPinToInterrupt(SPEEDOMETER_PIN), hallInterupt, RISING);
    } else {
        while (true) {
            DebugPrint("Unsupported speedometer pin");
        }
    }

    speedometer->addListener([](const speed_t &newValue) {
        CloudDatabase::instance.updateSpeed(newValue);
        printf("Speed: %d", newValue);
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

#if SENSOR_BRAKES == 1
    pinMode(BRAKES_OUTPUT_PIN, OUTPUT);
    digitalWrite(BRAKES_OUTPUT_PIN, HIGH);
    pinMode(BRAKES_INPUT_PIN, INPUT_PULLDOWN);
#endif

#if ENABLE_CAN == 1
    pinMode(MCP2515_CS_PIN, OUTPUT);
    printf("Starting CANInit!\n");
    CANInit(1024 * 10, (osPriority_t) 7, 400);
#endif

#if LOGGER_SD == 1
    printf("Calling LoggerInit\n");
    LoggerInit(1000, []() {
        char* row = new char[100];
        // TODO: Add pressure, and other metrics
        sprintf(row, "%d,%f,%f,%d,%f,%f,%f,%f,%f\n", 0, CloudDatabase::instance.getThrottle(), CloudDatabase::instance.getSpeed(), CloudDatabase::instance.getRPM(), CloudDatabase::instance.getBatteryCurrent(), CloudDatabase::instance.getBatteryVoltage());
        return row;
    }, "timestamp,throttle,speed,rpm,current,voltage\n");
#endif

#if LOGGER_IOT == 1
    iotThread.start(mbed::callback(IotTask));
#endif

//    statusLightThread.start(mbed::callback(StatusLightTask));
    printf("Setup Complete!\n");

    pinMode(LEDB, OUTPUT);
    pinMode(LEDG, OUTPUT);
    digitalWrite(LEDG, HIGH);
//    pinMode(LEDR, OUTPUT);
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

void SpeedDataCallback(iCommsMessage_t *msg) {}

/**
 * Listen for Event messages
 * @param msg
 */
void EventDataCallback(iCommsMessage_t *msg) {
    if (msg->dataLength == CANMessageLookUpTable[EVENT_DATA_ID].numberOfBytes) {
        auto code = (EventCode) msg->data[1];
        uint8_t status = msg->data[0];

        switch (code) {
            case DEADMAN:
                CloudDatabase::instance.updateMotorOn(status);
                break;
            case NEW_LAP:
                Serial.print("New Lap: "); Serial.println(status);
                CloudDatabase::instance.setLap(status);
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

void LightsDataCallback(iCommsMessage_t *msg) {
    // Do nothing, telemetry broadcasts this type of message
}

void PressureTemperatureDataCallback(iCommsMessage_t *msg) {

//    DebugPrint("Received PressureTemperature");

    int32 pressure, temperature;
    result_t r = IComms_ReadPressureTemperatureMessage(msg, &pressure, &temperature);

    if (r) {
        CloudDatabase::instance.updatePressure(pressure, temperature);
        printf("Update Pressure: %ld, %ld", pressure, temperature);
    }
}

void EfficiencyDataCallback(iCommsMessage_t *msg) {

}

#ifdef __cplusplus
}
#endif