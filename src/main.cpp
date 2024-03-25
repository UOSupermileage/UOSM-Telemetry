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

SPI_HandleTypeDef hspi2;


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
ValueSensor<percentage_t>* throttleSensor = new ValueSensor<percentage_t>();
#endif

#if SENSOR_SPEEDOMETER == 1
#include "Speedometer.hpp"

#define SPEEDOMETER_PIN 4

void hallInterupt();
Speedometer* speedometer;
PollingSensorTask<speed_t>* speedometerTask;
#endif

#if SENSOR_RPM == 1
ValueSensor<velocity_t>* rpmSensor = new ValueSensor<velocity_t >();
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
#endif

static rtos::Thread canThread;

uint16_t pollingRate;

#define EFFICIENCY_BROADCAST_RATE 2
#define VOLTAGE_CURRENT_BROADCAST_RATE 5
#define SPEED_BROADCAST_RATE 2

// RTOS Execution Loops
[[noreturn]] void CANTask() {
    pinMode(MCP2515_CS_PIN, OUTPUT);
    SPI.begin();
//    SPI.beginTransaction(SPISettings(100000, MSBFIRST, SPI_MODE3));

//    hspi2.Instance = SPI2;
//    hspi2.Init.Mode = SPI_MODE_MASTER;
//    hspi2.Init.Direction = SPI_DIRECTION_2LINES;
//    hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
//    hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
//    hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
//    hspi2.Init.NSS = SPI_NSS_SOFT;
//    hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
//    hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
//    hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
//    hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
//    hspi2.Init.CRCPolynomial = 0x0;
//    hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
//    hspi2.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
//    hspi2.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
//    hspi2.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
//    hspi2.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
//    hspi2.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
//    hspi2.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
//    hspi2.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
//    hspi2.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
//    hspi2.Init.IOSwap = SPI_IO_SWAP_DISABLE;
//    if (HAL_SPI_Init(&hspi2) != HAL_OK)
//    {
//        DebugPrint("Error SPI");
//        while (true) {}
//    }
//
//
//    GPIO_InitTypeDef GPIO_InitStruct = {0};
//    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
//        /* USER CODE BEGIN SPI2_MspInit 0 */
//
//        /* USER CODE END SPI2_MspInit 0 */
//
//        /** Initializes the peripherals clock
//        */
//        PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI2;
//        PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL;
//        if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
//        {
//            DebugPrint("Error SPI Pins");
//            while (true) {}
//        }
//
//        /* Peripheral clock enable */
//        __HAL_RCC_SPI2_CLK_ENABLE();
//
//        __HAL_RCC_GPIOI_CLK_ENABLE();
//        __HAL_RCC_GPIOC_CLK_ENABLE();
//        /**SPI2 GPIO Configuration
//        PI1     ------> SPI2_SCK
//        PC2     ------> SPI2_MISO
//        PC3     ------> SPI2_MOSI
//        */
//        GPIO_InitStruct.Pin = GPIO_PIN_1;
//        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//        GPIO_InitStruct.Pull = GPIO_NOPULL;
//        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//        GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
//        HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);
//
//        GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
//        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//        GPIO_InitStruct.Pull = GPIO_NOPULL;
//        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//        GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
//        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

        /* USER CODE BEGIN SPI2_MspInit 1 */

        /* USER CODE END SPI2_MspInit 1 */


    result_t isInitialized = IComms_Init();

//    uint8_t broadcast_voltage_current_counter = 0;
//    const ICommsMessageInfo *voltageCurrentInfo = CANMessageLookUpGetInfo(CURRENT_VOLTAGE_DATA_ID);
//
//    uint8_t speedTxCounter = 0;
//    const ICommsMessageInfo *speedInfo = CANMessageLookUpGetInfo(SPEED_DATA_ID);
//
//    uint8_t efficiencyTxCounter = 0;
//    const ICommsMessageInfo *efficiencyInfo = CANMessageLookUpGetInfo(EFFICIENCY_DATA_ID);

    while (true) {
        IComms_PeriodicReceive();

//        if (broadcast_voltage_current_counter++ > VOLTAGE_CURRENT_BROADCAST_RATE) {
//            iCommsMessage_t txMsg = IComms_CreatePairUInt16BitMessage(voltageCurrentInfo->messageID, CloudDatabase::instance.getBatteryVoltage(), CloudDatabase::instance.getBatteryCurrent());;
//            result_t _ = IComms_Transmit(&txMsg);
//            printf("Broadcast voltage and current.");
//            broadcast_voltage_current_counter = 0;
//        }
//
//        speedTxCounter++;
//        if (speedTxCounter > SPEED_BROADCAST_RATE) {
//            iCommsMessage_t speedTxMsg = IComms_CreateUint32BitMessage(speedInfo->messageID, CloudDatabase::instance.getSpeed());
//            result_t _ = IComms_Transmit(&speedTxMsg);
//            speedTxCounter = 0;
//        }
//
//        efficiencyTxCounter++;
//        if (efficiencyTxCounter > EFFICIENCY_BROADCAST_RATE) {
//            lap_efficiencies_t efficiencies;
//            CloudDatabase::instance.getLapEfficiencies(&efficiencies);
//            printf("Lap Efficiencies: %d %d %d %d\n", efficiencies.lap_0, efficiencies.lap_1, efficiencies.lap_2, efficiencies.lap_3);
//            iCommsMessage_t efficiencyTxMsg = IComms_CreateEfficiencyMessage(efficiencyInfo->messageID, &efficiencies);
//            result_t r = IComms_Transmit(&efficiencyTxMsg);
//            printf("Eff Transmission Result: %d", r);
//
//            efficiencyTxCounter = 0;
//        }

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

#if SENSOR_VOLTAGE == 1
    voltageSensor = new VoltageSensor(Wire, VoltageSensor::VoltageSensorMode::Differential_0_1, 0, 0x40, defaultBufferSize);
    voltageSensor->addListener([](const voltage_t& newValue) {
        CloudDatabase::instance.updateBatteryVoltage(newValue);
        printf("Voltage: %f", newValue);
    });
    voltageSensorTask = new PollingSensorTask<voltage_t>(voltageSensor, 200, "T_VoltageSensor", 1024 * 10, osPriorityNormal);
#endif

#if SENSOR_CURRENT == 1
    currentSensor = new VoltageSensor(Wire, VoltageSensor::VoltageSensorMode::Differential_2_3, 0, 0x40);
    currentSensor->addListener([](const voltage_t& newValue) {
        CloudDatabase::instance.updateBatteryCurrent(newValue / 12.5f);
        printf("Current: %fA\n", newValue / 12.5f);
    });
    currentSensorTask = new PollingSensorTask<voltage_t>(currentSensor, 200, "T_CurrentSensor", 1024 * 10, osPriorityNormal);
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
    CloudDatabase::instance.updateBatteryVoltage(48);

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
    speedometer = new Speedometer();

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
//    SPI.begin();
//    SPI.beginTransaction(SPISettings(100000, MSBFIRST, SPI_MODE3));
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
    DebugPrint("Throttle");

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
                break;
            case NEW_LAP:
                CloudDatabase::instance.triggerLap();
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
    int32 pressure, temperature;
    result_t r = IComms_ReadPressureTemperatureMessage(msg, &pressure, &temperature);

    if (r) {
        CloudDatabase::instance.updatePressure(pressure, temperature);
    }
}

void EfficiencyDataCallback(iCommsMessage_t *msg) {

}

#ifdef __cplusplus
}
#endif