//
// Created by Jeremy Cote on 2023-09-15.
//

#include "CANTask.hpp"
#include "UOSMCoreConfig.h"

#include "InternalCommsModule.h"
#include "Arduino_PortentaBreakout.h"
#include "CANMessageLookUpModule.h"
#include <Arduino.h>
#include "ThingProperties.hpp"
#include <rtos.h>

// RTOS Handles
static rtos::Thread canThread;

uint16_t pollingRate;

#define EFFICIENCY_BROADCAST_RATE 10
#define VOLTAGE_CURRENT_BROADCAST_RATE 5
#define SPEED_BROADCAST_RATE 2

// RTOS Execution Loops
[[noreturn]] void CANTask() {
    pinMode(MCP2515_CS_PIN, OUTPUT);

    printf("Starting CANTask");
    SPI.begin();
    SPI.beginTransaction(SPISettings(100000, MSBFIRST, SPI_MODE0));

    result_t isInitialized = RESULT_FAIL;

    uint8_t broadcast_voltage_current_counter = 0;
    const ICommsMessageInfo *voltageCurrentInfo = CANMessageLookUpGetInfo(CURRENT_VOLTAGE_DATA_ID);

    uint8_t speedTxCounter = 0;
    const ICommsMessageInfo *speedInfo = CANMessageLookUpGetInfo(SPEED_DATA_ID);

    uint8_t efficiencyTxCounter = 0;
    const ICommsMessageInfo *efficiencyInfo = CANMessageLookUpGetInfo(EFFICIENCY_DATA_ID);

    while (true) {
        if (isInitialized == RESULT_FAIL) {
            printf("Initializing CAN Hardware");
            isInitialized = IComms_Init();
        } else {
            printf("Periodic Receive");
            IComms_PeriodicReceive();
        }

        if (broadcast_voltage_current_counter++ > VOLTAGE_CURRENT_BROADCAST_RATE) {
            iCommsMessage_t txMsg = IComms_CreatePairUInt16BitMessage(voltageCurrentInfo->messageID, CloudDatabase::instance.getBatteryVoltage(), CloudDatabase::instance.getBatteryCurrent());;
            result_t _ = IComms_Transmit(&txMsg);
            printf("Broadcast voltage and current.");
            broadcast_voltage_current_counter = 0;
        }

        speedTxCounter++;
        if (speedTxCounter > SPEED_BROADCAST_RATE) {
            iCommsMessage_t speedTxMsg = IComms_CreateUint32BitMessage(speedInfo->messageID, CloudDatabase::instance.getSpeed());
            result_t _ = IComms_Transmit(&speedTxMsg);
            speedTxCounter = 0;
        }

        efficiencyTxCounter++;
        if (efficiencyTxCounter > EFFICIENCY_BROADCAST_RATE) {
            lap_efficiencies_t efficiencies;
            CloudDatabase::instance.getLapEfficiencies(&efficiencies);
            iCommsMessage_t efficiencyTxMsg = IComms_CreateEfficiencyMessage(speedInfo->messageID, &efficiencies);
            result_t _ = IComms_Transmit(&efficiencyTxMsg);

            speedTxCounter = 0;
        }

        rtos::ThisThread::sleep_for(std::chrono::milliseconds(pollingRate));
    }
}

void CANInit(
        uint32_t stackSize,
        osPriority_t priority,
        uint16_t _pollingRate
) {
    printf("CANInit");
    pollingRate = _pollingRate;
    canThread.start(mbed::callback(CANTask));
}

