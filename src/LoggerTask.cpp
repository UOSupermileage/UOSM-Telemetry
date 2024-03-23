//
// Created by Jeremy Cote on 2023-09-08.
//
//
#include "LoggerTask.hpp"
#include "ApplicationTypes.h"
#include <rtos.h>

#define SD_CHIP_SELECT_PIN PIN_A0

#define SDCARD_MOSI_PIN 8
#define SDCARD_MISO_PIN 10
#define SDCARD_SCK_PIN 9

#include <SD.h>

String createFilename();
static rtos::Thread loggerThread( OS_STACK_SIZE * 4, osPriorityHigh);

uint16_t logRate;

String header;
std::function<String()> constructRow;

[[noreturn]] void LoggerTask() {
    pinMode(SD_CHIP_SELECT_PIN, OUTPUT);
    DebugPrint("Starting LoggerTask");

    SD.begin(SD_CHIP_SELECT_PIN);

    String filename = createFilename();
    File fp;
    bool wroteHeader = false;

    while (true) {
        DebugPrint("Executing SD Loop");

//        fp = SD.open(filename, FILE_WRITE);
//        fp.close();
//
//        if (fp) {
//            if (!wroteHeader) {
//                fp.println(header.c_str());
//                wroteHeader = true;
//            }
//
//            fp.println(constructRow().c_str());
//            fp.close();
//
//            DebugPrint(constructRow().c_str());
//        }

        rtos::ThisThread::sleep_for(std::chrono::milliseconds(300));
    }
}

void LoggerInit(
        uint16_t _logRate,
        const std::function<String()>& _constructRow,
        const String& _header
    ) {

    constructRow = _constructRow;
    header = _header;

    logRate= _logRate;

    DebugPrint("Starting Logger Thread");
    loggerThread.start(mbed::callback(LoggerTask));
}

// Utilities

String createFilename() {
    uint16_t i;
    String f = "/sdcard/logs000.csv";
    for (i = 0; i < 1000; i++) {
        f[12] = ((i/10) / 10) % 10 + '0';
        f[13] = (i/10) % 10 + '0';
        f[14] = i%10 + '0';

        File fp = SD.open(f, FILE_READ);
        if (fp) {
            fp.close();
            break; //   leave the loop!
        }
        fp.close();
    }

    DebugPrint(f.c_str());

    return f;
}