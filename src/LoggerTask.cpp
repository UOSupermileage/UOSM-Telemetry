//
// Created by Jeremy Cote on 2023-09-08.
//
//
#include "LoggerTask.hpp"
#include "ApplicationTypes.h"
#include <rtos.h>
#include <Arduino_POSIXStorage.h>

String createFilename();
static rtos::Thread loggerThread;

uint16_t logRate;

String header;
std::function<String()> constructRow;

[[noreturn]] void LoggerTask() {
    DebugPrint("Starting LoggerTask");

    String filename;
    FILE *fp;
    bool wroteHeader = false;

    while (true) {
        DebugPrint("Executing SD Loop");
        if (mount(DEV_SDCARD, FS_FAT, MNT_DEFAULT) == 0) {
            if (!wroteHeader) {
                filename = createFilename();
            }

            DebugPrint("Writing to file: [%s]", filename.c_str());
            fp = fopen(filename.c_str(), "a");

            if (fp != nullptr) {
                if (!wroteHeader) {
                    fprintf(fp, "%s", header.c_str());
                    wroteHeader = true;
                } else {
                    fprintf(fp, "%s", constructRow().c_str());
                }
            }

            fclose(fp);
        } else {
            DebugPrint("Failed to mount SD Card");
        }

        umount(DEV_SDCARD);
        rtos::ThisThread::sleep_for(std::chrono::milliseconds(logRate));
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

        FILE *fp = fopen(f.c_str(), "r");
        if (fp == nullptr) {
            fclose(fp);
            break; //   leave the loop!
        }
        fclose(fp);
    }

    printf("Available filename at index: 0x%04x", i);
    DebugPrint("Available filename at index: 0x%04x", i);

    return f;
}