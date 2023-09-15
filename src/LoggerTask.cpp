//
// Created by Jeremy Cote on 2023-09-08.
//

#include "LoggerTask.hpp"

String createFilename();
void mount();
void unmount();

static TaskHandle_t handle = NULL;

uint8_t chipSelectPin;
uint8_t signalLightPin;
uint8_t sdDetectPin;
uint8_t logButtonPin;

uint16_t logRate;

String header;
std::function<String()> constructRow;

File dataFile;
bool fileMounted = false;
String filename;


[[noreturn]] void LoggerTask(void* args) {
    while (true) {
        if (digitalRead(logButtonPin)) {
            if (fileMounted) {
                if (dataFile) {
                    String row = constructRow();
                    dataFile.println(row);
                    dataFile.flush();
                }
            } else {
                mount();
            }
        } else if (fileMounted) {
            unmount();
        }

        digitalWrite(signalLightPin, fileMounted);
        vTaskDelay(logRate);
    }
}

void LoggerInit(
        uint8_t _chipSelectPin,
        uint8_t _signalLightPin,
        uint8_t _sdDetectPin,
        uint8_t _logButtonPin,
        const std::function<String()>& _constructRow,
        const String& _header,
        uint32_t stackSize,
        UBaseType_t priority,
        uint16_t _logRate
    ) {
    chipSelectPin = _chipSelectPin;
    signalLightPin = _signalLightPin;
    sdDetectPin = _sdDetectPin;
    logButtonPin = _logButtonPin;

    constructRow = _constructRow;
    header = _header;

    logRate= _logRate;

    pinMode(chipSelectPin, OUTPUT);
    pinMode(signalLightPin, OUTPUT);
    pinMode(sdDetectPin, INPUT);
    pinMode(logButtonPin, INPUT);

    xTaskCreate(LoggerTask, "LoggerTask", stackSize, nullptr, priority, &handle);
}

// Utilities

String createFilename() {
    String f = "/logs000.csv";
    for (int i = 0; i < 1000; i++) {
        f[5] = ((i/10) / 10) % 10 + '0';
        f[6] = (i/10) % 10 + '0';
        f[7] = i%10 + '0';
        Serial.println(f);
        if (! SD.exists(f)) {
            break;  // leave the loop!
        }
    }
    return f;
}

void mount() {
    Serial.print("Mounting SD card...");

    if (digitalRead(sdDetectPin) == LOW) {
        Serial.println("No SD card in slot...");
        return;
    }

    // see if the card is present and can be initialized:
    if (!SD.begin(chipSelectPin)) {
        Serial.println("Card failed, or not present");
        fileMounted = false;
        return;
    }

    filename = createFilename();

    dataFile = SD.open(filename, FILE_APPEND);

    if (dataFile) {
        // Successfully opened file
        Serial.println("Opened file");
        dataFile.println(header);
        fileMounted = true;
    } else {
        Serial.println("Failed to open file");
        fileMounted = false;
    }
}

void unmount() {
    Serial.println("Unmounting SD Card");

    if (fileMounted) {
        dataFile.close();
        SD.end();
    }

    fileMounted = false;
}
