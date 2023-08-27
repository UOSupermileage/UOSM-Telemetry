//
// Created by Jeremy Cote on 2023-08-27.
//
#include "UOSMCoreConfig.h"

#include <Arduino.h>
#include <SPI.h>

#ifdef __cplusplus
extern "C" {
#endif

static char messageBuf[MAX_SERIAL_PRINT_LENGTH];

void GPIO_DigitalWrite(uint8_t pin, uint8_t val) {
    digitalWrite(pin, val);
}

void SPI_Transfer(void *data, uint32_t size) {
    SPI.transfer(data, size);
}

void ExternalSerialPrint(const char * message, ...) {
    va_list args;
    va_start(args, message);
    Serial.printf(message, args);
    va_end(args);
}
void ExternalSerialPrintln(const char * message, ...) {
    va_list args;
    va_start(args, message);
    uint16_t len = vsprintf(messageBuf, message, args);
    messageBuf[len] = '\n';
    messageBuf[len+1] = '\r';
    Serial.printf(messageBuf, args);
    va_end(args);
}

#ifdef __cplusplus
}
#endif