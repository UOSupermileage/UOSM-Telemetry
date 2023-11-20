//
// Created by Jeremy Cote on 2023-08-27.
//
#include "../Inc/UOSMCoreConfig.h"

#ifdef ESP32
#include <Arduino.h>
#include <SPI.h>
#elif MBED
#include <Arduino.h>
#include <Arduino_PortentaBreakout.h>
#include <LibPrintf.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

static char messageBuf[MAX_SERIAL_PRINT_LENGTH];

void GPIO_DigitalWrite(uint8_t pin, uint8_t val) {
#ifdef ESP32
    digitalWrite(pin, val);
#elif MBED
    Breakout.digitalWrite((breakoutPin) pin, (PinStatus) val);
#endif
}

void SPI_Transfer(void *data, uint32_t size) {
#ifdef ESP32
    SPI.transfer(data, size);
#elif MBED
    Breakout.SPI_0.transfer(data, size);
#endif
}

void ExternalSerialPrint(const char * message, ...) {
#ifdef ESP32
    va_list args;
    va_start(args, message);
    Serial.printf(message, args);
    va_end(args);
#elif MBED
    va_list args;
    va_start(args, message);
//    sprintf(messageBuf, message, args);
    printf(message, args);
//    Serial.print(message);
    va_end(args);
#endif
}
void ExternalSerialPrintln(const char * message, ...) {
#ifdef ESP32
    va_list args;
    va_start(args, message);
    uint16_t len = vsprintf(messageBuf, message, args);
    messageBuf[len] = '\n';
    messageBuf[len+1] = '\r';
    Serial.printf(messageBuf, args);
    va_end(args);
#elif MBED
    va_list args;
    va_start(args, message);
//    printf(message, args);
//    printf("\n");
    uint16_t len = vsprintf(messageBuf, message, args);
    messageBuf[len] = '\n';
    messageBuf[len+1] = '\r';
    printf(messageBuf, args);
    va_end(args);
#endif
}

#ifdef __cplusplus
}
#endif