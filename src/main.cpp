#include "VoltageSensor.hpp"
#include "Accelerometer.hpp"
#ifdef ESP32

#include <Arduino.h>
#include "PollingSensorTask.hpp"

#include "ThingProperties.h"
#include "Conversions.h"
#include "GPSSensor.hpp"

#define FONA_TX 10
#define FONA_RX 9
#define FONA_RESET 14
#define fonaSerial Serial1

#define DEFAULT_BUFFER_SIZE 10

Fona3G* fona;

VoltageSensor* voltageSensor;
Accelerometer* accelerationSensor;
GPSSensor* gpsSensor;

PollingSensorTask<voltage_t>* voltageSensorTask;
PollingSensorTask<acceleration_t>* accelerationSensorTask;
PollingSensorTask<gps_coordinate_t>* gpsSensorTask;

TaskHandle_t loggerHandle = NULL;

void loggerTask(void* args) {
    vTaskDelay(500);
}

void setup() {
    Serial.begin(115200);
    fonaSerial.begin(4800, SERIAL_8N1, FONA_TX, FONA_RX);

    fona = new Fona3G(&fonaSerial, FONA_RESET);

    voltageSensor = new VoltageSensor(DEFAULT_BUFFER_SIZE);
    accelerationSensor = new Accelerometer(DEFAULT_BUFFER_SIZE);
    gpsSensor = new GPSSensor(fona, DEFAULT_BUFFER_SIZE);

    voltageSensor->addListener([](const voltage_t& newValue) {
        updateBatteryVoltage(convertVoltageToFloat(newValue));
    });

    accelerationSensor->addListener([](const acceleration_t & newValue){
        updateAcceleration(newValue);
    });

    gpsSensor->addListener([](const gps_coordinate_t& newValue) {
        updateGPS(newValue);
    });

    // TODO: Note that sensors will throw an exception if collect is not called before get(). See if we can apply RAII
    voltageSensorTask = new PollingSensorTask<voltage_t>(voltageSensor, 200, "T_VoltageSensor", 1024 * 20, 5);
    accelerationSensorTask = new PollingSensorTask<acceleration_t>(accelerationSensor, 200, "T_AccelSensor", 1024 * 20, 5);
    gpsSensorTask = new PollingSensorTask<gps_coordinate_t>(gpsSensor, 200, "T_GPSSensor", 1024 * 20, 5);

    initProperties();
    ArduinoCloud.begin(ArduinoIoTPreferredConnection);

    Serial.println("Begin Connection");

    setDebugMessageLevel(2);
    ArduinoCloud.printDebugInfo();

   // TODO: Note that voltageSensor will throw an exception if collect is not called before get(). See if we can apply RAII
    voltageSensorTask = new PollingSensorTask<voltage_t>(voltageSensor, 200, "T_VoltageSensor", 1024 * 5, 5);

    xTaskCreate(loggerTask, "LoggerTask", 1024 * 2, nullptr, 3, &loggerHandle);
}

void loop() {
    ArduinoCloud.update();

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


/***************************************************
  This is an example for our Adafruit FONA Cellular Module

  Designed specifically to work with the Adafruit FONA
  ----> http://www.adafruit.com/products/1946
  ----> http://www.adafruit.com/products/1963
  ----> http://www.adafruit.com/products/2468
  ----> http://www.adafruit.com/products/2542

  These cellular modules use TTL Serial to communicate, 2 pins are
  required to interface
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ****************************************************/

/*
THIS CODE IS STILL IN PROGRESS!

Open up the serial console on the Arduino at 115200 baud to interact with FONA

Note that if you need to set a GPRS APN, username, and password scroll down to
the commented section below at the end of the setup() function.
*/
//#include "Adafruit_FONA.h"
//
//#define FONA_RST 12
//
//// this is a large buffer for replies
//char replybuffer[255];
//
////#if (defined(__AVR__) || defined(ESP8266)) && !defined(__AVR_ATmega2560__)
//// For UNO and others without hardware serial,
//// we default to using software serial. If you want to use hardware serial
//// (because softserial isnt supported) comment out the following three lines
//// and uncomment the HardwareSerial line
////#include <SoftwareSerial.h>
////
//#define FONA_RX 10
//#define FONA_TX 9
////
////SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
////SoftwareSerial *fonaSerial = &fonaSS;
//
////#else
////// On Leonardo/M0/etc, others with hardware serial, use hardware serial!
//HardwareSerial *fonaSerial = &Serial1;
////
////#endif
//
//// Use this for FONA 800 and 808s
////Adafruit_FONA fona = Adafruit_FONA(FONA_RST);
//// Use this one for FONA 3G
//Adafruit_FONA_3G fona = Adafruit_FONA_3G(FONA_RST);
//
//uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout = 0);
//
//uint8_t type;
//
//void flushSerial();
//void printMenu();
//uint16_t readnumber();
//
//void setup() {
////    while (!Serial);
//
//    Serial.begin(115200);
//    Serial.println(F("FONA basic test"));
//    Serial.println(F("Initializing....(May take 3 seconds)"));
//
//    fonaSerial->begin(4800, SERIAL_8N1, FONA_TX, FONA_RX);
//    if (! fona.begin(*fonaSerial)) {
//        Serial.println(F("Couldn't find FONA"));
//        while (1);
//    }
//    type = fona.type();
//    Serial.println(F("FONA is OK"));
//    Serial.print(F("Found "));
//    switch (type) {
//        case FONA800L:
//            Serial.println(F("FONA 800L")); break;
//        case FONA800H:
//            Serial.println(F("FONA 800H")); break;
//        case FONA808_V1:
//            Serial.println(F("FONA 808 (v1)")); break;
//        case FONA808_V2:
//            Serial.println(F("FONA 808 (v2)")); break;
//        case FONA3G_A:
//            Serial.println(F("FONA 3G (American)")); break;
//        case FONA3G_E:
//            Serial.println(F("FONA 3G (European)")); break;
//        default:
//            Serial.println(F("???")); break;
//    }
//
//    // Print module IMEI number.
//    char imei[16] = {0}; // MUST use a 16 character buffer for IMEI!
//    uint8_t imeiLen = fona.getIMEI(imei);
//    if (imeiLen > 0) {
//        Serial.print("Module IMEI: "); Serial.println(imei);
//    }
//
//    // Optionally configure a GPRS APN, username, and password.
//    // You might need to do this to access your network's GPRS/data
//    // network.  Contact your provider for the exact APN, username,
//    // and password values.  Username and password are optional and
//    // can be removed, but APN is required.
//    //fona.setGPRSNetworkSettings(F("your APN"), F("your username"), F("your password"));
//
//    // Optionally configure HTTP gets to follow redirects over SSL.
//    // Default is not to follow SSL redirects, however if you uncomment
//    // the following line then redirects over SSL will be followed.
//    //fona.setHTTPSRedirect(true);
//
//    printMenu();
//}
//
//void printMenu(void) {
//    Serial.println(F("-------------------------------------"));
//    Serial.println(F("[?] Print this menu"));
//    Serial.println(F("[a] read the ADC 2.8V max (FONA800 & 808)"));
//    Serial.println(F("[b] read the Battery V and % charged"));
//    Serial.println(F("[C] read the SIM CCID"));
//    Serial.println(F("[U] Unlock SIM with PIN code"));
//    Serial.println(F("[i] read RSSI"));
//    Serial.println(F("[n] get Network status"));
//    Serial.println(F("[v] set audio Volume"));
//    Serial.println(F("[V] get Volume"));
//    Serial.println(F("[H] set Headphone audio (FONA800 & 808)"));
//    Serial.println(F("[e] set External audio (FONA800 & 808)"));
//    Serial.println(F("[T] play audio Tone"));
//    Serial.println(F("[P] PWM/Buzzer out (FONA800 & 808)"));
//
//    // FM (SIM800 only!)
//    Serial.println(F("[f] tune FM radio (FONA800)"));
//    Serial.println(F("[F] turn off FM (FONA800)"));
//    Serial.println(F("[m] set FM volume (FONA800)"));
//    Serial.println(F("[M] get FM volume (FONA800)"));
//    Serial.println(F("[q] get FM station signal level (FONA800)"));
//
//    // Phone
//    Serial.println(F("[c] make phone Call"));
//    Serial.println(F("[A] get call status"));
//    Serial.println(F("[h] Hang up phone"));
//    Serial.println(F("[p] Pick up phone"));
//
//    // SMS
//    Serial.println(F("[N] Number of SMSs"));
//    Serial.println(F("[r] Read SMS #"));
//    Serial.println(F("[R] Read All SMS"));
//    Serial.println(F("[d] Delete SMS #"));
//    Serial.println(F("[s] Send SMS"));
//    Serial.println(F("[u] Send USSD"));
//
//    // Time
//    Serial.println(F("[y] Enable network time sync (FONA 800 & 808)"));
//    Serial.println(F("[Y] Enable NTP time sync (GPRS FONA 800 & 808)"));
//    Serial.println(F("[t] Get network time"));
//
//    // GPRS
//    Serial.println(F("[G] Enable GPRS"));
//    Serial.println(F("[g] Disable GPRS"));
//    Serial.println(F("[l] Query GSMLOC (GPRS)"));
//    Serial.println(F("[w] Read webpage (GPRS)"));
//    Serial.println(F("[W] Post to website (GPRS)"));
//
//    // GPS
//    if ((type == FONA3G_A) || (type == FONA3G_E) || (type == FONA808_V1) || (type == FONA808_V2)) {
//        Serial.println(F("[O] Turn GPS on (FONA 808 & 3G)"));
//        Serial.println(F("[o] Turn GPS off (FONA 808 & 3G)"));
//        Serial.println(F("[L] Query GPS location (FONA 808 & 3G)"));
//        if (type == FONA808_V1) {
//            Serial.println(F("[x] GPS fix status (FONA808 v1 only)"));
//        }
//        Serial.println(F("[E] Raw NMEA out (FONA808)"));
//    }
//
//    Serial.println(F("[S] create Serial passthru tunnel"));
//    Serial.println(F("-------------------------------------"));
//    Serial.println(F(""));
//
//}
//void loop() {
//    Serial.print(F("FONA> "));
//    while (! Serial.available() ) {
//        if (fona.available()) {
//            Serial.write(fona.read());
//        }
//    }
//
//    char command = Serial.read();
//    Serial.println(command);
//
//
//    switch (command) {
//        case '?': {
//            printMenu();
//            break;
//        }
//
//        case 'a': {
//            // read the ADC
//            uint16_t adc;
//            if (! fona.getADCVoltage(&adc)) {
//                Serial.println(F("Failed to read ADC"));
//            } else {
//                Serial.print(F("ADC = ")); Serial.print(adc); Serial.println(F(" mV"));
//            }
//            break;
//        }
//
//        case 'b': {
//            // read the battery voltage and percentage
//            uint16_t vbat;
//            if (! fona.getBattVoltage(&vbat)) {
//                Serial.println(F("Failed to read Batt"));
//            } else {
//                Serial.print(F("VBat = ")); Serial.print(vbat); Serial.println(F(" mV"));
//            }
//
//
//            if (! fona.getBattPercent(&vbat)) {
//                Serial.println(F("Failed to read Batt"));
//            } else {
//                Serial.print(F("VPct = ")); Serial.print(vbat); Serial.println(F("%"));
//            }
//
//            break;
//        }
//
//        case 'U': {
//            // Unlock the SIM with a PIN code
//            char PIN[5];
//            flushSerial();
//            Serial.println(F("Enter 4-digit PIN"));
//            readline(PIN, 3);
//            Serial.println(PIN);
//            Serial.print(F("Unlocking SIM card: "));
//            if (! fona.unlockSIM(PIN)) {
//                Serial.println(F("Failed"));
//            } else {
//                Serial.println(F("OK!"));
//            }
//            break;
//        }
//
//        case 'C': {
//            // read the CCID
//            fona.getSIMCCID(replybuffer);  // make sure replybuffer is at least 21 bytes!
//            Serial.print(F("SIM CCID = ")); Serial.println(replybuffer);
//            break;
//        }
//
//        case 'i': {
//            // read the RSSI
//            uint8_t n = fona.getRSSI();
//            int8_t r;
//
//            Serial.print(F("RSSI = ")); Serial.print(n); Serial.print(": ");
//            if (n == 0) r = -115;
//            if (n == 1) r = -111;
//            if (n == 31) r = -52;
//            if ((n >= 2) && (n <= 30)) {
//                r = map(n, 2, 30, -110, -54);
//            }
//            Serial.print(r); Serial.println(F(" dBm"));
//
//            break;
//        }
//
//        case 'n': {
//            // read the network/cellular status
//            uint8_t n = fona.getNetworkStatus();
//            Serial.print(F("Network status "));
//            Serial.print(n);
//            Serial.print(F(": "));
//            if (n == 0) Serial.println(F("Not registered"));
//            if (n == 1) Serial.println(F("Registered (home)"));
//            if (n == 2) Serial.println(F("Not registered (searching)"));
//            if (n == 3) Serial.println(F("Denied"));
//            if (n == 4) Serial.println(F("Unknown"));
//            if (n == 5) Serial.println(F("Registered roaming"));
//            break;
//        }
//
//            /*** Audio ***/
//        case 'v': {
//            // set volume
//            flushSerial();
//            if ( (type == FONA3G_A) || (type == FONA3G_E) ) {
//                Serial.print(F("Set Vol [0-8] "));
//            } else {
//                Serial.print(F("Set Vol % [0-100] "));
//            }
//            uint8_t vol = readnumber();
//            Serial.println();
//            if (! fona.setVolume(vol)) {
//                Serial.println(F("Failed"));
//            } else {
//                Serial.println(F("OK!"));
//            }
//            break;
//        }
//
//        case 'V': {
//            uint8_t v = fona.getVolume();
//            Serial.print(v);
//            if ( (type == FONA3G_A) || (type == FONA3G_E) ) {
//                Serial.println(" / 8");
//            } else {
//                Serial.println("%");
//            }
//            break;
//        }
//
//        case 'H': {
//            // Set Headphone output
//            if (! fona.setAudio(FONA_HEADSETAUDIO)) {
//                Serial.println(F("Failed"));
//            } else {
//                Serial.println(F("OK!"));
//            }
//            fona.setMicVolume(FONA_HEADSETAUDIO, 15);
//            break;
//        }
//        case 'e': {
//            // Set External output
//            if (! fona.setAudio(FONA_EXTAUDIO)) {
//                Serial.println(F("Failed"));
//            } else {
//                Serial.println(F("OK!"));
//            }
//
//            fona.setMicVolume(FONA_EXTAUDIO, 10);
//            break;
//        }
//
//        case 'T': {
//            // play tone
//            flushSerial();
//            Serial.print(F("Play tone #"));
//            uint8_t kittone = readnumber();
//            Serial.println();
//            // play for 1 second (1000 ms)
//            if (! fona.playToolkitTone(kittone, 1000)) {
//                Serial.println(F("Failed"));
//            } else {
//                Serial.println(F("OK!"));
//            }
//            break;
//        }
//
//            /*** FM Radio ***/
//
//        case 'f': {
//            // get freq
//            flushSerial();
//            Serial.print(F("FM Freq (eg 1011 == 101.1 MHz): "));
//            uint16_t station = readnumber();
//            Serial.println();
//            // FM radio ON using headset
//            if (fona.FMradio(true, FONA_HEADSETAUDIO)) {
//                Serial.println(F("Opened"));
//            }
//            if (! fona.tuneFMradio(station)) {
//                Serial.println(F("Failed"));
//            } else {
//                Serial.println(F("Tuned"));
//            }
//            break;
//        }
//        case 'F': {
//            // FM radio off
//            if (! fona.FMradio(false)) {
//                Serial.println(F("Failed"));
//            } else {
//                Serial.println(F("OK!"));
//            }
//            break;
//        }
//        case 'm': {
//            // Set FM volume.
//            flushSerial();
//            Serial.print(F("Set FM Vol [0-6]:"));
//            uint8_t vol = readnumber();
//            Serial.println();
//            if (!fona.setFMVolume(vol)) {
//                Serial.println(F("Failed"));
//            } else {
//                Serial.println(F("OK!"));
//            }
//            break;
//        }
//        case 'M': {
//            // Get FM volume.
//            int8_t fmvol = fona.getFMVolume();
//            if (fmvol < 0) {
//                Serial.println(F("Failed"));
//            } else {
//                Serial.print(F("FM volume: "));
//                Serial.println(fmvol, DEC);
//            }
//            break;
//        }
//        case 'q': {
//            // Get FM station signal level (in decibels).
//            flushSerial();
//            Serial.print(F("FM Freq (eg 1011 == 101.1 MHz): "));
//            uint16_t station = readnumber();
//            Serial.println();
//            int8_t level = fona.getFMSignalLevel(station);
//            if (level < 0) {
//                Serial.println(F("Failed! Make sure FM radio is on (tuned to station)."));
//            } else {
//                Serial.print(F("Signal level (dB): "));
//                Serial.println(level, DEC);
//            }
//            break;
//        }
//
//            /*** PWM ***/
//
//        case 'P': {
//            // PWM Buzzer output @ 2KHz max
//            flushSerial();
//            Serial.print(F("PWM Freq, 0 = Off, (1-2000): "));
//            uint16_t freq = readnumber();
//            Serial.println();
//            if (! fona.setPWM(freq)) {
//                Serial.println(F("Failed"));
//            } else {
//                Serial.println(F("OK!"));
//            }
//            break;
//        }
//
//            /*** Call ***/
//        case 'c': {
//            // call a phone!
//            char number[30];
//            flushSerial();
//            Serial.print(F("Call #"));
//            readline(number, 30);
//            Serial.println();
//            Serial.print(F("Calling ")); Serial.println(number);
//            if (!fona.callPhone(number)) {
//                Serial.println(F("Failed"));
//            } else {
//                Serial.println(F("Sent!"));
//            }
//
//            break;
//        }
//        case 'A': {
//            // get call status
//            int8_t callstat = fona.getCallStatus();
//            switch (callstat) {
//                case 0: Serial.println(F("Ready")); break;
//                case 1: Serial.println(F("Could not get status")); break;
//                case 3: Serial.println(F("Ringing (incoming)")); break;
//                case 4: Serial.println(F("Ringing/in progress (outgoing)")); break;
//                default: Serial.println(F("Unknown")); break;
//            }
//            break;
//        }
//
//        case 'h': {
//            // hang up!
//            if (! fona.hangUp()) {
//                Serial.println(F("Failed"));
//            } else {
//                Serial.println(F("OK!"));
//            }
//            break;
//        }
//
//        case 'p': {
//            // pick up!
//            if (! fona.pickUp()) {
//                Serial.println(F("Failed"));
//            } else {
//                Serial.println(F("OK!"));
//            }
//            break;
//        }
//
//            /*** SMS ***/
//
//        case 'N': {
//            // read the number of SMS's!
//            int8_t smsnum = fona.getNumSMS();
//            if (smsnum < 0) {
//                Serial.println(F("Could not read # SMS"));
//            } else {
//                Serial.print(smsnum);
//                Serial.println(F(" SMS's on SIM card!"));
//            }
//            break;
//        }
//        case 'r': {
//            // read an SMS
//            flushSerial();
//            Serial.print(F("Read #"));
//            uint8_t smsn = readnumber();
//            Serial.print(F("\n\rReading SMS #")); Serial.println(smsn);
//
//            // Retrieve SMS sender address/phone number.
//            if (! fona.getSMSSender(smsn, replybuffer, 250)) {
//                Serial.println("Failed!");
//                break;
//            }
//            Serial.print(F("FROM: ")); Serial.println(replybuffer);
//
//            // Retrieve SMS value.
//            uint16_t smslen;
//            if (! fona.readSMS(smsn, replybuffer, 250, &smslen)) { // pass in buffer and max len!
//                Serial.println("Failed!");
//                break;
//            }
//            Serial.print(F("***** SMS #")); Serial.print(smsn);
//            Serial.print(" ("); Serial.print(smslen); Serial.println(F(") bytes *****"));
//            Serial.println(replybuffer);
//            Serial.println(F("*****"));
//
//            break;
//        }
//        case 'R': {
//            // read all SMS
//            int8_t smsnum = fona.getNumSMS();
//            uint16_t smslen;
//            int8_t smsn;
//
//            if ( (type == FONA3G_A) || (type == FONA3G_E) ) {
//                smsn = 0; // zero indexed
//                smsnum--;
//            } else {
//                smsn = 1;  // 1 indexed
//            }
//
//            for ( ; smsn <= smsnum; smsn++) {
//                Serial.print(F("\n\rReading SMS #")); Serial.println(smsn);
//                if (!fona.readSMS(smsn, replybuffer, 250, &smslen)) {  // pass in buffer and max len!
//                    Serial.println(F("Failed!"));
//                    break;
//                }
//                // if the length is zero, its a special case where the index number is higher
//                // so increase the max we'll look at!
//                if (smslen == 0) {
//                    Serial.println(F("[empty slot]"));
//                    smsnum++;
//                    continue;
//                }
//
//                Serial.print(F("***** SMS #")); Serial.print(smsn);
//                Serial.print(" ("); Serial.print(smslen); Serial.println(F(") bytes *****"));
//                Serial.println(replybuffer);
//                Serial.println(F("*****"));
//            }
//            break;
//        }
//
//        case 'd': {
//            // delete an SMS
//            flushSerial();
//            Serial.print(F("Delete #"));
//            uint8_t smsn = readnumber();
//
//            Serial.print(F("\n\rDeleting SMS #")); Serial.println(smsn);
//            if (fona.deleteSMS(smsn)) {
//                Serial.println(F("OK!"));
//            } else {
//                Serial.println(F("Couldn't delete"));
//            }
//            break;
//        }
//
//        case 's': {
//            // send an SMS!
//            char sendto[21], message[141];
//            flushSerial();
//            Serial.print(F("Send to #"));
//            readline(sendto, 20);
//            Serial.println(sendto);
//            Serial.print(F("Type out one-line message (140 char): "));
//            readline(message, 140);
//            Serial.println(message);
//            if (!fona.sendSMS(sendto, message)) {
//                Serial.println(F("Failed"));
//            } else {
//                Serial.println(F("Sent!"));
//            }
//
//            break;
//        }
//
//        case 'u': {
//            // send a USSD!
//            char message[141];
//            flushSerial();
//            Serial.print(F("Type out one-line message (140 char): "));
//            readline(message, 140);
//            Serial.println(message);
//
//            uint16_t ussdlen;
//            if (!fona.sendUSSD(message, replybuffer, 250, &ussdlen)) { // pass in buffer and max len!
//                Serial.println(F("Failed"));
//            } else {
//                Serial.println(F("Sent!"));
//                Serial.print(F("***** USSD Reply"));
//                Serial.print(" ("); Serial.print(ussdlen); Serial.println(F(") bytes *****"));
//                Serial.println(replybuffer);
//                Serial.println(F("*****"));
//            }
//            break;
//        }
//
//            /*** Time ***/
//
//        case 'y': {
//            // enable network time sync
//            if (!fona.enableNetworkTimeSync(true))
//                Serial.println(F("Failed to enable"));
//            break;
//        }
//
//        case 'Y': {
//            // enable NTP time sync
//            if (!fona.enableNTPTimeSync(true, F("pool.ntp.org")))
//                Serial.println(F("Failed to enable"));
//            break;
//        }
//
//        case 't': {
//            // read the time
//            char buffer[23];
//
//            fona.getTime(buffer, 23);  // make sure replybuffer is at least 23 bytes!
//            Serial.print(F("Time = ")); Serial.println(buffer);
//            break;
//        }
//
//
//            /*********************************** GPS (SIM808 only) */
//
//        case 'o': {
//            // turn GPS off
//            if (!fona.enableGPS(false))
//                Serial.println(F("Failed to turn off"));
//            break;
//        }
//        case 'O': {
//            // turn GPS on
//            if (!fona.enableGPS(true))
//                Serial.println(F("Failed to turn on"));
//            break;
//        }
//        case 'x': {
//            int8_t stat;
//            // check GPS fix
//            stat = fona.GPSstatus();
//            if (stat < 0)
//                Serial.println(F("Failed to query"));
//            if (stat == 0) Serial.println(F("GPS off"));
//            if (stat == 1) Serial.println(F("No fix"));
//            if (stat == 2) Serial.println(F("2D fix"));
//            if (stat == 3) Serial.println(F("3D fix"));
//            break;
//        }
//
//        case 'L': {
//            // check for GPS location
//            char gpsdata[120];
//            fona.getGPS(0, gpsdata, 120);
//            if (type == FONA808_V1)
//                Serial.println(F("Reply in format: mode,longitude,latitude,altitude,utctime(yyyymmddHHMMSS),ttff,satellites,speed,course"));
//            else
//                Serial.println(F("Reply in format: mode,fixstatus,utctime(yyyymmddHHMMSS),latitude,longitude,altitude,speed,course,fixmode,reserved1,HDOP,PDOP,VDOP,reserved2,view_satellites,used_satellites,reserved3,C/N0max,HPA,VPA"));
//            Serial.println(gpsdata);
//
//            break;
//        }
//
//        case 'E': {
//            flushSerial();
//            if (type == FONA808_V1) {
//                Serial.print(F("GPS NMEA output sentences (0 = off, 34 = RMC+GGA, 255 = all)"));
//            } else {
//                Serial.print(F("On (1) or Off (0)? "));
//            }
//            uint8_t nmeaout = readnumber();
//
//            // turn on NMEA output
//            fona.enableGPSNMEA(nmeaout);
//
//            break;
//        }
//
//            /*********************************** GPRS */
//
//        case 'g': {
//            // turn GPRS off
//            if (!fona.enableGPRS(false))
//                Serial.println(F("Failed to turn off"));
//            break;
//        }
//        case 'G': {
//            // turn GPRS on
//            if (!fona.enableGPRS(true))
//                Serial.println(F("Failed to turn on"));
//            break;
//        }
//        case 'l': {
//            // check for GSMLOC (requires GPRS)
//            uint16_t returncode;
//
//            if (!fona.getGSMLoc(&returncode, replybuffer, 250))
//                Serial.println(F("Failed!"));
//            if (returncode == 0) {
//                Serial.println(replybuffer);
//            } else {
//                Serial.print(F("Fail code #")); Serial.println(returncode);
//            }
//
//            break;
//        }
//        case 'w': {
//            // read website URL
//            uint16_t statuscode;
//            int16_t length;
//            char url[80];
//
//            flushSerial();
//            Serial.println(F("NOTE: in beta! Use small webpages to read!"));
//            Serial.println(F("URL to read (e.g. wifitest.adafruit.com/testwifi/index.html):"));
//            Serial.print(F("http://")); readline(url, 79);
//            Serial.println(url);
//
//            Serial.println(F("****"));
//            if (!fona.HTTP_GET_start(url, &statuscode, (uint16_t *)&length)) {
//                Serial.println("Failed!");
//                break;
//            }
//            while (length > 0) {
//                while (fona.available()) {
//                    char c = fona.read();
//
//                    // Serial.write is too slow, we'll write directly to Serial register!
//#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
//                    loop_until_bit_is_set(UCSR0A, UDRE0); /* Wait until data register empty. */
//            UDR0 = c;
//#else
//                    Serial.write(c);
//#endif
//                    length--;
//                    if (! length) break;
//                }
//            }
//            Serial.println(F("\n****"));
//            fona.HTTP_GET_end();
//            break;
//        }
//
//        case 'W': {
//            // Post data to website
//            uint16_t statuscode;
//            int16_t length;
//            char url[80];
//            char data[80];
//
//            flushSerial();
//            Serial.println(F("NOTE: in beta! Use simple websites to post!"));
//            Serial.println(F("URL to post (e.g. httpbin.org/post):"));
//            Serial.print(F("http://")); readline(url, 79);
//            Serial.println(url);
//            Serial.println(F("Data to post (e.g. \"foo\" or \"{\"simple\":\"json\"}\"):"));
//            readline(data, 79);
//            Serial.println(data);
//
//            Serial.println(F("****"));
//            if (!fona.HTTP_POST_start(url, F("text/plain"), (uint8_t *) data, strlen(data), &statuscode, (uint16_t *)&length)) {
//                Serial.println("Failed!");
//                break;
//            }
//            while (length > 0) {
//                while (fona.available()) {
//                    char c = fona.read();
//
//#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
//                    loop_until_bit_is_set(UCSR0A, UDRE0); /* Wait until data register empty. */
//            UDR0 = c;
//#else
//                    Serial.write(c);
//#endif
//
//                    length--;
//                    if (! length) break;
//                }
//            }
//            Serial.println(F("\n****"));
//            fona.HTTP_POST_end();
//            break;
//        }
//            /*****************************************/
//
//        case 'S': {
//            Serial.println(F("Creating SERIAL TUBE"));
//            while (1) {
//                while (Serial.available()) {
//                    delay(1);
//                    fona.write(Serial.read());
//                }
//                if (fona.available()) {
//                    Serial.write(fona.read());
//                }
//            }
//            break;
//        }
//
//        default: {
//            Serial.println(F("Unknown command"));
//            printMenu();
//            break;
//        }
//    }
//    // flush input
//    flushSerial();
//    while (fona.available()) {
//        Serial.write(fona.read());
//    }
//
//}
//
//void flushSerial() {
//    while (Serial.available())
//        Serial.read();
//}
//
//char readBlocking() {
//    while (!Serial.available());
//    return Serial.read();
//}
//uint16_t readnumber() {
//    uint16_t x = 0;
//    char c;
//    while (! isdigit(c = readBlocking())) {
//        //Serial.print(c);
//    }
//    Serial.print(c);
//    x = c - '0';
//    while (isdigit(c = readBlocking())) {
//        Serial.print(c);
//        x *= 10;
//        x += c - '0';
//    }
//    return x;
//}
//
//uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout) {
//    uint16_t buffidx = 0;
//    boolean timeoutvalid = true;
//    if (timeout == 0) timeoutvalid = false;
//
//    while (true) {
//        if (buffidx > maxbuff) {
//            //Serial.println(F("SPACE"));
//            break;
//        }
//
//        while (Serial.available()) {
//            char c =  Serial.read();
//
//            //Serial.print(c, HEX); Serial.print("#"); Serial.println(c);
//
//            if (c == '\r') continue;
//            if (c == 0xA) {
//                if (buffidx == 0)   // the first 0x0A is ignored
//                    continue;
//
//                timeout = 0;         // the second 0x0A is the end of the line
//                timeoutvalid = true;
//                break;
//            }
//            buff[buffidx] = c;
//            buffidx++;
//        }
//
//        if (timeoutvalid && timeout == 0) {
//            //Serial.println(F("TIMEOUT"));
//            break;
//        }
//        delay(1);
//    }
//    buff[buffidx] = 0;  // null term
//    return buffidx;
//}