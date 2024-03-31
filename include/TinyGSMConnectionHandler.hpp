//
// Created by Jeremy Cote on 2024-02-11.
//

#ifndef UOSM_TELEMETRY_PIO_TINYGSMCONNECTIONHANDLER_HPP
#define UOSM_TELEMETRY_PIO_TINYGSMCONNECTIONHANDLER_HPP

#include <Arduino.h>
#include "Arduino_ConnectionHandler.h"
#include <TinyGsmClient.h>
#include <time.h>

class TinyGSMUDP: public UDP {
private:
    TinyGsmClient& _gsmClient;
    IPAddress _remoteIP;
    uint16_t _remotePort;
public:
    TinyGSMUDP(TinyGsmClient& gsmClient) : _gsmClient(gsmClient) {}

    virtual uint8_t begin(uint16_t) { return 1; }
    virtual uint8_t beginMulticast(IPAddress, uint16_t) {
        Serial.println("beginMulticast not supported");
        return 0;
    }
    virtual void stop() {}

    virtual int beginPacket(IPAddress ip, uint16_t port) {
        _remoteIP = ip;
        _remotePort = port;
        return 1;
    }
    virtual int beginPacket(const char *host, uint16_t port) {
        return beginPacket(IPAddress(host), port);
    }
    virtual int endPacket() { return 1; }
    virtual size_t write(uint8_t t) {
        return _gsmClient.write(&t, 1);
    }
    virtual size_t write(const uint8_t *buffer, size_t size) {
        return _gsmClient.write(buffer, size);
    }
    virtual int parsePacket() {
        return _gsmClient.available();
    }
    virtual int available() {
        return _gsmClient.available();
    }
    virtual int read() {
        return _gsmClient.read();
    }
    virtual int read(unsigned char* buffer, size_t len) {
        return _gsmClient.read(buffer, len);
    }
    virtual int read(char* buffer, size_t len) {
        Serial.println("read prob not supported");
        return _gsmClient.readBytes(buffer, len);
    }
    virtual int peek() {
        return _gsmClient.peek();
    }
    virtual void flush() {
        _gsmClient.flush();
    }
    virtual IPAddress remoteIP() { return _remoteIP; }
    virtual uint16_t remotePort() { return _remotePort; }
};

class TinyGSMConnectionHandler: public ConnectionHandler {
public:
    TinyGSMConnectionHandler(UART& serial, const char* pin, const char* apn, const char* login, const char* pass, bool const keep_alive = true):
        ConnectionHandler{keep_alive, NetworkAdapter::GSM},
        _pin(pin),
        _apn(apn),
        _login(login),
        _pass(pass),
        _modem(serial),
        _client(_modem, 0),
        _udp(_client),
        _serial(serial)
    {
        _serial.begin(115200);
    }

    virtual unsigned long getTime() override {
        int year, month, day, hour, minute, second;
        float timezone;

        _modem.getNetworkTime(&year, &month, &day, &hour, &minute, &second, &timezone);

        Serial.print("TIMEZONE:");
        Serial.println(timezone);


        hour += 4;
        if (hour > 24) {
            day++;
            hour -= 24;
        }

        struct tm timeinfo = {0};
        timeinfo.tm_year = year - 1900;
        timeinfo.tm_mon = month - 1;
        timeinfo.tm_mday = day;
        timeinfo.tm_hour = hour;
        timeinfo.tm_min = minute;
        timeinfo.tm_sec = second;

        time_t epoch_time = mktime(&timeinfo);

        DebugPrint("Returning Network Epoch Time");
        Serial.println(epoch_time);

        return epoch_time;
    }

    virtual Client & getClient() override { return _client; };
    virtual UDP & getUDP() override { return _udp; };

protected:
    // TODO: Implement state switching
    NetworkConnectionState update_handleInit() override {
        if (!_modem.init()) {
            return NetworkConnectionState::INIT;
        }

        // Connect to anything supported
        _modem.setNetworkMode(2);

        _modem.waitForNetwork();
        return NetworkConnectionState::CONNECTING;
    }
    NetworkConnectionState update_handleConnecting   () override {
        if (!_modem.isNetworkConnected()) {
            _modem.waitForNetwork();
            return NetworkConnectionState::CONNECTING;
        }

        if (!_modem.gprsConnect(_apn, _login, _pass)) {
            return NetworkConnectionState::CONNECTING;
        }

        return NetworkConnectionState::CONNECTED;
    }
    virtual NetworkConnectionState update_handleConnected    () override {
        if (!_modem.isNetworkConnected() || !_modem.isGprsConnected()) {
            return NetworkConnectionState::DISCONNECTED;
        }

        return NetworkConnectionState::CONNECTED;
    }
    virtual NetworkConnectionState update_handleDisconnecting() override {
        return NetworkConnectionState::DISCONNECTED;
    }
    virtual NetworkConnectionState update_handleDisconnected () override {
        return NetworkConnectionState::CONNECTING;
    }

private:
    const char * _pin;
    const char * _apn;
    const char * _login;
    const char * _pass;

    TinyGsm _modem;
    TinyGsmClient _client;
    TinyGSMUDP _udp;

    UART& _serial;
};



#endif //UOSM_TELEMETRY_PIO_TINYGSMCONNECTIONHANDLER_HPP
