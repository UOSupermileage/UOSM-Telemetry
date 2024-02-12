//
// Created by Jeremy Cote on 2024-02-11.
//

#ifndef UOSM_TELEMETRY_PIO_TINYGSMCONNECTIONHANDLER_HPP
#define UOSM_TELEMETRY_PIO_TINYGSMCONNECTIONHANDLER_HPP

#include <Arduino.h>
#include "Arduino_ConnectionHandler.h"
#include <TinyGsmClient.h>

class TinyGSMUDP: public UDP {
public:
    virtual uint8_t begin(uint16_t) { return 0; }
    virtual uint8_t beginMulticast(IPAddress, uint16_t) { return 0; }
    virtual void stop() {}

    virtual int beginPacket(IPAddress ip, uint16_t port) { return 0; }
    virtual int beginPacket(const char *host, uint16_t port) { return 0; }
    virtual int endPacket() { return 0; }
    virtual size_t write(uint8_t) { return 0; }
    virtual size_t write(const uint8_t *buffer, size_t size) { return 0; }
    virtual int parsePacket() { return 0; }
    virtual int available() { return 0; }
    virtual int read() { return 0; }
    virtual int read(unsigned char* buffer, size_t len) { return 0; }
    virtual int read(char* buffer, size_t len) { return 0; }
    virtual int peek() { return 0; }
    virtual void flush() {}
    virtual IPAddress remoteIP() { return {}; }
    virtual uint16_t remotePort() { return 0; }
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
        _client(_modem, 0)
    {}

    virtual unsigned long getTime() override {
        // TODO: Implement this method
        return 0;
    }

    virtual Client & getClient() override { return _client; };
    virtual UDP & getUDP() override { return _udp; };

protected:
    // TODO: Implement state switching
    virtual NetworkConnectionState update_handleInit         () override { return NetworkConnectionState::INIT; }
    virtual NetworkConnectionState update_handleConnecting   () override { return NetworkConnectionState::INIT; }
    virtual NetworkConnectionState update_handleConnected    () override { return NetworkConnectionState::INIT; }
    virtual NetworkConnectionState update_handleDisconnecting() override { return NetworkConnectionState::INIT; }
    virtual NetworkConnectionState update_handleDisconnected () override { return NetworkConnectionState::INIT; }

private:
    const char * _pin;
    const char * _apn;
    const char * _login;
    const char * _pass;

    TinyGsm _modem;
    TinyGsmClient _client;
    TinyGSMUDP _udp;
};



#endif //UOSM_TELEMETRY_PIO_TINYGSMCONNECTIONHANDLER_HPP
