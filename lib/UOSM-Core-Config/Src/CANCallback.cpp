//
// Created by Jeremy Cote on 2023-08-27.
//

#include "ApplicationTypes.h"

#ifdef __cplusplus
extern "C" {
#endif

void MotorRPMDataCallback(iCommsMessage_t* msg) {
    DebugPrint("MotorRPMDataCallback not implemented! %d", msg->standardMessageID);
}

void VoltageDataCallback(iCommsMessage_t* msg) {
    DebugPrint("VoltageDataCallback not implemented! %d", msg->standardMessageID);
}

void ThrottleDataCallback(iCommsMessage_t* msg) {
    DebugPrint("ThrottleDataCallback not implemented! %d", msg->standardMessageID);
}

void ErrorDataCallback(iCommsMessage_t* msg) {
    DebugPrint("ErrorDataCallback not implemented! %d", msg->standardMessageID);
}

void SpeedDataCallback(iCommsMessage_t* msg) {
    DebugPrint("SpeedDataCallback not implemented! %d", msg->standardMessageID);
}

void EventDataCallback(iCommsMessage_t* msg) {
    DebugPrint("EventDataCallback not implemented! %d", msg->standardMessageID);
}

void CurrentVoltageDataCallback(iCommsMessage_t* msg) {
    DebugPrint("CurrentVoltageDataCallback not implemented! %d", msg->standardMessageID);
}

#ifdef __cplusplus
}
#endif