//
// Created by Jeremy Cote on 2024-03-04.
//

#ifndef UOSM_TELEMETRY_PIO_CONFIG_H
#define UOSM_TELEMETRY_PIO_CONFIG_H

#define SENSOR_GPS 0
#define SENSOR_VOLTAGE 0
#define SENSOR_CURRENT 1
#define SENSOR_ACCELEROMETER 0
#define SENSOR_CAN_LOG 0
#define SENSOR_THROTTLE 1
#define SENSOR_SPEEDOMETER 1
#define SENSOR_RPM 1

#define BROADCAST_CAN 1

#define ENABLE_CAN (SENSOR_CAN_LOG | SENSOR_THROTTLE | SENSOR_RPM | BROADCAST_CAN)

#define LOGGER_SD 0
#define LOGGER_IOT 1

#endif //UOSM_TELEMETRY_PIO_CONFIG_H
