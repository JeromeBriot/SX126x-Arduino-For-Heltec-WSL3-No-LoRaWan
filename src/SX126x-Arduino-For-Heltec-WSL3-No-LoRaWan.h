/**
 * @file SX126x-Arduino.h
 *
 * @author Bernd Giesecke
 * @author Insight SIP
 * @author SEMTECH S.A.
 *
 * Arduino library for LoRa communication with Semtech SX126x chips.
 * It is based on Semtech's SX126x libraries and adapted to the Arduino framework for ESP32,
 * ESP8266 and nRF52832. It will not work with other uC's like AVR.
 *
 * \example   DeepSleep\DeepSleep.ino
 * \example   PingPong\PingPong.ino
 * \example   Sensor-Gateway-Deepsleep\LoRa-Gateway\src\main.cpp
 * \example   Sensor-Gateway-Deepsleep\LoRa-TempSensor\src\main.cpp
 */
#ifndef _SX126X_ARDUINO_HELTEC_WSL3_NO_LORAWAN_H
#define _SX126X_ARDUINO_HELTEC_WSL3_NO_LORAWAN_H

#include "boards/mcu/board.h"
#include "radio/radio.h"

// If not on PIO or not defined in platformio.ini
#ifndef LIB_DEBUG
// Debug output set to 0 to disable app debug output
#define LIB_DEBUG 0
#endif

#if LIB_DEBUG > 0
#define LOG_LIB(tag, ...)                \
    do                                   \
    {                                    \
        if (tag)                         \
            Serial.printf("<%s> ", tag); \
        Serial.printf(__VA_ARGS__);      \
        Serial.printf("\n");             \
    } while (0)
#else
#define LOG_LIB(...)
#endif

#endif // _SX126X_ARDUINO_HELTEC_WSL3_NO_LORAWAN_H