// Copyright 2025 Balachandra Bhat (https://github.com/bnbhat)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef BNO08X_DRIVER__COMM_INTERFACE_HPP_
#define BNO08X_DRIVER__COMM_INTERFACE_HPP_

#include <sys/time.h>

#include <cstdint>

#include "bno08x_driver/logger.h"
/**
 * @brief Interface for the communication with the BNO08x sensor
 * This class defines the interface for the communication with the BNO08x sensor.
 * It is used by the BNO08x class to abstract
 * the communication with the sensor, so that the BNO08x class can be used with
 * different communication interfaces (I2C, UART, SPI).
 */
class CommInterface {
public:
    virtual ~CommInterface() {}
    virtual int open() = 0;
    virtual void close() = 0;
    virtual int read(uint8_t *pBuffer, unsigned len, uint32_t *t_us) = 0;
    virtual int write(uint8_t *pBuffer, unsigned len) = 0;

    uint32_t getTimeUs() {
        struct timeval tv;
        gettimeofday(&tv, NULL);
        uint32_t t = tv.tv_sec * 1000000 + tv.tv_usec;
        return t;
    }
};

#endif  // BNO08X_DRIVER__COMM_INTERFACE_HPP_
