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

#ifndef BNO08X_DRIVER__BNO08X_HPP_
#define BNO08X_DRIVER__BNO08X_HPP_

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <functional>
#include <iostream>
#include "bno08x_driver/comm_interface.hpp"
#include "bno08x_driver/logger.h"
#include "sh2/sh2.h"
#include "sh2/sh2_SensorValue.h"
#include "sh2/sh2_err.h"

/* Additional Activities not listed in SH-2 lib */
#define PAC_ON_STAIRS 8     ///< Activity code for being on stairs
#define PAC_OPTION_COUNT 9  ///< The number of current options for the activity classifier

/**
 * @brief  Class that stores state and functions for interacting with
 *         the BNO08x 9-DOF Orientation IMU Fusion Breakout
 */
class BNO08x
{
public:
  BNO08x(
    CommInterface * comm, std::function<void(void *, sh2_SensorValue_t *)> sensor_callback,
    void * cookie);
  ~BNO08x();

  bool begin(int32_t sensor_id = 0);
  void hardware_reset(void);
  bool was_reset(void);
  bool enable_report(sh2_SensorId_t sensor, uint32_t interval_us = 10000);
  void poll(void);

  // Set reorientation (i.e. axis remap) of the sensor
  //   Contributed by githhub user LazaroFilm (Victor Lazaro) Nov 2023.
  //   https://github.com/sparkfun/SparkFun_BNO08x_Arduino_Library/pull/15
  bool setReorientation(sh2_Quaternion_t * pOrientation);

  sh2_ProductIds_t prodIds;  ///< The product IDs returned by the sensor

protected:
  bool init(int32_t sensor_id);
  sh2_Hal_t HAL_;  // The struct representing the SH2 Hardware Abstraction Layer

private:
  static inline void sensor_event_callback(void * cookie, sh2_SensorEvent_t * event);
  static void hal_callback(void * cookie, sh2_AsyncEvent_t * pEvent);
  static inline int open_wrapper(sh2_Hal_t * HAL);
  static inline void close_wrapper(sh2_Hal_t * HAL);
  static inline int read_wrapper(sh2_Hal_t * HAL, uint8_t * pBuffer, unsigned len, uint32_t * t_us);
  static inline int write_wrapper(sh2_Hal_t * HAL, uint8_t * pBuffer, unsigned len);
  CommInterface * comm_;
  void * cookie_;
  std::function<void(void *, sh2_SensorValue_t *)> host_callback_;
  bool reset_occurred_;
};

#endif  // BNO08X_DRIVER__BNO08X_HPP_
