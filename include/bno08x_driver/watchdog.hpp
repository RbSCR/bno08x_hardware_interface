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

#pragma once

#include <atomic>
#include <chrono>
#include <functional>
#include <mutex>
#include <thread>
#include <iostream>
#include <utility>

class Watchdog
{
public:
  explicit Watchdog(
      std::chrono::milliseconds timeout = std::chrono::milliseconds(1000),
      std::chrono::milliseconds check_interval = std::chrono::milliseconds(500),
      std::function<void()> callback = []() {})
    : timeout_(timeout)
    , check_interval_(check_interval)
    , callback_(callback)
    , enable_watchdog_(false)
    , last_reset_(std::chrono::steady_clock::now())
  {
    watchdog_thread_ = std::thread([this]() {
      //  pthread_setname_np(pthread_self(), "watchdog_thread");
      while (enable_watchdog_)
      {
        std::this_thread::sleep_for(check_interval_);
        auto now = std::chrono::steady_clock::now();
        std::chrono::steady_clock::time_point last_reset_cp;
        {
          std::lock_guard<std::mutex> lock(reset_mutex_);
          last_reset_cp = last_reset_;
        }
        std::function<void()> current_callback;
        {
          std::lock_guard<std::mutex> lock(callback_mutex_);
          if (enable_watchdog_ && (now - last_reset_cp) >= timeout_)
          {
            current_callback = callback_;
          } else {
            continue;
          }
        }

        if (current_callback)
        {
          current_callback();
        }
      }
    });
  }

  ~Watchdog()
  {
    enable_watchdog_ = false;
    if (watchdog_thread_.joinable())
    {
      watchdog_thread_.join();
    }
  }

  void reset()
  {
    std::lock_guard<std::mutex> lock(reset_mutex_);
    last_reset_ = std::chrono::steady_clock::now();
  }

  void stop() noexcept
  {
    enable_watchdog_ = false;
  }

  void start()
  {
    last_reset_ = std::chrono::steady_clock::now();
    enable_watchdog_ = true;
  }

  void set_timeout(std::chrono::milliseconds timeout)
  {
    timeout_ = timeout;
  }

  std::chrono::milliseconds get_timeout() const noexcept
  {
    return timeout_;
  }

  std::function<void()> get_callback() const noexcept
  {
    return callback_;
  }

  void set_callback(std::function<void()> callback)
  {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    callback_ = std::move(callback);
  }

  void set_check_interval(std::chrono::milliseconds interval)
  {
    check_interval_ = interval;
  }

  std::chrono::milliseconds get_check_interval() const noexcept
  {
    return check_interval_;
  }

private:
  std::chrono::milliseconds timeout_;
  std::chrono::milliseconds check_interval_;
  std::function<void()> callback_;
  std::atomic<bool> enable_watchdog_;
  std::mutex callback_mutex_;
  std::mutex reset_mutex_;
  std::chrono::steady_clock::time_point last_reset_;
  std::thread watchdog_thread_;
};
