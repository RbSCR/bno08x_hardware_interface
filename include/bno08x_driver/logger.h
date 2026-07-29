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

#ifndef BNO08X_DRIVER__LOGGER_H_
#define BNO08X_DRIVER__LOGGER_H_

#include <unistd.h>

#include <iostream>
#include <cstring>
#include <sstream>

// Define DEBUG_LOG_ENABLED to enable debug logs at compile time
// #define DEBUG_LOG_ENABLED

#ifdef DEBUG_LOG_ENABLED
#define DEBUG_ONLY(code)                                                   \
  do  \
  {                                                                        \
    std::cout << "DEBUG: " << std::endl;                                   \
    code                                                                   \
  } while (0)  // End DEBUG_ONLY

#define DEBUG_LOG(message)                                                 \
  do                                                                       \
  {                                                                        \
    std::ostringstream oss;                                                \
    oss << message;                                                        \
    std::cout << "DEBUG: " << oss.str() << std::endl;                      \
  } while (0)  // End DEBUG_LOG

#define DEBUG_LOG_BUFFER(buffer, length)                                   \
  do                                                                       \
  {                                                                        \
    std::cout << "DEBUG BUFFER: ";                                         \
    for (size_t i = 0; i < length; ++i)                                    \
    {                                                                      \
      std::cout << std::hex << static_cast<int>(buffer[i]) << " ";         \
    }                                                                      \
    std::cout << std::dec << std::endl; /* Reset to default formatting */  \
  } while (0)  // End DEBUG_LOG_BUFFER
#else
// When DEBUG_LOG_ENABLED is not defined, compile out debug logs
#define DEBUG_ONLY(code)                                                    \
  do                                                                        \
  {                                                                         \
  } while (0)  // End DEBUG_ONLY (No-op)
#define DEBUG_LOG(message)                                                  \
  do                                                                        \
  {                                                                         \
  } while (0)  // End DEBUG_LOG (No-op)
#define DEBUG_LOG_BUFFER(buffer, length)                                    \
  do                                                                        \
  {  \
  } while (0)  // End DEBUG_LOG_BUFFER (No-op)
#endif

#endif  // BNO08X_DRIVER__LOGGER_H_
