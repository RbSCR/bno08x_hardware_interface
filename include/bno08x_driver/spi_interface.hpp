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

#ifndef BNO08X_DRIVER__SPI_INTERFACE_HPP_
#define BNO08X_DRIVER__SPI_INTERFACE_HPP_

#include <string>
#include <stdexcept>

#include "comm_interface.hpp"

/**
 * @brief SPI communication interface (not implemented)
 */
class SPIInterface : public CommInterface
{
public:
  explicit SPIInterface(const std::string& spi_device) : spi_device_(spi_device)
  {
    throw std::runtime_error("SPI Interface not implemented");
  }

  int open() override
  {
    throw std::runtime_error("SPI Interface not implemented");
  }

  void close() override
  {
    throw std::runtime_error("SPI Interface not implemented");
  }

  int read(uint8_t* pBuffer, unsigned len, uint32_t* t_us) override
  {
    throw std::runtime_error("SPI Interface not implemented");
  }

  int write(uint8_t* pBuffer, unsigned len) override
  {
    throw std::runtime_error("SPI Interface not implemented");
  }

private:
  std::string spi_device_;
};

#endif  // BNO08X_DRIVER__SPI_INTERFACE_HPP_
