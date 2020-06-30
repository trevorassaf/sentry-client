#include <cassert>
#include <chrono>
#include <cstdlib>
#include <deque>
#include <iostream>
#include <memory>
#include <sstream>
#include <thread>
#include <utility>

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "Gpio/OutputPin.h"
#include "Gpio/RpiPinManager.h"
#include "Uart/UartClient.h"
#include "Uart/UartClientFactory.h"
#include "Uart/RpiUartContext.h"
#include "System/RpiSystemContext.h"

#include "dynamixel/AxA12.h"

namespace
{
using dynamixel::AxA12;
using Gpio::OutputPin;
using Gpio::RpiPinManager;
using Uart::RpiUartContext;
using Uart::UartClient;
using System::RpiSystemContext;

constexpr int kInvalidGpio = -1;
constexpr int kServoNeutralPosition = 512;

bool ValidateGpio(const char *cli_param_name, int32_t pin_index)
{
  if (pin_index == kInvalidGpio)
  {
    LOG(ERROR) << "Invalid -- " << cli_param_name << "=" << pin_index;
    return false;
  }

  return true;
}

bool ValidateRotation(const char *cli_param_name, int32_t rotation)
{
  LOG(ERROR) << "Invalid -- " << cli_param_name << "=" << rotation;

  if (rotation < 0 || rotation > 1023)
  {
    LOG(ERROR) << "Invalid rotation: " << rotation;
    return false;
  }
  return true;
}

DEFINE_int32(gpio, kInvalidGpio, "GPIO pin");
DEFINE_int32(rotation, kServoNeutralPosition, "Rotation in hex (0-1023)");

DEFINE_validator(gpio, &ValidateGpio);
DEFINE_validator(rotation, &ValidateRotation);

bool PrintModelNumber(uint8_t id, AxA12 *axa12)
{
  assert(axa12);

  uint16_t model_number;
  if (!axa12->GetModelNumber(&model_number))
  {
    LOG(ERROR) << "Failed to get model number from axa12 " << static_cast<int>(id);
    return false;
  }

  std::ostringstream stream;
  stream << std::hex << static_cast<size_t>(model_number);
  LOG(ERROR) << "Servo " << static_cast<int>(id) << " model number: 0x" << stream.str();

  return true;
}

bool PrintVersionNumber(uint8_t id, AxA12 *axa12)
{
  assert(axa12);

  uint8_t version_number;
  if (!axa12->GetVersionNumber(&version_number))
  {
    LOG(ERROR) << "Failed to get version number from axa12 " << static_cast<int>(id);
    return false;
  }

  std::ostringstream stream;
  stream << std::hex << static_cast<size_t>(version_number);
  LOG(ERROR) << "Servo " << static_cast<int>(id) << " version number: 0x" << stream.str();

  return true;
}

}  // namespace

int main(int argc, char **argv)
{
  FLAGS_logtostderr = 1;
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InitGoogleLogging(argv[0]);

  RpiSystemContext system_context;
  RpiPinManager gpio_manager;

  LOG(ERROR) << "Gpio index: " << FLAGS_gpio;
  LOG(ERROR) << "Rotation: " << FLAGS_rotation;

  AxA12 axa12;
  uint8_t id = 1;
  if (!AxA12::Create(
          &system_context,
          &gpio_manager,
          FLAGS_gpio,
          id,
          &axa12))
  {
    LOG(ERROR) << "Failed to initialize AxA12 servo";
    return false;
  }

  if (!PrintModelNumber(1, &axa12))
  {
    LOG(ERROR) << "Failed to print model number for servo 1";
    return EXIT_FAILURE;
  }

  if (!PrintVersionNumber(1, &axa12))
  {
    LOG(ERROR) << "Failed to print version number for servo 1";
    return EXIT_FAILURE;
  }

  uint8_t read_id;
  if (!axa12.GetId(&read_id))
  {
    LOG(ERROR) << "Failed to read id for id=1";
    return EXIT_FAILURE;
  }

  LOG(ERROR) << "Id for servo with id=1: " << static_cast<int>(read_id);

  if (!axa12.SetGoalPosition(static_cast<uint16_t>(FLAGS_rotation)))
  {
    LOG(ERROR) << "Failed to rotate servo 1";
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
