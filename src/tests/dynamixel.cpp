#include <cassert>
#include <chrono>
#include <cstdlib>
#include <deque>
#include <iostream>
#include <memory>
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
  if (!AxA12::Create(
          &system_context,
          &gpio_manager,
          FLAGS_gpio,
          &axa12))
  {
    LOG(ERROR) << "Failed to initialize AxA12 servo";
    return EXIT_FAILURE;
  }

  if (!axa12.Rotate(1, static_cast<uint16_t>(FLAGS_rotation)))
  {
    LOG(ERROR) << "Failed to rotate servo 1";
    return EXIT_FAILURE;
  }

  if (!axa12.Rotate(2, static_cast<uint16_t>(FLAGS_rotation)))
  {
    LOG(ERROR) << "Failed to rotate servo 1";
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
