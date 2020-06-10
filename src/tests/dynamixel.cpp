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

bool ValidateGpio(const char *cli_param_name, int32_t pin_index)
{
  if (pin_index == kInvalidGpio)
  {
    LOG(ERROR) << "Invalid --" << cli_param_name << "=" << pin_index;
    return false;
  }

  return true;
}

DEFINE_bool(state, false, "Way to switch between the two types of instructions");
DEFINE_int32(gpio, -1, "GPIO pin");

DEFINE_validator(gpio, &ValidateGpio);
}  // namespace

int main(int argc, char **argv)
{
  FLAGS_logtostderr = 1;
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InitGoogleLogging(argv[0]);

  RpiSystemContext system_context;
  RpiPinManager gpio_manager;

  LOG(ERROR) << "State: " << static_cast<int>(FLAGS_state);
  LOG(ERROR) << "Gpio index: " << FLAGS_gpio;

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

  if (FLAGS_state)
  {
    if (!axa12.Rotate1())
    {
      LOG(ERROR) << "Failed to rotate 1";
      return EXIT_FAILURE;
    }
  }
  else
  {
    if (!axa12.Rotate2())
    {
      LOG(ERROR) << "Failed to rotate 2";
      return EXIT_FAILURE;
    }
  }

  return EXIT_SUCCESS;
}
