#include <cassert>
#include <chrono>
#include <cstdlib>
#include <deque>
#include <iostream>
#include <memory>
#include <thread>
#include <utility>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "Gpio/OutputPin.h"
#include "Gpio/RpiPinManager.h"
#include "Uart/UartClient.h"
#include "Uart/UartClientFactory.h"
#include "Uart/RpiUartContext.h"
#include "System/RpiSystemContext.h"

namespace
{
using Gpio::OutputPin;
using Gpio::RpiPinManager;
using Uart::RpiUartContext;
using Uart::UartClient;
using System::RpiSystemContext;

DEFINE_bool(state, false, "Way to switch between the two types of instructions");

constexpr uint8_t HEADER_BYTE = 0xFF;
constexpr uint8_t ID_BYTE = 0xFE;
constexpr uint8_t WRITE_INSTRUCTION = 0x03;
constexpr uint8_t GOAL_POSITION_L = 0x1E;

uint8_t CalculateCrc(const std::deque<uint8_t> &bytes)
{
  size_t sum = 0;
  for (uint8_t byte : bytes)
  {
    sum += byte;
  }

  return static_cast<uint8_t>(~sum);
}

bool Write(UartClient *client, const std::deque<uint8_t> &custom_byte_array)
{
  uint8_t length_byte = custom_byte_array.size() + 3;
  std::deque<uint8_t> bytes{
      ID_BYTE,
      length_byte,
      WRITE_INSTRUCTION,
      GOAL_POSITION_L};

  bytes.insert(
      bytes.end(),
      custom_byte_array.begin(),
      custom_byte_array.end());

  uint8_t crc = CalculateCrc(bytes);
  bytes.push_back(crc);
  bytes.push_front(HEADER_BYTE);
  bytes.push_front(HEADER_BYTE);

  for (uint8_t b : bytes)
  {
    std::cout << "Byte: 0x" << std::hex << static_cast<size_t>(b) << std::endl;

    if (!client->Write(&b, sizeof(b)))
    {
      LOG(ERROR) << "Failed to write() byte to uart: "
                 << strerror(errno);
      return false;
    }
  }

  return true;
}

bool Rotate(
    UartClient *client,
    OutputPin *gpio_pin,
    uint8_t position1,
    uint8_t position2)
{

  LOG(INFO) << "Setting GPIO 21 pin";
  gpio_pin->Set();

  if (!Write(client, std::deque<uint8_t>{position1, position2}))
  {
    LOG(ERROR) << "Failed to write UART command to dynamixel servo";
    return false;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds{100});

  LOG(INFO) << "Unsetting GPIO 21 pin";
  gpio_pin->Clear();

  return true;
}

bool Rotate1(UartClient *client, OutputPin *gpio_pin)
{
  return Rotate(client, gpio_pin, 0x32, 0x03);
}

bool Rotate2(UartClient *client, OutputPin *gpio_pin)
{
  return Rotate(client, gpio_pin, 0xCD, 0x00);
}

}  // namespace

int main(int argc, char **argv)
{
  FLAGS_logtostderr = 1;
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InitGoogleLogging(argv[0]);

  RpiSystemContext system_context;
  RpiUartContext uart_context{&system_context};

  LOG(ERROR) << "bozkurtus -- Before initialize RpiPinManager";
  RpiPinManager gpio_manager;
  LOG(ERROR) << "bozkurtus -- Before instantiate OutputPin 21";
  std::unique_ptr<OutputPin> gpio_pin =
      gpio_manager.BindOutputPin(21);

  LOG(ERROR) << "bozkurtus -- Before GetSerial0()";
  //*out_fd = open(UART_DEVICE_NAME, O_RDWR | O_NDELAY | O_NOCTTY);
  UartClient *serial_0 = nullptr;
  if (!uart_context.GetSerial0(&serial_0))
  {
    LOG(ERROR) << "Failed to initialize serial_0";
    return EXIT_FAILURE;
  }

  if (FLAGS_state)
  {
    LOG(INFO) << "bozkurtus -- Rotate1";
    Rotate1(serial_0, gpio_pin.get());
  }
  else
  {
    LOG(INFO) << "bozkurtus -- Rotate2";
    Rotate2(serial_0, gpio_pin.get());
  }

  return EXIT_SUCCESS;
}
