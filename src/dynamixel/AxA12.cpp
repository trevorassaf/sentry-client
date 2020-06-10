#include "dynamixel/AxA12.h"

#include <cassert>
#include <chrono>
#include <memory>
#include <thread>
#include <utility>

#include <glog/logging.h>

#include "Gpio/OutputPin.h"
#include "Gpio/RpiPinManager.h"
#include "System/RpiSystemContext.h"
#include "Uart/RpiUartContext.h"
#include "Uart/UartClient.h"

#include "dynamixel/AxA12ReaderWriter.h"
#include "dynamixel/AxA12RegisterMap.h"

namespace
{
using Gpio::OutputPin;
using Gpio::RpiPinManager;
using System::RpiSystemContext;
using Uart::RpiUartContext;
using Uart::UartClient;

const std::chrono::milliseconds SLEEP_TIME_AFTER_GPIO_ASSERT{100};
const std::chrono::milliseconds SLEEP_TIME_AFTER_UART_OPERATION{100};
}  // namespace

namespace dynamixel
{

bool AxA12::Create(
    RpiSystemContext *system,
    RpiPinManager *gpio_manager,
    size_t gpio_index,
    AxA12 *out_axa12)
{
  assert(system);
  assert(gpio_manager);
  assert(out_axa12);

  auto uart_context = std::make_unique<RpiUartContext>(system);
  UartClient *uart_client = nullptr;
  if (!uart_context->GetSerial0(&uart_client))
  {
    LOG(ERROR) << "Failed to initialize uart context";
    return false;
  }

  std::unique_ptr<OutputPin> gpio_pin =
      gpio_manager->BindOutputPin(gpio_index);

  *out_axa12 = AxA12{
    uart_client,
    std::move(uart_context),
    std::move(gpio_pin)};

  return true;
}

AxA12::AxA12() : is_initialized_{false} {}

AxA12::AxA12(
  UartClient *uart,
  std::unique_ptr<RpiUartContext> uart_context,
  std::unique_ptr<OutputPin> gpio)
: is_initialized_{true},
  axa12_reader_writer_{uart},
  uart_context_{std::move(uart_context)},
  gpio_{std::move(gpio)} {}

AxA12::AxA12(AxA12 &&other)
{
  StealResources(&other);
}

AxA12 &AxA12::operator=(AxA12 &&other)
{
  if (this != &other)
  {
    StealResources(&other);
  }
  return *this;
}

AxA12::~AxA12() {}

bool AxA12::SendPacket(const AxA12Packet &packet)
{
  gpio_->Set();
  std::this_thread::sleep_for(SLEEP_TIME_AFTER_GPIO_ASSERT);

  if (!axa12_reader_writer_.Write(packet))
  {
    LOG(ERROR) << "Failed to write UART message";
    return false;
  }

  std::this_thread::sleep_for(SLEEP_TIME_AFTER_UART_OPERATION);
  gpio_->Clear();

  return true;
}

bool AxA12::Rotate1()
{
  std::vector<uint8_t> parameters =
  {
    static_cast<uint8_t>(AxA12RegisterType::GOAL_POSITION_L),
    0x32,
    0x03,
  };

  AxA12Packet packet{
      AxA12Packet::BROADCAST_ID,
      AxA12InstructionType::WRITE_DATA,
      std::move(parameters)};

  if (!SendPacket(packet))
  {
    LOG(ERROR) << "Failed to send rotate1 packet";
    return false;
  }

  return true;
}

bool AxA12::Rotate2()
{
  std::vector<uint8_t> parameters =
  {
    static_cast<uint8_t>(AxA12RegisterType::GOAL_POSITION_L),
    0xCD,
    0x00,
  };

  AxA12Packet packet{
      AxA12Packet::BROADCAST_ID,
      AxA12InstructionType::WRITE_DATA,
      std::move(parameters)};

  if (!SendPacket(packet))
  {
    LOG(ERROR) << "Failed to send rotate1 packet";
    return false;
  }

  return true;
}

void AxA12::StealResources(AxA12 *other)
{
  assert(other);
  is_initialized_ = other->is_initialized_;
  other->is_initialized_ = false;
  axa12_reader_writer_ = std::move(other->axa12_reader_writer_);
  uart_context_ = std::move(other->uart_context_);
  gpio_ = std::move(other->gpio_);
}

}  // namespace dynamixel
