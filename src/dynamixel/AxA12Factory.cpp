#include "dynamixel/AxA12Factory.h"

#include <cassert>
#include <utility>

#include "glog/logging.h"

namespace
{
using Gpio::OutputPin;
using Gpio::RpiPinManager;
using System::RpiSystemContext;
using Uart::RpiUartContext;
using Uart::UartClient;
}  // namespace

namespace dynamixel
{

bool AxA12Factory::Create(
    RpiSystemContext *system,
    RpiPinManager *gpio_manager,
    size_t gpio_index,
    AxA12Factory* out_factory)
{
  LOG(ERROR) << "bozkurtus -- AxA12Factory::Create() -- call";

  assert(system);
  assert(gpio_manager);
  assert(out_factory);

  auto uart_context = std::make_unique<RpiUartContext>(system);
  UartClient *uart_client = nullptr;
  if (!uart_context->GetSerial0(&uart_client))
  {
    LOG(ERROR) << "Failed to initialize uart context";
    return false;
  }

  std::unique_ptr<OutputPin> gpio_pin;
  try
  {
    gpio_pin = gpio_manager->BindOutputPin(gpio_index);
  }
  catch (...)
  {
    LOG(ERROR) << "Failed to create output pin w/index=" << gpio_index;
    return false;
  }

  *out_factory = AxA12Factory{
      std::move(uart_context),
      uart_client,
      std::move(gpio_pin)};

  LOG(ERROR) << "bozkurtus -- AxA12Factory::Create() -- end";
  return true;
}

AxA12Factory::AxA12Factory() : is_initialized_{false} {}

AxA12Factory::AxA12Factory(
    std::unique_ptr<Uart::RpiUartContext> uart_context,
    UartClient *uart_client,
    std::unique_ptr<OutputPin> gpio_pin)
  : is_initialized_{true},
    uart_context_{std::move(uart_context)},
    uart_client_{uart_client},
    gpio_pin_{std::move(gpio_pin)} {}

AxA12Factory::AxA12Factory(AxA12Factory&& other)
{
  StealResources(&other);
}

AxA12Factory& AxA12Factory::operator=(AxA12Factory&& other)
{
  if (&other != this)
  {
    StealResources(&other);
  }
  return *this;
}

bool AxA12Factory::Get(uint8_t id, AxA12** out_axa12)
{
  LOG(ERROR) << "bozkurtus -- AxA12Factory::Get() -- call";

  assert(is_initialized_);
  assert(out_axa12);

  if (axa12_table_.count(id) != 0)
  {
    *out_axa12 = &axa12_table_.at(id);
    return true;
  }

  AxA12 axa12{uart_client_, gpio_pin_.get(), id};
  axa12_table_.emplace(id, std::move(axa12));
  *out_axa12 = &axa12_table_.at(id);
  LOG(ERROR) << "bozkurtus -- AxA12Factory::Get() -- end";

  return true;
}

void AxA12Factory::StealResources(AxA12Factory *other)
{
  assert(other);
  is_initialized_ = other->is_initialized_;
  other->is_initialized_ = false;
  uart_context_ = std::move(other->uart_context_);
  uart_client_ = other->uart_client_;
  other->uart_client_ = nullptr;
  gpio_pin_ = std::move(other->gpio_pin_);
  axa12_table_ = std::move(other->axa12_table_);
}

}  // namespace dynamixel
