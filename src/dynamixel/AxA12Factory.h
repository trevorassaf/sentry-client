#ifndef SENTRY_CLIENT_DYNAMIXEL_AXA12FACTORY_H_
#define SENTRY_CLIENT_DYNAMIXEL_AXA12FACTORY_H_

#include <cstdint>
#include <memory>
#include <unordered_map>

#include "Gpio/OutputPin.h"
#include "Gpio/RpiPinManager.h"
#include "System/RpiSystemContext.h"
#include "Uart/RpiUartContext.h"
#include "Uart/UartClient.h"

#include "dynamixel/AxA12.h"

namespace dynamixel
{

class AxA12Factory
{
public:
  static bool Create(
      System::RpiSystemContext *system,
      Gpio::RpiPinManager *gpio_manager,
      size_t gpio_index,
      AxA12Factory* out_factory);

public:
  AxA12Factory();
  AxA12Factory(
      std::unique_ptr<Uart::RpiUartContext> uart_context,
      Uart::UartClient *uart_client,
      std::unique_ptr<Gpio::OutputPin> gpio_pin);
  AxA12Factory(AxA12Factory&& other);
  AxA12Factory& operator=(AxA12Factory&& other);

  bool Get(uint8_t id, AxA12** out_axa12);

private:
  AxA12Factory(const AxA12Factory& other) = delete;
  AxA12Factory& operator=(const AxA12Factory& other) = delete;

private:
  void StealResources(AxA12Factory* other);

private:
  bool is_initialized_;
  std::unique_ptr<Uart::RpiUartContext> uart_context_;
  Uart::UartClient *uart_client_;
  std::unique_ptr<Gpio::OutputPin> gpio_pin_;
  std::unordered_map<uint8_t, AxA12> axa12_table_;
};

}  // namespace dynamixel

#endif // SENTRY_CLIENT_DYNAMIXEL_AXA12FACTORY_H_
