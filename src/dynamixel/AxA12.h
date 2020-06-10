#ifndef SENTRY_CLIENT_DYNAMIXEL_AXA12_H_
#define SENTRY_CLIENT_DYNAMIXEL_AXA12_H_

#include <cstdint>
#include <cstdlib>
#include <memory>

#include "Gpio/RpiPinManager.h"
#include "Gpio/OutputPin.h"
#include "System/RpiSystemContext.h"
#include "Uart/UartClient.h"
#include "Uart/RpiUartContext.h"

#include "dynamixel/AxA12Packet.h"
#include "dynamixel/AxA12ReaderWriter.h"

namespace dynamixel
{

class AxA12
{
public:
  static bool Create(
      System::RpiSystemContext *system,
      Gpio::RpiPinManager *gpio_manager,
      size_t gpio_index,
      AxA12 *out_axa12);

public:
  AxA12();
  AxA12(
      Uart::UartClient *uart_client,
      std::unique_ptr<Uart::RpiUartContext> uart_context,
      std::unique_ptr<Gpio::OutputPin> gpio);
  AxA12(AxA12 &&other);
  AxA12 &operator=(AxA12 &&other);
  ~AxA12();

  bool Rotate1();
  bool Rotate2();
 
private:
  void StealResources(AxA12 *other);
  bool SendPacket(const AxA12Packet &packet);

private:
  bool is_initialized_;
  AxA12ReaderWriter axa12_reader_writer_;
  std::unique_ptr<Uart::RpiUartContext> uart_context_;
  std::unique_ptr<Gpio::OutputPin> gpio_;
};

} // namespace dynamixel

#endif // SENTRY_CLIENT_DYNAMIXEL_AXA12_H_
