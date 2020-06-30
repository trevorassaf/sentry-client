#ifndef SENTRY_CLIENT_DYNAMIXEL_AXA12_H_
#define SENTRY_CLIENT_DYNAMIXEL_AXA12_H_

#include <cstdint>
#include <cstdlib>
#include <memory>
#include <unordered_set>

#include "Gpio/RpiPinManager.h"
#include "Gpio/OutputPin.h"
#include "System/RpiSystemContext.h"
#include "Uart/UartClient.h"
#include "Uart/RpiUartContext.h"

#include "dynamixel/AxA12BaudRate.h"
#include "dynamixel/AxA12InstructionPacket.h"
#include "dynamixel/AxA12ReaderWriter.h"
#include "dynamixel/AxA12RegisterMap.h"
#include "dynamixel/AxA12StatusPacket.h"

namespace dynamixel
{

class AxA12
{
public:
  static bool Create(
      System::RpiSystemContext *system,
      Gpio::RpiPinManager *gpio_manager,
      size_t gpio_index,
      uint8_t id,
      AxA12 *out_axa12);

public:
  AxA12();
  AxA12(
      Uart::UartClient *uart_client,
      std::unique_ptr<Uart::RpiUartContext> uart_context,
      std::unique_ptr<Gpio::OutputPin> gpio,
      uint8_t id);
  AxA12(AxA12 &&other);
  AxA12 &operator=(AxA12 &&other);
  ~AxA12();

  bool GetModelNumber(uint16_t *out_model_number);
  bool GetVersionNumber(uint8_t *out_version_number);
  bool GetId(uint8_t *out_id);
  bool SetId(uint8_t to_id);
  bool GetBaudRate(AxA12BaudRate *out_rate);
  bool SetBaudRate(AxA12BaudRate baud_rate);

  bool GetClockWiseAngleLimit(uint16_t *angle);
  bool SetClockWiseAngleLimit(uint16_t angle);
  bool GetCounterClockWiseAngleLimit(uint16_t *angle);
  bool SetCounterClockWiseAngleLimit(uint16_t angle);

  bool GetUpperTemperatureLimit(uint8_t *out_temp);
  bool SetUpperTemperatureLimit(uint8_t temp);

  bool GetLowerVoltageLimit(uint8_t *out_voltage);
  bool SetLowerVoltageLimit(uint8_t voltage);
  bool GetUpperVoltageLimit(uint8_t *out_voltage);
  bool SetUpperVoltageLimit(uint8_t voltage);

  bool GetMaxTorque(uint16_t *out_torque);
  bool SetMaxTorque(uint16_t torque);

  bool GetLedAlarm(std::unordered_set<AxA12ErrorType> *out_errors);
  bool SetLedAlarm(const std::unordered_set<AxA12ErrorType> &errors);
  bool GetShutdownAlarm(std::unordered_set<AxA12ErrorType> *out_errors);
  bool SetShutdownAlarm(const std::unordered_set<AxA12ErrorType> &errors);

  bool GetLedState(bool *out_enabled);
  bool SetLedState(bool enabled);

  bool GetGoalPosition(uint16_t *out_position);
  bool SetGoalPosition(uint16_t position);
  bool GetMovingSpeed(uint16_t *out_speed);
  bool SetMovingSpeed(uint16_t speed);
  bool GetTorqueLimit(uint16_t *out_limit);
  bool SetTorqueLimit(uint16_t limit);
 
  bool IsMoving(bool *out_moving);
 
private:
  void StealResources(AxA12 *other);
  bool SendPacket(const AxA12InstructionPacket &instruction);
  bool ReadPacket(AxA12StatusPacket *out_status);
  bool BasicWriteWrapper(
      const std::vector<uint8_t> &instruction_parameters);
  bool BasicWriteWrapper(
      AxA12RegisterType reg,
      uint8_t data);
  bool BasicWriteWrapper(
      AxA12RegisterType reg,
      uint16_t data);
  bool BasicReadWrapper(
      const std::vector<uint8_t> &instruction_parameters,
      uint8_t status_parameter_count,
      std::vector<uint8_t> *out_status_parmeters);
  bool BasicReadWrapper(
      AxA12RegisterType reg,
      uint8_t *out_data);
  bool BasicReadWrapper(
      AxA12RegisterType reg,
      uint16_t *out_data);

private:
  bool is_initialized_;
  AxA12ReaderWriter axa12_reader_writer_;
  std::unique_ptr<Uart::RpiUartContext> uart_context_;
  std::unique_ptr<Gpio::OutputPin> gpio_;
  uint8_t id_;
};

} // namespace dynamixel

#endif // SENTRY_CLIENT_DYNAMIXEL_AXA12_H_
