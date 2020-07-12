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

const std::chrono::microseconds SLEEP_TIME_AFTER_WRITE_OPERATION{100};
const std::chrono::milliseconds SLEEP_TIME_BEFORE_READ_OPERATION{1};

constexpr uint16_t MAX_TORQUE = 0x3FF;

}  // namespace

namespace dynamixel
{

AxA12::AxA12() : is_initialized_{false} {}

AxA12::AxA12(
  UartClient *uart,
  OutputPin *gpio,
  uint8_t id)
: is_initialized_{true},
  axa12_reader_writer_{uart},
  gpio_{gpio},
  id_{id} {}

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

bool AxA12::GetModelNumber(uint16_t *out_model_number)
{
  if (id_ == AxA12InstructionPacket::BROADCAST_ID)
  {
    LOG(ERROR) << "GetModelNumber() unsupprted for broadcast ID: "
               << static_cast<size_t>(id_);
    return false;
  }

  assert(out_model_number);
  if (!BasicReadWrapper(
          AxA12RegisterType::MODEL_NUMBER_L,
          out_model_number))
  {
    LOG(ERROR) << "Failed to read model number";
    return false;
  }

  return true;
}

bool AxA12::GetVersionNumber(uint8_t *out_version_number)
{
  assert(out_version_number);

  if (id_ == AxA12InstructionPacket::BROADCAST_ID)
  {
    LOG(ERROR) << "GetVersionNumber() unsupprted for broadcast ID: "
               << static_cast<size_t>(id_);
    return false;
  }

  if (!BasicReadWrapper(
          AxA12RegisterType::FW_VERSION,
          out_version_number))
  {
    LOG(ERROR) << "Failed to read version number";
    return false;
  }

  return true;
}

bool AxA12::GetId(uint8_t *out_id)
{
  assert(out_id);

  if (id_ == AxA12InstructionPacket::BROADCAST_ID)
  {
    *out_id = id_;
    return true;
  }

  if (!BasicReadWrapper(
          AxA12RegisterType::ID,
          out_id))
  {
    LOG(ERROR) << "Failed to read id";
    return false;
  }

  if (*out_id != id_)
  {
    LOG(ERROR) << "Empirical ID " << static_cast<size_t>(id_)
               << " does not match expected ID " << static_cast<size_t>(*out_id);
    return false;
  }

  return true;
}

bool AxA12::SetId(uint8_t to_id)
{
  if (id_ == AxA12InstructionPacket::BROADCAST_ID)
  {
    LOG(ERROR) << "SetId() unsupprted for broadcast ID: "
               << static_cast<size_t>(id_);
    return false;
  }

  if (!BasicWriteWrapper(
          AxA12RegisterType::ID,
          to_id))
  {
    LOG(ERROR) << "Failed to change ID=" << static_cast<int>(id_) << " to ID="
               << static_cast<int>(to_id);
    return false;
  }

  id_ = to_id;
  return true;
}

bool AxA12::GetBaudRate(AxA12BaudRate *out_rate)
{
  assert(out_rate);

  uint8_t rate_value;
  if (!BasicReadWrapper(
          AxA12RegisterType::BAUD_RATE,
          &rate_value))
  {
    LOG(ERROR) << "Failed to read baud rate";
    return false;
  }

  *out_rate = static_cast<AxA12BaudRate>(rate_value);
  return true;
}

bool AxA12::SetBaudRate(AxA12BaudRate rate)
{
  if (!BasicWriteWrapper(
          AxA12RegisterType::BAUD_RATE,
          static_cast<uint8_t>(rate)))
  {
    LOG(ERROR) << "Failed to set baud rate: " << ToString(rate);
    return false;
  }

  return true;
}

bool AxA12::GetClockWiseAngleLimit(uint16_t *out_angle)
{
  assert(out_angle);
  if (!BasicReadWrapper(
          AxA12RegisterType::CW_ANGLE_LIMIT_L,
          out_angle))
  {
    LOG(ERROR) << "Failed to read clockwise angle limit";
    return false;
  }

  return true;
}

bool AxA12::SetClockWiseAngleLimit(uint16_t angle)
{
  if (!BasicWriteWrapper(
          AxA12RegisterType::CW_ANGLE_LIMIT_L,
          angle))
  {
    LOG(ERROR) << "Failed to write clockwise angle limit: " << static_cast<int>(angle);
    return false;
  }

  return true;
}

bool AxA12::GetCounterClockWiseAngleLimit(uint16_t *out_angle)
{
  assert(out_angle);
  if (!BasicReadWrapper(
          AxA12RegisterType::CCW_ANGLE_LIMIT_L,
          out_angle))
  {
    LOG(ERROR) << "Failed to read counter clockwise angle limit";
    return false;
  }

  return true;
}

bool AxA12::SetCounterClockWiseAngleLimit(uint16_t angle)
{
  if (!BasicWriteWrapper(
          AxA12RegisterType::CCW_ANGLE_LIMIT_L,
          angle))
  {
    LOG(ERROR) << "Failed to write counter clockwise angle limit: " << static_cast<int>(angle);
    return false;
  }

  return true;
}

bool AxA12::GetUpperTemperatureLimit(uint8_t *out_temp)
{
  assert(out_temp);
  if (!BasicReadWrapper(
          AxA12RegisterType::HIGHEST_LIMIT_TEMPERATURE,
          out_temp))
  {
    LOG(ERROR) << "Failed to read upper temperature limit";
    return false;
  }

  return true;
}

bool AxA12::SetUpperTemperatureLimit(uint8_t temp)
{
  if (!BasicWriteWrapper(
          AxA12RegisterType::HIGHEST_LIMIT_TEMPERATURE,
          temp))
  {
    LOG(ERROR) << "Failed to set upper temperature limit: " << static_cast<int>(temp);
    return false;
  }

  return true;
}

bool AxA12::GetLowerVoltageLimit(uint8_t *out_voltage)
{
  assert(out_voltage);
  if (!BasicReadWrapper(
          AxA12RegisterType::LOWEST_LIMIT_VOLTAGE,
          out_voltage))
  {
    LOG(ERROR) << "Failed to read read lower voltage limit";
    return false;
  }

  return true;
}

bool AxA12::SetLowerVoltageLimit(uint8_t voltage)
{
  if (!BasicWriteWrapper(
          AxA12RegisterType::LOWEST_LIMIT_VOLTAGE,
          voltage))
  {
    LOG(ERROR) << "Failed to write lower voltage limit: " << static_cast<int>(voltage);
    return false;
  }

  return true;
}

bool AxA12::GetUpperVoltageLimit(uint8_t *out_voltage)
{
  assert(out_voltage);
  if (!BasicReadWrapper(
          AxA12RegisterType::HIGHEST_LIMIT_VOLTAGE,
          out_voltage))
  {
    LOG(ERROR) << "Failed to read upper voltage limit";
    return false;
  }

  return true;
}

bool AxA12::SetUpperVoltageLimit(uint8_t voltage)
{
  if (!BasicWriteWrapper(
          AxA12RegisterType::HIGHEST_LIMIT_VOLTAGE,
          voltage))
  {
    LOG(ERROR) << "Failed to write upper voltage limit: " << static_cast<int>(voltage);
    return false;
  }

  return true;
}

bool AxA12::GetMaxTorque(uint16_t *out_torque)
{
  assert(out_torque);
  if (!BasicReadWrapper(
          AxA12RegisterType::MAX_TORQUE_L,
          out_torque))
  {
    LOG(ERROR) << "Failed to read max torque";
    return false;
  }

  return true;
}

bool AxA12::SetMaxTorque(uint16_t torque)
{
  if (torque > MAX_TORQUE)
  {
    LOG(ERROR) << "Requested torque " << static_cast<int>(torque) << " exceeds max "
               << "torque value " << MAX_TORQUE;
    return false;
  }

  if (!BasicWriteWrapper(
          AxA12RegisterType::MAX_TORQUE_L,
          torque))
  {
    LOG(ERROR) << "Failed to write max torque: " << static_cast<int>(torque);
    return false;
  }

  return true;
}

bool AxA12::GetLedAlarm(std::unordered_set<AxA12ErrorType> *out_errors)
{
  assert(out_errors);

  uint8_t errors;
  if (!BasicReadWrapper(
          AxA12RegisterType::ALARM_LED,
          &errors))
  {
    LOG(ERROR) << "Failed to read LED alarm";
    return false;
  }

  *out_errors = FromBitset(errors);
  return true;
}

bool AxA12::SetLedAlarm(const std::unordered_set<AxA12ErrorType> &errors)
{
  if (!BasicWriteWrapper(
          AxA12RegisterType::ALARM_LED,
          ToBitset(errors)))
  {
    LOG(ERROR) << "Failed to set LED alarm";
    return false;
  }

  return true;
}

bool AxA12::GetShutdownAlarm(std::unordered_set<AxA12ErrorType> *out_errors)
{
  assert(out_errors);

  uint8_t errors;
  if (!BasicReadWrapper(
          AxA12RegisterType::ALARM_SHUTDOWN,
          &errors))
  {
    LOG(ERROR) << "Failed to read shutdown alarm";
    return false;
  }

  *out_errors = FromBitset(errors);
  return true;
}

bool AxA12::SetShutdownAlarm(const std::unordered_set<AxA12ErrorType> &errors)
{
  if (!BasicWriteWrapper(
          AxA12RegisterType::ALARM_SHUTDOWN,
          ToBitset(errors)))
  {
    LOG(ERROR) << "Failed to set shutdown alarm";
    return false;
  }

  return true;
}

bool AxA12::GetLedState(bool *out_enabled)
{
  assert(out_enabled);
  uint8_t enabled;
  if (!BasicReadWrapper(
          AxA12RegisterType::LED,
          &enabled))
  {
    LOG(ERROR) << "Failed to read LED state";
    return false;
  }

  *out_enabled = static_cast<bool>(enabled);
  return true;
}

bool AxA12::SetLedState(bool enabled)
{
  if (!BasicWriteWrapper(
          AxA12RegisterType::LED,
          static_cast<uint8_t>(enabled)))
  {
    LOG(ERROR) << "Failed to set LED state: " << static_cast<int>(enabled);
    return false;
  }

  return true;
}

bool AxA12::GetGoalPosition(uint16_t *out_position)
{
  assert(out_position);
  if (!BasicReadWrapper(
          AxA12RegisterType::GOAL_POSITION_L,
          out_position))
  {
    LOG(ERROR) << "Failed to read goal position";
    return false;
  }

  return true;
}

bool AxA12::SetGoalPosition(uint16_t rotation)
{
  if (id_ == AxA12InstructionPacket::BROADCAST_ID)
  {
    LOG(ERROR) << "SetGoalPosition() unsupprted for broadcast ID: "
               << static_cast<size_t>(id_);
    return false;
  }

  uint8_t most_significant_byte = rotation >> 8;
  uint8_t least_significant_byte = rotation & 0xFF;

  std::vector<uint8_t> parameters =
  {
    static_cast<uint8_t>(AxA12RegisterType::GOAL_POSITION_L),
    least_significant_byte,
    most_significant_byte,
  };

  if (!BasicWriteWrapper(parameters))
  {
    LOG(ERROR) << "Failed to write goal position registers";
    return false;
  }

  return true;
}

bool AxA12::GetMovingSpeed(uint16_t *out_position)
{
  assert(out_position);
  if (!BasicReadWrapper(
          AxA12RegisterType::MOVING_SPEED_L,
          out_position))
  {
    LOG(ERROR) << "Failed to read moving speed";
    return false;
  }

  return true;
}

bool AxA12::SetMovingSpeed(uint16_t speed)
{
  if (!BasicWriteWrapper(
          AxA12RegisterType::MOVING_SPEED_L,
          speed))
  {
    LOG(ERROR) << "Failed to set moving speed: " << static_cast<int>(speed);
    return false;
  }

  return true;
}

bool AxA12::GetTorqueLimit(uint16_t *out_limit)
{
  assert(out_limit);
  if (!BasicReadWrapper(
          AxA12RegisterType::TORQUE_LIMIT_L,
          out_limit))
  {
    LOG(ERROR) << "Failed to read torque limit";
    return false;
  }

  return true;
}

bool AxA12::SetTorqueLimit(uint16_t limit)
{
  if (!BasicWriteWrapper(
          AxA12RegisterType::TORQUE_LIMIT_L,
          limit))
  {
    LOG(ERROR) << "Failed to set torque limit: " << static_cast<int>(limit);
    return false;
  }

  return true;
}

bool AxA12::IsMoving(bool *out_moving)
{
  uint8_t is_moving = 0;
  if (!BasicReadWrapper(
          AxA12RegisterType::IS_MOVING,
          &is_moving))
  {
    LOG(ERROR) << "Failed to read is moving register";
    return false;
  }

  *out_moving = static_cast<bool>(is_moving);
  return true;
}

void AxA12::StealResources(AxA12 *other)
{
  assert(other);
  is_initialized_ = other->is_initialized_;
  other->is_initialized_ = false;
  axa12_reader_writer_ = std::move(other->axa12_reader_writer_);
  gpio_ = std::move(other->gpio_);
  id_ = other->id_;
}

bool AxA12::SendPacket(const AxA12InstructionPacket &packet)
{
  gpio_->Set();
  std::this_thread::sleep_for(SLEEP_TIME_AFTER_WRITE_OPERATION);

  LOG(ERROR) << "AxA12 Instruction Packet dump: " << packet.ToString();

  if (!axa12_reader_writer_.Write(packet))
  {
    LOG(ERROR) << "Failed to write UART message";
    return false;
  }

  std::this_thread::sleep_for(SLEEP_TIME_AFTER_WRITE_OPERATION);
  return true;
}

bool AxA12::ReadPacket(AxA12StatusPacket *out_status)
{
  assert(id_ != AxA12InstructionPacket::BROADCAST_ID);
  assert(out_status);

  gpio_->Clear();

  std::this_thread::sleep_for(SLEEP_TIME_BEFORE_READ_OPERATION);

  if (!axa12_reader_writer_.Read(out_status))
  {
    LOG(ERROR) << "Failed to read UART message";
    return false;
  }

  LOG(ERROR) << "AxA12 Status Packet Dump: " << out_status->ToString();
  std::this_thread::sleep_for(SLEEP_TIME_AFTER_WRITE_OPERATION);
  return true;
}

bool AxA12::BasicWriteWrapper(
    const std::vector<uint8_t> &instruction_parameters)
{
  AxA12InstructionPacket packet{
      id_,
      AxA12InstructionType::WRITE_DATA,
      instruction_parameters};

  if (!SendPacket(packet))
  {
    LOG(ERROR) << "Failed to send packet";
    return false;
  }

  if (id_ == AxA12InstructionPacket::BROADCAST_ID)
  {
    return true;
  }

  AxA12StatusPacket status;
  if (!ReadPacket(&status))
  {
    LOG(ERROR) << "Failed to read status packet";
    return false;
  }

  return true;
}

bool AxA12::BasicWriteWrapper(
    AxA12RegisterType reg,
    uint8_t data)
{
  std::vector<uint8_t> parameters =
  {
    static_cast<uint8_t>(reg),
    data
  };
  return BasicWriteWrapper(parameters);
}

bool AxA12::BasicWriteWrapper(
    AxA12RegisterType reg,
    uint16_t data)
{
  uint8_t least_significant_byte = static_cast<uint8_t>(data);
  uint8_t most_significant_byte = static_cast<uint8_t>(data >> 8);
  std::vector<uint8_t> parameters =
  {
    static_cast<uint8_t>(reg),
    least_significant_byte,
    most_significant_byte,
  };
  return BasicWriteWrapper(parameters);
}

bool AxA12::BasicReadWrapper(
    const std::vector<uint8_t> &instruction_parameters,
    uint8_t status_parameter_count,
    std::vector<uint8_t> *out_status_parameters)
{
  assert(out_status_parameters);

  AxA12InstructionPacket packet{
      id_,
      AxA12InstructionType::READ_DATA,
      instruction_parameters};

  if (!SendPacket(packet))
  {
    LOG(ERROR) << "Failed to send read operation instruction";
    return false;
  }

  AxA12StatusPacket status;
  if (!ReadPacket(&status))
  {
    LOG(ERROR) << "Failed to read status packet";
    return false;
  }

  if (status.GetHeader().HasError())
  {
    LOG(ERROR) << "Status packet contains error";
    return false;
  }

  if (status.GetParameters().size() != status_parameter_count)
  {
    LOG(ERROR) << "Expected parameter count: " << status_parameter_count
               << ". Empirical parameter count: " << status.GetParameters().size();
    return false;
  }

  *out_status_parameters = status.GetParameters();
  return true;
}

bool AxA12::BasicReadWrapper(
    AxA12RegisterType reg,
    uint8_t *out_data)
{
  assert(out_data);

  std::vector<uint8_t> instruction_parameters =
  {
    static_cast<uint8_t>(reg),
    1
  };

  std::vector<uint8_t> status_parameters;
  if (!BasicReadWrapper(
          instruction_parameters,
          1,
          &status_parameters))
  {
    LOG(ERROR) << "Failed to read single byte";
    return false;
  }

  *out_data = status_parameters.at(0);
  return true;
}

bool AxA12::BasicReadWrapper(
    AxA12RegisterType reg,
    uint16_t *out_data)
{
  assert(out_data);

  std::vector<uint8_t> instruction_parameters =
  {
    static_cast<uint8_t>(reg),
    2
  };

  std::vector<uint8_t> status_parameters;
  if (!BasicReadWrapper(
          instruction_parameters,
          2,
          &status_parameters))
  {
    LOG(ERROR) << "Failed to read single byte";
    return false;
  }

  *out_data = status_parameters.at(0);
  *out_data |= static_cast<uint16_t>(status_parameters.at(1)) << 8;
  return true;
}

}  // namespace dynamixel
