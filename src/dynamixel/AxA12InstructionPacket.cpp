#include "dynamixel/AxA12InstructionPacket.h"

#include <cassert>
#include <sstream>
#include <vector>

namespace dynamixel
{

constexpr uint8_t AxA12InstructionPacket::BROADCAST_ID;

AxA12InstructionPacket::AxA12InstructionPacket() : is_initialized_{false} {}

AxA12InstructionPacket::AxA12InstructionPacket(
    uint8_t id,
    AxA12InstructionType instruction_type,
    std::vector<uint8_t> parameters)
  : is_initialized_{true},
    id_{id},
    instruction_type_{instruction_type},
    parameters_{std::move(parameters)} {}

uint8_t AxA12InstructionPacket::GetId() const
{
  assert(is_initialized_);
  return id_;
}

AxA12InstructionType AxA12InstructionPacket::GetInstructionType() const
{
  assert(is_initialized_);
  return instruction_type_;
}

const std::vector<uint8_t> &AxA12InstructionPacket::GetParameters() const
{
  assert(is_initialized_);
  return parameters_;
}

std::string AxA12InstructionPacket::ToString() const
{
  std::ostringstream stream;
  stream << "\n - id=" << static_cast<int>(id_);
  stream << "\n - instruction_type=" << InstructionTypeToString(instruction_type_);

  for (size_t i = 0; i < parameters_.size(); ++i)
  {
    stream << "\n - param[" << i << "]=0x" << std::hex
           << static_cast<int>(parameters_.at(i)) << std::dec;
  }

  return stream.str();
}

}  // namespace dynamixel
