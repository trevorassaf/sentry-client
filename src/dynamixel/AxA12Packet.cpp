#include "dynamixel/AxA12Packet.h"

#include <cassert>
#include <vector>

namespace dynamixel
{

constexpr uint8_t AxA12Packet::BROADCAST_ID;

AxA12Packet::AxA12Packet() : is_initialized_{false} {}

AxA12Packet::AxA12Packet(
    uint8_t id,
    AxA12InstructionType instruction_type,
    std::vector<uint8_t> parameters)
  : is_initialized_{true},
    id_{id},
    instruction_type_{instruction_type},
    parameters_{std::move(parameters)} {}

uint8_t AxA12Packet::GetId() const
{
  assert(is_initialized_);
  return id_;
}

AxA12InstructionType AxA12Packet::GetInstructionType() const
{
  assert(is_initialized_);
  return instruction_type_;
}

const std::vector<uint8_t> &AxA12Packet::GetParameters() const
{
  assert(is_initialized_);
  return parameters_;
}

}  // namespace dynamixel
