#ifndef SENTRY_CLIENT_DYNAMIXEL_AXA12INSTRUCTIONPACKET_H_
#define SENTRY_CLIENT_DYNAMIXEL_AXA12INSTRUCTIONPACKET_H_

#include <cstdint>
#include <string>
#include <vector>

#include "dynamixel/AxA12InstructionType.h"

namespace dynamixel
{

class AxA12InstructionPacket
{
public:
  AxA12InstructionPacket();
  AxA12InstructionPacket(
      uint8_t id,
      AxA12InstructionType instruction_type,
      std::vector<uint8_t> parameters);
  uint8_t GetId() const;
  AxA12InstructionType GetInstructionType() const;
  const std::vector<uint8_t> &GetParameters() const;
  std::string ToString() const;

public:
  static constexpr uint8_t BROADCAST_ID = 0xFE;

private:
  bool is_initialized_;
  uint8_t id_;
  AxA12InstructionType instruction_type_;
  std::vector<uint8_t> parameters_;
};

} // namespace dynamixel

#endif // SENTRY_CLIENT_DYNAMIXEL_AXA12INSTRUCTIONPACKET_H_
