#ifndef SENTRY_CLIENT_DYNAMIXEL_AXA12INSTRUCTIONTYPE_H_
#define SENTRY_CLIENT_DYNAMIXEL_AXA12INSTRUCTIONTYPE_H_

#include <cstdint>

namespace dynamixel
{

enum class AxA12InstructionType : uint8_t
{
  PING = 0x01,
  READ_DATA = 0x02,
  WRITE_DATA = 0x03,
  REG_WRITE = 0x04,
  ACTION = 0x05,
  RESET = 0x06,
  SYNC_WRITE = 0x83,
};

const char *InstructionTypeToString(AxA12InstructionType type);

} // namespace dynamixel

#endif // SENTRY_CLIENT_DYNAMIXEL_AXA12INSTRUCTIONTYPE_H_
