#include "dynamixel/AxA12InstructionType.h"

#include "glog/logging.h"

namespace dynamixel
{

const char *InstructionTypeToString(AxA12InstructionType type)
{
  switch (type)
  {
    case AxA12InstructionType::PING:
      return "PING";
    case AxA12InstructionType::READ_DATA:
      return "READ_DATA";
    case AxA12InstructionType::WRITE_DATA:
      return "WRITE_DATA";
    case AxA12InstructionType::REG_WRITE:
      return "REG_WRITE";
    case AxA12InstructionType::ACTION:
      return "ACTION";
    case AxA12InstructionType::RESET:
      return "RESET";
    case AxA12InstructionType::SYNC_WRITE:
      return "SYNC_WRITE";
    default:
      LOG(ERROR) << "Unknown AxA12InstructionType: " << static_cast<int>(type);
      return "Invalid instruction type";
  }
}

}  // namespace dynamixel
