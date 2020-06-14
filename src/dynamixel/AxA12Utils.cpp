#include "dynamixel/AxA12Utils.h"

#include <cassert>
#include <cstdint>
#include <cstdlib>
#include <vector>

#include "dynamixel/AxA12InstructionType.h"
#include "dynamixel/AxA12InstructionPacket.h"

namespace
{
const std::vector<uint8_t> HEADER_BYTES{0xFF, 0xFF};
/**
 * Two bytes for 0xFFFF header plus 4 bytes for id, length, instruction, and checksum.
 */
const size_t WRITE_BUFFER_SIZE_SANS_PARAMETERS = HEADER_BYTES.size() + 4;
constexpr size_t PARAMETER_COUNT_OFFSET = 2;
}  // namespace

namespace dynamixel
{

const std::vector<uint8_t> &GetHeaderBytes()
{
  return HEADER_BYTES;
}

size_t CalculateWriteBufferSize(size_t parameter_count)
{
  return WRITE_BUFFER_SIZE_SANS_PARAMETERS + parameter_count;
}

uint8_t CalculatePacketLengthParameter(size_t parameter_count)
{
  return parameter_count + PARAMETER_COUNT_OFFSET;
}

uint8_t CalculatePacketCrc(const AxA12InstructionPacket &packet)
{
  uint8_t scratch =
      packet.GetId() +
      static_cast<uint8_t>(packet.GetInstructionType()) +
      CalculatePacketLengthParameter(packet.GetParameters().size());

  for (uint8_t p : packet.GetParameters())
  {
    scratch += p;
  }

  return ~scratch;
}

}  // namespace dynamixel
