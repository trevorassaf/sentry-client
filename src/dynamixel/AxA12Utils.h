#ifndef SENTRY_CLIENT_DYNAMIXEL_AXA12UTILS_H_
#define SENTRY_CLIENT_DYNAMIXEL_AXA12UTILS_H_
#include <cstdint>
#include <cstdlib>
#include <vector>

#include "dynamixel/AxA12InstructionType.h"
#include "dynamixel/AxA12InstructionPacket.h"

namespace dynamixel
{

const std::vector<uint8_t> &GetHeaderBytes();
size_t CalculateWriteBufferSize(size_t parameter_count);
uint8_t CalculatePacketLengthParameter(size_t parameter_count);
uint8_t CalculatePacketCrc(const AxA12InstructionPacket &packet);

} // namespace dynamixel

#endif // SENTRY_CLIENT_DYNAMIXEL_AXA12UTILS_H_
