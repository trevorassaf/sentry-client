#ifndef SENTRY_CLIENT_DYNAMIXEL_AXA12READERWRITER_H_
#define SENTRY_CLIENT_DYNAMIXEL_AXA12READERWRITER_H_

#include <cstdint>
#include <string>
#include <vector>
#include <unordered_map>

#include "Uart/UartClient.h"

#include "dynamixel/AxA12InstructionType.h"
#include "dynamixel/AxA12Packet.h"

namespace dynamixel
{

class AxA12ReaderWriter
{
public:
  AxA12ReaderWriter();
  AxA12ReaderWriter(Uart::UartClient *uart);
  AxA12ReaderWriter(AxA12ReaderWriter &&other);
  AxA12ReaderWriter &operator=(AxA12ReaderWriter &&other);
  bool Write(const AxA12Packet &packet);

private:
  void StealResources(AxA12ReaderWriter *other);

private:
  bool is_initialized_;
  Uart::UartClient *uart_;
};

} // namespace dynamixel

#endif // SENTRY_CLIENT_DYNAMIXEL_AXA12READERWRITER_H_
